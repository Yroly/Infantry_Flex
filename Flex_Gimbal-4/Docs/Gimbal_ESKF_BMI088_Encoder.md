# Gimbal ESKF for BMI088 + Encoder

这份文档给出一套可直接移植到 STM32 工程里的云台 `ESKF` 实现。

目标：

- 预测：使用 `BMI088` 陀螺仪
- 俯仰/横滚修正：使用 `BMI088` 加速度计
- 航向修正：使用云台 `Yaw` 电机编码器
- 输出：四元数、欧拉角、连续航向角、去零偏后的角速度

适用场景：

- 两轴云台
- `BMI088` 安装在云台上
- `Yaw` 轴有连续编码器角度可读
- 控制中仍然使用陀螺角速度做速率环

注意：

- 这套代码是独立框架，不依赖你工程里现有的 `QuaternionEKF`
- 接入前必须确认 `BMI088` 三轴方向和编码器方向
- 编码器角度必须是连续角，不能是跳变的 `0~360`

---

## 1. 设计说明

名义状态：

```text
q = [qw qx qy qz]^T
bg = [bgx bgy bgz]^T
```

误差状态：

```text
dx = [dtheta_x dtheta_y dtheta_z dbg_x dbg_y dbg_z]^T
```

滤波逻辑：

1. 用陀螺积分四元数做预测
2. 用加速度重力方向更新 `roll/pitch`
3. 用编码器 `yaw` 更新航向
4. 用误差状态修正名义四元数和陀螺零偏

---

## 2. 头文件

建议新建文件：`Components/Algorithm/GimbalESKF.h`

```c
#ifndef GIMBAL_ESKF_H
#define GIMBAL_ESKF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float q[4];              // quaternion: [w x y z]
    float gyro_bias[3];      // rad/s
    float gyro_unbias[3];    // rad/s

    float roll;              // deg
    float pitch;             // deg
    float yaw;               // deg, [-180, 180]
    float yaw_total;         // deg, continuous

    float P[6][6];

    float gyro_noise;        // rad^2/s
    float bias_noise;        // (rad/s)^2/s
    float acc_noise;         // normalized gravity measurement noise
    float yaw_noise;         // rad^2

    float g_norm;            // usually 9.81
    float acc_gate;          // acceleration gate, e.g. 1.5

    float yaw_last_deg;
    int32_t yaw_round_count;

    uint8_t initialized;
} GimbalESKF_t;

void GimbalESKF_Init(GimbalESKF_t *eskf);

void GimbalESKF_SetNoise(GimbalESKF_t *eskf,
                         float gyro_noise,
                         float bias_noise,
                         float acc_noise,
                         float yaw_noise);

void GimbalESKF_Predict(GimbalESKF_t *eskf,
                        float gx, float gy, float gz,
                        float dt);

void GimbalESKF_UpdateAccel(GimbalESKF_t *eskf,
                            float ax, float ay, float az);

void GimbalESKF_UpdateYawEncoder(GimbalESKF_t *eskf,
                                 float yaw_encoder_rad);

void GimbalESKF_Update(GimbalESKF_t *eskf,
                       float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float yaw_encoder_rad,
                       uint8_t use_encoder_yaw,
                       float dt);

void GimbalESKF_GetEulerDeg(GimbalESKF_t *eskf,
                            float *roll_deg,
                            float *pitch_deg,
                            float *yaw_deg,
                            float *yaw_total_deg);

#ifdef __cplusplus
}
#endif

#endif
```

---

## 3. 源文件

建议新建文件：`Components/Algorithm/GimbalESKF.c`

```c
#include "GimbalESKF.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD (0.01745329251994329577f)
#define RAD2DEG (57.2957795130823208768f)

static float clampf(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static float wrap_pi(float x)
{
    while (x > M_PI)  x -= 2.0f * M_PI;
    while (x < -M_PI) x += 2.0f * M_PI;
    return x;
}

static float inv_sqrtf_safe(float x)
{
    if (x <= 1e-12f) return 0.0f;
    return 1.0f / sqrtf(x);
}

static void vec3_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static float vec3_norm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void quat_normalize(float q[4])
{
    float n_inv = inv_sqrtf_safe(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n_inv <= 0.0f)
    {
        q[0] = 1.0f;
        q[1] = q[2] = q[3] = 0.0f;
        return;
    }
    q[0] *= n_inv;
    q[1] *= n_inv;
    q[2] *= n_inv;
    q[3] *= n_inv;
}

static void quat_multiply(const float q1[4], const float q2[4], float out[4])
{
    out[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    out[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    out[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    out[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

static void quat_from_small_angle(const float dtheta[3], float dq[4])
{
    dq[0] = 1.0f;
    dq[1] = 0.5f * dtheta[0];
    dq[2] = 0.5f * dtheta[1];
    dq[3] = 0.5f * dtheta[2];
    quat_normalize(dq);
}

static void quat_integrate_gyro(float q[4], const float w[3], float dt)
{
    float angle = vec3_norm(w) * dt;
    float dq[4];
    float q_new[4];

    if (angle < 1e-6f)
    {
        dq[0] = 1.0f;
        dq[1] = 0.5f * w[0] * dt;
        dq[2] = 0.5f * w[1] * dt;
        dq[3] = 0.5f * w[2] * dt;
    }
    else
    {
        float half = 0.5f * angle;
        float s = sinf(half) / vec3_norm(w);
        dq[0] = cosf(half);
        dq[1] = w[0] * s;
        dq[2] = w[1] * s;
        dq[3] = w[2] * s;
    }

    quat_multiply(q, dq, q_new);
    memcpy(q, q_new, sizeof(q_new));
    quat_normalize(q);
}

static void quat_to_euler_deg(const float q[4], float *roll, float *pitch, float *yaw)
{
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    float sinp = 2.0f * (qw * qy - qz * qx);
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);

    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;
    *pitch = asinf(clampf(sinp, -1.0f, 1.0f)) * RAD2DEG;
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
}

static void quat_rotate_world_to_body_gravity(const float q[4], float g_body[3])
{
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    g_body[0] = 2.0f * (qx * qz - qw * qy);
    g_body[1] = 2.0f * (qw * qx + qy * qz);
    g_body[2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

static void mat6_identity(float A[6][6])
{
    uint8_t i, j;
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            A[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

static void mat6_mul(float A[6][6], float B[6][6], float C[6][6])
{
    uint8_t i, j, k;
    float tmp[6][6] = {0};
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            for (k = 0; k < 6; k++)
            {
                tmp[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    memcpy(C, tmp, sizeof(tmp));
}

static void mat6_transpose(float A[6][6], float AT[6][6])
{
    uint8_t i, j;
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            AT[j][i] = A[i][j];
        }
    }
}

static void eskf_update_3d(GimbalESKF_t *eskf,
                           const float H[3][6],
                           const float residual[3],
                           float R_diag)
{
    uint8_t i, j, k;
    float PHt[6][3] = {0};
    float S[3][3] = {0};
    float S_inv[3][3] = {0};
    float K[6][3] = {0};
    float dx[6] = {0};
    float I_KH[6][6];
    float KH[6][6] = {0};
    float P_new[6][6];

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 3; j++)
        {
            for (k = 0; k < 6; k++)
            {
                PHt[i][j] += eskf->P[i][k] * H[j][k];
            }
        }
    }

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            for (k = 0; k < 6; k++)
            {
                S[i][j] += H[i][k] * PHt[k][j];
            }
            if (i == j) S[i][j] += R_diag;
        }
    }

    {
        float a = S[0][0], b = S[0][1], c = S[0][2];
        float d = S[1][0], e = S[1][1], f = S[1][2];
        float g = S[2][0], h = S[2][1], ii = S[2][2];
        float det = a * (e * ii - f * h) - b * (d * ii - f * g) + c * (d * h - e * g);
        if (fabsf(det) < 1e-9f) return;

        S_inv[0][0] =  (e * ii - f * h) / det;
        S_inv[0][1] = -(b * ii - c * h) / det;
        S_inv[0][2] =  (b * f - c * e) / det;
        S_inv[1][0] = -(d * ii - f * g) / det;
        S_inv[1][1] =  (a * ii - c * g) / det;
        S_inv[1][2] = -(a * f - c * d) / det;
        S_inv[2][0] =  (d * h - e * g) / det;
        S_inv[2][1] = -(a * h - b * g) / det;
        S_inv[2][2] =  (a * e - b * d) / det;
    }

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 3; j++)
        {
            for (k = 0; k < 3; k++)
            {
                K[i][j] += PHt[i][k] * S_inv[k][j];
            }
        }
    }

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 3; j++)
        {
            dx[i] += K[i][j] * residual[j];
        }
    }

    {
        float dq[4];
        quat_from_small_angle(dx, dq);
        quat_multiply(eskf->q, dq, eskf->q);
        quat_normalize(eskf->q);
        eskf->gyro_bias[0] += dx[3];
        eskf->gyro_bias[1] += dx[4];
        eskf->gyro_bias[2] += dx[5];
    }

    mat6_identity(I_KH);

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            for (k = 0; k < 3; k++)
            {
                KH[i][j] += K[i][k] * H[k][j];
            }
            I_KH[i][j] -= KH[i][j];
        }
    }

    mat6_mul(I_KH, eskf->P, P_new);
    memcpy(eskf->P, P_new, sizeof(P_new));
}

static void eskf_update_1d(GimbalESKF_t *eskf,
                           const float H[6],
                           float residual,
                           float R)
{
    uint8_t i, j;
    float PHt[6] = {0};
    float K[6] = {0};
    float S = 0.0f;
    float dx[6] = {0};
    float I_KH[6][6];
    float P_new[6][6];

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            PHt[i] += eskf->P[i][j] * H[j];
        }
    }

    for (i = 0; i < 6; i++)
    {
        S += H[i] * PHt[i];
    }
    S += R;
    if (fabsf(S) < 1e-9f) return;

    for (i = 0; i < 6; i++)
    {
        K[i] = PHt[i] / S;
        dx[i] = K[i] * residual;
    }

    {
        float dq[4];
        quat_from_small_angle(dx, dq);
        quat_multiply(eskf->q, dq, eskf->q);
        quat_normalize(eskf->q);
        eskf->gyro_bias[0] += dx[3];
        eskf->gyro_bias[1] += dx[4];
        eskf->gyro_bias[2] += dx[5];
    }

    mat6_identity(I_KH);
    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 6; j++)
        {
            I_KH[i][j] -= K[i] * H[j];
        }
    }

    mat6_mul(I_KH, eskf->P, P_new);
    memcpy(eskf->P, P_new, sizeof(P_new));
}

static void GimbalESKF_UpdateEuler(GimbalESKF_t *eskf)
{
    float yaw_deg;
    quat_to_euler_deg(eskf->q, &eskf->roll, &eskf->pitch, &yaw_deg);
    eskf->yaw = yaw_deg;

    if ((yaw_deg - eskf->yaw_last_deg) > 180.0f)
    {
        eskf->yaw_round_count--;
    }
    else if ((yaw_deg - eskf->yaw_last_deg) < -180.0f)
    {
        eskf->yaw_round_count++;
    }

    eskf->yaw_total = 360.0f * (float)eskf->yaw_round_count + yaw_deg;
    eskf->yaw_last_deg = yaw_deg;
}

void GimbalESKF_Init(GimbalESKF_t *eskf)
{
    memset(eskf, 0, sizeof(GimbalESKF_t));

    eskf->q[0] = 1.0f;
    eskf->g_norm = 9.81f;
    eskf->acc_gate = 1.5f;

    eskf->gyro_noise = 2e-3f;
    eskf->bias_noise = 1e-5f;
    eskf->acc_noise = 2e-2f;
    eskf->yaw_noise = 1e-3f;

    eskf->P[0][0] = 1e-2f;
    eskf->P[1][1] = 1e-2f;
    eskf->P[2][2] = 1e-2f;
    eskf->P[3][3] = 1e-3f;
    eskf->P[4][4] = 1e-3f;
    eskf->P[5][5] = 1e-3f;

    eskf->initialized = 1;
}

void GimbalESKF_SetNoise(GimbalESKF_t *eskf,
                         float gyro_noise,
                         float bias_noise,
                         float acc_noise,
                         float yaw_noise)
{
    eskf->gyro_noise = gyro_noise;
    eskf->bias_noise = bias_noise;
    eskf->acc_noise = acc_noise;
    eskf->yaw_noise = yaw_noise;
}

void GimbalESKF_Predict(GimbalESKF_t *eskf,
                        float gx, float gy, float gz,
                        float dt)
{
    uint8_t i;
    float w[3];
    float F[6][6];
    float FT[6][6];
    float Q[6][6] = {0};
    float temp[6][6];

    if (!eskf->initialized || dt <= 0.0f) return;

    w[0] = gx - eskf->gyro_bias[0];
    w[1] = gy - eskf->gyro_bias[1];
    w[2] = gz - eskf->gyro_bias[2];

    eskf->gyro_unbias[0] = w[0];
    eskf->gyro_unbias[1] = w[1];
    eskf->gyro_unbias[2] = w[2];

    quat_integrate_gyro(eskf->q, w, dt);

    mat6_identity(F);
    F[0][1] =  w[2] * dt;
    F[0][2] = -w[1] * dt;
    F[1][0] = -w[2] * dt;
    F[1][2] =  w[0] * dt;
    F[2][0] =  w[1] * dt;
    F[2][1] = -w[0] * dt;

    F[0][3] = -dt;
    F[1][4] = -dt;
    F[2][5] = -dt;

    for (i = 0; i < 3; i++)
    {
        Q[i][i] = eskf->gyro_noise * dt;
        Q[i + 3][i + 3] = eskf->bias_noise * dt;
    }

    mat6_transpose(F, FT);
    mat6_mul(F, eskf->P, temp);
    mat6_mul(temp, FT, eskf->P);

    for (i = 0; i < 6; i++)
    {
        eskf->P[i][i] += Q[i][i];
    }

    GimbalESKF_UpdateEuler(eskf);
}

void GimbalESKF_UpdateAccel(GimbalESKF_t *eskf,
                            float ax, float ay, float az)
{
    float acc[3] = {ax, ay, az};
    float acc_norm = vec3_norm(acc);
    float z[3];
    float g_body[3];
    float residual[3];
    float H[3][6] = {0};

    if (!eskf->initialized) return;
    if (acc_norm < 1e-4f) return;
    if (fabsf(acc_norm - eskf->g_norm) > eskf->acc_gate) return;

    z[0] = ax / acc_norm;
    z[1] = ay / acc_norm;
    z[2] = az / acc_norm;

    quat_rotate_world_to_body_gravity(eskf->q, g_body);

    residual[0] = z[0] - g_body[0];
    residual[1] = z[1] - g_body[1];
    residual[2] = z[2] - g_body[2];

    H[0][1] = -g_body[2];
    H[0][2] =  g_body[1];
    H[1][0] =  g_body[2];
    H[1][2] = -g_body[0];
    H[2][0] = -g_body[1];
    H[2][1] =  g_body[0];

    eskf_update_3d(eskf, H, residual, eskf->acc_noise);
    GimbalESKF_UpdateEuler(eskf);
}

void GimbalESKF_UpdateYawEncoder(GimbalESKF_t *eskf,
                                 float yaw_encoder_rad)
{
    float yaw_est_rad;
    float residual;
    float H[6] = {0};

    if (!eskf->initialized) return;

    yaw_est_rad = eskf->yaw * DEG2RAD;
    residual = wrap_pi(yaw_encoder_rad - yaw_est_rad);

    H[2] = 1.0f;

    eskf_update_1d(eskf, H, residual, eskf->yaw_noise);
    GimbalESKF_UpdateEuler(eskf);
}

void GimbalESKF_Update(GimbalESKF_t *eskf,
                       float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float yaw_encoder_rad,
                       uint8_t use_encoder_yaw,
                       float dt)
{
    GimbalESKF_Predict(eskf, gx, gy, gz, dt);
    GimbalESKF_UpdateAccel(eskf, ax, ay, az);
    if (use_encoder_yaw)
    {
        GimbalESKF_UpdateYawEncoder(eskf, yaw_encoder_rad);
    }
}

void GimbalESKF_GetEulerDeg(GimbalESKF_t *eskf,
                            float *roll_deg,
                            float *pitch_deg,
                            float *yaw_deg,
                            float *yaw_total_deg)
{
    if (roll_deg) *roll_deg = eskf->roll;
    if (pitch_deg) *pitch_deg = eskf->pitch;
    if (yaw_deg) *yaw_deg = eskf->yaw;
    if (yaw_total_deg) *yaw_total_deg = eskf->yaw_total;
}
```

---

## 4. FreeRTOS / INS 线程接入示例

如果你想直接替换当前 `INS_Task()`，可以按下面这个方式改。

```c
#include "GimbalESKF.h"
#include "bsp_dwt.h"
#include "BMI088driver.h"
#include "dm_motor.h"

GimbalESKF_t gimbal_eskf;

extern BMI088_Data_t BMI088;
extern DM4310_TypeDef GimYaw;

static uint32_t ins_dwt_cnt = 0;

static float yaw_encoder_offset_deg = 0.0f;
static float yaw_encoder_dir = 1.0f;   // 反向就改成 -1.0f

static float YawEncoderToRad(void)
{
    float yaw_deg = yaw_encoder_dir * GimYaw.Angle_DEG + yaw_encoder_offset_deg;
    return yaw_deg * DEG2RAD;
}

void INS_Init(void)
{
    GimbalESKF_Init(&gimbal_eskf);

    // 这四个参数后面要根据你的云台调
    GimbalESKF_SetNoise(&gimbal_eskf,
                        2e-3f,   // gyro_noise
                        1e-5f,   // bias_noise
                        2e-2f,   // acc_noise
                        8e-4f);  // yaw_noise
}

void INS_Task(void)
{
    float dt;
    float yaw_enc_rad;

    dt = DWT_GetDeltaT(&ins_dwt_cnt);
    BMI088_Read(&BMI088);

    // 这里的轴映射必须按你的BMI088安装方向调整
    // 下面只是示例，不保证和你板子完全一致
    {
        float gx = BMI088.Gyro[0];
        float gy = BMI088.Gyro[1];
        float gz = BMI088.Gyro[2];
        float ax = BMI088.Accel[0];
        float ay = BMI088.Accel[1];
        float az = BMI088.Accel[2];

        yaw_enc_rad = YawEncoderToRad();

        GimbalESKF_Update(&gimbal_eskf,
                          gx, gy, gz,
                          ax, ay, az,
                          yaw_enc_rad,
                          1,
                          dt);
    }

    IMU.Angle_Roll = gimbal_eskf.roll;
    IMU.Angle_Pitch = gimbal_eskf.pitch;
    IMU.Angle_Yaw = gimbal_eskf.yaw;
    IMU.Angle_Yawcontinuous = gimbal_eskf.yaw_total;

    IMU.Gyro_Roll = gimbal_eskf.gyro_unbias[0];
    IMU.Gyro_Pitch = gimbal_eskf.gyro_unbias[1];
    IMU.Gyro_Yaw = gimbal_eskf.gyro_unbias[2];
}
```

---

## 5. 你工程里最关键的三处适配

### 5.1 BMI088 轴方向

这套 ESKF 能不能直接稳，第一件事不是滤波参数，而是轴方向必须对。

你要确认：

- `gx/gy/gz` 的正方向
- `ax/ay/az` 的正方向
- 输出欧拉角里哪一轴对应你的云台 `pitch/yaw`

如果方向错了，现象一般是：

- 静止时 `pitch/roll` 会缓慢跑飞
- 左右转云台时 `yaw` 越修越偏

### 5.2 编码器方向和零点

这两个参数必须调：

```c
static float yaw_encoder_offset_deg = 0.0f;
static float yaw_encoder_dir = 1.0f;
```

判断方法：

- 手动让云台向“正 yaw”方向转动
- 看 `GimYaw.Angle_DEG` 是增大还是减小
- 如果方向反了，把 `yaw_encoder_dir` 改成 `-1.0f`

然后让云台朝正前方时：

- 记录此时编码器角
- 通过 `yaw_encoder_offset_deg` 把它对齐到 IMU 的 `yaw = 0`

### 5.3 连续角

你工程里的 `DM4310_Receive()` 已经算出了连续角：

```c
Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
```

这个正好能直接拿来做 `yaw` 观测，比单圈角好得多。

---

## 6. 初始参数建议

先用这一组：

```c
gyro_noise = 2e-3f;
bias_noise = 1e-5f;
acc_noise  = 2e-2f;
yaw_noise  = 8e-4f;
acc_gate   = 1.5f;
g_norm     = 9.81f;
```

如果现象是：

- `pitch/roll` 抖：把 `acc_noise` 调大
- `yaw` 跟编码器不够紧：把 `yaw_noise` 调小
- `yaw` 被编码器拖得太死：把 `yaw_noise` 调大
- 长时间有缓慢偏航：把 `bias_noise` 适当调大一点

---

## 7. 推荐控制结构

云台控制建议保持：

- 位置环：`IMU.Angle_Yawcontinuous` / `IMU.Angle_Pitch`
- 速度环：`IMU.Gyro_Yaw` / `IMU.Gyro_Pitch`

不要改成：

- 速度环用编码器速度代替陀螺

原因很简单：

- 编码器速度带宽低、量化重
- 陀螺更适合做内环

---

## 8. 直接替换当前解算的最小步骤

1. 新建 `GimbalESKF.h`
2. 新建 `GimbalESKF.c`
3. 在你的 `INS_Init()` 里调用 `GimbalESKF_Init()`
4. 在你的 `INS_Task()` 里：
   - 读 `BMI088`
   - 读 `GimYaw.Angle_DEG`
   - 调 `GimbalESKF_Update()`
5. 用 `gimbal_eskf` 输出替换当前 `IMU.Angle_*` 和 `IMU.Gyro_*`

---

## 9. 重要说明

这份代码已经能直接作为工程骨架使用，但“直接拿来就稳”的前提是：

- BMI088 轴向映射正确
- 编码器方向正确
- 编码器零点和 IMU 参考方向对齐

如果你愿意继续，我下一步最有效的是直接按你当前工程的文件结构，把这套代码拆成：

- [GimbalESKF.h](/E:/Code/Infantry_Flex/Flex_Gimbal/Components/Algorithm/GimbalESKF.h)
- [GimbalESKF.c](/E:/Code/Infantry_Flex/Flex_Gimbal/Components/Algorithm/GimbalESKF.c)
- 修改 [ins_task.c](/E:/Code/Infantry_Flex/Flex_Gimbal/Init_Ctrl/ins_task.c)

这样你就不是“看文档自己搬”，而是我直接帮你落到工程里。
