#ifndef PID_H
#define PID_H
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
#define limitmax(input, max)   	 \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
typedef struct{
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;
	fp32 Kf;
	
  fp32 max;	
  fp32 measure;
	fp32 last_measure;
  fp32 target;

  fp32 out;  
	fp32 P_out;
  fp32 I_out;
  fp32 D_out;
	fp32 F_out;
  fp32 Dbuf[3];
  fp32 error[3];
} PidTypeDef;

class PID_Class{
public:
    void Init(PidTypeDef *pid,fp32 Kp,fp32 Ki,fp32 Kd,fp32 Kf,fp32 max);
    fp32 Calc(PidTypeDef *pid,fp32 measure,fp32 target);
    void Clear(PidTypeDef *pid);
};

extern PID_Class PID;

#endif
