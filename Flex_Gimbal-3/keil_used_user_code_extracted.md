# Keil工程中实际纳入工程的自定义代码提取

- 工程文件: `MDK-ARM/C_Board_Standard_Robot.uvprojx`
- 提取范围: Keil工程里已添加的源码/头文件
- 提取规则: 自定义目录保留整文件，`Src/Inc` 仅保留非空且有实际内容的 `USER CODE` 区块

## USER CODE: Src/main.c

### BLOCK: Includes

```c
#include "BMI088driver.h"
#include "bsp_dwt.h"
#include "stdio.h"
#include "Gimbal.h"
#include "Variate.h"
```

### BLOCK: PV

```c
extern TaskHandle_t Music_Task_handle;
```

### BLOCK: 2

```c
DWT_Init(168);
    while (BMI088Init(&hspi1, 1) != BMI088_NO_ERROR);
```

### BLOCK: Error_Handler_Debug

```c
/* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
```

## USER CODE: Src/freertos.c

### BLOCK: FunctionPrototypes

```c
void INS_Init();
void INS_Task();
void Init_Task();
```

### BLOCK: StartINSTask

```c
INS_Init();
  /* Infinite loop */
  for(;;)
  {
	  INS_Task();
    osDelay(1);
  }
```

### BLOCK: StartInit_Task

```c
MX_USB_DEVICE_Init();
     Init_Task();
  /* Infinite loop */
```

## USER CODE: Src/usbd_conf.c

### BLOCK: PFP

```c
/* Private function prototypes -----------------------------------------------*/
USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);
```

### BLOCK: 2

```c
if (hpcd->Init.low_power_enable)
  {
    /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register. */
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
```

## USER CODE: Src/usbd_cdc_if.c

### BLOCK: PV

```c
/* Private variables ---------------------------------------------------------*/
#define MAX_RESEND_CNT 8192
```

### BLOCK: 3

```c
/* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
```

### BLOCK: 4

```c
return (USBD_OK);
```

### BLOCK: 5

```c
switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
```

### BLOCK: 6

```c
USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
```

### BLOCK: 7

```c
USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
```

### BLOCK: 13

```c
UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
```

### BLOCK: PRIVATE_FUNCTIONS_IMPLEMENTATION

```c
uint8_t USB_Transmit(uint8_t* Buf, uint16_t Len)
{
    uint16_t resend_cnt = 0;
    uint8_t usb_send_state = USBD_FAIL;
    while (usb_send_state != USBD_OK && resend_cnt < MAX_RESEND_CNT) {
        usb_send_state = CDC_Transmit_FS(Buf, Len);
        resend_cnt++;
    }
    return usb_send_state;
}

int8_t USB_Receive(uint8_t* Buf, uint32_t *Len){
    return CDC_Receive_FS(Buf,Len);
}
```

## USER CODE: Src/stm32f4xx_it.c

### BLOCK: Includes

```c
#include "Variate.h"
#include "remote_control.h"
```

### BLOCK: 0

```c
WatchDog_TypeDef Remote_Dog;
```

### BLOCK: NonMaskableInt_IRQn 1

```c
while (1)
  {
  }
```

### BLOCK: USART3_IRQn 0

```c
if(huart3.Instance->SR & UART_FLAG_RXNE)//锟斤拷锟斤拷锟斤拷?锟絬
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);
		Feed_Dog(&Remote_Dog);
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //锟斤拷锟斤拷DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //?锟斤拷锟斤拷锟斤拷?锟絬?锟斤拷,?锟斤拷 = ?锟絯?锟斤拷 - 锟窖?锟斤拷
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //锟斤拷锟絪?锟絯?锟絬?锟斤拷
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //?锟絯???1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //锟较拷DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
				Remote_Rx(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //锟斤拷锟斤拷DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //?锟斤拷锟斤拷锟斤拷?锟絬?锟斤拷,?锟斤拷 = ?锟絯?锟斤拷 - 锟窖?锟斤拷
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //锟斤拷锟絪?锟絯?锟絬?锟斤拷
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //?锟絯???0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //锟较拷DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //?锟絲?锟斤拷锟斤拷?锟絬
				Remote_Rx(sbus_rx_buf[1]);
            }
        }
    }

#if 0
```

### BLOCK: USART3_IRQn 1

```c
#endif
```

### BLOCK: DMA2_Stream7_IRQn 1

```c
BaseType_t xHigherPriorityTaskWoken;
	//printf("ooooooo\r\n");
```

## FILE: Bsp/bsp_dwt.c

```c
/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bsp_dwt.h"

DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;
static void DWT_CNT_Update(void);

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 浣胯兘DWT澶栬 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT瀵勫瓨鍣ㄨ鏁版竻0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 浣胯兘Cortex-M DWT CYCCNT瀵勫瓨鍣?*/
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;

    CYCCNT_LAST = cnt_now;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
    {
    }
}
```

## FILE: Bsp/bsp_PWM.c

```c
/**
  ******************************************************************************
  * @file	 bsp_PWM.c
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/3/1
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "bsp_PWM.h"

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;
        break;
    }
}
```

## FILE: Bsp/bsp_rc.c

```c
#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);


}
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);

}
```

## FILE: Bsp/bsp_buzzer.c

```c
#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
void buzzer_note(uint16_t note,float volume)
{
    if(volume > 1.0f)
    {
        volume = 1.0f;
    }else if(volume < 0.0f)
    {
        volume = 0.0f;
    }
    // 禁用定时器
    __HAL_TIM_DISABLE(&htim4);

    // 重置定时器计数器
    htim4.Instance->CNT = 0;
    
    // 设置自动重装载寄存器（ARR），以控制PWM信号的频率
    htim4.Instance->ARR = (8*21000 / note - 1) * 1u;
    
    // 设置比较寄存器（CCR3），以控制PWM信号的占空比
    htim4.Instance->CCR3 = (8*10500 / note - 1) * volume * 1u;
    
    // 重新启用定时器
    __HAL_TIM_ENABLE(&htim4);
    
    // 启动PWM信号
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
```

## FILE: Components/Variate.c

```c
#include "Variate.h"
int StuckFlag = 0;
int16_t pluck_speed;
uint8_t GimbalInitFlag = 0;

Gimbal_data_t Gimbal_data;
Gimbal_action_t Gimbal_action;
Chassis_RefereeMsg_t Referee_data_Rx;
Chassis_Msg_t Chassis_data_Rx;

uint8_t NormalModeFlag = 0,GyroscopeModeFlag = 0;
eSystemState SystemState;
DeviceStates DeviceState;
eMidMode MidMode = FRONT;
```

## FILE: Components/Time.c

```c
#include "Time.h"
#include "Variate.h"
eTime Time;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }

	if(htim == &htim3)  //判断中断是否来自于定时器3     1000hz
	{
		 if(GimbalInitFlag == 1) Time.GimbalInit++;
		 if(StuckFlag == 1) Time.ShootStuck++;
		 Time.Single++;

	}
}
```

## FILE: Components/kalman_filter.c

```c
/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   C implementation of kalman filter
 ******************************************************************************
 * @attention
 * 璇ュ崱灏旀浖婊ゆ尝鍣ㄥ彲浠ュ湪浼犳劅鍣ㄩ噰鏍烽鐜囦笉鍚岀殑鎯呭喌涓嬶紝鍔ㄦ€佽皟鏁寸煩闃礖 R鍜孠鐨勭淮鏁颁笌鏁板€笺€? * This implementation of kalman filter can dynamically adjust dimension and
 * value of matrix H R and K according to the measurement validity under any
 * circumstance that the sampling rate of component sensors are different.
 *
 * 鍥犳鐭╅樀H鍜孯鐨勫垵濮嬪寲浼氫笌鐭╅樀P A鍜孮鏈夋墍涓嶅悓銆傚彟澶栫殑锛屽湪鍒濆鍖栭噺娴嬪悜閲弞鏃堕渶瑕侀澶栧啓
 * 鍏ヤ紶鎰熷櫒閲忔祴鎵€瀵瑰簲鐨勭姸鎬佷笌杩欎釜閲忔祴鐨勬柟寮忥紝璇︽儏璇疯渚嬬▼
 * Therefore, the initialization of matrix P, F, and Q is sometimes different
 * from that of matrices H R. when initialization. Additionally, the corresponding
 * state and the method of the measurement should be provided when initializing
 * measurement vector z. For more details, please see the example.
 *
 * 鑻ヤ笉闇€瑕佸姩鎬佽皟鏁撮噺娴嬪悜閲弞锛屽彲绠€鍗曞皢缁撴瀯浣撲腑鐨刄se_Auto_Adjustment鍒濆鍖栦负0锛屽苟鍍忓垵
 * 濮嬪寲鐭╅樀P閭ｆ牱鐢ㄥ父瑙勬柟寮忓垵濮嬪寲z H R鍗冲彲銆? * If automatic adjustment is not required, assign zero to the UseAutoAdjustment
 * and initialize z H R in the normal way as matrix P.
 *
 * 瑕佹眰閲忔祴鍚戦噺z涓庢帶鍒跺悜閲弖鍦ㄤ紶鎰熷櫒鍥炶皟鍑芥暟涓洿鏂般€傛暣鏁?鎰忓懗鐫€閲忔祴鏃犳晥锛屽嵆鑷笂娆″崱灏旀浖
 * 婊ゆ尝鏇存柊鍚庢棤浼犳劅鍣ㄦ暟鎹洿鏂般€傚洜姝ら噺娴嬪悜閲弞涓庢帶鍒跺悜閲弖浼氬湪鍗″皵鏇兼护娉㈡洿鏂拌繃绋嬩腑琚竻闆? * MeasuredVector and ControlVector are required to be updated in the sensor
 * callback function. Integer 0 in measurement vector z indicates the invalidity
 * of current measurement, so MeasuredVector and ControlVector will be reset
 * (to 0) during each update.
 *
 * 姝ゅ锛岀煩闃礟杩囧害鏀舵暃鍚庢护娉㈠櫒灏嗛毦浠ュ啀閫傚簲鐘舵€佺殑缂撴參鍙樺寲锛屼粠鑰屼骇鐢熸护娉及璁″亸宸€傝绠楁硶
 * 閫氳繃闄愬埗鐭╅樀P鏈€灏忓€肩殑鏂规硶锛屽彲鏈夋晥鎶戝埗婊ゆ尝鍣ㄧ殑杩囧害鏀舵暃锛岃鎯呰瑙佷緥绋嬨€? * Additionally, the excessive convergence of matrix P will make filter incapable
 * of adopting the slowly changing state. This implementation can effectively
 * suppress filter excessive convergence through boundary limiting for matrix P.
 * For more details, please see the example.
 *
 * @example:
 * x =
 *   |   height   |
 *   |  velocity  |
 *   |acceleration|
 *
 * KalmanFilter_t Height_KF;
 *
 * void INS_Task_Init(void)
 * {
 *     static float P_Init[9] =
 *     {
 *         10, 0, 0,
 *         0, 30, 0,
 *         0, 0, 10,
 *     };
 *     static float F_Init[9] =
 *     {
 *         1, dt, 0.5*dt*dt,
 *         0, 1, dt,
 *         0, 0, 1,
 *     };
 *     static float Q_Init[9] =
 *     {
 *         0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt,
 *         0.5*dt*dt*dt,        dt*dt,         dt,
 *         0.5*dt*dt,              dt,         1,
 *     };
 *
 *     // 璁剧疆鏈€灏忔柟宸? *     static float state_min_variance[3] = {0.03, 0.005, 0.1};
 *
 *     // 寮€鍚嚜鍔ㄨ皟鏁? *     Height_KF.UseAutoAdjustment = 1;
 *
 *     // 姘斿帇娴嬪緱楂樺害 GPS娴嬪緱楂樺害 鍔犻€熷害璁℃祴寰梲杞磋繍鍔ㄥ姞閫熷害
 *     static uint8_t measurement_reference[3] = {1, 1, 3}
 *
 *     static float measurement_degree[3] = {1, 1, 1}
 *     // 鏍规嵁measurement_reference涓巑easurement_degree鐢熸垚H鐭╅樀濡備笅锛堝湪褰撳墠鍛ㄦ湡鍏ㄩ儴娴嬮噺鏁版嵁鏈夋晥鎯呭喌涓嬶級
 *       |1   0   0|
 *       |1   0   0|
 *       |0   0   1|
 *
 *     static float mat_R_diagonal_elements = {30, 25, 35}
 *     //鏍规嵁mat_R_diagonal_elements鐢熸垚R鐭╅樀濡備笅锛堝湪褰撳墠鍛ㄦ湡鍏ㄩ儴娴嬮噺鏁版嵁鏈夋晥鎯呭喌涓嬶級
 *       |30   0   0|
 *       | 0  25   0|
 *       | 0   0  35|
 *
 *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 *
 *     // 璁剧疆鐭╅樀鍊? *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
 *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
 *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
 *     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
 *     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
 *     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
 *     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 * }
 *
 * void INS_Task(void const *pvParameters)
 * {
 *     // 寰幆鏇存柊
 *     Kalman_Filter_Update(&Height_KF);
 *     vTaskDelay(ts);
 * }
 *
 * // 娴嬮噺鏁版嵁鏇存柊搴旀寜鐓т互涓嬪舰寮?鍗冲悜MeasuredVector璧嬪€? * void Barometer_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[0] = baro_height;
 * }
 * void GPS_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[1] = GPS_height;
 * }
 * void Acc_Data_Process(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[2] = acc.z;
 * }
 ******************************************************************************
 */

#include "kalman_filter.h"

uint16_t sizeof_float, sizeof_double;

static void H_K_R_Adjustment(KalmanFilter_t *kf);

/**
 * @brief 鍒濆鍖栫煩闃电淮搴︿俊鎭苟涓虹煩闃靛垎閰嶇┖闂? *
 * @param kf kf绫诲瀷瀹氫箟
 * @param xhatSize 鐘舵€佸彉閲忕淮搴? * @param uSize 鎺у埗鍙橀噺缁村害
 * @param zSize 瑙傛祴閲忕淮搴? */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;

    // measurement flags
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // filter data
    kf->FilteredValue = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // xhat x(k|k)
    kf->xhat_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (float *)kf->xhat_data);

    // xhatminus x(k|k-1)
    kf->xhatminus_data = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (float *)kf->xhatminus_data);

    if (uSize != 0)
    {
        // control vector u
        kf->u_data = (float *)user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (float *)kf->u_data);
    }

    // measurement vector z
    kf->z_data = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (float *)kf->z_data);

    // covariance matrix P(k|k)
    kf->P_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (float *)kf->P_data);

    // create covariance matrix P(k|k-1)
    kf->Pminus_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Pminus_data);

    // state transition matrix F FT
    kf->F_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (float *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (float *)kf->FT_data);

    if (uSize != 0)
    {
        // control matrix B
        kf->B_data = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (float *)kf->B_data);
    }

    // measurement matrix H
    kf->H_data = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (float *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (float *)kf->HT_data);

    // process noise covariance matrix Q
    kf->Q_data = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (float *)kf->Q_data);

    // measurement noise covariance matrix R
    kf->R_data = (float *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (float *)kf->R_data);

    // kalman gain K
    kf->K_data = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (float *)kf->K_data);

    kf->S_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float *)user_malloc(sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (float *)user_malloc(sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (float *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (float *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (float *)kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // 鐭╅樀H K R鏍规嵁閲忔祴鎯呭喌鑷姩璋冩暣
    // matrix H K R auto adjustment
    if (kf->UseAutoAdjustment != 0)
        H_K_R_Adjustment(kf);
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H路P'(k)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H路P'(k)路HT
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H路P'(k)路HT + R)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)路HT
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H路xhat'(k)
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)路(z(k) - H路xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)路H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)路H路P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
 * @brief 鎵ц鍗″皵鏇兼护娉㈤粍閲戜簲寮?鎻愪緵浜嗙敤鎴峰畾涔夊嚱鏁?鍙互鏇夸唬浜斾釜涓殑浠绘剰涓€涓幆鑺?鏂逛究鑷鎵╁睍涓篍KF/UKF/ESKF/AUKF绛? * 
 * @param kf kf绫诲瀷瀹氫箟
 * @return float* 杩斿洖婊ゆ尝鍊? */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. 鑾峰彇閲忔祴淇℃伅
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 鍏堥獙浼拌
    // 1. xhat'(k)= A路xhat(k-1) + B路u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 棰勬祴鏇存柊
    // 2. P'(k) = A路P(k-1)路AT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 閲忔祴鏇存柊
        // 3. K(k) = P'(k)路HT / (H路P'(k)路HT + R)
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 铻嶅悎
        // 4. xhat(k) = xhat'(k) + K(k)路(z(k) - H路xhat'(k))
        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 淇鏂瑰樊
        // 5. P(k) = (1-K(k)路H)路P'(k) ==> P(k) = P'(k)-K(k)路H路P'(k)
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // 鏃犳湁鏁堥噺娴?浠呴娴?        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    // 鑷畾涔夊嚱鏁?鍙互鎻愪緵鍚庡鐞嗙瓑
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 閬垮厤婊ゆ尝鍣ㄨ繃搴︽敹鏁?    // suppress filter excessive convergence
    for (uint8_t i = 0; i < kf->xhatSize; i++)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // 璇嗗埆閲忔祴鏁版嵁鏈夋晥鎬у苟璋冩暣鐭╅樀H R K
    // recognize measurement validity and adjust matrices H R K
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; i++)
    {
        if (kf->z_data[i] != 0)
        {
            // 閲嶆瀯鍚戦噺z
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // 閲嶆瀯鐭╅樀H
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++)
    {
        // 閲嶆瀯鐭╅樀R
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // 璋冩暣鐭╅樀缁存暟
    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
```

## FILE: Components/user_lib.c

```c
/**
  ******************************************************************************
  * @file	 user_lib.c
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2021/2/18
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"
#include "math.h"
#include "main.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

uint8_t GlobalDebugMode = 7;

//蹇€熷紑鏂?float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

//蹇€熸眰骞虫柟鏍瑰€掓暟
/*
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86- (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}*/

/**
  * @brief          鏂滄尝鍑芥暟鍒濆鍖?  * @author         RM
  * @param[in]      鏂滄尝鍑芥暟缁撴瀯浣?  * @param[in]      闂撮殧鐨勬椂闂达紝鍗曚綅 s
  * @param[in]      鏈€澶у€?  * @param[in]      鏈€灏忓€?  * @retval         杩斿洖绌?  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          鏂滄尝鍑芥暟璁＄畻锛屾牴鎹緭鍏ョ殑鍊艰繘琛屽彔鍔狅紝 杈撳叆鍗曚綅涓?/s 鍗充竴绉掑悗澧炲姞杈撳叆鐨勫€?  * @author         RM
  * @param[in]      鏂滄尝鍑芥暟缁撴瀯浣?  * @param[in]      杈撳叆鍊?  * @retval         杩斿洖绌?  */
float ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
    return ramp_source_type->out;
}

//缁濆鍊奸檺鍒?float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}

//鍒ゆ柇绗﹀彿浣?float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//娴偣姝诲尯
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26姝诲尯
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//闄愬箙鍑芥暟
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//闄愬箙鍑芥暟
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//寰幆闄愬箙鍑芥暟
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//寮у害鏍煎紡鍖栦负-PI~PI

//瑙掑害鏍煎紡鍖栦负-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

/**
  * @brief          鏈€灏忎簩涔樻硶鍒濆鍖?  * @param[in]      鏈€灏忎簩涔樻硶缁撴瀯浣?  * @param[in]      鏍锋湰鏁?  * @retval         杩斿洖绌?  */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
  * @brief          鏈€灏忎簩涔樻硶鎷熷悎
  * @param[in]      鏈€灏忎簩涔樻硶缁撴瀯浣?  * @param[in]      淇″彿鏂版牱鏈窛涓婁竴涓牱鏈椂闂撮棿闅?  * @param[in]      淇″彿鍊?  */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;
}

/**
  * @brief          鏈€灏忎簩涔樻硶鎻愬彇淇″彿寰垎
  * @param[in]      鏈€灏忎簩涔樻硶缁撴瀯浣?  * @param[in]      淇″彿鏂版牱鏈窛涓婁竴涓牱鏈椂闂撮棿闅?  * @param[in]      淇″彿鍊?  * @retval         杩斿洖鏂滅巼k
  */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k;
}

/**
  * @brief          鑾峰彇鏈€灏忎簩涔樻硶鎻愬彇淇″彿寰垎
  * @param[in]      鏈€灏忎簩涔樻硶缁撴瀯浣?  * @retval         杩斿洖鏂滅巼k
  */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}

/**
  * @brief          鏈€灏忎簩涔樻硶骞虫粦淇″彿
  * @param[in]      鏈€灏忎簩涔樻硶缁撴瀯浣?  * @param[in]      淇″彿鏂版牱鏈窛涓婁竴涓牱鏈椂闂撮棿闅?  * @param[in]      淇″彿鍊?  * @retval         杩斿洖骞虫粦杈撳嚭
  */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
  * @brief          鑾峰彇鏈€灏忎簩涔樻硶骞虫粦淇″彿
  * @param[in]      鏈€灏忎簩涔樻硶缁撴瀯浣?  * @retval         杩斿洖骞虫粦杈撳嚭
  */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}
```

## FILE: Components/remote_control.c

```c
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      閬ユ帶鍣ㄥ鐞嗭紝閬ユ帶鍣ㄦ槸閫氳繃绫讳技SBUS鐨勫崗璁紶杈擄紝鍒╃敤DMA浼犺緭鏂瑰紡鑺傜害CPU
  *             璧勬簮锛屽埄鐢ㄤ覆鍙ｇ┖闂蹭腑鏂潵鎷夎捣澶勭悊鍑芥暟锛屽悓鏃舵彁渚涗竴浜涙帀绾块噸鍚疍MA锛屼覆鍙?  *             鐨勬柟寮忎繚璇佺儹鎻掓嫈鐨勭ǔ瀹氭€с€?  * @note       璇ヤ换鍔℃槸閫氳繃涓插彛涓柇鍚姩锛屼笉鏄痜reeRTOS浠诲姟
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 瀹屾垚
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "bsp_rc.h"
#include "main.h"


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          閬ユ帶鍣ㄥ崗璁В鏋?  * @param[in]      sbus_buf: 鍘熺敓鏁版嵁鎸囬拡
  * @param[out]     rc_ctrl: 閬ユ帶鍣ㄦ暟鎹寚
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//閬ユ帶鍣ㄦ帶鍒跺彉閲?RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes 
//鎺ユ敹鍘熷鏁版嵁锛屼负18涓瓧鑺傦紝缁欎簡36涓瓧鑺傞暱搴︼紝闃叉DMA浼犺緭瓒婄晫
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          閬ユ帶鍣ㄥ垵濮嬪寲
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          鑾峰彇閬ユ帶鍣ㄦ暟鎹寚閽?  * @param[in]      none
  * @retval         閬ユ帶鍣ㄦ暟鎹寚閽?  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          閬ユ帶鍣ㄥ崗璁В鏋?  * @param[in]      sbus_buf: 鍘熺敓鏁版嵁鎸囬拡
  * @param[out]     rc_ctrl: 閬ユ帶鍣ㄦ暟鎹寚
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
```

## FILE: Components/Algorithm/QuaternionEKF.c

```c
/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  鈥斺€斺€斺€斺€斺€斺€? *  as + 1
 ******************************************************************************
 */
#include "QuaternionEKF.h"
#include "USB_Task.h"
QEKF_INS_t QEKF_INS;

const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void IMU_QuaternionEKF_Init(float* init_quaternion,float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
  QEKF_INS.Initialized = 1;
  QEKF_INS.Q1 = process_noise1;
  QEKF_INS.Q2 = process_noise2;
  QEKF_INS.R = measure_noise;
  QEKF_INS.ChiSquareTestThreshold = 1e-8;
  QEKF_INS.ConvergeFlag = 0;
  QEKF_INS.ErrorCount = 0;
  QEKF_INS.UpdateCount = 0;
  if (lambda > 1)	lambda = 1;
  QEKF_INS.lambda = lambda;
  QEKF_INS.accLPFcoef = lpf;

  // 鍒濆鍖栫煩闃电淮搴︿俊鎭?  Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
  Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, (float *)QEKF_INS.ChiSquare_Data);
    
  // 濮挎€佸垵濮嬪寲
  for(int i = 0; i < 4; i++){
		QEKF_INS.IMU_QuaternionEKF.xhat_data[i] = init_quaternion[i];
  }
	// 鑷畾涔夊嚱鏁板垵濮嬪寲,鐢ㄤ簬鎵╁睍鎴栧鍔爇f鐨勫熀纭€鍔熻兘
	QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
	QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
	QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
	QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;	
	// 璁惧畾鏍囧織浣?鐢ㄨ嚜瀹氬嚱鏁版浛鎹f鏍囧噯姝ラ涓殑SetK(璁＄畻澧炵泭)浠ュ強xhatupdate(鍚庨獙浼拌/铻嶅悎)
	QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
	QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;
	
	memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
	memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s虏
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,鐢ㄤ簬鏇存柊宸ヤ綔鐐瑰鐨勭姸鎬佽浆绉籉鐭╅樀
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    QEKF_INS.dt = dt;
    
    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // set F
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    // 姝ら儴鍒嗚瀹氱姸鎬佽浆绉荤煩闃礔鐨勫乏涓婅閮ㄥ垎 4x4瀛愮煩闃?鍗?.5(Ohm-Ohm^bias)*deltaT,鍙充笅瑙掓湁涓€涓?x2鍗曚綅闃靛凡缁忓垵濮嬪寲濂戒簡
    // 娉ㄦ剰鍦╬redict姝鐨勫彸涓婅鏄?x2鐨勯浂鐭╅樀,鍥犳姣忔predict鐨勬椂鍊欓兘浼氳皟鐢╩emcpy鐢ㄥ崟浣嶉樀瑕嗙洊鍓嶄竴杞嚎鎬у寲鍚庣殑鐭╅樀
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // accel low pass filter,鍔犻€熷害杩囦竴涓嬩綆閫氭护娉㈠钩婊戞暟鎹?闄嶄綆鎾炲嚮鍜屽紓甯哥殑褰卞搷
    if (QEKF_INS.UpdateCount == 0){
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    }
    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ax * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ay * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + az * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);

    // set z,鍗曚綅鍖栭噸鍔涘姞閫熷害鍚戦噺
    accelInvNorm = invSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] + QEKF_INS.Accel[1] * QEKF_INS.Accel[1] + QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    for (uint8_t i = 0; i < 3; ++i)
    {
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm; // 鐢ㄥ姞閫熷害鍚戦噺鏇存柊閲忔祴鍊?    }
    // get body state
    QEKF_INS.gyro_norm = 1.0f / invSqrt(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
                                        QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                                        QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);
    QEKF_INS.accl_norm = 1.0f / accelInvNorm;
    // 濡傛灉瑙掗€熷害灏忎簬闃堝€间笖鍔犻€熷害澶勪簬璁惧畾鑼冨洿鍐?璁や负杩愬姩绋冲畾,鍔犻€熷害鍙互鐢ㄤ簬淇瑙掗€熷害
    // 绋嶅悗鍦ㄦ渶鍚庣殑濮挎€佹洿鏂伴儴鍒嗕細鍒╃敤StableFlag鏉ョ‘瀹?    if (QEKF_INS.gyro_norm < 0.3f && QEKF_INS.accl_norm > 9.8f - 0.5f && QEKF_INS.accl_norm < 9.8f + 0.5f){
        QEKF_INS.StableFlag = 1;
    }else{
        QEKF_INS.StableFlag = 0;
    }

    // set Q R,杩囩▼鍣０鍜岃娴嬪櫔澹扮煩闃?    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

    // 璋冪敤kalman_filter.c灏佽濂界殑鍑芥暟,娉ㄦ剰鍑犱釜User_Funcx_f鐨勮皟鐢?    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    // 鑾峰彇铻嶅悎鍚庣殑鏁版嵁,鍖呮嫭鍥涘厓鏁板拰xy闆堕鍊?    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0; // 澶ч儴鍒嗘椂鍊檢杞撮€氬ぉ,鏃犳硶瑙傛祴yaw鐨勬紓绉?    // 鍒╃敤鍥涘厓鏁板弽瑙ｆ鎷夎
    QEKF_INS.Yaw = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f) * 57.295779513f;
    QEKF_INS.Pitch = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]), 2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f) * 57.295779513f;
    QEKF_INS.Roll = asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) * 57.295779513f;
    // get Yaw total, yaw鏁版嵁鍙兘浼氳秴杩?60,澶勭悊涓€涓嬫柟渚垮叾浠栧姛鑳戒娇鐢?濡傚皬闄€铻?
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f){
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f){
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
    QEKF_INS.UpdateCount++; // 鍒濆鍖栦綆閫氭护娉㈢敤,璁℃暟娴嬭瘯鐢?}
/**
 * @brief 鐢ㄤ簬鏇存柊绾挎€у寲鍚庣殑鐘舵€佽浆绉荤煩闃礔鍙充笂瑙掔殑涓€涓?x2鍒嗗潡鐭╅樀,绋嶅悗鐢ㄤ簬鍗忔柟宸煩闃礟鐨勬洿鏂?
 *        骞跺闆舵紓鐨勬柟宸繘琛岄檺鍒?闃叉杩囧害鏀舵暃骞堕檺骞呴槻姝㈠彂鏁? *
 * @param kf
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
    static float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // quaternion normalize
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++){
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F_data[4] = q1 * QEKF_INS.dt / 2;
    kf->F_data[5] = q2 * QEKF_INS.dt / 2;

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2;
    kf->F_data[11] = q3 * QEKF_INS.dt / 2;

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2;
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2;

    kf->F_data[22] = q2 * QEKF_INS.dt / 2;
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2;

    // fading filter,闃叉闆堕鍙傛暟杩囧害鏀舵暃
    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    // 闄愬箙,闃叉鍙戞暎
    if (kf->P_data[28] > 10000){
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000){
        kf->P_data[35] = 10000;
    }
}
/**
 * @brief 鍦ㄥ伐浣滅偣澶勮绠楄娴嬪嚱鏁癶(x)鐨凧acobi鐭╅樀H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf){
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}
/**
 * @brief 鍒╃敤瑙傛祴鍊煎拰鍏堥獙浼拌寰楀埌鏈€浼樼殑鍚庨獙浼拌
 *        鍔犲叆浜嗗崱鏂规楠屼互鍒ゆ柇铻嶅悎鍔犻€熷害鐨勬潯浠舵槸鍚︽弧瓒? *        鍚屾椂寮曞叆鍙戞暎淇濇姢淇濊瘉鎭跺姡宸ュ喌涓嬬殑蹇呰閲忔祴鏇存柊
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf){
    static float q0, q1, q2, q3;

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H路P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H路P'(k)路HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H路P'(k)路HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 璁＄畻棰勬祴寰楀埌鐨勯噸鍔涘姞閫熷害鏂瑰悜(閫氳繃濮挎€佽幏鍙栫殑)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 璁＄畻棰勬祴鍊煎拰鍚勪釜杞寸殑鏂瑰悜浣欏鸡
    for (uint8_t i = 0; i < 3; i++)
    {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 鍒╃敤鍔犻€熷害璁℃暟鎹慨姝?    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,鍗℃柟妫€楠?    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H路P'(k)路HT + R)路(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);
    // rk is small,filter converged/converging
    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        if (QEKF_INS.StableFlag)
        {
            QEKF_INS.ErrorCount++; // 杞戒綋闈欐鏃朵粛鏃犳硶閫氳繃鍗℃柟妫€楠?        }
        else
        {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50)
        {
            // 婊ゆ尝鍣ㄥ彂鏁?            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = FALSE; // step-5 is cov mat P updating
        }
        else
        {
            //  娈嬪樊鏈€氳繃鍗℃柟妫€楠?浠呴娴?            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE; // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk瓒婂皬鍒欏鐩婅秺澶?鍚﹀垯鏇寸浉淇￠娴嬪€?        if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
        {
            QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        }
        else
        {
            QEKF_INS.AdaptiveGainScale = 1;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)路HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++)
    {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)路(z(k) - H路xhat'(k))

    // 闆舵紓淇闄愬箙,涓€鑸笉浼氭湁杩囧ぇ鐨勬紓绉?    if (QEKF_INS.ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    // 涓嶄慨姝aw杞存暟鎹?    kf->temp_vector.pData[3] = 0;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}
/**
 * @brief EKF瑙傛祴鐜妭,鍏跺疄灏辨槸鎶婃暟鎹鍒朵竴涓? *
 * @param kf kf绫诲瀷瀹氫箟
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}
/**
 * @brief 鑷畾涔?/sqrt(x),閫熷害鏇村揩
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
```

## FILE: Components/Devices/BMI088driver.c

```c
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "bsp_dwt.h"
#include <math.h>

float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static uint8_t res = 0;
static uint8_t write_reg_num = 0;
static uint8_t error = BMI088_NO_ERROR;
float gyroDiff[3], gNormDiff;

uint8_t caliOffset = 1;
int16_t caliCount = 0;

IMU_Data_t BMI088;

#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)
#endif

static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

static void Calibrate_MPU_Offset(IMU_Data_t *bmi088);

uint8_t BMI088Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate)
{
    BMI088_SPI = bmi088_SPI;
    error = BMI088_NO_ERROR;

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();
    if (calibrate)
        Calibrate_MPU_Offset(&BMI088);
    else
    {
        BMI088.GyroOffset[0] = GxOFFSET;
        BMI088.GyroOffset[1] = GyOFFSET;
        BMI088.GyroOffset[2] = GzOFFSET;
			
				BMI088.AccelOffset[0]= AxOFFSET;
			  BMI088.AccelOffset[1]= AyOFFSET;
			  BMI088.AccelOffset[2]= AzOFFSET;
			
        BMI088.gNorm = gNORM;
        BMI088.AccelScale = 9.81f / BMI088.gNorm;
        BMI088.TempWhenCali = 40;
    }

    return error;
}

void Calibrate_MPU_Offset(IMU_Data_t *bmi088)
{
    static float startTime;
    static uint16_t CaliTimes = 6000; 
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;

    startTime = DWT_GetTimeline_s();
    do
    {
        if (DWT_GetTimeline_s() - startTime > 12)
        {
            bmi088->GyroOffset[0] = GxOFFSET;
            bmi088->GyroOffset[1] = GyOFFSET;
            bmi088->GyroOffset[2] = GzOFFSET;
            bmi088->gNorm = gNORM;
            bmi088->TempWhenCali = 40;
            break;
        }

        DWT_Delay(0.005);
        bmi088->gNorm = 0;
        bmi088->GyroOffset[0] = 0;
        bmi088->GyroOffset[1] = 0;
        bmi088->GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; ++i)
        {
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(bmi088->Accel[0] * bmi088->Accel[0] +
                              bmi088->Accel[1] * bmi088->Accel[1] +
                              bmi088->Accel[2] * bmi088->Accel[2]);
            bmi088->gNorm += gNormTemp;

            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[0] += bmi088->Gyro[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[1] += bmi088->Gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[2] += bmi088->Gyro[2];
            }

            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    gyroMax[j] = bmi088->Gyro[j];
                    gyroMin[j] = bmi088->Gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    if (bmi088->Gyro[j] > gyroMax[j])
                        gyroMax[j] = bmi088->Gyro[j];
                    if (bmi088->Gyro[j] < gyroMin[j])
                        gyroMin[j] = bmi088->Gyro[j];
                }
            }

            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; ++j)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
            {
                break;
            }

            DWT_Delay(0.0005);
        }

        bmi088->gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; ++i)
            bmi088->GyroOffset[i] /= (float)CaliTimes;

        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        bmi088->TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        caliCount++;
    } while (gNormDiff > 0.5f ||
             fabsf(bmi088->gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(bmi088->GyroOffset[0]) > 0.01f ||
             fabsf(bmi088->GyroOffset[1]) > 0.01f ||
             fabsf(bmi088->GyroOffset[2]) > 0.01f);

    bmi088->AccelScale = 9.81f / bmi088->gNorm;
}

uint8_t bmi088_accel_init(void)
{
    // check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    DWT_Delay(0.001);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    DWT_Delay(0.001);
    // accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    // HAL_Delay(BMI088_LONG_DELAY_TIME);
    DWT_Delay(0.08);
    // check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    DWT_Delay(0.001);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    DWT_Delay(0.001);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(BMI088_Accel_Init_Table[write_reg_num][0], BMI088_Accel_Init_Table[write_reg_num][1]);
        DWT_Delay(0.001);

        BMI088_accel_read_single_reg(BMI088_Accel_Init_Table[write_reg_num][0], res);
        DWT_Delay(0.001);

        if (res != BMI088_Accel_Init_Table[write_reg_num][1])
        {
            // write_reg_num--;
            // return BMI088_Accel_Init_Table[write_reg_num][2];
            error |= BMI088_Accel_Init_Table[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    // check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    DWT_Delay(0.001);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    DWT_Delay(0.001);

    // reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    // HAL_Delay(BMI088_LONG_DELAY_TIME);
    DWT_Delay(0.08);
    // check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    DWT_Delay(0.001);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    DWT_Delay(0.001);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(BMI088_Gyro_Init_Table[write_reg_num][0], BMI088_Gyro_Init_Table[write_reg_num][1]);
        DWT_Delay(0.001);

        BMI088_gyro_read_single_reg(BMI088_Gyro_Init_Table[write_reg_num][0], res);
        DWT_Delay(0.001);

        if (res != BMI088_Gyro_Init_Table[write_reg_num][1])
        {
            write_reg_num--;
            // return BMI088_Gyro_Init_Table[write_reg_num][2];
            error |= BMI088_Accel_Init_Table[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

void BMI088_Read(IMU_Data_t *bmi088)
{
    static uint8_t buf[8] = {0};
    static int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
        }
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    bmi088->Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
```

## FILE: Components/Devices/BMI088Middleware.c

```c
#include "BMI088Middleware.h"
#include "main.h"

SPI_HandleTypeDef *BMI088_SPI;

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
```

## FILE: gimbal_ws/middlewares/motor/dm_motor.c

```c
#include "dm_motor.h"

DM4310_TypeDef GimYaw,GimPitch;

int float_to_uint(float x_float, float x_min, float x_max, int bits){
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits){
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
void DM_Motor_Init(DM_Motor_t *motor,uint16_t id,uint16_t mode){
	motor->mode = mode;
	motor->para.id = id;
}
void DM_Motor_read(DM_Motor_t *motor,uint8_t *rx_data){
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int,P_MIN,P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int,V_MIN,V_MAX, 12); // (-30.0,30.0)
	motor->para.tor = uint_to_float(motor->para.t_int,T_MIN,T_MAX, 12);  // (-10.0,10.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}
void DM4310_Receive(DM4310_TypeDef *Dst, uint8_t *Data) {
    Dst->MchanicalAngle = (uint16_t)(Data[0] << 8 | Data[1]);
    Dst->Speed = (int16_t)(Data[2] << 8 | Data[3]) / 1000;
    Dst->TorqueCurrent = (int16_t)(Data[4] << 8 | Data[5]);
    Dst->temp = Data[6];
    Dst->PCBtemp = Data[7];

    int16_t diff = Dst->MchanicalAngle - Dst->LsatAngle;

    if (diff > 4095)
        Dst->r--;
    if (diff < -4095)
        Dst->r++;

    Dst->Angle = Dst->r * 8192 + Dst->MchanicalAngle;
    Dst->Angle_DEG = Dst->Angle * 0.0439453125f;
//    Dst->Power = GetChassisMotorPower(Dst->Speed, Dst->TorqueCurrent, &Dst->PowerCOF);
    Dst->LsatAngle = Dst->MchanicalAngle;
}
/**
*@brief 0 enable
*@brief 1 disable
*@brief 2 set zero
*@brief 3 clear error
*/
HAL_StatusTypeDef DM_Motor_Ctrl(CAN_HandleTypeDef *hcan,uint16_t motor_id,uint16_t mode_id,uint8_t mode){
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	switch(mode){
		case 0 :	data[7] = 0xFC; break;
		case 1 :  data[7] = 0xFD; break;
		case 2 :  data[7] = 0xFE; break;
		case 3 :  data[7] = 0xFB; break;
		default : break;
	}
	return CAN_Send_StdDataFrame(hcan, id, data);
}
HAL_StatusTypeDef mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq){
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	return CAN_Send_StdDataFrame(hcan, id, data);
}
HAL_StatusTypeDef pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	return CAN_Send_StdDataFrame(hcan, id, data);
}
HAL_StatusTypeDef speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel){
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + VEL_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	return CAN_Send_StdDataFrame(hcan, id, data);
}
```

## FILE: gimbal_ws/middlewares/motor/dm_motor.h

```c
#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "can.h"
#include "CANDrive.h"
 
#define MIT_MODE 0x000
#define POS_MODE 0x100
#define VEL_MODE 0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef struct {
    uint16_t MchanicalAngle;    //!<@brief 鏈烘瑙掑害
    int16_t Speed;              //!<@brief 杞€?
    int16_t TorqueCurrent;      //!<@brief 杞煩鐢垫祦
    uint8_t temp;                //!<@brief 娓╁害
    uint8_t PCBtemp;            //!<@brief PCB娓╁害
    uint16_t LsatAngle;         //!<@brief 涓婁竴娆＄殑鏈烘瑙掑害
    int16_t r;                  //!<@brief 鍦堟暟
    int32_t Angle;              //!<@brief 杩炵画鍖栨満姊拌搴?@warning 鐢变簬鍚姩鏃惰搴︿笉纭畾锛屽惎鍔ㄦ椂杩炵画鍖栬搴﹀彲鑳芥湁涓€鍦堢殑鍋忓樊
    float Angle_DEG;            //!<@brief 杩炵画鍖栬搴﹀埗瑙掑害 @warning 鐢变簬鍚姩鏃惰搴︿笉纭畾锛屽惎鍔ㄦ椂杩炵画鍖栬搴﹀彲鑳芥湁涓€鍦堢殑鍋忓樊
}DM4310_TypeDef;


typedef struct{
	int id;                // 鐢垫満鍐呴儴璁剧疆鐨?can id
	int state;             // 鐢垫満鐘舵€?
	int p_int;             // 鏁村瀷浣嶇疆淇℃伅
	int v_int;             // 鏁村瀷閫熷害淇℃伅
	int t_int;             // 鏁村瀷鎵煩淇℃伅
	int kp_int;            // 鏁村瀷Kp淇℃伅
	int kd_int;            // 鏁村瀷Kd淇℃伅
	float pos;             // 鏈€缁堣В鏋愬嚭鏉ョ殑浣嶇疆淇℃伅  (rad)
	float vel;             // 鏈€缁堣В鏋愬嚭鏉ョ殑閫熷害淇℃伅  (rad/s)
	float tor;             // 鏈€缁堣В鏋愬嚭鏉ョ殑鎵煩淇℃伅
	float Kp;              // 鏈€缁堣В鏋愬嚭鏉ョ殑Kp鏁版嵁
	float Kd;              // 鏈€缁堣В鏋愬嚭鏉ョ殑Kd鏁版嵁
	float Tmos;            // 鏉垮瓙MOS娓╁害
	float Tcoil;           // 鐢垫満绾垮湀娓╁害
}DM_motor_fdpara_t;
typedef struct{
    float P_Ref;
    float V_Ref;
    float KP_Ref;
    float KD_Ref;
    float T_ff;
}DM_Ref_t;
typedef struct{
	DM_motor_fdpara_t para;
	DM_Ref_t ref;
	uint16_t mode;
}DM_Motor_t;

extern DM4310_TypeDef GimYaw,GimPitch;

float uint_to_float(int x_int,float x_min,float x_max,int bits);
int float_to_uint(float x_float,float x_min,float x_max,int bits);
void DM_Motor_Init(DM_Motor_t *motor,uint16_t id,uint16_t mode);
void DM_Motor_read(DM_Motor_t *motor,uint8_t * rx_data);
void DM4310_Receive(DM4310_TypeDef *Dst, uint8_t *Data);

HAL_StatusTypeDef DM_Motor_Ctrl(CAN_HandleTypeDef *hcan,uint16_t motor_id,uint16_t mode_id,uint8_t mode);
HAL_StatusTypeDef mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
HAL_StatusTypeDef pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);
HAL_StatusTypeDef speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel);
#endif
```

## FILE: RM_Lib/Src/CANDrive.c

```c
#include "CANDrive.h"

CAN_RxHeaderTypeDef RxHeader[2];
uint8_t CAN1_buff[8];
uint8_t CAN2_Rxbuff[64];
void CanFilter_Init(CAN_HandleTypeDef *hcan) {
	CAN_FilterTypeDef canfilter;

	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;
	canfilter.FilterMaskIdLow = 0x0000;

	/*! 浠巆an鐨勮繃婊ゅ櫒璧峰缂栧彿 鍙湁褰撹缃袱涓猚an鏃?璇ュ弬鏁版墠鏈夋剰涔?*/
	canfilter.SlaveStartFilterBank = 14;

	/*! can1鍜孋AN2浣跨敤涓嶅悓鐨勬护娉㈠櫒*/
	if (hcan->Instance == CAN1) {
			
			/*! 涓籧an鐨勮繃婊ゅ櫒缂栧彿 */
			canfilter.FilterBank = 0;

			/*! CAN_FilterFIFO0 */
			canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	}
#if defined(CAN2)
	if (hcan->Instance == CAN2) {
			/*! 浠巆an鐨勮繃婊ゅ櫒缂栧彿 */
			canfilter.FilterBank = 14;

			/*! CAN_FilterFIFO1 */
			canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
			
			/* 杩囨护瓒呯數淇℃伅 */
			canfilter.FilterMode = CAN_FILTERMODE_IDLIST;   //鍒楄〃妯″紡
			canfilter.FilterScale = CAN_FILTERSCALE_16BIT;  //16浣嶅
			
			canfilter.FilterIdHigh = 0x101<<5;              //涓嬫澘瑁佸垽绯荤粺淇℃伅ID
			canfilter.FilterIdLow = 0x301<<5;               //Pitch杞?			canfilter.FilterMaskIdHigh = 0x102<<5;          //Yaw杞?			canfilter.FilterMaskIdLow = 0x302<<5;							//涓婁笅鏉块€氫俊0x102
	}
#endif
	/*! 婵€娲昏繃婊ゅ櫒 */
	canfilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hcan, &canfilter);
}

HAL_StatusTypeDef CAN_Send_StdDataFrame(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *msg) {
	CAN_TxHeaderTypeDef CAN_Tx = {
		.StdId = StdId,                 //鏍囧噯鏍囪瘑绗?		.ExtId = 0,
		.IDE = CAN_ID_STD,              //浣跨敤鏍囧噯甯?		.RTR = CAN_RTR_DATA,            //鏁版嵁甯?		.DLC = 8,
		.TransmitGlobalTime = DISABLE,
	};

  uint32_t TxMailbox = 0;

	return HAL_CAN_AddTxMessage(hcan, &CAN_Tx, msg, &TxMailbox);
}

uint32_t CAN_Receive_DataFrame(CAN_HandleTypeDef *hcan, uint8_t *buf) {
    CAN_RxHeaderTypeDef CAN_Rx = { 0 };
    HAL_CAN_GetRxMessage(hcan, (hcan->Instance == CAN1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1, &CAN_Rx, buf);

    if(CAN_Rx.IDE == CAN_ID_STD)
        return CAN_Rx.StdId;
    else
        return CAN_Rx.ExtId;
}
```

## FILE: RM_Lib/Src/motor.c

```c
#include <motor.h>
/**
 * @brief 鐢ㄨ浆鐭╃數娴佽绠楀緱鍒板姛鐜囧€? * @param[in] speed 鐢垫満閫熷害
 * @param[in] current 杞煩鐢垫祦
 * @param[in] pcof 鍙傛暟
 * @return 鐢垫満鍔熺巼
 */
static inline float GetChassisMotorPower(int speed, int current, struct PowerCOF_s *pcof) {
    return (pcof->ss * speed * speed +
            pcof->sc * speed * current +
            pcof->cc * current * current +
            pcof->constant);
}
void RMMotor_Receive(RMMotor_t *motor,uint8_t *Data){
	motor->MechAngle = (uint16_t)(Data[0] << 8 | Data[1]);
	motor->RoSpeed = (int16_t)(Data[2] << 8 | Data[3]);
	motor->TorCurrent = (uint16_t)(Data[4] << 8 | Data[5]);
	motor->Temp = Data[6];
	motor->Err =  Data[7];
	int16_t diff = motor->MechAngle - motor->lastMechAngle;
		if (diff > 4096)
				motor->r--;
		if (diff < -4096)
				motor->r++;
	motor->continueMechAngle = motor->r * 8192 + motor->MechAngle;
	motor->lastMechAngle = motor->MechAngle;
	motor->angle = motor->continueMechAngle * 0.0439453125f;
}
//	Dst->Power = GetChassisMotorPower(Dst->Speed, Dst->TorqueCurrent, &Dst->PowerCOF);

HAL_StatusTypeDef MotorSend(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t *Data) {
    uint8_t temp[8];
    temp[0] = (uint8_t)(Data[0] >> 8);
    temp[1] = (uint8_t)(Data[0] & 0xff);
    temp[2] = (uint8_t)(Data[1] >> 8);
    temp[3] = (uint8_t)(Data[1] & 0xff);
    temp[4] = (uint8_t)(Data[2] >> 8);
    temp[5] = (uint8_t)(Data[2] & 0xff);
    temp[6] = (uint8_t)(Data[3] >> 8);
    temp[7] = (uint8_t)(Data[3] & 0xff);
    return CAN_Send_StdDataFrame(hcan, StdId, temp);
}

int16_t QuickCentering(uint16_t current, uint16_t target){
	int32_t diff = (int32_t)target - (int32_t)current;
	if (diff > 4096) {
			target -= 8192;
	} else if (diff < -4096) {
			target +=8192;
	}
	return target;
}
```

## FILE: RM_Lib/Src/PID.c

```c
#include "PID.h"
#include "Function.h"

#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

void PID_init(PID_TypeDef *pid,uint16_t max_out,float intergrallimit,float deadband,float Kp,float Ki,float Kd,float Kf,float dt){
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergrallimit;
	pid->MaxOut = max_out;
	pid->Target = 0;

	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Kf = Kf;
	pid->dt = dt;
	pid->ITerm = 0;
}
/***************************PID calculate**********************************/
float PID_Calc(PID_TypeDef *pid, float measure, float target){
	if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE){
			return 0; //Catch ERROR
	}
	pid->Measure = measure;
	pid->Target = target;
	pid->Err = pid->Target - pid->Measure;
  if (ABS(pid->Err) > pid->DeadBand){
		pid->Pout  = pid->Kp * pid->Err;
		pid->Iout += pid->Ki * pid->Err;
		pid->Dout  = pid->Kd * (pid->Err - pid->Last_Err);
		pid->Fout  = pid->Kf * (pid->Target - pid->Last_Target);

		pid->Output = pid->Pout + pid->Iout + pid->Dout + pid->Fout;
		//Output limit
		f_Output_Limit(pid);   
	}
	pid->Last_Measure = pid->Measure;
	pid->Last_Target  = pid->Target;
	pid->Last_Output  = pid->Output;
	pid->Last_Err = pid->Err;

	return pid->Output;
}
static void f_Output_Limit(PID_TypeDef *pid){
  if (pid->Output > pid->MaxOut){
    pid->Output = pid->MaxOut;
  }
	if (pid->Output < -(pid->MaxOut)){
    pid->Output = -(pid->MaxOut);
  }
}
void PID_Control(float current, float expected, PID *parameter) {
    parameter->error_last = parameter->error_now;
    parameter->error_now = expected - current;

    if(fabs(parameter->error_now) < parameter->DeadBand)
        parameter->error_now = 0.0f;
        if(fabs(parameter->error_now) < parameter->error_thre)
        {
            if(parameter->error_now <= 0)
                parameter->error_inter += (parameter->error_now + parameter->DeadBand);
            else
                parameter->error_inter += (parameter->error_now - parameter->DeadBand);
        }
        
        limit(parameter->error_inter, parameter->limit, -parameter->limit);
        
        parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                             parameter->Kd * (parameter->error_now - parameter->error_last);
}

void PID_Control_Smis(float current, float expected, PID_Smis *parameter, float speed) {
    parameter->error_now = expected - current;

    if(fabs(parameter->error_now) < parameter->DeadBand)
        parameter->error_now = 0.0f;
    
        if(fabs(parameter->error_now) < parameter->error_thre)
        {
            if(parameter->error_now <= 0)
                parameter->error_inter += (parameter->error_now + parameter->DeadBand);
            else
                parameter->error_inter += (parameter->error_now - parameter->DeadBand);
        }

        limit(parameter->error_inter, parameter->limit, -parameter->limit);

        parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                             parameter->Kd * speed;
}
float FeedForward_Calc(FeedForward_Typedef *FF){
    
    FF->Out = FF->Now_DeltIn*FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn)*FF->K2;
    FF->Last_DeltIn = FF->Now_DeltIn;
    
    limit(FF->Out,FF->OutMax,-FF->OutMax);

    return FF->Out;
}
```

## FILE: RM_Lib/Src/ramp.c

```c
#include "ramp.h"

__weak uint32_t Get_TimerTick() {
    return HAL_GetTick();
}

float Slope(Ramp_Typedef *Ramp) {
    if (!Ramp->flag) {
        Ramp->StartTick = Get_TimerTick();
        Ramp->flag = 1;
    }
    if (Get_TimerTick() > (Ramp->StartTick + Ramp->RampTime))return 1.0f;
    return ((Get_TimerTick() - Ramp->StartTick) / (float) Ramp->RampTime);
}
```

## FILE: RM_Lib/Src/remote.c

```c
#include "remote.h"
#include "Variate.h"

float Key_ch[4]   = {0};
float Mouse_ch[3] = {0};
InputMode_e RemoteMode;

RC_Ctl_t RC_CtrlData = {.rc = {1024, 1024, 1024, 1024, 2, 2}};     //!<@brief remote control data

void RemoteControlProcess(Remote *rc) {
  RemoteMode=REMOTE_INPUT;
	
	Key_ch[0] =(float )(rc->ch0 - 1024)/660;
	Key_ch[1] =(float )(rc->ch1 - 1024)/660;
	Key_ch[2] =(float )(rc->ch2 - 1024)/660;
	Key_ch[3] =(float )(rc->ch3 - 1024)/660;
	
	deadline_limit(Key_ch[0],0.1f);
	deadline_limit(Key_ch[1],0.1f);
	deadline_limit(Key_ch[2],0.1f);
	deadline_limit(Key_ch[3],0.1f);
}
void MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey) {
	RemoteMode=KEY_MOUSE_INPUT;
	
	limit (mouse ->x,200,-200);
	limit (mouse ->y,200,-200);
	limit (mouse ->z,200,-200);
	
	Mouse_ch[0]=(float)(mouse ->x)/200;
	Mouse_ch[1]=(float)(mouse ->y)/200;
	Mouse_ch[2]=(float)(mouse ->z)/200;
	
	deadline_limit(Mouse_ch[0],0.01f);
	deadline_limit(Mouse_ch[1],0.01f);
	deadline_limit(Mouse_ch[2],0.01f);;
}

void STOPControlProcess(void) {
	RemoteMode=STOP;
}

void RemoteClear() {
    RC_CtrlData.rc.ch0 = 1024;
    RC_CtrlData.rc.ch1 = 1024;
    RC_CtrlData.rc.ch2 = 1024;
    RC_CtrlData.rc.ch3 = 1024;
    RC_CtrlData.rc.s1 = 2;
    RC_CtrlData.rc.s2 = 2;
    RC_CtrlData.key.S = 0;
    RC_CtrlData.Lastkey.S = 0;
    RC_CtrlData.mouse.x = 0;
    RC_CtrlData.mouse.y = 0;
    RC_CtrlData.mouse.z = 0;
    RC_CtrlData.mouse.press_r = 0;
    RC_CtrlData.mouse.press_l = 0;
    RC_CtrlData.mouse.last_press_r = 0;
    RC_CtrlData.mouse.last_press_l = 0;
}

void Remote_Rx(unsigned char *RxMsg) {
    RC_CtrlData.rc.ch0 = (RxMsg[0] | (RxMsg[1] << 8)) & 0x07FF;
    RC_CtrlData.rc.ch1 = ((RxMsg[1] >> 3) | (RxMsg[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = ((RxMsg[2] >> 6) | (RxMsg[3] << 2) | (RxMsg[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = ((RxMsg[4] >> 1) | (RxMsg[5] << 7)) & 0x07FF;

    RC_CtrlData.rc.s1 = (RxMsg[5] >> 4 & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = (RxMsg[5] >> 4 & 0x0003);

    RC_CtrlData.mouse.x = (int16_t)(RxMsg[6] | (RxMsg[7] << 8));
    RC_CtrlData.mouse.y = (int16_t)(RxMsg[8] | (RxMsg[9] << 8));
    RC_CtrlData.mouse.z = (int16_t)(RxMsg[10] | (RxMsg[11] << 8));

    RC_CtrlData.mouse.press_l = RxMsg[12];
    RC_CtrlData.mouse.press_r = RxMsg[13];

    *(uint16_t * ) & (RC_CtrlData.key) = RxMsg[14] | RxMsg[15] << 8;

    switch (RC_CtrlData.rc.s2) {
        case REMOTE_INPUT:
            //閬ユ帶鍣ㄦ帶鍒舵ā寮?            RemoteControlProcess(&(RC_CtrlData.rc));
            break;
        case KEY_MOUSE_INPUT:
            //閿紶鎺у埗妯″紡
            MouseKeyControlProcess(&RC_CtrlData.mouse, RC_CtrlData.key, RC_CtrlData.Lastkey);
            break;
        case STOP:
            STOPControlProcess();
            break;
    }
    RC_CtrlData.mouse.last_press_l = RC_CtrlData.mouse.press_l;
    RC_CtrlData.mouse.last_press_r = RC_CtrlData.mouse.press_r;
    RC_CtrlData.Lastkey = RC_CtrlData.key;
}
```

## FILE: RM_Lib/Src/WatchDog.c

```c
/**
 * @file    WatchDog.c
 * @author  yao
 * @date    1-May-2020
 * @brief   鐪嬮棬鐙楁ā鍧? * @details 瑕佸惎鐢ㄧ湅闂ㄧ嫍闇€瑕佹坊鍔犲叏灞€瀹忓畾涔塛atchDoglength骞惰祴鍊? *  闇€瑕佺殑鏈€澶х湅闂ㄧ嫍鏁伴噺
 */

#include <WatchDog.h>

#if defined(WatchDoglength) && WatchDoglength > 0

/*!@brief 鐪嬮棬鐙楀垪琛?/
static WatchDogp List[WatchDoglength];

/*!@brief 鐪嬮棬鐙楅暱搴?/
static uint16_t Len = 0;

void WatchDog_Polling(void) {
    for (uint8_t i = 0; i < Len; ++i) {
        List[i]->Life++;
        if (List[i]->Life > List[i]->Max) {
            WatchDog_CallBack(List[i]);
        }
    }
}

void WatchDog_Init(WatchDogp handle, uint32_t Life) {
    if (Len >= WatchDoglength)
        return;
    handle->Max = Life;
    handle->ID  = Len + 1;
    List[Len++] = handle;
}

void Feed_Dog(WatchDogp handle) {
    handle->Life = 0;
    FeedDog_CallBack(handle);
}

__weak void FeedDog_CallBack(WatchDogp handle) {
    UNUSED(handle);
}

__weak void WatchDog_CallBack(WatchDogp handle) {
    UNUSED(handle);
}

#endif
```

## FILE: Components/Function.c

```c
/*!
* @file     Function.c
* @brief    鍏ㄥ眬璋冪敤鍔熻兘鍑芥暟
*/
#include "Function.h"

#define ENCODER_RANGE 8192
#define HALF_RANGE (ENCODER_RANGE / 2)

/* 璁惧鐘舵€佹娴嬪嚱鏁?*/
osStatus_t RM3508_Motor_Temp(RM3508_TypeDef *dst)
{
	if (dst->Measure.Temp > 80)
		return osError;
	else
		return osOK;
}

osStatus_t GM6020_Motor_Temp(GM6020_TypeDef *dst)
{
	if (dst->Measure.Temp > 80)
		return osError;
	else
		return osOK;
}

osStatus_t REMOTE_IfDataError( void )
{
if ( (RC_CtrlData.rc.s1 != 1 && RC_CtrlData.rc.s1 != 3 && RC_CtrlData.rc.s1 != 2)
|| (RC_CtrlData.rc.s2 != 1 && RC_CtrlData.rc.s2 != 3 && RC_CtrlData.rc.s2 != 2)
|| (RC_CtrlData.rc.ch0 > 1684 || RC_CtrlData.rc.ch0 < 364)
|| (RC_CtrlData.rc.ch1 > 1684 || RC_CtrlData.rc.ch1 < 364)
|| (RC_CtrlData.rc.ch2 > 1684 || RC_CtrlData.rc.ch2 < 364)
|| (RC_CtrlData.rc.ch3 > 1684 || RC_CtrlData.rc.ch3 < 364) )
    return osError;
else
    return osOK;
}

osStatus_t IMU_IfDataError( void )
{
    if(fabs(IMU.Angle_Pitch)>180||fabs (IMU.Angle_Roll)>180||fabs (IMU.Angle_Yaw )>180
        ||(IMU.Angle_Pitch ==0&&IMU.Angle_Roll==0&&IMU.Angle_Yaw))
        return osError;
    else
        return osOK;
}

/* 鏂滃潯鍑芥暟锛坒loat锛?*/
float RAMP_float( float final, float now, float ramp )
{
	float	buffer = final - now;
	if (buffer > 0){
			if (buffer > ramp)  
							now += ramp;  
			else
							now += buffer;
	} else {
			if (buffer < -ramp)
							now += -ramp;
			else
							now += buffer;
	}
	return now;
}

/**
 * @brief 灏嗙數鏈烘満姊拌搴?0~8191)灞曞紑涓鸿繛缁搴︼紝閬垮厤璺ㄩ浂璺冲彉
 * @param cur_raw 褰撳墠鏈烘瑙掑害(0~8191)
 * @param id 鐢垫満缂栧彿锛堝尯鍒哬aw/Pitch锛? * @return 杩炵画瑙掑害锛堝彲姝ｈ礋鏃犻檺绱姞锛? */
float GetContinuousAngle(uint16_t cur_raw, uint8_t id)
{
    static int32_t last_raw[2] = {0};    // 涓婁竴娆＄殑鍘熷鍊?    static int32_t round_cnt[2] = {0};   // 鍦堟暟璁板綍
    static int32_t continuous_angle[2] = {0};

    int32_t diff = (int32_t)cur_raw - last_raw[id];

    // 璺ㄩ浂鐐瑰鐞?    if (diff > ENCODER_HALF) {
        round_cnt[id]--;
    } else if (diff < -ENCODER_HALF) {
        round_cnt[id]++;
    }

    continuous_angle[id] = (int32_t)cur_raw + round_cnt[id] * ENCODER_MAX;
    last_raw[id] = cur_raw;

    return (float)continuous_angle[id];
}
```

## FILE: Components/Controller/controller.c

```c
/**
 ******************************************************************************
 * @file    controller.c
 * @author  Wang Hongxi
 * @author  Zhang Hongyu (fuzzy pid)
 * @version V1.1.3
 * @date    2021/7/3
 * @brief   DWT定时器用于计算控制周期 OLS用于提取信号微分
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "controller.h"
/******************************* PID CONTROL *********************************/
// PID优化环节函数声明
static void f_Trapezoid_Intergral(PID_t *pid);
static void f_Integral_Limit(PID_t *pid);
static void f_Derivative_On_Measurement(PID_t *pid);
static void f_Changing_Integration_Rate(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Derivative_Filter(PID_t *pid);
static void f_Output_Limit(PID_t *pid);
static void f_Proportion_Limit(PID_t *pid);
static void f_PID_ErrorHandle(PID_t *pid);

/**
 * @brief          PID初始化   PID initialize
 * @param[in]      PID结构体   PID structure
 * @param[in]      略
 * @retval         返回空      null
 */
void PID_Init(
    PID_t *pid,
    float max_out,
    float intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float A,
    float B,

    float output_lpf_rc,
    float derivative_lpf_rc,

    uint16_t ols_order,

    uint8_t improve)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->Ref = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    // 变速积分参数
    // coefficient of changing integration rate
    pid->CoefA = A;
    pid->CoefB = B;

    pid->Output_LPF_RC = output_lpf_rc;

    pid->Derivative_LPF_RC = derivative_lpf_rc;

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    pid->OLS_Order = ols_order;
    OLS_Init(&pid->OLS, ols_order);

    // DWT定时器计数变量清零
    // reset DWT Timer count counter
    pid->DWT_CNT = 0;

    // 设置PID优化环节
    pid->Improve = improve;

    // 设置PID异常处理 目前仅包含电机堵转保护
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}
float PID_Calculate(PID_t *pid, float measure, float ref)
{
    if (pid->Improve & ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT((void *)&pid->DWT_CNT);

    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (pid->User_Func1_f != NULL)
        pid->User_Func1_f(pid);

    if (abs(pid->Err) > pid->DeadBand)
    {
        if (pid->FuzzyRule == NULL)
        {
            pid->Pout = pid->Kp * pid->Err;
            pid->ITerm = pid->Ki * pid->Err * pid->dt;
            if (pid->OLS_Order > 2)
                pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
            else
                pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
        }
        else
        {
            pid->Pout = (pid->Kp + pid->FuzzyRule->KpFuzzy) * pid->Err;
            pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * pid->Err * pid->dt;
            if (pid->OLS_Order > 2)
                pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
            else
                pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Err - pid->Last_Err) / pid->dt;
        }

        if (pid->User_Func2_f != NULL)
            pid->User_Func2_f(pid);

        // 梯形积分
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;

        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        // 输出滤波
        if (pid->Improve & OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);

        // 无关紧要
        f_Proportion_Limit(pid);
    }

    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

static void f_Trapezoid_Intergral(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
        pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
    else
        pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

static void f_Changing_Integration_Rate(PID_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        // Integral still increasing
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            // 积分呈累积趋势
            // Integral still increasing
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
    {
        if (pid->OLS_Order > 2)
            pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        else
            pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
    else
    {
        if (pid->OLS_Order > 2)
            pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        else
            pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
}

static void f_Derivative_Filter(PID_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

static void f_Output_Filter(PID_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_t *pid)
{
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

// PID ERRORHandle Function
static void f_PID_ErrorHandle(PID_t *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/*************************** FEEDFORWARD CONTROL *****************************/
/**
 * @brief          前馈控制初始化
 * @param[in]      前馈控制结构体
 * @param[in]      略
 * @retval         返回空
 */
void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order)
{
    ffc->MaxOut = max_out;

    // 设置前馈控制器参数 详见前馈控制结构体定义
    // set parameters of feed-forward controller (see struct definition)
    if (c != NULL && ffc != NULL)
    {
        ffc->c[0] = c[0];
        ffc->c[1] = c[1];
        ffc->c[2] = c[2];
    }
    else
    {
        ffc->c[0] = 0;
        ffc->c[1] = 0;
        ffc->c[2] = 0;
        ffc->MaxOut = 0;
    }

    ffc->LPF_RC = lpf_rc;

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    ffc->Ref_dot_OLS_Order = ref_dot_ols_order;
    ffc->Ref_ddot_OLS_Order = ref_ddot_ols_order;
    if (ref_dot_ols_order > 2)
        OLS_Init(&ffc->Ref_dot_OLS, ref_dot_ols_order);
    if (ref_ddot_ols_order > 2)
        OLS_Init(&ffc->Ref_ddot_OLS, ref_ddot_ols_order);

    ffc->DWT_CNT = 0;

    ffc->Output = 0;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float Feedforward_Calculate(Feedforward_t *ffc, float ref)
{
    ffc->dt = DWT_GetDeltaT((void *)&ffc->DWT_CNT);

    ffc->Ref = ref * ffc->dt / (ffc->LPF_RC + ffc->dt) +
               ffc->Ref * ffc->LPF_RC / (ffc->LPF_RC + ffc->dt);

    // 计算一阶导数
    // calculate first derivative
    if (ffc->Ref_dot_OLS_Order > 2)
        ffc->Ref_dot = OLS_Derivative(&ffc->Ref_dot_OLS, ffc->dt, ffc->Ref);
    else
        ffc->Ref_dot = (ffc->Ref - ffc->Last_Ref) / ffc->dt;

    // 计算二阶导数
    // calculate second derivative
    if (ffc->Ref_ddot_OLS_Order > 2)
        ffc->Ref_ddot = OLS_Derivative(&ffc->Ref_ddot_OLS, ffc->dt, ffc->Ref_dot);
    else
        ffc->Ref_ddot = (ffc->Ref_dot - ffc->Last_Ref_dot) / ffc->dt;

    // 计算前馈控制输出
    // calculate feed-forward controller output
    ffc->Output = ffc->c[0] * ffc->Ref + ffc->c[1] * ffc->Ref_dot + ffc->c[2] * ffc->Ref_ddot;

    ffc->Output = float_constrain(ffc->Output, -ffc->MaxOut, ffc->MaxOut);

    ffc->Last_Ref = ffc->Ref;
    ffc->Last_Ref_dot = ffc->Ref_dot;

    return ffc->Output;
}

/*************************LINEAR DISTURBANCE OBSERVER *************************/
void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order)
{
    ldob->Max_Disturbance = max_d;

    ldob->DeadBand = deadband;

    // 设置线性扰动观测器参数 详见LDOB结构体定义
    // set parameters of linear disturbance observer (see struct definition)
    if (c != NULL && ldob != NULL)
    {
        ldob->c[0] = c[0];
        ldob->c[1] = c[1];
        ldob->c[2] = c[2];
    }
    else
    {
        ldob->c[0] = 0;
        ldob->c[1] = 0;
        ldob->c[2] = 0;
        ldob->Max_Disturbance = 0;
    }

    // 设置Q(s)带宽  Q(s)选用一阶惯性环节
    // set bandwidth of Q(s)    Q(s) is chosen as a first-order low-pass form
    ldob->LPF_RC = lpf_rc;

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    ldob->Measure_dot_OLS_Order = measure_dot_ols_order;
    ldob->Measure_ddot_OLS_Order = measure_ddot_ols_order;
    if (measure_dot_ols_order > 2)
        OLS_Init(&ldob->Measure_dot_OLS, measure_dot_ols_order);
    if (measure_ddot_ols_order > 2)
        OLS_Init(&ldob->Measure_ddot_OLS, measure_ddot_ols_order);

    ldob->DWT_CNT = 0;

    ldob->Disturbance = 0;
}

float LDOB_Calculate(LDOB_t *ldob, float measure, float u)
{
    ldob->dt = DWT_GetDeltaT((void *)&ldob->DWT_CNT);

    ldob->Measure = measure;

    ldob->u = u;

    // 计算一阶导数
    // calculate first derivative
    if (ldob->Measure_dot_OLS_Order > 2)
        ldob->Measure_dot = OLS_Derivative(&ldob->Measure_dot_OLS, ldob->dt, ldob->Measure);
    else
        ldob->Measure_dot = (ldob->Measure - ldob->Last_Measure) / ldob->dt;

    // 计算二阶导数
    // calculate second derivative
    if (ldob->Measure_ddot_OLS_Order > 2)
        ldob->Measure_ddot = OLS_Derivative(&ldob->Measure_ddot_OLS, ldob->dt, ldob->Measure_dot);
    else
        ldob->Measure_ddot = (ldob->Measure_dot - ldob->Last_Measure_dot) / ldob->dt;

    // 估计总扰动
    // estimate external disturbances and internal disturbances caused by model uncertainties
    ldob->Disturbance = ldob->c[0] * ldob->Measure + ldob->c[1] * ldob->Measure_dot + ldob->c[2] * ldob->Measure_ddot - ldob->u;
    ldob->Disturbance = ldob->Disturbance * ldob->dt / (ldob->LPF_RC + ldob->dt) +
                        ldob->Last_Disturbance * ldob->LPF_RC / (ldob->LPF_RC + ldob->dt);

    ldob->Disturbance = float_constrain(ldob->Disturbance, -ldob->Max_Disturbance, ldob->Max_Disturbance);

    // 扰动输出死区
    // deadband of disturbance output
    if (abs(ldob->Disturbance) > ldob->DeadBand * ldob->Max_Disturbance)
        ldob->Output = ldob->Disturbance;
    else
        ldob->Output = 0;

    ldob->Last_Measure = ldob->Measure;
    ldob->Last_Measure_dot = ldob->Measure_dot;
    ldob->Last_Disturbance = ldob->Disturbance;

    return ldob->Output;
}

/*************************** Tracking Differentiator ***************************/
void TD_Init(TD_t *td, float r, float h0)
{
    td->r = r;
    td->h0 = h0;

    td->x = 0;
    td->dx = 0;
    td->ddx = 0;
    td->last_dx = 0;
    td->last_ddx = 0;
}
float TD_Calculate(TD_t *td, float input)
{
    static float d, a0, y, a1, a2, a, fhan;

    td->dt = DWT_GetDeltaT((void *)&td->DWT_CNT);

    if (td->dt > 0.5f)
        return 0;

    td->Input = input;

    d = td->r * td->h0 * td->h0;
    a0 = td->dx * td->h0;
    y = td->x - td->Input + a0;
    a1 = sqrt(d * (d + 8 * abs(y)));
    a2 = a0 + sign(y) * (a1 - d) / 2;
    a = (a0 + y) * (sign(y + d) - sign(y - d)) / 2 + a2 * (1 - (sign(y + d) - sign(y - d)) / 2);
    fhan = -td->r * a / d * (sign(a + d) - sign(a - d)) / 2 -
           td->r * sign(a) * (1 - (sign(a + d) - sign(a - d)) / 2);

    td->ddx = fhan;
    td->dx += (td->ddx + td->last_ddx) * td->dt / 2;
    td->x += (td->dx + td->last_dx) * td->dt / 2;

    td->last_ddx = td->ddx;
    td->last_dx = td->dx;

    return td->x;
}
```

## FILE: gimbal_ws/middlewares/VT03/VT03.c

```c
#include "VT03.h"
#include "CRC.h"
#include "Referee.h"
#include "cmsis_os2.h"
#include "string.h"
#include "WatchDog.h"
#include "Variate.h"

VT03_t VT03;

Referee_t Referee = { .SOF = 0xA5,
											.HEAD_LEN = sizeof(FrameHeader_t),
											.CMD_ID_LEN = 2,
											.TAIL_LEN = 2,
											.DATA_START = 4
};

void VT03_Task(){
	VT03_Init(&huart6,true);
	VT03_Request();
	for(;;){
		osDelay(1);
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size){  
  if (huart == VT03.huart_) {
		VT03_Update(VT03.VT03_Buff_,Size);
    VT03_Request();
		Feed_Dog(&VT03_Dog);
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart){
  if (huart == VT03.huart_) {
    VT03_Request();
  }
}

void VT03_Init(UART_HandleTypeDef * huart,bool use_dma){
	VT03.huart_ = huart;
	VT03.use_dma_ = use_dma;
}
void VT03_Request(){
	if(VT03.use_dma_){
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(VT03.huart_, VT03.VT03_Buff_,VT03_BUFF_SIZE);
    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(VT03.huart_->hdmarx, DMA_IT_HT);		
	} else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(VT03.huart_,VT03.VT03_Buff_,VT03_BUFF_SIZE);		
	}	
}
void VT03_Update(uint8_t * frame_start, uint16_t size){
	VT03.has_read_ = true;
	
  if (frame_start[0] == 0xA9 && frame_start[1] == 0x53) {
    size_t frame_len = sizeof(VT03RemoteData_t);

    if (size < frame_len) return;
    if (!verify_CRC16_check_sum(frame_start, frame_len)) return;

    VT03_ToRemote((VT03RemoteData_t *)(frame_start));

    // 閫掑綊瑙ｆ瀽, 鍥犱负缂撳啿鍖轰腑鍙兘鍖呭惈澶氬抚瑁佸垽绯荤粺鐨勬暟鎹?
    VT03_Update(frame_start + frame_len,size - frame_len);
    return;
  }

  if (size < Referee.HEAD_LEN) return;
  if (frame_start[0] != Referee.SOF) return;
  if (!verify_CRC8_check_sum(frame_start, Referee.HEAD_LEN)) return;

  size_t data_len = (frame_start[2] << 8) | frame_start[1];
  size_t frame_len = Referee.HEAD_LEN + Referee.CMD_ID_LEN + data_len + Referee.TAIL_LEN;

  if (size < frame_len) return;
  if (!verify_CRC16_check_sum(frame_start, frame_len)) return;

  uint16_t cmd_id = (frame_start[6] << 8) | frame_start[5];
  // 閫掑綊瑙ｆ瀽, 鍥犱负缂撳啿鍖轰腑鍙兘鍖呭惈澶氬抚瑁佸垽绯荤粺鐨勬暟鎹?
  VT03_Update(frame_start + frame_len, size - frame_len);	
}
void VT03_ToRemote(const VT03RemoteData_t * data){
	VT03.ch_rx = (data->ch_0 - 1024) / 660.0f;
  VT03.ch_ry = (data->ch_1 - 1024) / 660.0f;
  VT03.ch_ly = (data->ch_2 - 1024) / 660.0f;
  VT03.ch_lx = (data->ch_3 - 1024) / 660.0f;
  VT03.wheel = (data->wheel - 1024) / 660.0f;

  VT03.fn_l = data->fn_1;
  VT03.fn_r = data->fn_2;
  VT03.pause = data->pause;
  VT03.trigger = data->trigger;

  VT03.mouse.vx = data->mouse_x / 32768.0f;
  VT03.mouse.vy = data->mouse_y / 32768.0f;
  VT03.mouse.vz = data->mouse_z / 32768.0f;
  VT03.mouse.left = data->mouse_left;
  VT03.mouse.middle = data->mouse_middle;
  VT03.mouse.right = data->mouse_right;

  VT03.keys.W = (data->keys & 0x0001);
  VT03.keys.S = (data->keys & 0x0002);
  VT03.keys.A = (data->keys & 0x0004);
  VT03.keys.D = (data->keys & 0x0008);
  VT03.keys.Shift = (data->keys & 0x0010);
  VT03.keys.Ctrl = (data->keys & 0x0020);
  VT03.keys.Q = (data->keys & 0x0040);
  VT03.keys.E = (data->keys & 0x0080);
  VT03.keys.R = (data->keys & 0x0100);
  VT03.keys.F = (data->keys & 0x0200);
  VT03.keys.G = (data->keys & 0x0400);
  VT03.keys.Z = (data->keys & 0x0800);
  VT03.keys.X = (data->keys & 0x1000);
  VT03.keys.C = (data->keys & 0x2000);
  VT03.keys.V = (data->keys & 0x4000);
  VT03.keys.B = (data->keys & 0x8000);

  VT03.mode = (data->mode_sw == 0) ? (VT03.mode = ComInput) : 
							(data->mode_sw == 1) ? (VT03.mode = Stop) : (VT03.mode = RcInput);
}
```

## FILE: gimbal_ws/middlewares/VT03/VT03.h

```c
#ifndef __VT03_H__
#define __VT03_H__

#include "main.h"
#include "stdbool.h"
#include "usart.h"

#define VT03_BUFF_SIZE 255

typedef struct{
  float yaw;
  float roll;
  float pitch;
  float roll2;
  float x;
  float y;
  float z;
  bool button;
  uint8_t reserved;  //淇濈暀浣?
}__PACKED RefereeCustomData_t; 
typedef struct {
  bool yaw_fdb_on;
  bool roll_fdb_on;
  bool pitch_fdb_on;
  bool roll2_fdb_on;
  bool x_fdb_on;
  bool y_fdb_on;
  bool z_fdb_on;
  uint8_t reserved[23];  //淇濈暀浣?23浣?
}__PACKED RefereeRobotData_t;

typedef struct{
  uint8_t sof_1;
  uint8_t sof_2;
  uint64_t ch_0 : 11;
  uint64_t ch_1 : 11;
  uint64_t ch_2 : 11;
  uint64_t ch_3 : 11;
  uint64_t mode_sw : 2;
  uint64_t pause : 1;
  uint64_t fn_1 : 1;
  uint64_t fn_2 : 1;
  uint64_t wheel : 11;
  uint64_t trigger : 1;
  uint64_t padding1 : 3;  

  int16_t mouse_x;  
  int16_t mouse_y;  
  int16_t mouse_z;  

  uint8_t mouse_left : 2;   
  uint8_t mouse_right : 2;   
  uint8_t mouse_middle : 2;  
  uint8_t padding2 : 2;      

  uint16_t keys;   
  uint16_t crc16;  
}__PACKED VT03RemoteData_t;
typedef struct{
  float vx;  // 鍙栧€艰寖鍥? [-1, 1]
  float vy;  // 鍙栧€艰寖鍥? [-1, 1]
  float vz;  // 鍙栧€艰寖鍥? [-1, 1], 榧犳爣婊氳疆
  bool left;
  bool middle;  //涓敭
  bool right;
}VT03MouseData_t;
typedef struct{
  bool W;
  bool S;
  bool A;
  bool D;
  bool Shift;
  bool Ctrl;
  bool Q;
  bool E;
  bool R;
  bool F;
  bool G;
  bool Z;
  bool X;
  bool C;
  bool V;
  bool B;
}__PACKED VT03KeysData_t;
typedef struct{
	enum VT03MODE{
	ComInput = 0,
	Stop = 1,
	RcInput = 2
	}mode;
	float ch_rx;
	float ch_ry;
	float ch_lx;
	float ch_ly;
	float wheel;
	float fn_l;
	float fn_r;
	float pause;
	float trigger;
	RefereeCustomData_t custom;
	RefereeRobotData_t robot; 
	VT03MouseData_t mouse;
	VT03KeysData_t keys;
	
	UART_HandleTypeDef * huart_;
	bool use_dma_;
	bool has_read_;
	uint8_t VT03_Buff_[VT03_BUFF_SIZE];
	
}VT03_t;
extern VT03_t VT03;

void VT03_Init(UART_HandleTypeDef * huart,bool use_dma);
void VT03_Request();
void VT03_Update(uint8_t * frame_start,uint16_t size);
void VT03_ToRemote(const VT03RemoteData_t * data);
extern void VT03_Task();

#endif
```

## FILE: RM_Lib/Src/CRC.c

```c
#include "CRC.h"
#include "stddef.h"

static const uint8_t CRC8_INIT = 0xff;
static const uint16_t CRC_INIT = 0xffff;

static const uint8_t CRC8_TAB[256] = {
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
        0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
        0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
        0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
        0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
        0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
        0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
        0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
        0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
        0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
        0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
        0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
        0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
        0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
        0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

static const uint16_t wCRC_Table[256] = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint8_t dwLength, uint8_t ucCRC8) {
    uint8_t ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint8_t dwLength) {
    if (pchMessage == NULL) return 0;
    return Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
}

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
    uint8_t chData;
    if (pchMessage == NULL)
        return CRC_INIT;
    while (dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

uint16_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {
    if (pchMessage == NULL) return 0;
    return Get_CRC16_Check_Sum(pchMessage, dwLength, CRC_INIT);
}
/***********************forback bool class*************************/
const uint8_t CRC8_table[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
uint16_t CRC16_INIT = 0xffff;
const uint16_t wCRC_table[256] =
{
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/**
  * @brief          calculate the crc8  
  * @param[in]      pch_message: data
  * @param[in]      dw_length: stream length = data + checksum
  * @param[in]      ucCRC8: init CRC8
  * @retval         calculated crc8
  */
/**
  * @brief          Calc CRC8
  * @param[in]      pch_message: data
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @param[in]      ucCRC8:Init CRC8
  * @retval         Calc all CRC8
  */
uint8_t get_CRC8_check_sum(unsigned char *pch_message,unsigned int dw_length,unsigned char ucCRC8)
{
    unsigned char uc_index;
    while (dw_length--)
    {
        uc_index = ucCRC8^(*pch_message++);
        ucCRC8 = CRC8_table[uc_index];
    }
    return(ucCRC8);
}
/**
  * @brief          CRC8 verify function  
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         true of false
  */
/**
  * @brief          CRC8鏍￠獙鍑芥暟
  * @param[in]      pch_message: 鏁版嵁
	* @param[in]      dw_length:   鏁版嵁鍜屾牎楠岀殑闀垮害 
  * @retval         鐪熸垨鍋?  */
uint32_t verify_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length)
{
    unsigned char ucExpected = 0;
    if ((pch_message == 0) || (dw_length <= 2))
    {
        return 0;
    }
    ucExpected = get_CRC8_check_sum (pch_message, dw_length - 1, CRC8_INIT);
    return ( ucExpected == pch_message[dw_length - 1] );
}


/**
  * @brief          append CRC8 to the end of data
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         none
  */
/**
  * @brief          娣诲姞CRC8鍒版暟鎹粨灏?  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @retval         none
  */
void append_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length)
{
    unsigned char ucCRC = 0;
    if ((pch_message == 0) || (dw_length <= 2))
    {
        return;
    }
    ucCRC = get_CRC8_check_sum((unsigned char *)pch_message, dw_length - 1, CRC8_INIT);
    pch_message[dw_length - 1] = ucCRC;
}


/**
  * @brief          calculate the crc16  
  * @param[in]      pch_message: data
  * @param[in]      dw_length: stream length = data + checksum
  * @param[in]      wCRC: init CRC16
  * @retval         calculated crc16
  */
/**
  * @brief          璁＄畻CRC16
  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @param[in]      wCRC:Init CRC16
  * @retval         璁＄畻瀹岀殑CRC16
  */
uint16_t get_CRC16_check_sum(uint8_t *pch_message,uint32_t dw_length,uint16_t wCRC)
{
    uint8_t chData;
    if (pch_message == NULL)
    {
        return 0xFFFF;
    }
    while(dw_length--)
    {
        chData = *pch_message++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


/**
  * @brief          CRC16 verify function  
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         true of false
  */
/**
  * @brief          CRC16鏍￠獙鍑芥暟
  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @retval         鐪熸垨鍋?  */
uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = get_CRC16_check_sum(pchMessage, dwLength - 2, CRC16_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


/**
  * @brief          append CRC16 to the end of data
  * @param[in]      pch_message: data
  * @param[in]      dw_length:stream length = data + checksum
  * @retval         none
  */
/**
  * @brief          娣诲姞CRC16鍒版暟鎹粨灏?  * @param[in]      pch_message: 鏁版嵁
  * @param[in]      dw_length: 鏁版嵁鍜屾牎楠岀殑闀垮害
  * @retval         none
  */
void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = get_CRC16_check_sum ( (uint8_t *)pchMessage, dwLength-2, CRC16_INIT );
    pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}
```

## FILE: gimbal_ws/middlewares/Referee.h

```c
#ifndef __REFEREE_H__
#define __REFEREE_H__

#include "main.h"
#include "stddef.h"
#include "stdint.h"

#define RED_HERO  1
#define RED_ENGINEER  2
 #define RED_STANDARD_3  3
 #define RED_STANDARD_4  4
 #define RED_STANDARD_5  5
 #define RED_AERIAL  6
 #define RED_SENTRY  7
 #define RED_RADAR  9
 #define RED_OUTPOST 10
 #define RED_BASE 11
 #define BLUE_HERO  101
 #define BLUE_ENGINEER 102
 #define BLUE_STANDARD_3 103
 #define BLUE_STANDARD_4 104
 #define BLUE_STANDARD_5 105
 #define BLUE_AERIAL 106
 #define BLUE_SENTRY 107
 #define BLUE_RADAR 109
 #define BLUE_OUTPOST 110
 #define BLUE_BASE 111

 #define RED_HERO_CLIENT 0x0101
 #define RED_ENGINEER_CLIENT 0x0102
 #define RED_STANDARD_3_CLIENT  0x0103
 #define RED_STANDARD_4_CLIENT  0x0104
 #define RED_STANDARD_5_CLIENT 0x0105
 #define RED_AERIAL_CLIENT  0x0106
 #define BLUE_HERO_CLIENT  0x0165
 #define BLUE_ENGINEER_CLIENT  0x0166
 #define BLUE_STANDARD_3_CLIENT  0x0167
 #define BLUE_STANDARD_4_CLIENT  0x0168
 #define BLUE_STANDARD_5_CLIENT  0x0169
 #define BLUE_AERIAL_CLIENT  0x016A
 #define REFEREE_SERVER  0x8080

 #define GAME_STATUS  0x0001    // 姣旇禌鐘舵€佹暟鎹?
 #define GAME_RESULT  0x0002    // 姣旇禌缁撴灉鏁版嵁
 #define GAME_ROBOT_HP  0x0003  // 姣旇禌鏈哄櫒浜鸿閲忔暟鎹?

 #define EVENT_DATA  0x0101       // 鍦哄湴浜嬩欢鏁版嵁
 #define REFEREE_WARNING  0x0104  // 瑁佸垽璀﹀憡鏁版嵁
 #define DART_INFO  0x0105        // 椋為晼鍙戝皠鐩稿叧鏁版嵁

 #define ROBOT_STATUS  0x0201           // 鏈哄櫒浜烘€ц兘浣撶郴鏁版嵁
 #define POWER_HEAT_DATA  0x0202        // 瀹炴椂搴曠洏缂撳啿鑳介噺鍜屽皠鍑荤儹閲忔暟鎹?
 #define ROBOT_POS  0x0203              // 鏈哄櫒浜轰綅缃暟鎹?
 #define BUFF  0x0204                   // 鏈哄櫒浜哄鐩婃暟鎹?
                                                    // 娌℃湁0x0205
 #define HURT_DATA  0x0206              // 浼ゅ鐘舵€佹暟鎹?
 #define SHOOT_DATA  0x0207             // 瀹炴椂灏勫嚮鏁版嵁
 #define PROJECTILE_ALLOWANCE  0x0208   // 鍏佽鍙戝脊閲?
 #define RFID_STATUS  0x0209            // 鏈哄櫒浜篟FID妯″潡鐘舵€?
 #define DART_CLIENT_CMD  0x020A        // 椋為晼閫夋墜绔寚浠ゆ暟鎹?
 #define GROUND_ROBOT_POSITION  0x020B  // 鍦伴潰鏈哄櫒浜轰綅缃暟鎹?
 #define RADAR_MARK_DATA  0x020C        // 闆疯揪鏍囪杩涘害鏁版嵁
 #define SENTRY_INFO  0x020D            // 鍝ㄥ叺鑷富鍐崇瓥淇℃伅鍚屾
 #define RADAR_INFO  0x020E             // 闆疯揪鑷富鍐崇瓥淇℃伅鍚屾

 #define ROBOT_INTERACTION_DATA  0x0301  // 鏈哄櫒浜轰氦浜掓暟鎹?
 #define CUSTOM_ROBOT_DATA  0x0302       // 鑷畾涔夋帶鍒跺櫒涓庢満鍣ㄤ汉浜や簰鏁版嵁 鍥句紶閾捐矾
 #define MAP_COMMAND  0x0303             // 閫夋墜绔皬鍦板浘浜や簰鏁版嵁
 #define MAP_ROBOT_DATA  0x0305          // 閫夋墜绔皬鍦板浘鎺ユ敹闆疯揪鏁版嵁
 #define CUSTOM_CLIENT_DATA  0x0306      // 鑷畾涔夋帶鍒跺櫒涓庨€夋墜绔氦浜掓暟鎹?
 #define MAP_DATA 0x0307                // 閫夋墜绔皬鍦板浘鎺ユ敹鍝ㄥ叺鏁版嵁
 #define CUSTOM_INFO  0x0308             // 閫夋墜绔皬鍦板浘鎺ユ敹鏈哄櫒浜烘暟鎹?
 #define ROBOT_CUSTOM_DATA  0x0309       // 鑷畾涔夋帶鍒跺櫒鎺ユ敹鏈哄櫒浜烘暟鎹?鍥句紶閾捐矾
 #define ROBOT_CLIENT_DATA  0x0310       // 鏈哄櫒浜哄彂閫佺粰鑷畾涔夊鎴风鐨勬暟鎹?鍥句紶閾捐矾
 #define SET_IMAGE_TRANSFER  0x0F01      //璁剧疆鍥句紶鍑哄浘淇￠亾 鍥句紶閾捐矾
 #define INQUIRY_IMAGE_TRANSFER  0x0F02  //鏌ヨ鍥句紶鍑哄浘淇￠亾 鍥句紶閾捐矾

 #define INTERACTION_LAYER_DELETE  0x0100    // 閫夋墜绔垹闄ゅ浘灞?
 #define INTERACTION_FIGURE 0x0101         // 閫夋墜绔粯鍒朵竴涓浘褰?
 #define INTERACTION_FIGURE_2  0x0102         // 閫夋墜绔粯鍒朵袱涓浘褰?
 #define INTERACTION_FIGURE_5  0x0103         // 閫夋墜绔粯鍒朵簲涓浘褰?
 #define INTERACTION_FIGURE_7  0x0104         // 閫夋墜绔粯鍒朵竷涓浘褰?
 #define EXT_CLIENT_CUSTOM_CHARACTER 0x0110  // 閫夋墜绔粯鍒跺瓧绗﹀浘褰?
 #define SENTRY_CMD 0x0120                   // 鍝ㄥ叺鑷富鍐崇瓥鎸囦护 TODO
 #define RADAR_CMD 0x0121                    // 闆疯揪鑷富鍐崇瓥鎸囦护 TODO

// 0x0001 姣旇禌鐘舵€佹暟鎹?
typedef struct{
  uint8_t game_type : 4;
  // bit 0-3: 姣旇禌绫诲瀷
  //       1: RoboMaster鏈虹敳澶у笀瓒呯骇瀵规姉璧?
  //       2: RoboMaster鏈虹敳澶у笀楂樻牎鍗曢」璧?
  //       3: ICRA RoboMaster楂樻牎浜哄伐鏅鸿兘鎸戞垬璧?
  //       4: RoboMaster鏈虹敳澶у笀楂樻牎鑱旂洘璧?V3瀵规姉
  //       5: RoboMaster鏈虹敳澶у笀楂樻牎鑱旂洘璧涙鍏靛鎶?
  uint8_t game_progress : 4;
  // bit 4-7: 褰撳墠姣旇禌闃舵
  //       0: 鏈紑濮嬫瘮璧?
  //       1: 鍑嗗闃舵
  //       2: 鍗佷簲绉掕鍒ょ郴缁熻嚜妫€闃舵
  //       3: 浜旂鍊掕鏃?
  //       4: 姣旇禌涓?
  //       5: 姣旇禌缁撶畻涓?
  uint16_t stage_remain_time;  // 鍗曚綅: s
  uint64_t sync_timestamp;     // UNIX鏃堕棿锛屽綋鏈哄櫒浜烘纭繛鎺ュ埌瑁佸垽绯荤粺鐨凬TP鏈嶅姟鍣ㄥ悗鐢熸晥
} __attribute__((packed)) GameStatus_t;

// 0x0002 姣旇禌缁撴灉鏁版嵁
typedef struct {
  uint8_t winner;  // 0: 骞冲眬, 1: 绾㈡柟鑳滃埄, 2: 钃濇柟鑳滃埄
}__attribute__((packed)) GameResult_t;

// 0x0003 姣旇禌鏈哄櫒浜鸿閲忔暟鎹?
typedef struct{
  uint16_t ally_1_robot_hp;  // 宸辨柟 1 鑻遍泟鏈哄櫒浜鸿閲?
  uint16_t ally_2_robot_hp;  // 宸辨柟 2 宸ョ▼鏈哄櫒浜鸿閲?
  uint16_t ally_3_robot_hp;  // 宸辨柟 3 姝ュ叺鏈哄櫒浜鸿閲?
  uint16_t ally_4_robot_hp;  // 宸辨柟 4 姝ュ叺鏈哄櫒浜鸿閲?
  uint16_t reserved;         // 淇濈暀
  uint16_t ally_7_robot_hp;  // 宸辨柟 7 鍝ㄥ叺鏈哄櫒浜鸿閲?
  uint16_t ally_outpost_hp;  // 宸辨柟鍓嶅摠绔欒閲?
  uint16_t ally_base_hp;     // 宸辨柟鍩哄湴琛€閲?
} __attribute__((packed)) GameRobotHP_t;

// 0x0101 鏈哄櫒浜轰簨浠舵暟鎹?
typedef struct {
  // 0:鏈崰棰?鏈縺娲?
  // 1:宸插崰棰?宸叉縺娲?
  uint32_t supply_status : 3;  // bit 0-2
  // bit 0锛氬繁鏂逛笌鍏戞崲鍖轰笉閲嶅彔鐨勮ˉ缁欏尯鍗犻鐘舵€侊紝1 涓哄凡鍗犻
  // bit 1锛氬繁鏂逛笌鍏戞崲鍖洪噸鍙犵殑琛ョ粰鍖哄崰棰嗙姸鎬侊紝1 涓哄凡鍗犻
  // bit 2锛氬繁鏂硅ˉ缁欏尯鐨勫崰棰嗙姸鎬侊紝1 涓哄凡鍗犻锛堜粎 RMUL 閫傜敤锛?
  uint32_t energy_status : 4;  // bit 3-6
  // bit 3-4锛氬繁鏂瑰皬鑳介噺鏈哄叧鐨勬縺娲荤姸鎬侊紝0 涓烘湭婵€娲伙紝1 涓哄凡婵€娲伙紝2 涓烘鍦ㄦ縺娲?
  // bit 5-6锛氬繁鏂瑰ぇ鑳介噺鏈哄叧鐨勬縺娲荤姸鎬侊紝0 涓烘湭婵€娲伙紝1 涓哄凡婵€娲伙紝2 涓烘鍦ㄦ縺娲?
  uint32_t central_highground_status : 2;  // bit 7-8
  // bit 7-8锛氬繁鏂逛腑澶珮鍦扮殑鍗犻鐘舵€侊紝1 涓鸿宸辨柟鍗犻锛? 涓鸿瀵规柟鍗犻
  uint32_t trapezoidal_highground_status : 2;  // bit 9-10
  // bit 7-8锛氬繁鏂规褰㈤珮鍦扮殑鍗犻鐘舵€侊紝1 涓哄凡鍗犻
  uint32_t last_hit_time : 9;  // bit 11-19
  // bit  9-17锛氬鏂归闀栨渶鍚庝竴娆″嚮涓繁鏂瑰墠鍝ㄧ珯鎴栧熀鍦扮殑鏃堕棿锛?-420锛屽紑灞€榛樿涓?0锛?
  uint32_t last_hit_target : 3;  // bit 20-22
  // bit 18-20锛氬鏂归闀栨渶鍚庝竴娆″嚮涓繁鏂瑰墠鍝ㄧ珯鎴栧熀鍦扮殑鍏蜂綋鐩爣锛屽紑灞€榛樿涓?0
  // 1 涓哄嚮涓墠鍝ㄧ珯锛? 涓哄嚮涓熀鍦板浐瀹氱洰鏍囷紝3 涓哄嚮涓熀鍦伴殢鏈哄浐瀹氱洰鏍囷紝4 涓哄嚮涓熀鍦伴殢鏈虹Щ鍔ㄧ洰鏍?
  uint32_t center_boost_status : 2;  // bit 23-24
  // 涓績澧炵泭鐐圭殑鍗犻鐘舵€侊紝0 涓烘湭琚崰棰嗭紝1 涓鸿宸辨柟鍗犻锛?涓鸿瀵规柟鍗犻锛? 涓鸿鍙屾柟鍗犻銆傦紙浠?RMUL 閫傜敤锛?
  uint32_t fortess_boost_status : 2;  // bit 25-26
  // 宸辨柟鍫″瀿澧炵泭鐐圭殑鍗犻鐘舵€侊紝0 涓烘湭琚崰棰嗭紝1 涓鸿宸辨柟鍗犻锛?涓鸿瀵规柟鍗犻锛? 涓鸿鍙屾柟鍗犻銆?
  uint32_t outpost_boost_status : 2;  // bit 27-28
  // 宸辨柟鍓嶅摠绔欏鐩婄偣鐨勫崰棰嗙姸鎬侊紝0 涓烘湭琚崰棰嗭紝1 涓鸿宸辨柟鍗犻锛?涓鸿鍙屾柟鍗犻銆?
  uint32_t base_boost_status : 2;  // bit 29
  // 宸辨柟鍩哄湴澧炵泭鐐圭殑鍗犻鐘舵€侊紝1 涓哄凡鍗犻銆?
  uint32_t reserved : 2;  // bit 30-31 : 淇濈暀
}__attribute__((packed)) EventData_t;

// 0x0104 瑁佸垽璀﹀憡鏁版嵁
typedef struct {
  uint8_t level;
  uint8_t offefending_robot_id;
  uint8_t count;
}__attribute__((packed)) RefereeWarning_t;

// 0x0105 椋為晼鍙戝皠鐩稿叧鏁版嵁
typedef struct {
  uint8_t dart_remaining_time;  //宸辨柟椋為晼鍙戝皠绔欏墿浣欓闀栧彂灏勬椂闂达紝鍗曚綅锛歴
  uint16_t dart_info;
  uint16_t last_hit_target : 3;
  // bit 0-2锛氬鏂归闀栨渶鍚庝竴娆″嚮涓洰鏍?
  //       0锛氬紑灞€榛樿鍊?
  //       1锛氬墠鍝ㄧ珯
  //       2锛氬熀鍦板浐瀹氱洰鏍?
  //       3锛氬熀鍦伴殢鏈哄浐瀹氱洰鏍?
  //       4锛氬熀鍦伴殢鏈虹Щ鍔ㄧ洰鏍?
  //       5锛氬熀鍦版湯绔Щ鍔ㄧ洰鏍?
  uint16_t total_hit_count : 3;
  // bit 3-5锛氬鏂规渶杩戣鍑讳腑鐩爣鐨勭疮璁℃鏁帮紝寮€灞€榛樿鍊间负 0锛岃嚦澶氫负 4
  uint16_t current_selected_target : 3;
  //bit 6-8锛氬綋鍓嶉闀栭€夋嫨鐨勫嚮鎵撶洰鏍囷紝寮€灞€榛樿鍊间负 0
  //       1锛氬熀鍦板浐瀹氱洰鏍?
  //       2锛氬熀鍦伴殢鏈哄浐瀹氱洰鏍?
  //       3锛氬熀鍦伴殢鏈虹Щ鍔ㄧ洰鏍?
  //       4锛氬熀鍦版湯绔Щ鍔ㄧ洰鏍?
  uint16_t reserved : 7;  // bit 9-15锛氫繚鐣?
}__attribute__((packed)) DartInfo_t;

// 0x0201 鏈哄櫒浜虹姸鎬佹暟鎹?
typedef struct {
  uint8_t robot_id;                             // 鏈哄櫒浜篒D
  uint8_t robot_level;                          // 鏈哄櫒浜虹瓑绾?
  uint16_t current_hp;                          // 褰撳墠琛€閲?
  uint16_t maximum_hp;                          // 鏈€澶ц閲?
  uint16_t shooter_barrel_cooling_value;        // 鍙戝皠鏈烘瀯鍐峰嵈閫熺巼
  uint16_t shooter_barrel_heat_limit;           // 鍙戝皠鏈烘瀯鐑噺涓婇檺
  uint16_t chassis_power_limit;                 // 搴曠洏鍔熺巼涓婇檺
  uint8_t power_management_gimbal_output : 1;   // gimbal鍙ｈ緭鍑? 0涓烘棤杈撳嚭, 1涓?4V杈撳嚭
  uint8_t power_management_chassis_output : 1;  // chassis鍙ｈ緭鍑? 0涓烘棤杈撳嚭, 1涓?4V杈撳嚭
  uint8_t power_management_shooter_output : 1;  // shooter鍙ｈ緭鍑? 0涓烘棤杈撳嚭, 1涓?4V杈撳嚭
}__attribute__((packed)) RobotStatus_t;

// 0x0202 瀹炴椂搴曠洏缂撳啿鑳介噺鍜屽皠鍑荤儹閲忔暟鎹?
typedef struct {
  uint16_t reserved_1;
  uint16_t reserved_2;
  float reserved_3;
  uint16_t buffer_energy;               // 缂撳啿鑳介噺锛堝崟浣嶏細J锛?
  uint16_t shooter_17mm_1_barrel_heat;  // 绗?1 涓?17mm 鍙戝皠鏈烘瀯鐨勫皠鍑荤儹閲?
  uint16_t shooter_17mm_2_barrel_heat;  // 绗?2 涓?17mm 鍙戝皠鏈烘瀯鐨勫皠鍑荤儹閲?
  uint16_t shooter_42mm_barrel_heat;    // 42mm 鍙戝皠鏈烘瀯鐨勫皠鍑荤儹閲?
}__attribute__((packed)) PowerHeatData_t;

// 0x0203 鏈哄櫒浜轰綅缃暟鎹?
typedef struct {
  float x;      // 鏈満鍣ㄤ汉浣嶇疆 x 鍧愭爣锛屽崟浣嶏細m
  float y;      // 鏈満鍣ㄤ汉浣嶇疆 y 鍧愭爣锛屽崟浣嶏細m
  float angle;  // 鏈満鍣ㄤ汉娴嬮€熸ā鍧楃殑鏈濆悜锛屽崟浣嶏細搴︺€傛鍖椾负 0 搴?
}__attribute__((packed)) RobotPos_t;

// 0x0204 鏈哄櫒浜哄鐩婃暟鎹?
typedef struct{
	uint8_t recovery_y_buff;  // 鏈哄櫒浜哄洖琛€澧炵泭锛堢櫨鍒嗘瘮锛屽€间负 10 琛ㄧず姣忕鎭㈠琛€閲忎笂闄愮殑 10%锛?
  uint16_t cooling_buff;    // 鏈哄櫒浜哄皠鍑荤儹閲忓喎鍗村鐩婂叿浣撳€硷紙鐩存帴鍊硷紝鍊间负 x 琛ㄧず鐑噺鍐峰嵈澧炲姞 x/s锛?
  uint8_t defence_buff;     // 鏈哄櫒浜洪槻寰″鐩婏紙鐧惧垎姣旓紝鍊间负 50 琛ㄧず 50%闃插尽澧炵泭锛?
  uint8_t vulnerability_buff;  // 鏈哄櫒浜鸿礋闃插尽澧炵泭锛堢櫨鍒嗘瘮锛屽€间负 30 琛ㄧず-30%闃插尽澧炵泭锛?
  uint16_t attack_buff;        // 鏈哄櫒浜烘敾鍑诲鐩婏紙鐧惧垎姣旓紝鍊间负 50 琛ㄧず 50%鏀诲嚮澧炵泭锛?
  uint8_t remaining_energy;
  // bit  0-6锛氭満鍣ㄤ汉鍓╀綑鑳介噺鍊煎弽棣堬紝
  // 浠?16 杩涘埗鏍囪瘑鏈哄櫒浜哄墿浣欒兘閲忓€兼瘮渚嬶紝
  // 浠呭湪鏈哄櫒浜哄墿浣欒兘閲忓皬浜?50%鏃跺弽棣堬紝鍏朵綑榛樿鍙嶉 0x80銆傛満鍣ㄤ汉鍒濆鑳介噺瑙嗕负100%
  // bit:0: 鍦ㄥ墿浣欒兘閲忊墺125%鏃朵负 1锛屽叾浣欐儏鍐典负 0
  // bit:1: 鍦ㄥ墿浣欒兘閲忊墺100%鏃朵负 1锛屽叾浣欐儏鍐典负 0
  // bit 2锛氬湪鍓╀綑鑳介噺鈮?0%鏃朵负 1锛屽叾浣欐儏鍐典负 0
  // bit 3锛氬湪鍓╀綑鑳介噺鈮?0%鏃朵负 1锛屽叾浣欐儏鍐典负 0
  // bit 4锛氬湪鍓╀綑鑳介噺鈮?5%鏃朵负 1锛屽叾浣欐儏鍐典负 0
  // bit 5锛氬湪鍓╀綑鑳介噺鈮?%鏃朵负 1锛屽叾浣欐儏鍐典负 0
  // bit 6锛氬湪鍓╀綑鑳介噺鈮?%鏃朵负 1锛屽叾浣欐儏鍐典负 0
} __attribute__((packed)) Buff_t;

// 0x0206 浼ゅ鐘舵€佹暟鎹?
typedef struct {
  uint8_t armor_id : 4;
  // bit 0-3锛氬綋鎵ｈ鍘熷洜涓鸿鐢叉ā鍧楄寮逛父鏀诲嚮銆佸彈鎾炲嚮銆佺绾挎垨娴嬮€熸ā鍧楃绾挎椂锛?
  // 璇?4 bit 缁勬垚鐨勬暟鍊间负瑁呯敳妯″潡鎴栨祴閫熸ā鍧楃殑 ID 缂栧彿锛?
  // 褰撳叾浠栧師鍥犲鑷存墸琛€鏃讹紝璇ユ暟鍊间负 0
  uint8_t HP_deduction_reason : 4;
  // bit 4-7锛氳閲忓彉鍖栫被鍨?
  // 0锛氳鐢叉ā鍧楄寮逛父鏀诲嚮瀵艰嚧鎵ｈ
  // 1锛氳鐢叉ā鍧楁垨瓒呯骇鐢靛绠＄悊妯″潡绂荤嚎瀵艰嚧鎵ｈ
  // 5锛氳鐢叉ā鍧楀彈鍒版挒鍑诲鑷存墸琛€
}__attribute__((packed)) HurtData_t;

// 0x0207 瀹炴椂灏勫嚮鏁版嵁
typedef struct 
{
  uint8_t bullet_type;          // 1: 17mm寮逛父, 2: 42mm寮逛父
  uint8_t shooter_number;       // 1: 绗?涓?7mm鍙戝皠鏈烘瀯, 2: 绗?涓?7mm鍙戝皠鏈烘瀯, 3: 42mm鍙戝皠鏈烘瀯
  uint8_t launching_frequency;  // 鍗曚綅: Hz
  float initial_speed;          // 鍗曚綅: m/s
}__attribute__((packed)) ShootData_t;

// 0x0208 鍏佽鍙戝脊閲?
typedef struct
{
  uint16_t projectile_allowance_17mm;  // 17mm 寮逛父鍏佽鍙戝脊閲?
  uint16_t projectile_allowance_42mm;  // 42mm 寮逛父鍏佽鍙戝脊閲?
  uint16_t remaining_gold_coin;        // 鍓╀綑閲戝竵鏁伴噺
  uint16_t
    projectile_allowance_fortress;  // 鍫″瀿澧炵泭鐐规彁渚涚殑鍌ㄥ 17mm 寮逛父鍏佽鍙戝脊閲忥紱璇ュ€间笌鏈哄櫒浜烘槸鍚﹀疄闄呭崰棰嗗牎鍨掓棤鍏?
} __attribute__((packed)) ProjectileAllowance_t;

// 0x0209 鏈哄櫒浜篟FID妯″潡鐘舵€?
typedef struct
{
  uint32_t friendly_base : 1;             // bit 0锛氬繁鏂瑰熀鍦板鐩婄偣
  uint32_t friendly_central : 1;          // bit 1锛氬繁鏂逛腑澶珮鍦板鐩婄偣
  uint32_t enemy_central : 1;             // bit 2锛氬鏂逛腑澶珮鍦板鐩婄偣
  uint32_t friendly_trapezoid : 1;        // bit 3锛氬繁鏂规褰㈤珮鍦板鐩婄偣
  uint32_t enemy_trapezoid : 1;           // bit 4锛氬鏂规褰㈤珮鍦板鐩婄偣
  uint32_t friendly_flying_slope_f : 1;   // bit 5锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥鍧″墠锛岄潬杩戝繁鏂逛竴渚э級
  uint32_t friendly_flying_slope_b : 1;   // bit 6锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥鍧″悗锛岄潬杩戝繁鏂逛竴渚э級
  uint32_t enemy_flying_slope_f : 1;      // bit 7锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥鍧″墠锛岄潬杩戝鏂逛竴渚э級
  uint32_t enemy_flying_slope_b : 1;      // bit 8锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥鍧″悗锛岄潬杩戝鏂逛竴渚э級
  uint32_t friendly_ch_down : 1;          // bit 9锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堜腑澶珮鍦颁笅鏂癸級
  uint32_t friendly_ch_up : 1;            // bit 10锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堜腑澶珮鍦颁笂鏂癸級
  uint32_t enemy_ch_down : 1;             // bit 11锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堜腑澶珮鍦颁笅鏂癸級
  uint32_t enemy_ch_up : 1;               // bit 12锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堜腑澶珮鍦颁笂鏂癸級
  uint32_t friendly_road_down : 1;        // bit 13锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堝叕璺笅鏂癸級
  uint32_t friendly_road_up : 1;          // bit 14锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堝叕璺笂鏂癸級
  uint32_t enemy_road_down : 1;           // bit 15锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堝叕璺笅鏂癸級
  uint32_t enemy_road_up : 1;             // bit 16锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堝叕璺笂鏂癸級
  uint32_t friendly_fort : 1;             // bit 17锛氬繁鏂瑰牎鍨掑鐩婄偣
  uint32_t friendly_outpost : 1;          // bit 18锛氬繁鏂瑰墠鍝ㄧ珯澧炵泭鐐?
  uint32_t friendly_supply_no_trade : 1;  // bit 19锛氬繁鏂逛笌鍏戞崲鍖轰笉閲嶅彔鐨勮ˉ缁欏尯 / RMUL 琛ョ粰鍖?
  uint32_t friendly_supply_trade : 1;     // bit 20锛氬繁鏂逛笌鍏戞崲鍖洪噸鍙犵殑琛ョ粰鍖?
  uint32_t friendly_big_resource : 1;     // bit 21锛氬繁鏂瑰ぇ璧勬簮宀涘鐩婄偣
  uint32_t enemy_big_resource : 1;        // bit 22锛氬鏂瑰ぇ璧勬簮宀涘鐩婄偣
  uint32_t center_point : 1;              // bit 23锛氫腑蹇冨鐩婄偣锛堜粎 RMUL 閫傜敤锛?
  uint32_t enemy_fort : 1;                // bit 24锛氬鏂瑰牎鍨掑鐩婄偣
  uint32_t enemy_outpost : 1;             // bit 25锛氬鏂瑰墠鍝ㄧ珯澧炵泭鐐?
  uint32_t friendly_tunnel_road_down
    : 1;  // bit 26锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂逛竴渚у叕璺尯涓嬫柟锛?
  uint32_t friendly_tunnel_road_mid
    : 1;  // bit 27锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂逛竴渚у叕璺尯涓棿锛?
  uint32_t friendly_tunnel_road_up
    : 1;  // bit 28锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂逛竴渚у叕璺尯涓婃柟锛?
  uint32_t friendly_tunnel_trapezoid_low
    : 1;  //bit 29锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂规褰㈤珮鍦拌緝浣庡锛?
  uint32_t friendly_tunnel_trapezoid_mid
    : 1;  //bit 30锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂规褰㈤珮鍦拌緝涓棿澶勶級
  uint32_t friendly_tunnel_trapezoid_high
    : 1;  //bit 31锛氬繁鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂规褰㈤珮鍦拌緝楂樺锛?
  uint32_t enemy_tunnel_road_down
    : 1;  // bit 0锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂逛竴渚у叕璺尯涓嬫柟锛?
  uint32_t enemy_tunnel_road_mid
    : 1;                              // bit 1锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂逛竴渚у叕璺尯涓棿锛?
  uint32_t enemy_tunnel_road_up : 1;  // bit 2锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂逛竴渚у叕璺尯涓婃柟锛?
  uint8_t enemy_tunnel_trapezoid_low
    : 1;  //bit 3锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂规褰㈤珮鍦拌緝浣庡锛?
  uint8_t enemy_tunnel_trapezoid_mid
    : 1;  //bit 4锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂规褰㈤珮鍦拌緝涓棿澶勶級
  uint8_t enemy_tunnel_trapezoid_high
    : 1;  //bit 5锛氬鏂瑰湴褰㈣法瓒婂鐩婄偣锛堥毀閬擄級锛堥潬杩戝繁鏂规褰㈤珮鍦拌緝楂樺锛?
} __attribute__((packed)) RFID_Status_t;

// 0x020A 椋為晼閫夋墜绔寚浠ゆ暟鎹?
typedef struct
{
  uint8_t
    dart_launch_opening_status;  // 褰撳墠椋為晼鍙戝皠绔欑殑鐘舵€侊細 1锛氬叧闂?2锛氭鍦ㄥ紑鍚垨鑰呭叧闂腑 0锛氬凡缁忓紑鍚?
  uint8_t reserved;
  uint16_t target_change_time;  // 鍒囨崲鍑绘墦鐩爣鏃剁殑姣旇禌鍓╀綑鏃堕棿锛屽崟浣嶏細绉掞紝鏃?鏈垏鎹㈠姩浣滐紝榛樿涓?0銆?
  uint16_t
    latest_launch_cmd_time;  // 鍙戦€佹渶鏂伴闀栧彂灏勬寚浠ょ殑姣旇禌鍓╀綑鏃堕棿锛屽崟浣嶏細绉掞紝鏃?鏈彂灏勶紝榛樿涓?0銆?
} __attribute__((packed)) DartClientCmd_t;

// 0x020B 鍦伴潰鏈哄櫒浜轰綅缃暟鎹?
typedef struct 
{
  float hero_x;        // 宸辨柟鑻遍泟鏈哄櫒浜轰綅缃?x 杞村潗鏍囷紝鍗曚綅锛歮
  float hero_y;        // 宸辨柟鑻遍泟鏈哄櫒浜轰綅缃?y 杞村潗鏍囷紝鍗曚綅锛歮
  float engineer_x;    // 宸辨柟宸ョ▼鏈哄櫒浜轰綅缃?x 杞村潗鏍囷紝鍗曚綅锛歮
  float engineer_y;    // 宸辨柟宸ョ▼鏈哄櫒浜轰綅缃?y 杞村潗鏍囷紝鍗曚綅锛歮
  float standard_3_x;  // 宸辨柟 3 鍙锋鍏垫満鍣ㄤ汉浣嶇疆 x 杞村潗鏍囷紝鍗曚綅锛歮
  float standard_3_y;  // 宸辨柟 3 鍙锋鍏垫満鍣ㄤ汉浣嶇疆 y 杞村潗鏍囷紝鍗曚綅锛歮
  float standard_4_x;  // 宸辨柟 4 鍙锋鍏垫満鍣ㄤ汉浣嶇疆 x 杞村潗鏍囷紝鍗曚綅锛歮
  float standard_4_y;  // 宸辨柟 4 鍙锋鍏垫満鍣ㄤ汉浣嶇疆 y 杞村潗鏍囷紝鍗曚綅锛歮
  float reserved1;     // 淇濈暀
  float reserved2;     // 淇濈暀
}__attribute__((packed)) GroundRobotPosition_t;

// 0x020C 闆疯揪鏍囪杩涘害鏁版嵁
typedef struct
{
  // 瀵规柟鏈哄櫒浜猴細鍦ㄥ搴旀満鍣ㄤ汉琚爣璁拌繘搴︹墺100 鏃跺彂閫?1锛岃鏍囪杩涘害<100 鏃跺彂閫?0銆?
  // 宸辨柟鏈哄櫒浜猴細鍦ㄥ搴旀満鍣ㄤ汉琚爣璁拌繘搴︹墺50 鏃跺彂閫?1锛岃鏍囪杩涘害<50 鏃跺彂閫?0銆?
  uint16_t enemy_hero_1_vulnerable : 1;      // bit 0: 瀵规柟 1鍙疯嫳闆勬満鍣ㄤ汉鏄撲激鎯呭喌
  uint16_t enemy_engineer_2_vulnerable : 1;  // bit 1: 瀵规柟 2鍙峰伐绋嬫満鍣ㄤ汉鏄撲激鎯呭喌
  uint16_t enemy_infantry_3_vulnerable : 1;  // bit 2: 瀵规柟 3鍙锋鍏垫満鍣ㄤ汉鏄撲激鎯呭喌
  uint16_t enemy_infantry_4_vulnerable : 1;  // bit 3: 瀵规柟 4鍙锋鍏垫満鍣ㄤ汉鏄撲激鎯呭喌
  uint16_t enemy_uav_vulnerable : 1;         // bit 4: 瀵规柟绌轰腑鏈哄櫒浜烘槗浼ゆ儏鍐?
  uint16_t enemy_sentry_vulnerable : 1;      // bit 5: 瀵规柟鍝ㄥ叺鏈哄櫒浜烘槗浼ゆ儏鍐?
  uint16_t friendly_hero_1_special : 1;      // bit 6: 宸辨柟 1鍙疯嫳闆勬満鍣ㄤ汉鐗规畩鏍囪瘑鎯呭喌
  uint16_t friendly_engineer_2_special : 1;  // bit 7: 宸辨柟 2鍙峰伐绋嬫満鍣ㄤ汉鐗规畩鏍囪瘑鎯呭喌
  uint16_t friendly_infantry_3_special : 1;  // bit 8: 宸辨柟 3鍙锋鍏垫満鍣ㄤ汉鐗规畩鏍囪瘑鎯呭喌
  uint16_t friendly_infantry_4_special : 1;  // bit 9: 宸辨柟 4鍙锋鍏垫満鍣ㄤ汉鐗规畩鏍囪瘑鎯呭喌
  uint16_t friendly_uav_special : 1;         // bit 10: 宸辨柟绌轰腑鏈哄櫒浜虹壒娈婃爣璇嗘儏鍐?
  uint16_t friendly_sentry_special : 1;      // bit 11: 宸辨柟鍝ㄥ叺鏈哄櫒浜虹壒娈婃爣璇嗘儏鍐?
  uint16_t reserved : 4;                     // bit 12-15: 淇濈暀
} __attribute__((packed)) RadarMarkData_t;

// 0x020D 鍝ㄥ叺鑷富鍐崇瓥淇℃伅鍚屾
typedef struct 
{
  uint32_t allowed_fire_amount : 11;  // bits 0-10  : 闄よ繙绋嬪厬鎹㈠锛屽摠鍏垫満鍣ㄤ汉鎴愬姛鍏戞崲鐨勫厑璁稿彂寮归噺
  uint32_t remote_exchange_fire_count : 4;    // bits 11-14 : 鍝ㄥ叺鏈哄櫒浜烘垚鍔熻繙绋嬪厬鎹㈠厑璁稿彂寮归噺鐨勬鏁?
  uint32_t remote_exchange_health_count : 4;  // bits 15-18 : 鍝ㄥ叺鏈哄櫒浜烘垚鍔熻繙绋嬪厬鎹㈣閲忕殑娆℃暟
  uint32_t can_confirm_free_resurrect : 1;    // bit 19     : 鍝ㄥ叺鏈哄櫒浜哄綋鍓嶆槸鍚﹀彲浠ョ‘璁ゅ厤璐瑰娲?
  uint32_t can_exchange_immediate_resurrect : 1;  // bit 20     : 鍝ㄥ叺鏈哄櫒浜哄綋鍓嶆槸鍚﹀彲浠ュ厬鎹㈢珛鍗冲娲?
  uint32_t immediate_resurrect_cost
    : 10;                      // bits 21-30 : 鍝ㄥ叺鏈哄櫒浜哄綋鍓嶈嫢鍏戞崲绔嬪嵆澶嶆椿闇€瑕佽姳璐圭殑閲戝竵鏁?
  uint32_t reserved1 : 1;      // bit 31     : 淇濈暀
  uint16_t out_of_combat : 1;  // bit 0     : 鍝ㄥ叺褰撳墠鏄惁澶勪簬鑴辨垬鐘舵€侊紝1 涓鸿劚鎴橈紝0 涓哄湪鎴?
  uint16_t ammo_exchange_allowance : 11;  // bit 1-11  : 闃熶紞 17mm 鍏佽鍙戝脊閲忕殑鍓╀綑鍙厬鎹㈡暟
  uint16_t sentry_pose : 2;               // bit 12-13 : 鍝ㄥ叺褰撳墠濮挎€侊細1杩涙敾锛?闃插尽锛?绉诲姩
  uint16_t energy_mechanism_status : 1;   // bit 14    : 宸辨柟鑳介噺鏈哄叧鏄惁鍙繘鍏ユ縺娲荤姸鎬侊紝1涓哄彲婵€娲?
  uint16_t reserved : 1;                  // bit 15    : 淇濈暀浣?
}__attribute__((packed)) SentryInfo_t;

// 0x020E 闆疯揪鑷富鍐崇瓥淇℃伅鍚屾
typedef struct 
{
  uint8_t double_vul_trigger_chance
    : 2;  // bits 0-1 : 闆疯揪鎷ユ湁瑙﹀彂鍙屽€嶆槗浼ょ殑鏈轰細锛?~2锛夛紝寮€灞€涓?0锛屾渶澶у彲杈?2
  uint8_t opponent_in_double_vulnerability
    : 1;  // bit 2 : 瀵规柟鏄惁姝ｅ湪琚Е鍙戝弻鍊嶆槗浼?0锛氬鏂规湭琚Е鍙戝弻鍊嶆槗浼?1锛氬鏂规鍦ㄨ瑙﹀彂鍙屽€嶆槗浼?
  uint8_t friendly_encryption_level
    : 2;  // bit 3-4锛氬繁鏂瑰姞瀵嗙瓑绾э紙鍗冲鏂瑰共鎵版尝闅惧害绛夌骇锛夛紝寮€灞€涓?1锛屾渶楂樹负 3
  uint8_t can_ket_be_modified : 1;  // bit 5锛氬綋鍓嶆槸鍚﹀彲浠ヤ慨鏀瑰瘑閽ワ紝1 涓哄彲淇敼
  uint8_t reserved : 2;             // bit 6-7锛氫繚鐣?
}__attribute__((packed)) RadarInfo_t;

// 0x0301 鏈哄櫒浜轰氦浜掓暟鎹?
typedef struct
{
  uint16_t data_cmd_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t user_data[112];  // 鏈€澶т负112
} __attribute__((packed)) RobotInteractionData_t;

// 0x0301 0x0100 閫夋墜绔垹闄ゅ浘灞?
typedef struct 
{
  uint8_t delete_type;  // 0锛氱┖鎿嶄綔 1锛氬垹闄ゅ浘灞?2锛氬垹闄ゆ墍鏈?
  uint8_t layer;        // 鍥惧眰鏁帮細0~9
}__attribute__((packed)) InteractionLayerDelete_t;

// 0x0301 0x0101 閫夋墜绔粯鍒朵竴涓浘褰?
typedef struct{
  uint8_t figure_name[3];  // 鍦ㄥ浘褰㈠垹闄ゃ€佷慨鏀圭瓑鎿嶄綔涓紝浣滀负绱㈠紩

  // bit 0-2锛氬浘褰㈡搷浣?0锛氱┖鎿嶄綔 1锛氬鍔?2锛氫慨鏀?3锛氬垹闄?
  uint32_t operate_type : 3;
  // bit 3-5锛氬浘褰㈢被鍨?0锛氱洿绾?1锛氱煩褰?2锛氭鍦?3锛氭き鍦?4锛氬渾寮?5锛氭诞鐐规暟 6锛氭暣鍨嬫暟 7锛氬瓧绗?
  uint32_t figure_type : 3;
  // bit 6-9锛氬浘灞傛暟锛?~9锛?
  uint32_t layer : 4;
  // bit 10-13锛氶鑹?0锛氱孩/钃濓紙宸辨柟棰滆壊锛?1锛氶粍鑹?2锛氱豢鑹?3锛氭鑹?4锛氱传绾㈣壊 5锛氱矇鑹?6锛氶潚鑹?7锛氶粦鑹?8锛氱櫧鑹?
  uint32_t color : 4;
  // bit 14-22锛氭牴鎹粯鍒剁殑鍥惧舰涓嶅悓锛屽惈涔変笉鍚?
  uint32_t details_a : 9;
  // bit 23-31锛氭牴鎹粯鍒剁殑鍥惧舰涓嶅悓锛屽惈涔変笉鍚?
  uint32_t details_b : 9;

  // bit 0-9锛氱嚎瀹斤紝寤鸿瀛椾綋澶у皬涓庣嚎瀹芥瘮渚嬩负 10锛?
  uint32_t width : 10;
  // bit 10-20锛氳捣鐐?鍦嗗績 x 鍧愭爣
  uint32_t start_x : 11;
  // bit 21-31锛氳捣鐐?鍦嗗績 y 鍧愭爣
  uint32_t start_y : 11;

  // bit 0-9锛氭牴鎹粯鍒剁殑鍥惧舰涓嶅悓锛屽惈涔変笉鍚?
  uint32_t details_c : 10;
  // bit 10-20锛氭牴鎹粯鍒剁殑鍥惧舰涓嶅悓锛屽惈涔変笉鍚?
  uint32_t details_d : 11;
  // bit 21-31锛氭牴鎹粯鍒剁殑鍥惧舰涓嶅悓锛屽惈涔変笉鍚?
  uint32_t details_e : 11;
}__attribute__((packed)) InteractionFigure_t;
// 0x0301 0x0102 閫夋墜绔粯鍒朵袱涓浘褰?
typedef struct {
  InteractionFigure_t interaction_figures[2];
}__attribute__((packed)) InteractionFigure2_t;

// 0x0301 0x0103 閫夋墜绔粯鍒朵簲涓浘褰?
typedef struct {
  InteractionFigure_t interaction_figures[5];
}__attribute__((packed)) InteractionFigure5_t;

// 0x0301 0x0104 閫夋墜绔粯鍒朵竷涓浘褰?
typedef struct{
  InteractionFigure_t interaction_figures[7];
} __attribute__((packed)) InteractionFigure7_t;

// 0x0301 0x0110 閫夋墜绔粯鍒跺瓧绗﹀浘褰?
typedef struct {
  InteractionFigure_t interaction_figure;
  uint8_t data[30];
}__attribute__((packed)) ExtClientCustomCharacter_t;

// 0x0301 0x0120 鍝ㄥ叺鑷富鍐崇瓥鎸囦护 TODO
typedef struct {
  uint32_t resurrect : 1;             // bit 0锛氭槸鍚︾‘璁ゅ娲?0锛氬惁 1锛氭槸
  uint32_t immediate_resurrect : 1;   // bit 1锛氭槸鍚︽秷鑰楅噾甯佸厬鎹㈢珛鍗冲娲? 0锛氬惁 1锛氭槸
  uint32_t exchange_17mm_value : 11;  // bit 2-12锛氬彂寮归噺鍏戞崲鍊硷紙閫掑鏈夋晥锛?
  uint32_t remote_exchange_17mm_count
    : 4;  // bit 13-16锛氳繙绋嬪厬鎹㈠彂寮归噺娆℃暟锛屽紑灞€涓?锛屽崟璋冮€掑锛屾瘡娆″姞1
  uint32_t remote_exchange_blood_count
    : 4;  // bit 17-20锛氳繙绋嬪厬鎹㈣閲忔鏁帮紝寮€灞€涓?锛屽崟璋冮€掑锛屾瘡娆″姞1
  uint32_t sentry_change_pose
    : 2;  // bit 21-22锛氬摠鍏典慨鏀瑰綋鍓嶅Э鎬佹寚浠わ紝1 涓鸿繘鏀诲Э鎬侊紝2 涓洪槻寰″Э鎬侊紝3 涓虹Щ鍔ㄥЭ鎬侊紝榛樿涓?3锛涗慨鏀规鍊煎嵆鍙敼鍙樺摠鍏靛Э鎬併€?
  uint32_t confirm_energy_activated
    : 1;  // bit 23锛氬摠鍏垫満鍣ㄤ汉鏄惁纭浣胯兘閲忔満鍏宠繘鍏ユ鍦ㄦ縺娲荤姸鎬侊紝1 涓虹‘璁ゃ€傞粯璁や负 0銆?
  uint32_t reserved : 8;  // bit 24-31锛氫繚鐣?
}__attribute__((packed)) SentryCmd_t;

// 0x0301 0x0121 闆疯揪鑷富鍐崇瓥鎸囦护 TODO
typedef struct{
  uint8_t radar_cmd;     // 瑙﹀彂鍙屽€嶆槗浼ゆ鏁帮紝 寮€灞€涓?锛屽崟璋冮€掑锛屾瘡娆″姞1锛屾渶澶т负2
  uint8_t password_cmd;  //瀵嗛挜鏇存柊鎴栭獙璇佹寚浠?
  uint8_t password_1;
  uint8_t password_2;
  uint8_t password_3;
  uint8_t password_4;
  uint8_t password_5;
  uint8_t password_6;
} __attribute__((packed)) RadarCmd_t;

// 0x0302 鑷畾涔夋帶鍒跺櫒涓庢満鍣ㄤ汉浜や簰鏁版嵁 鍥句紶閾捐矾
typedef struct {
  uint8_t data[30];
}__attribute__((packed)) CustomRobotData_t;

// 0x0303 閫夋墜绔皬鍦板浘浜や簰鏁版嵁
typedef struct {
  float target_position_x;  // 鐩爣浣嶇疆 x 杞村潗鏍囷紝鍗曚綅锛歮
  float target_position_y;  // 鐩爣浣嶇疆 y 杞村潗鏍囷紝鍗曚綅锛歮
  uint8_t cmd_keyboard;     // 浜戝彴鎵嬫寜涓嬬殑閿洏鎸夐敭閫氱敤閿€?
  uint8_t target_robot_id;  // 瀵规柟鏈哄櫒浜?ID
  uint16_t cmd_source;      // 淇℃伅鏉ユ簮 ID
}__attribute__((packed)) MapCommand_t;

// 0x0305 閫夋墜绔皬鍦板浘鎺ユ敹闆疯揪鏁版嵁
typedef struct{
  uint16_t hero_position_x;        // 鑻遍泟鏈哄櫒浜?x 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t hero_position_y;        // 鑻遍泟鏈哄櫒浜?y 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t engineer_position_x;    // 宸ョ▼鏈哄櫒浜?x 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t engineer_position_y;    // 宸ョ▼鏈哄櫒浜?y 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t infantry_3_position_x;  // 3鍙锋鍏垫満鍣ㄤ汉 x 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t infantry_3_position_y;  // 3鍙锋鍏垫満鍣ㄤ汉 y 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t infantry_4_position_x;  // 4鍙锋鍏垫満鍣ㄤ汉 x 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t infantry_4_position_y;  // 4鍙锋鍏垫満鍣ㄤ汉 y 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t infantry_5_position_x;  // 5鍙锋鍏垫満鍣ㄤ汉 x 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t infantry_5_position_y;  // 5鍙锋鍏垫満鍣ㄤ汉 y 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t sentry_position_x;      // 鍝ㄥ叺鏈哄櫒浜?x 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
  uint16_t sentry_position_y;      // 鍝ㄥ叺鏈哄櫒浜?y 浣嶇疆鍧愭爣锛屽崟浣嶏細cm
} __attribute__((packed)) MapRobotData_t;

// 0x0306 鑷畾涔夋帶鍒跺櫒涓庨€夋墜绔氦浜掓暟鎹?
typedef struct {
  uint16_t key_value_1 : 8;  // bit 0-7锛氭寜閿?1 閿€?
  uint16_t key_value_2 : 8;  // bit 8-15锛氭寜閿?2 閿€?
  uint16_t x_position : 12;  // bit 0-11锛氶紶鏍?X 杞村儚绱犱綅缃?
  uint16_t mouse_left : 4;   // bit 12-15锛氶紶鏍囧乏閿姸鎬?
  uint16_t y_position : 12;  // bit 0-11锛氶紶鏍?Y 杞村儚绱犱綅缃?
  uint16_t mouse_right : 4;  // bit 12-15锛氶紶鏍囧彸閿姸鎬?
  uint16_t reserved;
}__attribute__((packed)) CustomClientData_t;

// 0x0307 閫夋墜绔皬鍦板浘鎺ユ敹鍝ㄥ叺鏁版嵁
typedef struct {
  uint8_t intention;          // 1锛氬埌鐩爣鐐规敾鍑?2锛氬埌鐩爣鐐归槻瀹?3锛氱Щ鍔ㄥ埌鐩爣鐐?
  uint16_t start_position_x;  // 璺緞璧风偣 x 杞村潗鏍囷紝鍗曚綅锛歞m
  uint16_t start_position_y;  // 璺緞璧风偣 y 杞村潗鏍囷紝鍗曚綅锛歞m
  int8_t delta_x[49];         // 璺緞鐐?x 杞村閲忔暟缁勶紝鍗曚綅锛歞m
  int8_t delta_y[49];         // 璺緞鐐?y 杞村閲忔暟缁勶紝鍗曚綅锛歞m
  uint16_t sender_id;         // 鍙戦€佹柟 ID
}__attribute__((packed)) MapData_t;

// 0x0308 閫夋墜绔皬鍦板浘鎺ユ敹鏈哄櫒浜烘暟鎹?
typedef struct {
  uint16_t sender_id;     // 鍙戦€佽€呯殑 ID
  uint16_t receiver_id;   // 鎺ユ敹鑰呯殑 ID
  uint8_t user_data[30];  // 瀛楃
}__attribute__((packed)) CustomInfo_t;

// 0x0309 鑷畾涔夋帶鍒跺櫒鎺ユ敹鏈哄櫒浜烘暟鎹?鍥句紶閾捐矾
typedef struct  {
  uint8_t data[30];
}__attribute__((packed)) RobotCustomData_t;
typedef struct{
  uint8_t sof;
  uint16_t data_len;
  uint8_t seq;
  uint8_t crc8;
}__attribute__((packed)) FrameHeader_t;
typedef struct{
	uint8_t SOF;

	size_t HEAD_LEN;
	size_t CMD_ID_LEN;
	size_t TAIL_LEN;
	size_t DATA_START;
	
	GameStatus_t GameStatus;
	GameResult_t GameResult;
	GameRobotHP_t GameRobotHP;
	EventData_t EventData;
	RefereeWarning_t RefereeWarning;
	DartInfo_t DartInfo;
	RobotStatus_t RobotStatus;
	PowerHeatData_t PowerHeatData;
	RobotPos_t RobotPos;
	Buff_t Buff;
	HurtData_t HurtData;
	ShootData_t ShootData;
	ProjectileAllowance_t ProjectileAllowance;
	RFID_Status_t RFID_Status;
	DartClientCmd_t DartClientCmd;
	GroundRobotPosition_t GroundRobotPosition;
	RadarMarkData_t RadarMarkData;
	SentryInfo_t SentryInfo;
	RadarInfo_t RadarInfo;
	RobotInteractionData_t RobotInteractionData;
	InteractionLayerDelete_t InteractionLayerDelete;
	InteractionFigure_t InteractionFigure;
	InteractionFigure2_t InteractionFigure2;
	InteractionFigure5_t InteractionFigure3;
	InteractionFigure7_t InteractionFigure7;
	ExtClientCustomCharacter_t ExtClientCustomCharacter;
	SentryCmd_t  SentryCmd;
	RadarCmd_t RadarCmd;
	CustomRobotData_t CustomRobotData;
	MapCommand_t MapCommand;
	MapRobotData_t MapRobotData;
	CustomClientData_t CustomClientData;
	MapData_t MapData;
	CustomInfo_t CustomInfo;
	RobotCustomData_t RobotCustomData;
}Referee_t;

#endif
```

## FILE: Init_Ctrl/Init_Task.c

```c
#include "Init_Task.h"
WatchDog_TypeDef Gimbal_Dog[GIMBAL_SUM], Shoot_Dog[FRIC_SUM], Pluck_Dog, Down_Dog, PC_Dog,Referee_Dog,VT03_Dog;
void Init_Task(){
	taskENTER_CRITICAL();

	CanFilter_Init(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	CanFilter_Init(&hcan2);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

	remote_control_init();

	WatchDog_Init(&Remote_Dog, 20);
	WatchDog_Init(&IMU_Dog, 20);
	WatchDog_Init(&Gimbal_Dog[YAW], 10);
	WatchDog_Init(&Gimbal_Dog[PITCH], 10);
	WatchDog_Init(&Shoot_Dog[LEFT], 10); 
	WatchDog_Init(&Shoot_Dog[RIGHT], 10);
	WatchDog_Init(&Pluck_Dog, 10);
	WatchDog_Init(&Down_Dog, 50);
	WatchDog_Init(&PC_Dog, 100);
	WatchDog_Init(&Referee_Dog, 50);
	WatchDog_Init(&VT03_Dog, 20);

	HAL_TIM_Base_Start_IT(&htim3); 

	xTaskCreate((TaskFunction_t)MainCtrl_Task,    "MainCtrl_Task",     256,   NULL, 7, &MainCtrl_Task_handle);
	xTaskCreate((TaskFunction_t)Shoot_Task,		   	"Shoot_Task",        256,   NULL, 4, &Shoot_Task_handle);		
	xTaskCreate((TaskFunction_t)usb_task,         "usb_task",          256 *2,NULL, 4, &usb_task_handle);				
	xTaskCreate((TaskFunction_t)Gimbal_Task,      "Gimbal_Task",       256,   NULL, 4, &Gimbal_Task_handle);
//	xTaskCreate((TaskFunction_t)Plotter_Task,   	"Plotter_Task",			 128,   NULL, 4, &Plotter_Task_handle);
	xTaskCreate((TaskFunction_t)VT03_Task,   	"VT03_Task",			 128,   NULL, 5, &VT03_Task_handle);
	xTaskCreate((TaskFunction_t)Chassis_Task,     "Chassis_Task",      256,   NULL, 4, &Chassis_Task_handle);
	xTaskCreate((TaskFunction_t)Music_Task,       "Music_Task",        256,   NULL, 4, &Music_Task_handle);

	taskEXIT_CRITICAL();
	vTaskDelete(NULL);
}
```

## FILE: Init_Ctrl/ins_task.c

```c
/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "WatchDog.h"
#include "spi.h"
#include "bsp_dwt.h"

INS_t INS;
IMU_t IMU;
PID_t TempCtrl = {0};
WatchDog_TypeDef IMU_Dog;

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
float RefTemp = 50.0;

void INS_Init(void){
	float init_quaternion[4] = {1,0,0,0};
	IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0.98f);  
	// imu heat init
	PID_Init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0,0,0,0,0);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	INS.AccelLPF = 0.008;
}
void INS_Task(void){
	static uint32_t count = 0;
	const float gravity[3] = {0, 0, 9.7997f};
	dt = DWT_GetDeltaT(&INS_DWT_Count);
	t += dt;
	// ins update
	BMI088_Read(&BMI088);
	INS.Accel[0] = BMI088.Accel[0];
	INS.Accel[1] = BMI088.Accel[1];
	INS.Accel[2] = BMI088.Accel[2];
	INS.Gyro[0] = BMI088.Gyro[0];
	INS.Gyro[1] = BMI088.Gyro[1];
	INS.Gyro[2] = BMI088.Gyro[2];
	// 鏍稿績鍑芥暟,EKF鏇存柊鍥涘厓鏁?	IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2], INS.Accel[0], INS.Accel[1], INS.Accel[2], dt);
	memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
	// 鑾峰彇鏈€缁堟暟鎹?	INS.Yaw = QEKF_INS.Yaw;
	INS.Pitch = QEKF_INS.Pitch;
	INS.Roll = QEKF_INS.Roll;
	INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
	IMU_Rx();
	Feed_Dog(&IMU_Dog);
	if ((count % 2) == 0){			// 500hz
		IMU_Temperature_Ctrl();
	}
	count++;
}
void IMU_Rx(){
	IMU.Angle_Roll          = INS.Roll;
	IMU.Angle_Pitch         = INS.Pitch;
	IMU.Angle_Yaw           = INS.Yaw;
	IMU.Angle_Yawcontinuous = INS.YawTotalAngle;
	IMU.Gyro_Roll           = INS.Gyro[1];
	IMU.Gyro_Pitch          = INS.Gyro[0];
	IMU.Gyro_Yaw            = INS.Gyro[2];
	IMU.r = QEKF_INS.YawRoundCount;
	for(int i = 0;i < 4;i++)IMU.q[i] = INS.q[i];
}
/**
 * @brief 娓╁害鎺у埗
 * 
 */
void IMU_Temperature_Ctrl(void){
	PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);
	TIM_Set_PWM(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT16_MAX));
}
```

## FILE: Init_Ctrl/MainCtrl_Task.c

```c
#include "MainCtrl_Task.h"
#include "USB_Task.h"
#include "Shoot.h" 
#include "Task_Music.h"
void MainCtrl_Task(){
	static portTickType currentTime;
	for (;;){
    static uint8_t cnt=0;
		currentTime = xTaskGetTickCount();
		if(DeviceState.VT03_State != Device_Online){		
			osThreadSuspend(Chassis_Task_handle);
			osThreadSuspend(Gimbal_Task_handle);
			osThreadSuspend(Shoot_Task_handle);
			RemoteClear();
      for(int i = 0;i<4;i++)Key_ch[i] = 0;
			SystemState = SYSTEM_STARTING;
			GimbalInitFlag = 0;
		} else {
			osThreadResume(Chassis_Task_handle);
			osThreadResume(Shoot_Task_handle);
			osThreadResume(Gimbal_Task_handle);
		}
		if(RC_CtrlData.key.Z){
			Gimbal_action.Key = 1;
		} else if (RC_CtrlData.key.Ctrl){
			Gimbal_action.Key = 2;
		} else {
			Gimbal_action.Key = 0;		
		}
	  switch(cnt++){
      case 0:CAN_Send_StdDataFrame(&hcan2, 0x120, (uint8_t *)&Gimbal_action);			 break;
      case 1:CAN_Send_StdDataFrame(&hcan2, 0x130, (uint8_t *)&Gimbal_data);cnt = 0;break;
    }
		WatchDog_Polling();
		vTaskDelayUntil(&currentTime, 15);
	}	   
}
```

## FILE: Agency/Shoot/Shoot.c

```c
#include "Shoot.h"
#include "Time.h"
#include "Function.h"
#include "Time.h"
#include "Gimbal.h"
#include "Music.h"
#include "VT03.h"

PID Shoot_Speed_PID[FRIC_SUM] = {{.Kp = 15, .Ki = 0, .Kd = 0, .limit = 5000},
																 {.Kp = 15, .Ki = 0, .Kd = 0, .limit = 5000}};
PID_Smis Pluck_Place_PIDS = {.Kp = 30, .Ki = 0, .Kd = -0.8, .limit = 5000}; 
PID Pluck_Speed_PID = {.Kp = 10, .Ki = 0, .Kd = 0, .limit = 5000};                   
PID Pluck_Continue_PID = {.Kp = 20, .Ki = 0, .Kd = 0, .limit = 5000};               
int16_t Can1Send_Shoot[4] ={0};

struct SHOOT{	
	int16_t Ref_3508[FRIC_SUM];
	int16_t Ref_2006;
	int16_t Ref_2006_Angle;
	enum{
		SHOOT_STOP = 0,
		SHOOT_READY = 1,
		SHOOT_NORMAL = 2,
		SHOOT_RUNNING = 3,
		SHOOT_STUCKING = 4,
	}Action;
}SHOOT;
RM3508_TypeDef Shoot_Motor[FRIC_SUM];
M2006_TypeDef  Pluck_Motor;

uint8_t Add_Angle_Flag = 0;
uint8_t Shoot_One_Flag = 0;
uint8_t Lose_Angle_Flag = 0;
uint8_t	Running_Flag = 0;
float 	Angle_Target = 0;
float 	RAMP_Angle_Target = 0;
uint16_t tim = 0;
uint16_t Stuck_time = 0;
/**
 *@breif 鍙戝皠鎺у埗妯″紡
 */
void ShootCtrl_Decide(){   
	if(DeviceState.VT03_State == Device_Online){
		VT03.mode == ComInput ? Shoot_Key_Ctrl() :
		VT03.mode == RcInput ? Shoot_Rc_Ctrl() :
		Shoot_Close();
	}else Shoot_Close();
}
/**
 *@breif 閬ユ帶鍣ㄦ帶鍒? */
void Shoot_Rc_Ctrl(){
	if(GimbalCtrl != gAim){
		if(VT03.trigger == 1){
			SHOOT.Action = SHOOT_READY;
			SHOOT.Action = SHOOT_RUNNING;	
		} else if(VT03.fn_r == 1){
			SHOOT.Action = SHOOT_READY;
			SHOOT.Action = SHOOT_NORMAL;		
		} else SHOOT.Action = SHOOT_STOP;			
	}
}
/**
 *@breif 閿紶鎺у埗
 */
void Shoot_Key_Ctrl(){
	static char mouse_middle_flag = 0;
  static uint16_t normal_time = 0,shoot_tim = 0;
	if(GimbalCtrl != gAim){							
		if(VT03.mouse.left == 1){
			SHOOT.Action = SHOOT_READY;							
			shoot_tim = 0;
			SHOOT.Action = SHOOT_RUNNING;  
		}
		if(SHOOT.Action == SHOOT_RUNNING && VT03.mouse.left == 0){
			SHOOT.Action = SHOOT_READY;											   
		}
			shoot_tim ++;					
		if(SHOOT.Action != SHOOT_STUCKING && shoot_tim > 3000){
			SHOOT.Action = SHOOT_STOP;								
		}
	 } else {
		SHOOT.Action = SHOOT_READY;
		if(ReceiveVisionData.data.dis > 0.1f){
			if(ReceiveVisionData.data.FireFlag == 0){
				if(VT03.mouse.left == 1)	SHOOT.Action = SHOOT_RUNNING;
			} else {
				if(VT03.mouse.middle == 1 && mouse_middle_flag == 0){
					if(SHOOT.Action != SHOOT_NORMAL) SHOOT.Action = SHOOT_NORMAL;
					else SHOOT.Action = SHOOT_READY;
					mouse_middle_flag = 1;
				}
				if(VT03.mouse.middle == 0) mouse_middle_flag = 0;
				if(SHOOT.Action != SHOOT_NORMAL) {
					if(Referee_data_Rx.game_state == 1) SHOOT.Action = SHOOT_RUNNING;
					else SHOOT.Action = SHOOT_READY;
				}
			}
		}
	if(Gimbal.LastCtrl == gAim && GimbalCtrl != gAim) 
		SHOOT.Action = SHOOT_STOP;
	}       
}
void Shoot_Stop(){
	SHOOT.Ref_3508[LEFT]    =  0;
  SHOOT.Ref_3508[RIGHT]   =  0; 
	SHOOT.Ref_2006          =  0;
	PID_Control(Pluck_Motor.Measure.RoSpeed, SHOOT.Ref_2006, &Pluck_Speed_PID);
	PID_Control(Shoot_Motor[LEFT].Measure.RoSpeed, SHOOT.Ref_3508[LEFT], &Shoot_Speed_PID[LEFT]);
	PID_Control(Shoot_Motor[RIGHT].Measure.RoSpeed, SHOOT.Ref_3508[RIGHT], &Shoot_Speed_PID[RIGHT]);
	limit(Pluck_Speed_PID.pid_out, PLUCK_SPEED, -PLUCK_SPEED);
  limit(Shoot_Speed_PID[LEFT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
  limit(Shoot_Speed_PID[RIGHT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
	Can1Send_Shoot[0] = (int16_t)Pluck_Speed_PID.pid_out;       
	Can1Send_Shoot[2] = (int16_t)Shoot_Speed_PID[LEFT].pid_out;
	Can1Send_Shoot[1] = (int16_t)Shoot_Speed_PID[RIGHT].pid_out; 
#if SHOOT_RUN
  MotorSend(&hcan1, 0X200, Can1Send_Shoot);
#endif
}
void Shoot_Close(){
  Can1Send_Shoot[0] = 0;       
  Can1Send_Shoot[1] = 0;
	Can1Send_Shoot[2] = 0; 
#if SHOOT_RUN
	MotorSend(&hcan1, 0X200, Can1Send_Shoot);
#endif
}
void Shoot_SendDown(){
	if(DeviceState.Pluck_State != Device_Online || DeviceState.Shoot_State[LEFT] != Device_Online || DeviceState.Shoot_State[RIGHT] != Device_Online)
	 Gimbal_action.shoot_status = shoot_offline;
	else Gimbal_action.shoot_status = shoot_online;
	
	if(SHOOT.Action == SHOOT_STOP) Gimbal_action.shoot_mode = shoot_mode_stop;
	else if(SHOOT.Action == SHOOT_RUNNING && GimbalCtrl != gAim) Gimbal_action.shoot_mode = shoot_mode_fire;
	else if(GimbalCtrl == gAim) Gimbal_action.shoot_mode = shoot_mode_follow;
	
	if(SHOOT.Action == SHOOT_STUCKING)Gimbal_action.shoot_mode = shoot_mode_stucking;
} 
void ShootRef_Set(){
	SHOOT.Ref_3508[LEFT]  =  SHOOT_SPEED;
	SHOOT.Ref_3508[RIGHT] = -SHOOT_SPEED; 
	switch(SHOOT.Action){
		case SHOOT_STOP:
			Shoot_Stop();
			Time.Single             = 0;
			SHOOT.Ref_2006_Angle    = Pluck_Motor.Measure.continueMechAngle;
			break;
		case SHOOT_READY:
			SHOOT.Ref_2006          =  0;
			Add_Angle_Flag			    =  1;
			Running_Flag            =  0;
			Pluck_Motor.Measure.r   =  0;
			Time.Single             =  0;
			SHOOT.Ref_2006_Angle    = Pluck_Motor.Measure.continueMechAngle;
			break;
		case SHOOT_NORMAL:
			if(Add_Angle_Flag == 1){
				if(Shoot_One_Flag == 0){
					SHOOT.Ref_2006_Angle    = Pluck_Motor.Measure.continueMechAngle + PLUCK_MOTOR_ONE;
					Add_Angle_Flag = 0;
					Shoot_One_Flag = 1;
				}
			}
			break;
		case SHOOT_RUNNING:
			if(Referee_data_Rx.game_state == 1){
				ShootHeat_Limit();
			} else {
				SHOOT.Ref_2006 = -PLUCK_SPEED;
			}			break;
		case SHOOT_STUCKING:
			SHOOT.Ref_2006 = PLUCK_SPEED;
		break;
	}
}
void Shoot_Console(){
	if(SHOOT.Action == SHOOT_NORMAL){
		if(Shoot_One_Flag == 1){
			PID_Control_Smis(Pluck_Motor.Measure.continueMechAngle,SHOOT.Ref_2006_Angle,&Pluck_Place_PIDS,Pluck_Motor.Measure.RoSpeed);
			SHOOT.Ref_2006 = Pluck_Place_PIDS.pid_out;
			Shoot_One_Flag = 0;
		}
	 }
	PID_Control(Pluck_Motor.Measure.RoSpeed, SHOOT.Ref_2006, &Pluck_Speed_PID);
	PID_Control(Shoot_Motor[LEFT].Measure.RoSpeed, SHOOT.Ref_3508[LEFT], &Shoot_Speed_PID [LEFT]);
	PID_Control(Shoot_Motor[RIGHT].Measure.RoSpeed, SHOOT.Ref_3508[RIGHT], &Shoot_Speed_PID [RIGHT]);
	limit(Pluck_Speed_PID.pid_out, M2006_LIMIT, -M2006_LIMIT);
	limit(Shoot_Speed_PID[LEFT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
	limit(Shoot_Speed_PID[RIGHT].pid_out, RM3508_LIMIT, -RM3508_LIMIT);
}
void Shoot_Send(){
	Can1Send_Shoot[0] = (int16_t)Pluck_Speed_PID.pid_out;       
	Can1Send_Shoot[2] = (int16_t)Shoot_Speed_PID[LEFT].pid_out;
	Can1Send_Shoot[1] = (int16_t)Shoot_Speed_PID[RIGHT].pid_out; 
#if SHOOT_RUN
	if(SHOOT.Action != SHOOT_STOP)	MotorSend(&hcan1, 0X200, Can1Send_Shoot);
#endif
}
void Detect_Shoot(){
	if(DeviceState.Pluck_State != Device_Online || DeviceState.Shoot_State[LEFT] != Device_Online || DeviceState.Shoot_State[RIGHT] != Device_Online)
		SHOOT.Action = SHOOT_STOP;
	if(SHOOT.Action != SHOOT_STOP){
		if(SHOOT.Action == SHOOT_RUNNING){
			if(ABS( Pluck_Motor.Measure.RoSpeed ) <= 20)
				Stuck_time++;
		}
		if(Stuck_time >= 50){
			SHOOT.Action = SHOOT_STUCKING;
			Stuck_time++;
		}
		if( Stuck_time > 100){
			Stuck_time = 0;
			SHOOT.Action = SHOOT_RUNNING;
		}
	}
}
/**
 * @brief 璁＄畻鐢垫満杞€熶笌寮归鐨勫叧绯? * @param target_freq 鐩爣寮归 (Hz)锛岄粯璁?20Hz
 * @return uint32_t 璁＄畻鍚庣殑杞€熼檺鍒跺€?(RPM) * 
 * @note 杞€熸渶澶т负5400RPM鏃讹紝寮归鍗冲彲杈惧埌20Hz reduction_ratio 鍑忛€熸瘮 caliper 鎷ㄩ娇鏁?shoot_speed 寮归
 * @see SHOOT.Ref[2006] / 36 / 60 * 8 <= 20  杞€?= 鏈熸湜寮归 * 鍑忛€熸瘮 * 60s / 鎷ㄩ娇鏁? * @warning 杈撳叆鍙傛暟瓒呭嚭鑼冨洿鍙兘瀵艰嚧鐢垫満澶辨帶
 */
float cooling,heat_now,heat_limit,Consumption,shoot_speed,K = 2;
uint16_t ShootTime,shoot_time;
uint8_t reduction_ratio = 36.0,caliper = 8.0;
void ShootHeat_Limit(){
	heat_limit = Referee_data_Rx.heat_limit;
	cooling = Referee_data_Rx.heat_cooling;
	heat_now = Referee_data_Rx.heat_now;
	Consumption = 10.0f;//娑堣€?	if(heat_limit - heat_now > 100){
		shoot_speed = 20.0f;
	} else if (100 > (heat_limit - heat_now) && (heat_limit - heat_now) > 50){
		shoot_speed = (10.0f * heat_limit + 10.0f * cooling * shoot_time / 1000.0f - cooling) / (10.0f * Consumption * shoot_time / 1000.0f);	
	} else {
		shoot_speed =  cooling / Consumption;	
	}
	if( (GimbalCtrl == gAim && ReceiveVisionData.data.FireFlag == 1) || SHOOT.Action == SHOOT_RUNNING){
		shoot_time ++ ;	
	} else {
		shoot_time -- ;
	}
	SHOOT.Ref_2006 = shoot_speed * reduction_ratio * 60.0f / caliper ;//杞€?= 鏈熸湜寮归 * 鍑忛€熸瘮 * 60s / 鎷ㄩ娇鏁?}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if (hcan->Instance == CAN1){
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader[0],CAN1_buff);
    switch (RxHeader[0].StdId){
			case 0x201: RMMotor_Receive(&Pluck_Motor.Measure, CAN1_buff);
									Feed_Dog(&Pluck_Dog);
									break;
			case 0x203: RMMotor_Receive(&Shoot_Motor[LEFT].Measure, CAN1_buff); 
									Feed_Dog(&Shoot_Dog[LEFT]);
									break;   
			case 0x202: RMMotor_Receive(&Shoot_Motor[RIGHT].Measure, CAN1_buff);
									Feed_Dog(&Shoot_Dog[RIGHT]);
									break; 
			default:    break;
		}
  }
}
```

## FILE: Agency/Shoot/Shoot.h

```c
#ifndef __SHOOT_H
#define __SHOOT_H

#include "Variate.h"
#include "USB_Task.h"

extern void Shoot_Stop();
extern void Shoot_Rc_Ctrl();    //!< @brief 鍙戝皠鏈烘瀯閬ユ帶鍣ㄦā寮?extern void Shoot_Key_Ctrl();   //!< @brief 鍙戝皠鏈烘瀯閿紶妯″紡
extern void Shoot_Drive();     //!< @brief 鍙戝皠鏈烘瀯鐢垫満椹卞姩
extern void Shoot_Close();
/* 妫€娴嬪彂灏勬満鏋?*/
extern void Detect_Shoot();
/* 鍙戝竷鍙戝皠鏈烘瀯 */
extern void ShootPublish();
/* 鏇存柊鐘舵€侀噺态*/
extern void ShootData_Update();
/* 鍐冲畾鎺у埗鏂瑰紡 */
extern void ShootCtrl_Decide();
/* 澶勭悊寮傚父 */
extern void ShootHandleEception();
/* 璁剧疆鐩爣閲?*/
extern void ShootRef_Set();
/* 鏋彛鐑噺闄愬埗 */
extern void ShootHeat_Limit();
/* 璁＄畻鎺у埗閲?*/
extern void Shoot_Console();
/* 鍙戦€佹帶鍒堕噺 */
extern void Shoot_Send();
extern void Shoot_SendDown();
/* 鑷瀯 */
extern void Aim_Shoot();
#endif
```

## FILE: Agency/Shoot/Shoot_Task.c

```c
#include "Shoot_Task.h"

void Shoot_Task(void *pvParameters){
	static portTickType currentTime;	 
	for(;;){
		currentTime = xTaskGetTickCount(); 
		ShootCtrl_Decide();
		Detect_Shoot();
		ShootRef_Set();
		Shoot_Console();
		Shoot_Send();
		Shoot_SendDown();		
		vTaskDelayUntil (&currentTime,1);
	}
}
```

## FILE: Agency/Gimbal/Gimbal.c

```c
#include "Gimbal.h"
#include "Time.h"
#include "USB_Task.h"
#include "dm_motor.h"
#include "VT03.h"

eGimbal Gimbal;
eGimbalCtrl GimbalCtrl;
int16_t Can2Send[4] = {0};

PID_TypeDef Gimbal_Speed_pid_Yaw[GIMBAL_MODE];
PID_TypeDef Gimbal_Place_pid_Yaw[GIMBAL_MODE];

PID_TypeDef Gimbal_Place_pid_Pitch[GIMBAL_MODE];
PID_TypeDef Gimbal_Speed_pid_Pitch[GIMBAL_MODE];

void GimbalInit(){
	GimbalCtrl = gNormal;
	GimbalInitFlag = 1;
	Time.GimbalInit = 0;

	PID_init(&Gimbal_Place_pid_Yaw[INIT],25000,0,0,0.5f,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[INIT],25000,0,0,0.1f,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Yaw[GYRO],25000,1,0,1.5f,0,50.0f,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[GYRO],25000,1,0,0.8f,0.002f,0,0,0);	
	PID_init(&Gimbal_Place_pid_Yaw[AIM],25000,0,0,3.20f,0,3.15f,0,0);	
	PID_init(&Gimbal_Speed_pid_Yaw[AIM],25000,0,0,1.0f,0.0008f,0,0,0);	

	PID_init(&Gimbal_Place_pid_Pitch[INIT],25000,0,0,0.0,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_Pitch[INIT],25000,0,0,0.0,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Pitch[GYRO],20000,1,0,3.0,0,20,0,0);	
	PID_init(&Gimbal_Speed_pid_Pitch[GYRO],20000,1,0,-1.0f,0,0,0,0);	
	PID_init(&Gimbal_Place_pid_Pitch[AIM],25000,0,0,2.25,0,0,0,0);	
	PID_init(&Gimbal_Speed_pid_Pitch[AIM],25000,0,0,-0.9,0,0,0,0);	
}
/*鎺у埗鏂瑰紡鍐冲畾鍑芥暟*/
void GimbalCtrl_Decide(){
	if(DeviceState.VT03_State == Device_Online){
		VT03.mode == ComInput ? Gimbal_Key_Ctrl() :
		VT03.mode == RcInput ? Gimbal_RC_Ctrl() :
		Gimbal_Stop();
	} else Gimbal_Stop();
}
void Gimbal_RC_Ctrl(){
	static char Rc_FNl_flag = 0;
	if(VT03.fn_l == 1 && Rc_FNl_flag == 0 ){
		if(GimbalCtrl != gAim) GimbalCtrl = gAim;
		else GimbalCtrl = gNormal;
		Rc_FNl_flag = 1;
	}
	if(VT03.fn_l == 0) Rc_FNl_flag = 0;
//	switch (RC_CtrlData.rc.s1){
//		case 1:
////			GimbalCtrl = gNormal;
//			GimbalCtrl = gAim;
//			break;
//		case 3:
//			GimbalCtrl = gNormal;
//			break;
//		case 2:
//			GimbalCtrl = gNormal;
//		break;
//    }	
}	
void Gimbal_Key_Ctrl(){
	static char Key_Q_flag = 0,Key_F_flag = 0;
	static char mouse_r_flag = 0;
	if(GimbalCtrl != gAim){
		if(RC_CtrlData.key.F == 1 && Key_F_flag == 0){
			Gimbal.Ref[YAW] += 180;
			Key_F_flag = 1;
		}
		if(RC_CtrlData.key.F == 0)
			Key_F_flag = 0;
			GimbalCtrl = gNormal;
	}
	if(RC_CtrlData.mouse.press_r == 1 && mouse_r_flag == 0 ){
		if(GimbalCtrl != gAim) GimbalCtrl = gAim;
		else GimbalCtrl = gNormal;
		mouse_r_flag = 1;
	}
	if(RC_CtrlData.mouse.press_r == 0) mouse_r_flag = 0;		
}
/**
 *@brief 浜戝彴鎬ュ仠
 */
void Gimbal_Stop(){	
	Gimbal.increase[YAW]   = 0;
	Gimbal.increase[PITCH] = 0;
  Gimbal.Ref[PITCH] = IMU.Angle_Pitch;
	Gimbal.Ref[YAW] = IMU.Angle_Yawcontinuous;

	Can2Send[0] = 0;
	Can2Send[1] = 0;
	MotorSend(&hcan2,0x3FE,Can2Send);
}
/**
 *@brief 浜戝彴鏈熸湜鍊兼洿鏂? */
void GimbalRef_Update(){
	if(GimbalCtrl == gAim){
		float yaw_diff = 0;
		yaw_diff = ReceiveVisionData.data.Ref_Yaw - IMU.Angle_Yaw;
		if(yaw_diff > 180.0f){
			yaw_diff -= 360.0f;
		} else if (yaw_diff < -180.0f){
			yaw_diff += 360.0f;
		}
		IMU.VisionAngle = IMU.Angle_Yawcontinuous + yaw_diff;
	}
switch(GimbalCtrl){
	case gNormal:
		if(VT03.mode == ComInput){
			Gimbal.increase[YAW]    = Mouse_ch[0] * 0.1f;
			Gimbal.increase[PITCH]  = -Mouse_ch[1] * 0.1f;
		} else if (VT03.mode == RcInput){
			Gimbal.increase[YAW]   = VT03.ch_lx * 0.3f;
			Gimbal.increase[PITCH] = -VT03.ch_ly * 0.2f;
		}                                                
		Gimbal.Ref[PITCH] -= Gimbal.increase[PITCH];
		Gimbal.Ref[YAW] -= Gimbal.increase[YAW];			
		limit(Gimbal.Ref[PITCH],P_ADD_limit,P_LOSE_limit);		 
		break;
	case gAim:
		if(DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
			Gimbal.increase[YAW]   = 0;
			Gimbal.increase[PITCH] = 0;
			Gimbal.Ref[YAW] = IMU.VisionAngle;
			Gimbal.Ref[PITCH] = ReceiveVisionData.data.Ref_Pitch;
		} else {
			if(VT03.mode == ComInput){
				Gimbal.increase[YAW]   = Mouse_ch[0] * 0.25;
				Gimbal.increase[PITCH] = Mouse_ch[1] * 0.01;
			} else if (VT03.mode == RcInput){				
				Gimbal.increase[YAW]   = VT03.ch_lx * 0.3f;
				Gimbal.increase[PITCH] = -VT03.ch_ly * 0.01f;
			}
			Gimbal.Ref[PITCH] -= Gimbal.increase[PITCH];
			Gimbal.Ref[YAW] -= Gimbal.increase[YAW];
		}
		Gimbal.LastCtrl = gAim;
		break;
	default :
		Gimbal.Ref[PITCH] = IMU.Angle_Pitch;
		Gimbal.Ref[YAW]   = IMU.Angle_Yawcontinuous;
		break;
  }
}
float GimbalPitchOffset = 0.0f;
float FF_Yaw = 0.0f, K_FF = 0.0f;
void Gimbal_Calc(){
	limit(ReceiveVisionData.data.Ref_Pitch,P_ADD_limit,P_LOSE_limit);
	FF_Yaw = Chassis_data_Rx.Chassis_Speed * K_FF;
	if(GimbalCtrl == gAim && DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
		PID_Calc(&Gimbal_Place_pid_Pitch[AIM],IMU.Angle_Pitch,ReceiveVisionData.data.Ref_Pitch);
		PID_Calc(&Gimbal_Speed_pid_Pitch[AIM],IMU.Gyro_Pitch,Gimbal_Place_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_Vpitch);		
				
		PID_Calc(&Gimbal_Place_pid_Yaw[AIM],IMU.Angle_Yawcontinuous,Gimbal.Ref[YAW]);
		PID_Calc(&Gimbal_Speed_pid_Yaw[AIM],IMU.Gyro_Yaw,Gimbal_Place_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_Vyaw + FF_Yaw);		
	} else {
		PID_Calc(&Gimbal_Place_pid_Pitch[GYRO],IMU.Angle_Pitch,Gimbal.Ref[PITCH]);
		PID_Calc(&Gimbal_Speed_pid_Pitch[GYRO],IMU.Gyro_Pitch,Gimbal_Place_pid_Pitch[GYRO].Output);		
		
		PID_Calc(&Gimbal_Place_pid_Yaw[GYRO],IMU.Angle_Yawcontinuous,Gimbal.Ref[YAW]);
		PID_Calc(&Gimbal_Speed_pid_Yaw[GYRO],IMU.Gyro_Yaw,Gimbal_Place_pid_Yaw[GYRO].Output);		
	}
}
void Gimbal_Send(){
	if(GimbalCtrl == gAim && DeviceState.PC_State == 1 && ReceiveVisionData.data.dis > 0.1f){
		Can2Send[0] = (int16_t)(Gimbal_Speed_pid_Pitch[AIM].Output + ReceiveVisionData.data.Ref_aPitch + GimbalPitchOffset);	
		Can2Send[1] = (int16_t)(Gimbal_Speed_pid_Yaw[AIM].Output + ReceiveVisionData.data.Ref_aYaw);	
	} else {
		Can2Send[0] = (int16_t)(Gimbal_Speed_pid_Pitch[GYRO].Output);	
		Can2Send[1] = (int16_t)(Gimbal_Speed_pid_Yaw[GYRO].Output);	
	}
#if GIMBAL_RUN
	MotorSend(&hcan2,0x3FE,Can2Send);
#endif
	if(DeviceState.Gimbal_State[PITCH] == Device_Online) Gimbal_action.Gimbal_status.Pitch = Gimbal_online;
		else Gimbal_action.Gimbal_status.Pitch 	= 	Gimbal_offline;
	if( DeviceState.Gimbal_State[YAW]  == Device_Online)  Gimbal_action.Gimbal_status.Yaw  = Gimbal_online;
		else Gimbal_action.Gimbal_status.Yaw 	= 	Gimbal_offline; 
}

void MedianInit(){
	static float Expect_PitchInit = 0;
  static float Expect_YawInit = 0;
  uint16_t Expect_PitchRamp = GimPitch.MchanicalAngle;
  uint16_t Expect_YawRamp   = GimYaw.MchanicalAngle;
	/* 鑾峰緱褰掍腑浣嶇疆 */
	if(Time.GimbalInit < 100){
#if   Yaw_Mid_Right < Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) && (GimYaw.MchanicalAngle >= Yaw_Mid_Right) )
#elif Yaw_Mid_Right > Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) || (GimYaw.MchanicalAngle >= Yaw_Mid_Right) )
#endif
		MidMode = FRONT; else MidMode = BACK;
	} else {
		if (MidMode == FRONT) Expect_YawInit = QuickCentering( GimYaw.MchanicalAngle, Yaw_Mid_Front );
		else Expect_YawInit = QuickCentering( GimYaw.MchanicalAngle, Yaw_Mid_Back );
	}
	/* Yaw come mid */
	Expect_YawRamp = RAMP_float(Expect_YawInit,Expect_YawRamp,200); 
	PID_Calc(&Gimbal_Place_pid_Yaw[INIT],GimYaw.MchanicalAngle,Expect_YawRamp);
	PID_Calc(&Gimbal_Speed_pid_Yaw[INIT],GimYaw.Speed,Gimbal_Place_pid_Yaw[INIT].Output);
  limit( Gimbal_Speed_pid_Yaw[INIT].Output,GM6020_LIMIT,-GM6020_LIMIT);
	/* Pitch come mid*/	
	Expect_PitchInit = QuickCentering(GimPitch.MchanicalAngle,Pitch_Mid);
	Expect_PitchRamp = RAMP_float(Pitch_Mid,Expect_PitchRamp,50); 
	PID_Calc(&Gimbal_Place_pid_Pitch[INIT],GimPitch.MchanicalAngle,Expect_PitchRamp);
	PID_Calc(&Gimbal_Speed_pid_Pitch[INIT],GimPitch.Speed,Gimbal_Place_pid_Pitch[INIT].Output);
	limit( Gimbal_Speed_pid_Pitch[INIT].Output,GM6020_LIMIT,-GM6020_LIMIT );

	Can2Send[0] = (int16_t)Gimbal_Place_pid_Pitch[INIT].Output;
	Can2Send[1] = (int16_t)Gimbal_Speed_pid_Yaw[INIT].Output;
	limit(Can2Send[GIMBAL_SUM],GM6020_LIMIT,-GM6020_LIMIT);	
#if GIMBAL_RUN
	MotorSend(&hcan2,0x3FE,Can2Send);
#endif	
	if(Time.GimbalInit >= 1000){
		Time.GimbalInit = 0;
		GimbalInitFlag  = 0;
		Gimbal.YawInit   = Expect_YawInit;
		Gimbal.PitchInit = Expect_PitchInit;

		Gimbal.increase[PITCH] = 0;
		Gimbal.increase[YAW]   = 0;
		Gimbal.Ref[YAW] = IMU.Angle_Yawcontinuous;
		Gimbal.Ref[PITCH] = IMU.Angle_Pitch;
		
		SystemState = SYSTEM_RUNNING;
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if (hcan->Instance == CAN2) {
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&RxHeader[1],CAN2_Rxbuff);
    switch (RxHeader[1].StdId){
			case 0x101:
						Referee_data_Rx.game_state = CAN2_Rxbuff[0];
						Referee_data_Rx.robot_color = CAN2_Rxbuff[1];
						Referee_data_Rx.heat_limit = (uint16_t)(CAN2_Rxbuff[2] << 8 | CAN2_Rxbuff[3]);
						Referee_data_Rx.heat_cooling = (uint16_t)(CAN2_Rxbuff[4] << 8 | CAN2_Rxbuff[5]);
						Referee_data_Rx.heat_now = (uint16_t)(CAN2_Rxbuff[6] << 8 | CAN2_Rxbuff[7]);
                    Feed_Dog(&Referee_Dog);
				break;
			case 0x102:
						memcpy(&Chassis_data_Rx.Chassis_Speed,&CAN2_Rxbuff[0],sizeof(float));
						memcpy(&Chassis_data_Rx.bullet_speed,&CAN2_Rxbuff[4],sizeof(float));
										Feed_Dog(&Down_Dog);
			case 0x301 : DM4310_Receive(&GimPitch,CAN2_Rxbuff);
					Feed_Dog(&Gimbal_Dog[PITCH]);
				break;
			case 0x302 : DM4310_Receive(&GimYaw,CAN2_Rxbuff);
					Feed_Dog(&Gimbal_Dog[YAW]);
				break;
      default:	break;
    }
  }
}
```

## FILE: Agency/Gimbal/Gimbal.h

```c
#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "Variate.h"
#include "Function.h"
typedef enum{
    INIT = 0,
    GYRO = 1,
    AIM = 2,
    GIMBAL_MODE = 3
}eGimbalPidMode;
extern eGimbalPidMode GimbalPidMode;
typedef enum {
	gNormal    = 0,
	gAim       = 1,
	gFllow     = 2,
	gTest      = 3
}eGimbalCtrl;
extern eGimbalCtrl GimbalCtrl;

typedef struct 
{
	float Pitch;
	float Roll;
	float Yaw;
	int16_t r;
	float Last;
	float ContinuousYaw;
}eAngle;

typedef struct{
	int LastCtrl;
	float Ref[GIMBAL_SUM];
	float YawInit,PitchInit;
	float increase[GIMBAL_SUM];
}eGimbal;
extern eGimbal Gimbal;
extern PID_TypeDef Gimbal_Speed_pid_Yaw[GIMBAL_MODE], Gimbal_Place_pid_Yaw[GIMBAL_MODE];																						

extern void GimbalInit();//浜戝彴鍒濆鍖?extern void MedianInit();//褰掍腑
extern void Gimbal_RC_Ctrl();//閬ユ帶鍣ㄦ帶鍒?extern void Gimbal_Key_Ctrl();//閿紶鎺у埗
extern void Gimbal_Stop();//鎬ュ仠

extern void GimbalCtrl_Decide();//鍐冲畾鎺у埗鏂瑰紡
extern void GimbalRef_Update();//鏇存柊鏈熸湜鍊?extern void GimbalReal_Update();//鏇存柊褰撳墠鍊?extern void Detect_Gimbal();
extern void Gimbal_Calc();
extern void Gimbal_Send();
extern void Gimbal_SendDown();

extern float Kff_v,Kff_a;

#endif
```

## FILE: Agency/Gimbal/Gimbal_Task.c

```c
#include "Gimbal_Task.h"

void Gimbal_Task(){
	static portTickType currentTime;
	for(;;){
		currentTime = xTaskGetTickCount();			 
		if(SystemState != SYSTEM_RUNNING){
			if(GimbalInitFlag == 0) GimbalInit();
			MedianInit();
#if !GIMBAL_RUN
		SystemState = SYSTEM_RUNNING;
#endif				
		}else{
			GimbalCtrl_Decide();
			GimbalRef_Update();
			Gimbal_Calc();	
			Gimbal_Send();				
		}
      vTaskDelayUntil(&currentTime, 1);		 
		}
}
```

## FILE: RM_Lib/Src/Chassis.c

```c
#include "Chassis.h"
#include "arm_math.h"
#include "rng.h"
#include "dm_motor.h"
#include "Gimbal.h"
#include "VT03.h"

PID_TypeDef Chassis_Place_pid_Rotate;
PID_TypeDef Chassis_Speed_pid_Rotate;
FeedForward_Typedef Chassis_FF = {.K1 = 1000.00, .OutMax = RM3508_LIMIT};        //鍓嶉
uint16_t Mid_Left,Mid_Right,Mid_Back,Mid_Front;

uint8_t RecodeAngle = 0; //璁板綍闄€铻轰华瑙掑害
uint16_t Angle_rotate_ref;
static	Gimbal_board_send_t send_data;
struct{
enum{
ChassisStop = 0,
ChassisFollow = 1,
ChassisNormal = 2,
ChassisGyroscope = 3,
ChassisCheck = 4,//妫€褰曟鍙嶈浆
Chassis_Rise_Mid = 5,
Chassis_Rise_High = 6
}Action;
int16_t MidAngle;
}CHASSIS;

/**
*@brief up to under 
*/
Communication_Speed_t Communication_Speed_Tx;
/**
*@brief chassis init
*/
void ChassisInit(){
	   Mid_Front = Yaw_Mid_Front;
		 PID_init(&Chassis_Place_pid_Rotate,8000,5,0, 1,0,0,0,0.001);
		 PID_init(&Chassis_Speed_pid_Rotate,8000,5,0, 2,0,0,0,0.001);
     CHASSIS.Action = ChassisNormal;
}
/**
*@brief chassis ctrl decicde
*/
void ChassisCtrl_Decide(){
	if(DeviceState.VT03_State == Device_Online){
		VT03.mode == ComInput ? Chassis_Key_Ctrl() :
		VT03.mode == RcInput ? Chassis_RC_Ctrl() :
		Chassis_Stop();
	}else Chassis_Close();
}
/**
*@brief chassis rc ctrl
*/
void Chassis_RC_Ctrl(){
	Communication_Speed_Tx.Close_flag = 0;			
	CHASSIS.Action = ChassisFollow;
}
static char Key_F_flag = 0;
/**&
*@brief WSAD 鍓嶅悗宸﹀彸
*@brief r 灏忛檧铻?*@brief F 杞悜180
*/
uint8_t LegFlag = 0;
void Chassis_Key_Ctrl(){
	static char Key_R_flag = 0,Key_Ctrl_flag = 0,Key_Q_flag = 0;
	static uint16_t tim;
	tim ++;

	if (RC_CtrlData.key.W)
			VT03.ch_ry = 1;
	else if (RC_CtrlData.key.S)
			VT03.ch_ry = -1;
	else
			VT03.ch_ry = 0;

	if (RC_CtrlData.key.A)
			VT03.ch_rx = -1;
	else if (RC_CtrlData.key.D)
			VT03.ch_rx = 1;
	else
			VT03.ch_rx = 0;
	
	if(CHASSIS.Action == ChassisFollow || CHASSIS.Action == Chassis_Rise_High || CHASSIS.Action == Chassis_Rise_Mid){
		if(RC_CtrlData.key.Q){
			osDelay(2);
			Key_Q_flag = 1;
			if(CHASSIS.Action == ChassisFollow){
				CHASSIS.Action = Chassis_Rise_Mid;
				Key_Q_flag = 1;
			} else if(CHASSIS.Action == Chassis_Rise_Mid){
				CHASSIS.Action = Chassis_Rise_High;
				Key_Q_flag = 1;
			} else if(CHASSIS.Action == Chassis_Rise_High){
				CHASSIS.Action = Chassis_Rise_Mid;
				Key_Q_flag = 1;
			}	
		}
	}
	if (RC_CtrlData.key.G == 1){
			CHASSIS.Action = ChassisFollow;                                             
			Key_Q_flag = 0;
	}
	Communication_Speed_Tx.Close_flag = 0;			
	if(NormalModeFlag != 0 && GyroscopeModeFlag != 1 && Key_Q_flag != 1){
		CHASSIS.Action = ChassisNormal;
	}else if(CHASSIS.Action != ChassisGyroscope && Key_Q_flag != 1){
		CHASSIS.Action = ChassisFollow;		
	}
	if (RC_CtrlData.key.R == 1 && Key_R_flag == 0){
		if (CHASSIS.Action != ChassisGyroscope){
			CHASSIS.Action = ChassisGyroscope;
			GyroscopeModeFlag = 1;
		}else{
			CHASSIS.Action = ChassisFollow;
			GyroscopeModeFlag = 0;
		}
		Key_R_flag = 1;
	}
	if (RC_CtrlData.key.R == 0)
			Key_R_flag = 0;
	
	if (RC_CtrlData.key.F == 1 && Key_F_flag == 0){
			Key_F_flag = 1;
			tim = 0;
	} else if(Key_F_flag == 1 && tim > 500){
		if (RC_CtrlData.key.F == 0 && Key_F_flag == 1){
			Key_F_flag = 0;
		}
	}
	if(GimbalCtrl == gAim){
		if(RC_CtrlData.key.Ctrl == 1) CHASSIS.Action = ChassisFollow;
		else CHASSIS.Action = ChassisNormal;
	}
}
void Chassis_Stop(){
	CHASSIS.Action = ChassisStop;
}
void ChassisRef_Update(){
	static uint16_t Follow_Speed_MAX = 2000;		 
	static uint16_t Speed,tim;		 
	static uint16_t Ramp_rotate_ref;	

	switch(CHASSIS.Action){
		case ChassisFollow:
			Chassis_FF.Now_DeltIn = VT03.ch_lx + Mouse_ch[0] * 0.8;
#if   Yaw_Mid_Right < Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) && (GimYaw.MchanicalAngle >= Yaw_Mid_Right) ){
#elif Yaw_Mid_Right > Yaw_Mid_Left
        if ( (GimYaw.MchanicalAngle <= Yaw_Mid_Left) || (GimYaw.MchanicalAngle >= Yaw_Mid_Right) ){
#endif
        CHASSIS.MidAngle = Yaw_Mid_Front; 
        MidMode = FRONT;
    } else {
        CHASSIS.MidAngle = Yaw_Mid_Back;
        MidMode = BACK;
    }
        PID_Calc(&Chassis_Place_pid_Rotate,GimYaw.MchanicalAngle,QuickCentering(GimYaw.MchanicalAngle, CHASSIS.MidAngle));
				PID_Calc(&Chassis_Speed_pid_Rotate,GimYaw.Speed,Chassis_Place_pid_Rotate.Output);
        Chassis_Speed_pid_Rotate.Output -= Chassis_Place_pid_Rotate.Output;//鍓嶉
				Communication_Speed_Tx.Chassis_Speed.rotate_ref = Chassis_Speed_pid_Rotate.Output + FeedForward_Calc(&Chassis_FF);
				limit(Communication_Speed_Tx.Chassis_Speed.rotate_ref, Follow_Speed_MAX, -Follow_Speed_MAX);
	      if(Key_F_flag) Communication_Speed_Tx.Chassis_Speed.rotate_ref = 0;
			break;
				
		case ChassisNormal:
			Ramp_rotate_ref   = 0;
			Communication_Speed_Tx.Chassis_Speed.rotate_ref   = RAMP_float(Ramp_rotate_ref,Communication_Speed_Tx.Chassis_Speed.rotate_ref,15) ;
		break;	

		case ChassisGyroscope:
			if(VT03.ch_rx == 0 && VT03.ch_ry == 0 &&RC_CtrlData.key.Shift == 0){
				Ramp_rotate_ref = 4000;
				Communication_Speed_Tx.Chassis_Speed.rotate_ref   = RAMP_float(Ramp_rotate_ref,Communication_Speed_Tx.Chassis_Speed.rotate_ref,15) ;	
			}			
		break;	

		case ChassisStop:
			Communication_Speed_Tx.Chassis_Speed.rotate_ref       = 0;
			Communication_Speed_Tx.Chassis_Speed.forward_back_ref = 0;		
			Communication_Speed_Tx.Chassis_Speed.left_right_ref   = 0;
			Communication_Speed_Tx.Close_flag = 1;				
		break;
				
		case ChassisCheck: break;
		case Chassis_Rise_Mid:
			if(RC_CtrlData.key.Ctrl){
				PID_Calc(&Chassis_Place_pid_Rotate,GimYaw.MchanicalAngle,QuickCentering(GimYaw.MchanicalAngle, CHASSIS.MidAngle));
				PID_Calc(&Chassis_Speed_pid_Rotate,GimYaw.Speed,Chassis_Place_pid_Rotate.Output);
				Chassis_Speed_pid_Rotate.Output -= Chassis_Place_pid_Rotate.Output;//鍓嶉
				Communication_Speed_Tx.Chassis_Speed.rotate_ref = Chassis_Speed_pid_Rotate.Output + FeedForward_Calc(&Chassis_FF);
				limit(Communication_Speed_Tx.Chassis_Speed.rotate_ref, Follow_Speed_MAX, -Follow_Speed_MAX);				
			}
		break;
		case Chassis_Rise_High :
			if(RC_CtrlData.key.Ctrl){
				PID_Calc(&Chassis_Place_pid_Rotate,GimYaw.MchanicalAngle,QuickCentering(GimYaw.MchanicalAngle, CHASSIS.MidAngle));
				PID_Calc(&Chassis_Speed_pid_Rotate,GimYaw.Speed,Chassis_Place_pid_Rotate.Output);
				Chassis_Speed_pid_Rotate.Output -= Chassis_Place_pid_Rotate.Output;//鍓嶉
				Communication_Speed_Tx.Chassis_Speed.rotate_ref = Chassis_Speed_pid_Rotate.Output + FeedForward_Calc(&Chassis_FF);
				limit(Communication_Speed_Tx.Chassis_Speed.rotate_ref, Follow_Speed_MAX, -Follow_Speed_MAX);				
			}
break;
	}
}
uint16_t R_C,L_C;
/*搴曠洏琛ュ伩璁＄畻*/
void Chassis_Offset(){
	static float Level_Gain, chassis_offset;
	static int16_t forward_back_ref = 0, left_right_ref = 0,rotate_ref = 0;
	static int16_t Speed_Gain;
	 static float Ramp_forward_back_ref,Ramp_left_right_ref,Ramp_rotate_ref;	

	chassis_offset = (GimYaw.MchanicalAngle - Yaw_Mid_Front) / 1303.64f;//搴曠洏琛ュ伩瑙?	Gimbal_data.Offset_Angle = chassis_offset * 1000;
	
	R_C = Yaw_Mid_Right;
	L_C = Yaw_Mid_Left;
	/* 鎸変綇Shift鍔犻€?*/
	if(RC_CtrlData.key.Shift){
		Speed_Gain = 6000; 
	} else Speed_Gain = 4000; 

	if(CHASSIS.Action != ChassisCheck){
		 if(MidMode == FRONT){
			forward_back_ref = -VT03.ch_ry * Speed_Gain;
			left_right_ref   = -VT03.ch_rx * Speed_Gain * 0.7;
		 }else if(MidMode == BACK){
			forward_back_ref = VT03.ch_ry * Speed_Gain;
			left_right_ref   = VT03.ch_rx * Speed_Gain * 0.7;
		 }	
	}
    if(CHASSIS.Action == ChassisCheck){
			left_right_ref = 0;
			Communication_Speed_Tx.Chassis_Speed.rotate_ref   = VT03.ch_rx * 4000;
		}
		if(CHASSIS.Action == ChassisGyroscope &&(VT03.ch_rx || VT03.ch_ry || RC_CtrlData.key.Shift == 1)){
			rotate_ref = 4000 * 0.6;
			Ramp_rotate_ref = RAMP_float(rotate_ref,Ramp_rotate_ref,Speed_Gain/750.0); 
			Communication_Speed_Tx.Chassis_Speed.rotate_ref = RAMP_float(rotate_ref,Communication_Speed_Tx.Chassis_Speed.rotate_ref,Speed_Gain/750.0); 
		}
		                                  	
    /* 搴曠洏琛ュ伩璁＄畻 灏忛檧铻虹Щ鍔?鍙湁YAW杞村湪绾挎墠鍙В绠?*/
    if(DeviceState.Gimbal_State[YAW] == Device_Online && CHASSIS.Action == ChassisGyroscope){
        Communication_Speed_Tx.Chassis_Speed.forward_back_ref = forward_back_ref * arm_sin_f32( -chassis_offset)
                                                                + left_right_ref * arm_cos_f32( -chassis_offset);
        Communication_Speed_Tx.Chassis_Speed.left_right_ref   = forward_back_ref * arm_cos_f32( chassis_offset) 
                                                                + left_right_ref * arm_sin_f32( chassis_offset);
    } else {
        Communication_Speed_Tx.Chassis_Speed.forward_back_ref =  forward_back_ref;
        Communication_Speed_Tx.Chassis_Speed.left_right_ref   =  left_right_ref;
    }
}
void Chassis_Close(){
	Communication_Speed_Tx.Chassis_Speed.rotate_ref       = 0;
	Communication_Speed_Tx.Chassis_Speed.forward_back_ref = 0;
	Communication_Speed_Tx.Chassis_Speed.left_right_ref   = 0;
	Communication_Speed_Tx.Close_flag = 1;

	send_data.vx         = Communication_Speed_Tx.Chassis_Speed.forward_back_ref;
	send_data.vy         = Communication_Speed_Tx.Chassis_Speed.left_right_ref;
	send_data.rotate     = Communication_Speed_Tx.Chassis_Speed.rotate_ref;
	send_data.Close_flag = Communication_Speed_Tx.Close_flag;
	send_data.Shift_flag = 0;
#if CHASSIS_RUN
    CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&send_data);
#endif
}
void ChassisDown_Send(){
	send_data.vx         = Communication_Speed_Tx.Chassis_Speed.forward_back_ref;
	send_data.vy         = Communication_Speed_Tx.Chassis_Speed.left_right_ref;
	send_data.rotate     = Communication_Speed_Tx.Chassis_Speed.rotate_ref;
	send_data.Close_flag = Communication_Speed_Tx.Close_flag;
	send_data.Shift_flag = 0;
	switch(CHASSIS.Action) {
		case ChassisStop: Gimbal_action.move_status = stop; break;
		case ChassisNormal: Gimbal_action.move_status = normal; break;
		case ChassisGyroscope: Gimbal_action.move_status = rotate; break;
		case Chassis_Rise_Mid:Gimbal_action.move_status = rise_mid; break;
		case Chassis_Rise_High:Gimbal_action.move_status = rise_high; break;
		case ChassisFollow:Gimbal_action.move_status = follow; break;	
		default: Gimbal_action.move_status = stop; break;
	}   
#if CHASSIS_RUN
    CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&send_data);	
#endif
}
void ChassisMotorSpeed_clean(ChassisSpeed_Ref_t *ref) {
    ref->forward_back_ref = 0;
    ref->left_right_ref = 0;
    ref->rotate_ref = 0;
}
__weak void ChassisMotorSpeed_get(Chassis_Motor_Speed *motor, ChassisSpeed_Ref_t *ref) {
    motor->speed_3 = -ref->forward_back_ref -
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_2 = ref->forward_back_ref -
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_1 = ref->forward_back_ref +
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_4 = -ref->forward_back_ref +
                     ref->left_right_ref + ref->rotate_ref;
}
```

## FILE: RM_Lib/Inc/Chassis.h

```c
#pragma once
#include "Variate.h"
#include "RMLibHead.h"

RMLIB_CPP_BEGIN
extern void ChassisInit();
extern void ChassisCtrl_Decide();
extern void ChassisRef_Update();
extern void Chassis_Offset();
extern void ChassisDown_Send();
extern void Chassis_RC_Ctrl();
extern void Chassis_Key_Ctrl();
extern void Chassis_Stop();
extern void Chassis_Close();
extern void ChassisDown_Send();

enum Chassis_Status{
	Chassis_Disable = 0,
	Chassis_Enable,
};

/**
 * @brief 鐭㈤噺閫熷害缁撴瀯浣? */
typedef struct {
  int16_t forward_back_ref;  //!<@brief 鍓嶈繘閫熷害
  int16_t left_right_ref;    //!<@brief 宸﹀彸閫熷害
	int16_t rotate_ref;        //!<@brief 鏃嬭浆閫熷害
} ChassisSpeed_Ref_t;

typedef struct{
	ChassisSpeed_Ref_t Chassis_Speed; //!< @brief 鏈熸湜閫熷害
	uint8_t Close_flag;				  		//!< @brief 鍚姩鏍囧織浣?	uint8_t Shift_flag;        		//!< @brief Shift鍔犻€?} Communication_Speed_t;
// up to under 缁撴瀯浣?typedef struct  {
	int16_t vx;            // X 鏂瑰悜閫熷害锛堥€氬父涓哄綊涓€/鏍囧畾鍚庣殑閫熷害閲忥級
	int16_t vy;            // Y 鏂瑰悜閫熷害锛堥€氬父涓哄綊涓€/鏍囧畾鍚庣殑閫熷害閲忥級
	int16_t rotate;        // 鏃嬭浆閫熷害锛堣閫熷害锛屽崟浣嶆寜绯荤粺绾﹀畾锛氬 deg/s 鎴?rpm 鏄犲皠鍊硷級
	uint8_t Close_flag;    // 搴曠洏鍏抽棴鏍囧織锛?=鍏抽棴/鍋滄锛?	uint8_t Shift_flag;    // Shift 鍔熻兘閿爣蹇楋紙1=鎸変笅锛?} Gimbal_board_send_t;
/**
 * @brief 浜戝彴瑙掑害缁撴瀯浣? */
typedef struct {
    float Pitch;               //!< @brief Pitch
    float Yaw;                 //!< @brief Yaw
} PTZAngle_Ref_t;

/**
 * @brief 杞粍閫熷害缁撴瀯浣? */
typedef struct {
	int16_t speed_1;    //!< @brief 鐢垫満1閫熷害
	int16_t speed_2;    //!< @brief 鐢垫満2閫熷害
	int16_t speed_3;    //!< @brief 鐢垫満3閫熷害
	int16_t speed_4;    //!< @brief 鐢垫満4閫熷害
} Chassis_Motor_Speed;

/**
 * @brief 娓呴浂棰勬湡閫熷害
 * @param[out] ref 鐭㈤噺閫熷害缁撴瀯浣? */
void ChassisMotorSpeed_clean(ChassisSpeed_Ref_t *ref);
/**
 * @brief 閫熷害鐭㈤噺璁＄畻
 * @param[out] motor 杞粍閫熷害缁撴瀯浣? * @param[in] ref 鐭㈤噺閫熷害缁撴瀯浣? */
void ChassisMotorSpeed_get(Chassis_Motor_Speed *motor, ChassisSpeed_Ref_t *ref);

RMLIB_CPP_END
```

## FILE: Agency/Chassis/Chassis_Task.c

```c
#include "Chassis_Task.h"

void Chassis_Task(){
		static portTickType currentTime;
		ChassisInit();
		for(;;){
		currentTime = xTaskGetTickCount();
		ChassisCtrl_Decide();
		ChassisRef_Update();
		Chassis_Offset();
		ChassisDown_Send();
		vTaskDelayUntil(&currentTime, 1);		 
	   }
}
```

## FILE: Agency/Serialport/plotter.cpp

```cpp
#include "plotter.h"

namespace at
{
Plotter::Plotter(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma), hal_status_(HAL_OK)
{
}

void Plotter::plot(float value1)
{
  static_assert(PLOTTER_FLOAT_NUM >= 1);
  plot_frame_.size = 4 * 1;
  plot_frame_.data[0] = value1;
  send();
}

void Plotter::plot(float value1, float value2)
{
  static_assert(PLOTTER_FLOAT_NUM >= 2);
  plot_frame_.size = 4 * 2;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  send();
}

void Plotter::plot(float value1, float value2, float value3)
{
  static_assert(PLOTTER_FLOAT_NUM >= 3);
  plot_frame_.size = 4 * 3;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  send();
}

void Plotter::plot(float value1, float value2, float value3, float value4)
{
  static_assert(PLOTTER_FLOAT_NUM >= 4);
  plot_frame_.size = 4 * 4;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  send();
}

void Plotter::plot(float value1, float value2, float value3, float value4, float value5)
{
  static_assert(PLOTTER_FLOAT_NUM >= 5);
  plot_frame_.size = 4 * 5;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6)
{
  static_assert(PLOTTER_FLOAT_NUM >= 6);
  plot_frame_.size = 4 * 6;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7)
{
  static_assert(PLOTTER_FLOAT_NUM >= 7);
  plot_frame_.size = 4 * 7;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8)
{
  static_assert(PLOTTER_FLOAT_NUM >= 8);
  plot_frame_.size = 4 * 8;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9)
{
  static_assert(PLOTTER_FLOAT_NUM >= 9);
  plot_frame_.size = 4 * 9;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10)
{
  static_assert(PLOTTER_FLOAT_NUM >= 10);
  plot_frame_.size = 4 * 10;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11)
{
  static_assert(PLOTTER_FLOAT_NUM >= 11);
  plot_frame_.size = 4 * 11;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  send();
}

void Plotter::plot(
  float value1, float value2, float value3, float value4, float value5, float value6, float value7,
  float value8, float value9, float value10, float value11, float value12)
{
  static_assert(PLOTTER_FLOAT_NUM >= 12);
  plot_frame_.size = 4 * 12;
  plot_frame_.data[0] = value1;
  plot_frame_.data[1] = value2;
  plot_frame_.data[2] = value3;
  plot_frame_.data[3] = value4;
  plot_frame_.data[4] = value5;
  plot_frame_.data[5] = value6;
  plot_frame_.data[6] = value7;
  plot_frame_.data[7] = value8;
  plot_frame_.data[8] = value9;
  plot_frame_.data[9] = value10;
  plot_frame_.data[10] = value11;
  plot_frame_.data[11] = value12;
  send();
}
void Plotter::send()
{
  if (use_dma_) {
    hal_status_ = HAL_UART_Transmit_DMA(
      huart_, (uint8_t *)&plot_frame_,
      sizeof(plot_frame_.start) + sizeof(plot_frame_.size) + plot_frame_.size);
  }
  else {
    hal_status_ = HAL_UART_Transmit(
      huart_, (uint8_t *)&plot_frame_,
      sizeof(plot_frame_.start) + sizeof(plot_frame_.size) + plot_frame_.size, 0xff);
  }
}

}  // namespace at
```

## FILE: Agency/Serialport/plotter_task.cpp

```cpp
#include "cmsis_os.h"
#include "plotter.h"
#include "ins_task.h"
#include "Gimbal.h"
#include "dm_motor.h"

at::Plotter plotter(&huart1);

extern "C" void Plotter_Task(){
	while(true){
		plotter.plot(IMU.Angle_Pitch,Gimbal.Ref[PITCH]);
		osDelay(10);
	}
}
```

## FILE: Agency/WatchDog/Callback_Function.c

```c
#include "Callback_Function.h"
#include "WatchDog.h"
#include "dm_motor.h"
/* 鍠傜嫍鍥炶皟鍑芥暟 */
void FeedDog_CallBack(WatchDogp handle){
  /*
  ID
  1    WatchDog_Init(&Remote_Dog,30);
  2    WatchDog_Init(&IMU_Dog,15);
  3    WatchDog_Init(&Gimbal_Dog[YAW],10);
  4    WatchDog_Init(&Gimbal_Dog[PITCH],10);
  5    WatchDog_Init(&Shoot_Dog[Left],10);
  6    WatchDog_Init(&Shoot_Dog[Right],10);
  7    WatchDog_Init(&Pluck_Dog,10);
  8    WatchDog_Init(&Down_Dog,15);
  9    WatchDog_Init(&PC_Dog,50);
	10   WatchDog_Init(&Referee_Dog, 50);

  */
  switch (handle->ID)
  {

      case 1:
            if (REMOTE_IfDataError() == osError){
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,15000);
              DeviceState.Remote_State = Device_Error;
            } else {
              DeviceState.Remote_State = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
            }
        break;

      case 2:
            if (IMU_IfDataError() == osError){
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,15000);
              DeviceState.IMU_State = Device_Error;
            } else {
              DeviceState.IMU_State = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0);
            }
        break;

      case 3:
              DeviceState.Gimbal_State[YAW] = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,0);
        break;

      case 4:
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,0);
              DeviceState.Gimbal_State[PITCH] = Device_Online;
        break;

      case 5:
           if (RM3508_Motor_Temp(&Shoot_Motor[LEFT]) == osError){
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,15000);
              DeviceState.Shoot_State[LEFT] = Device_Error;
            } else {
              DeviceState.Shoot_State[LEFT] = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,0);
            }
        break;

      case 6:
            if (RM3508_Motor_Temp(&Shoot_Motor[RIGHT]) == osError){
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,15000);
              DeviceState.Shoot_State[RIGHT] = Device_Error;
            } else {
              DeviceState.Shoot_State[RIGHT] = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,0);
            }
        break;

      case 7:
          DeviceState.Pluck_State = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,0);
        break;

      case 8:
            DeviceState.Down_State = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,0);
        break;

      case 9:
            DeviceState.PC_State = Device_Online;
        break;
			case 10:
				    DeviceState.Referee_State = Device_Online;
				break;
			case 11:
						DeviceState.VT03_State = Device_Online;
				break;
  }
}

/* 楗挎鍥炶皟鍑芥暟 */
void WatchDog_CallBack(WatchDogp handle){
	if(IS_Dog(handle,Remote_Dog)){
		DeviceState.Remote_State = Device_Offline;
	}
	if(IS_Dog(handle,IMU_Dog)){
		DeviceState.IMU_State = Device_Offline;
	}
	if(IS_Dog(handle,Gimbal_Dog[PITCH])){
		DM_Motor_Ctrl(&hcan2,0x01,0,3);
		osDelay(10);
    DeviceState.Gimbal_State[PITCH] = Device_Offline;
	}	
	if(IS_Dog(handle,Gimbal_Dog[YAW])){
		DM_Motor_Ctrl(&hcan2,0x02,0,3);
		osDelay(10);
    DeviceState.Gimbal_State[YAW] = Device_Offline;
	}
	if(IS_Dog(handle,Shoot_Dog[LEFT])){
		DeviceState.Shoot_State[LEFT] = Device_Offline;
	}
	if(IS_Dog(handle,Shoot_Dog[RIGHT])){
		DeviceState.Shoot_State[RIGHT] = Device_Offline;
	}
	if(IS_Dog(handle,Pluck_Dog)){
		DeviceState.Pluck_State = Device_Offline;
	}
	if(IS_Dog(handle,Down_Dog)){
		DeviceState.Down_State  = Device_Offline;
	}
	if(IS_Dog(handle,PC_Dog)){
		DeviceState.PC_State = Device_Offline;
	}
	if(IS_Dog(handle,Referee_Dog)){
		DeviceState.Referee_State = Device_Offline;
	}
	if(IS_Dog(handle,VT03_Dog)){
		DeviceState.VT03_State = Device_Offline;
	}
}
```

## FILE: Agency/WatchDog/Task_Music.c

```c
#include "Task_Music.h"

#include "bsp_buzzer.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "music.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t music_high_water;
#endif

#define STEP_INIT 1
#define STEP_NORMAL 2

#define is_play_cali()                                                                 \
    (is_play == CALI_BEGIN || is_play == CALI_MIDDLE_TIME || is_play == CALI_GIMBAL || \
     is_play == CALI_IMU || is_play == CALI_CHASSIS)
	 
#define cali_buzzer_begin() buzzer_on(50, 10000)   // 铚傞福鍣ㄧ殑璁剧疆棰戠巼鍜屽己搴?
#define cali_buzzer_middle() buzzer_on(20, 10000)  // 铚傞福鍣ㄧ殑璁剧疆棰戠巼鍜屽己搴?
#define cali_buzzer_gimbal() buzzer_on(30, 19999)  // 褰撲簯鍙板湪鏍″噯,铚傞福鍣ㄧ殑璁剧疆棰戠巼鍜屽己搴?
#define cali_buzzer_imu() buzzer_on(60, 19999)  // 褰搃mu鍦ㄦ牎鍑?铚傞福鍣ㄧ殑璁剧疆棰戠巼鍜屽己搴?
#define cali_buzzer_chassis() buzzer_on(100, 19999)  // 褰撳簳鐩樺湪鏍″噯,铚傞福鍣ㄧ殑璁剧疆棰戠巼鍜屽己搴?
#define cali_buzzer_off() buzzer_off()               // buzzer off锛屽叧闂渹楦ｅ櫒



extern uint32_t play_id;       // Index of the note to be played
void Music_Task(void const * pvParameters)
{
	static int16_t tim = 0;
    // 绌洪棽涓€娈垫椂闂?
    vTaskDelay(MUSIC_TASK_INIT_TIME);
    // 鍒濆鍖栭煶涔?
    MusicStartInit();
     static portTickType currentTime;
	   for(;;){
			currentTime = xTaskGetTickCount();		
      tim++;			 
      MusicStartPlay();
			if(tim >= 900){
      buzzer_off();
			osThreadSuspend(Music_Task_handle);			 
			}
			vTaskDelayUntil(&currentTime, 1);		 

			}
}
```

## FILE: Agency/WatchDog/Music.c

```c
#include "Music.h"
#include "stm32f4xx_hal.h"
#include "bsp_buzzer.h"
//瀹氫箟浣庨煶  
#define A1  131
#define A2  147
#define A3  165
#define A4  175
#define A5  196
#define A6  220
#define A7  247
  
//瀹氫箟涓煶  
#define B1  262
#define B2  296
#define B3  330
#define B4  349
#define B5  392
#define B6  440
#define B7  494
  
//瀹氫箟楂橀煶  
#define C1  523
#define C2  587
#define C3  659
#define C4  698
#define C4p 741
#define C5  784
#define C6  880
#define C7  988
  
//瀹氫箟楂樹簩搴? 
#define D1  1047
#define D2  1175
#define D3  1319
#define D4  1397
#define D5  1568
#define D6  1760
#define D7  1976

//瀹氫箟鑺傛媿  
#define OneBeat   200//涓€鎷嶅瓙涓や釜1beat 
#define HalfBeat  100

#define NOTE_NUM 10
static Note Notes[NOTE_NUM];  // Array of notes

static uint32_t last_note_id = 0;  // Index of the last note
static uint32_t write_id = 1;      // Index of the note to be written
uint32_t play_id = 1;       // Index of the note to be played

static uint32_t start_time = 0;  // Start time of the music
static uint32_t now = 0;

         /* Music_Start */
static void WriteNote(int note, float Long)
{
    Notes[write_id].note = note;
    Notes[write_id].Long = Long;
    Notes[write_id].end = Notes[write_id - 1].end + Long;
    write_id++;
}
/**
 * @brief 鎾斁闊充箰
 * @param  none
 * @return 缁撴潫1 鏈粨鏉?
 */
bool MusicStartPlay(void ){
	now = HAL_GetTick();
	bool end = false;
	if(now - start_time >= Notes[play_id].end){
	play_id++;
		if(play_id > last_note_id){
			end = true;
			play_id = 1;
			start_time = now; 
		}
		buzzer_note(Notes[play_id].note,0.1);
	}
	return end;
}
void MusicStartInit(void){
	WriteNote(0,2);
	WriteNote(B1,HalfBeat*2);
	WriteNote(0,3);
	WriteNote(C3,HalfBeat*2);
	WriteNote(0,3);
	WriteNote(D5,HalfBeat*2);
	WriteNote(0,5);
	WriteNote(D1,HalfBeat*5);
	
	last_note_id = write_id -1;
	write_id = 1;
}
```

## FILE: Agency/USB/USB_Task.c

```c
#include "usb_task.h"

static SendDataImu_s SEND_DATA_IMU = {.header.sof = 0x5A,
																			.header.len = (uint8_t)(sizeof(SendDataImu_s) - 6),
																			.header.id  = 0x01,
																			.eof = 0xA5};
ReceiveVisionData_t ReceiveVisionData = {.header.sof = 0x5A,
																				 .header.id  = 0X02,
																				 .eof = 0xA5,
																				 .data.dis = -1};
ReceiveVisionData_t last_ReceiveVisionData = {.header.sof = 0x5A,
																				 .header.id  = 0X02,
																				 .eof = 0xA5,
																				 .data.dis = -1};

static void UsbInit(void);
static void UsbReceiveData(void);
static void UsbSendImuData(void);

void usb_task(void *pvParameters){
	UsbInit();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
			UsbSendImuData();
			UsbReceiveData();
			vTaskDelayUntil(&xLastWakeTime,1);
	}
}
static void UsbInit(void){
	memset(&SEND_DATA_IMU.data,0,sizeof(SEND_DATA_IMU.data));
	memset(&ReceiveVisionData.data,0,sizeof(ReceiveVisionData.data));
}

static void UsbSendImuData(void){
	SEND_DATA_IMU.time_stamp = HAL_GetTick();//鑾峰彇褰撳墠鏃堕棿鎴?
//	SEND_DATA_IMU.data.bullet_speed = SHOOT_SPEED * 0.10472f * 0.03;//m/s 733.04鏄痳pm杞寲涓虹嚎閫熷害 v=wr;
	SEND_DATA_IMU.data.bullet_speed = Chassis_data_Rx.bullet_speed;
	SEND_DATA_IMU.data.pitch = 	IMU.Angle_Pitch * Pi / 180.0f;//rad
	SEND_DATA_IMU.data.yaw 	 = 	IMU.Angle_Yaw * Pi / 180.0f;
	SEND_DATA_IMU.data.roll  =	IMU.Angle_Roll * Pi / 180.0f;
	SEND_DATA_IMU.data.pitch_vel = IMU.Gyro_Pitch;
	SEND_DATA_IMU.data.yaw_vel 	 = IMU.Gyro_Yaw;  
	SEND_DATA_IMU.data.roll_vel  = IMU.Gyro_Roll; 
	if(Referee_data_Rx.robot_color == 1){
		SEND_DATA_IMU.data.self_color = 0;
	}else{
		SEND_DATA_IMU.data.self_color = 1; 
	}
  USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}
static void UsbReceiveData(void) {
	static uint8_t data_buffer[64] = {0};
  uint32_t actual_len = 0;
  USB_Receive(data_buffer, &actual_len);
	last_ReceiveVisionData = ReceiveVisionData;
	memcpy(&ReceiveVisionData, data_buffer, sizeof(ReceiveVisionData_t));
	if(memcmp(&ReceiveVisionData,&last_ReceiveVisionData,sizeof(ReceiveVisionData_t))!=0){
		Feed_Dog(&PC_Dog);	
	} else {
//		memset(&data_buffer,0,sizeof(data_buffer));
//		memset(&ReceiveVisionData.data,0,sizeof(ReceiveVisionData.data));
	}
}
```


