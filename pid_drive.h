#ifndef _PID_DRIVE_H_
#define _PID_DRIVE_H_

#include <board.h>

/* PID 内部统一使用 Q10 定点数，寄存器值进入 PID 前乘以 Q_SCALE。 */
#define Q_SCALE 1024L

/* 功率内环的最终输出是 PWM 百分比，100% = 100 * Q_SCALE。 */
#define PID_PERCENT_MAX (100L * Q_SCALE)

typedef struct {
    int32_t Kp;             /* 比例增益，Q10 */
    int32_t Ki;             /* 积分增益，Q10 */
    int32_t Kd;             /* 微分增益，Q10 */
    int32_t setpoint;       /* 目标值，Q10。温度 PID 为温度，功率 PID 为真实功率 */
    int32_t integral;       /* 积分累计值 */
    int32_t prev_error;     /* 上一次误差 */
    int32_t prev_prev_error;/* 上上次误差，增量式 PID 使用 */
    int32_t prev_output;    /* 上一次输出，Q10 */
    int32_t prev_measured;  /* 上一次测量值，用于一阶滤波 */
    int32_t output_min;     /* 输出下限，Q10 */
    int32_t output_max;     /* 输出上限，Q10 */
    int32_t integral_limit; /* 积分限幅，防止积分饱和 */
    int32_t max_rise;       /* 输出最大上升率，Q10/秒 */
    uint8_t filter_coeff;   /* 测量滤波系数，0=不滤波，越大越平滑 */
    uint8_t measured_valid; /* prev_measured 是否已经有效 */
    uint8_t is_incremental; /* 0=位置式，1=增量式 */
} PID_Handle_t;

extern PID_Handle_t xdata g_temp_pid;
extern PID_Handle_t xdata g_power_pid;

void PID_Init(PID_Handle_t xdata *pid, int32_t Kp, int32_t Ki, int32_t Kd,
              int32_t out_min, int32_t out_max, int32_t integral_limit, uint8_t is_inc);
void PID_Reset(PID_Handle_t xdata *pid);
int32_t PID_Calc(PID_Handle_t xdata *pid, int32_t measurement, uint16_t dt);

/* PWM、温度 ADC、外部 UART ADC 初始化。 */
void PID_Hardware_Init(void);

/* 从 4x 保持寄存器重新装载 PID 参数和输出限幅。 */
void PID_LoadParams(void);

/* 刷新 3x 输入寄存器：原始值、校准后实际值、真实功率。 */
void Modbus_Input_Reg_Update(void);

/* duty 范围 0~1000，对应 0.0%~100.0% PWM。 */
void Set_PWM_Duty(uint16_t duty);

/* 以下接口返回值均已乘 Q_SCALE，用于 PID 计算。 */
int32_t Get_Temperature(void);
int32_t Get_Power(void);
int32_t Get_Target_Temp(void);
int32_t Get_Power_Setpoint(void);

/* 40027 总功率限制，单位与 30003 一致；0 表示不限制。 */
uint16_t Get_Power_Limit_Value(void);
uint16_t PID_Get_Temp_Sample_Time(void);
uint16_t PID_Get_Power_Sample_Time(void);

#endif
