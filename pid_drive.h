#ifndef _PID_DRIVE_H_
#define _PID_DRIVE_H_
#include <board.h>
// 定点数缩放因子：Q10，放大1024倍，精度约0.001
#define Q_SCALE 1024UL
// PID句柄结构体（纯整数版本）
typedef struct {
    int32_t Kp;                  // 比例增益（Q10格式）
    int32_t Ki;                  // 积分增益（Q10格式）
    int32_t Kd;                  // 微分增益（Q10格式）
    int32_t setpoint;            // 设定值（Q10格式）
    int32_t integral;            // 积分项（Q10格式）
    int32_t prev_error;          // 上一次误差（Q10格式）
    int32_t prev_output;         // 上一次输出（Q10格式）
    int32_t output_min;          // 输出最小值（Q10格式）
    int32_t output_max;          // 输出最大值（Q10格式）
    int32_t integral_limit;      // 积分限幅（Q10格式）
    int32_t max_rise;            // 最大上升率 %/s（Q10格式）
    uint8_t filter_coeff;        // 滤波系数 0~255
    int32_t prev_measured;       // 上一次测量值（Q10格式，用于滤波）
    uint8_t is_incremental;      // 是否增量式
} PID_Handle_t;
// 全局PID实例
extern PID_Handle_t xdata g_temp_pid;
extern PID_Handle_t xdata g_power_pid;
// 函数声明
void PID_Init(PID_Handle_t xdata *pid, int32_t Kp, int32_t Ki, int32_t Kd,
              int32_t out_min, int32_t out_max, int32_t integral_limit, uint8_t is_inc);
void PID_Reset(PID_Handle_t xdata *pid);
int32_t PID_Calc(PID_Handle_t xdata *pid, int32_t measurement, uint16_t dt);
void PID_Hardware_Init(void);
void PID_LoadParams(void);
void Modbus_Input_Reg_Update(void);
// PWM占空比设置（duty: 0~1000 对应 0.0%~100.0%）
void Set_PWM_Duty(uint16_t duty);
// 数据获取（Q10格式）
int32_t Get_Temperature(void);
int32_t Get_Power(void);
int32_t Get_Target_Temp(void);
int32_t Get_Power_Setpoint(void);
#endif
