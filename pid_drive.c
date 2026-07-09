#include "pid_drive.h"
#include "adc_sensor.h"
#include "uart_adc.h"
#include "modbus_reg_config.h"
#include "modbus.h"
#ifndef PWMA_CR1
#define PWMA_CR1    (*(xdata volatile unsigned char *)0xFEC0)
#define PWMA_CCMR1  (*(xdata volatile unsigned char *)0xFEC8)
#define PWMA_CCER1  (*(xdata volatile unsigned char *)0xFECC)
#define PWMA_PSCRH  (*(xdata volatile unsigned char *)0xFED0)
#define PWMA_PSCRL  (*(xdata volatile unsigned char *)0xFED1)
#define PWMA_ARRH   (*(xdata volatile unsigned char *)0xFED2)
#define PWMA_ARRL   (*(xdata volatile unsigned char *)0xFED3)
#define PWMA_CCR1H  (*(xdata volatile unsigned char *)0xFED5)
#define PWMA_CCR1L  (*(xdata volatile unsigned char *)0xFED6)
#endif
#ifndef IR_TEMP_OFFSET
#define IR_TEMP_OFFSET REG_EXT_ADC1_OFFSET
#endif
#ifndef IR_REAL_POWER_OFFSET
#define IR_REAL_POWER_OFFSET REG_REAL_POWER_OFFSET
#endif
// 定点数缩放因子 Q_SCALE 已在 pid_drive.h 中定义
#define Q_MUL(a,b) (((int32_t)(a) * (int32_t)(b)) / Q_SCALE)
#define Q_DIV(a,b) (((int32_t)(a) * Q_SCALE) / (b))
/*============================================================================
 * 全局变量定义
 *============================================================================*/
// PID 实例
PID_Handle_t xdata g_temp_pid;
PID_Handle_t xdata g_power_pid;
// 系统状态（用于业务逻辑）
static uint16_t xdata g_pwm_duty = 0;      // 当前 PWM 占空比 (0~1000 对应 0~100%)
/*============================================================================
 * PID 算法实现（纯整数定点版本，无浮点）
 *============================================================================*/
// 初始化 PID
void PID_Init(PID_Handle_t xdata *pid, int32_t Kp, int32_t Ki, int32_t Kd,
              int32_t out_min, int32_t out_max, int32_t integral_limit, uint8_t is_inc)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_limit = integral_limit;
    pid->is_incremental = is_inc;
    pid->max_rise = 10 * Q_SCALE; // 默认最大上升率10%/s
    pid->filter_coeff = 0; // 默认无滤波
    PID_Reset(pid);
}
// 重置 PID 内部状态
void PID_Reset(PID_Handle_t xdata *pid)
{
    pid->setpoint = 0;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_output = 0;
    pid->prev_measured = 0;
}
// PID 计算（位置式 + 增量式混合，带滤波和上升率限制，纯整数运算）
// dt: 调用间隔时间（单位ms，默认100ms）
int32_t PID_Calc(PID_Handle_t xdata *pid, int32_t measurement, uint16_t dt)
{
    int32_t error = pid->setpoint - measurement;
    int32_t output = 0;
    // 一阶低通滤波（整数版本）
    if (pid->filter_coeff > 0) {
        measurement = (measurement * (256 - pid->filter_coeff) + pid->prev_measured * pid->filter_coeff) / 256;
    }
    pid->prev_measured = measurement;
    if (pid->is_incremental)
    {
        // ========== 增量式 PID ==========
        int32_t p_term;
        int32_t i_term;
        int32_t d_term;
        int32_t delta_max;
        int32_t delta;
        p_term = Q_MUL(pid->Kp, (error - pid->prev_error));
        i_term = Q_MUL(pid->Ki, error) * dt / 1000;
        d_term = Q_MUL(pid->Kd, (error - pid->prev_error)) * 1000 / dt;
        output = pid->prev_output + p_term + i_term + d_term;
        // 输出限幅
        if (output > pid->output_max)
            output = pid->output_max;
        if (output < pid->output_min)
            output = pid->output_min;
        // 上升率限制
        delta_max = pid->max_rise * dt / 1000;
        delta = output - pid->prev_output;
        if (delta > delta_max) output = pid->prev_output + delta_max;
        else if (delta < -delta_max) output = pid->prev_output - delta_max;
        // 更新状态
        pid->prev_output = output;
        pid->prev_error = error;
    }
    else
    {
        // ========== 位置式 PID ==========
        int32_t derivative;
        int32_t delta_max;
        int32_t delta;
        // 积分累加（带积分限幅）
        pid->integral += Q_MUL(error, dt) / 1000;
        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;
        // 微分
        derivative = Q_MUL((error - pid->prev_error), 1000) / dt;
        // 计算输出
        output = Q_MUL(pid->Kp, error) + Q_MUL(pid->Ki, pid->integral) + Q_MUL(pid->Kd, derivative);
        // 输出限幅
        if (output > pid->output_max)
            output = pid->output_max;
        else if (output < pid->output_min)
            output = pid->output_min;
        // 上升率限制
        delta_max = pid->max_rise * dt / 1000;
        delta = output - pid->prev_output;
        if (delta > delta_max) output = pid->prev_output + delta_max;
        else if (delta < -delta_max) output = pid->prev_output - delta_max;
        // 更新状态
        pid->prev_output = output;
        pid->prev_error = error;
    }
    return output;
}
/*============================================================================
 * 从保持寄存器加载PID参数
 *============================================================================*/
void PID_LoadParams(void)
{
    // ---------------- 加载温度PID参数 ----------------
    // 增益参数（放大100倍存储，转为Q10格式）
    uint16_t kp_reg;
    uint16_t ki_reg;
    uint16_t kd_reg;
    uint16_t max_rise_reg;
    kp_reg = g_holding_regs[HLD_TEMP_PID_PROP_GAIN_OFFSET];
    ki_reg = g_holding_regs[HLD_TEMP_PID_INT_GAIN_OFFSET];
    kd_reg = g_holding_regs[HLD_TEMP_PID_DER_GAIN_OFFSET];
    if (kp_reg == 0 || kp_reg > 10000) kp_reg = 100; // 默认1.0
    if (ki_reg > 10000) ki_reg = 0;
    if (kd_reg > 10000) kd_reg = 0;
    g_temp_pid.Kp = (int32_t)kp_reg * 1024 / 100; // 转为Q10
    g_temp_pid.Ki = (int32_t)ki_reg * 1024 / 100;
    g_temp_pid.Kd = (int32_t)kd_reg * 1024 / 100;
    // 最大上升率（放大10倍存储，单位%/s，转为Q10）
    max_rise_reg = g_holding_regs[HLD_TEMP_PID_MAX_RISE_OFFSET];
    if (max_rise_reg == 0 || max_rise_reg > 1000) max_rise_reg = 100; // 默认10%/s
    g_temp_pid.max_rise = (int32_t)max_rise_reg * 1024 / 10;
    // 滤波系数
    g_temp_pid.filter_coeff = (uint8_t)(g_holding_regs[HLD_TEMP_PID_FILTER_OFFSET] & 0xFF);
    // ---------------- 加载功率PID参数 ----------------
    kp_reg = g_holding_regs[HLD_POWER_PID_PROP_GAIN_OFFSET];
    ki_reg = g_holding_regs[HLD_POWER_PID_INT_GAIN_OFFSET];
    kd_reg = g_holding_regs[HLD_POWER_PID_DER_GAIN_OFFSET];
    if (kp_reg == 0 || kp_reg > 10000) kp_reg = 100; // 默认1.0
    if (ki_reg > 10000) ki_reg = 0;
    if (kd_reg > 10000) kd_reg = 0;
    g_power_pid.Kp = (int32_t)kp_reg * 1024 / 100; // 转为Q10
    g_power_pid.Ki = (int32_t)ki_reg * 1024 / 100;
    g_power_pid.Kd = (int32_t)kd_reg * 1024 / 100;
    // 最大上升率（放大10倍存储，单位%/s，转为Q10）
    max_rise_reg = g_holding_regs[HLD_POWER_PID_MAX_RISE_OFFSET];
    if (max_rise_reg == 0 || max_rise_reg > 1000) max_rise_reg = 100; // 默认10%/s
    g_power_pid.max_rise = (int32_t)max_rise_reg * 1024 / 10;
    // 滤波系数
    g_power_pid.filter_coeff = (uint8_t)(g_holding_regs[HLD_POWER_PID_FILTER_OFFSET] & 0xFF);
}
/*============================================================================
 * 硬件初始化（PWM、内部ADC、外部串口ADC）
 *============================================================================*/
void PID_Hardware_Init(void)
{
    // PWM输出引脚由 board.h 定义为 P2.0，用于 PID 调节功率。
    P2M0 |= BIT(IO_PWM_OUT_PIN);
    P2M1 &= ~BIT(IO_PWM_OUT_PIN);
    // PWMA_CH1 使用 500 计数周期，Set_PWM_Duty() 按 0~1000 直接写入 CCR1.
    // 当前工程 board 配置为 11.0592MHz，PWM 频率 22.1184kHz，超出音频范围无啸叫
    PWMA_CR1 = 0x00;
    PWMA_CCER1 &= ~0x01;
    PWMA_PSCRH = 0x00;
    PWMA_PSCRL = 0x00;
    PWMA_ARRH = 0x01;
    PWMA_ARRL = 0xF4; // 重载值500
    PWMA_CCR1H = 0x00;
    PWMA_CCR1L = 0x00;
    g_pwm_duty = 0;
    PWMA_CCMR1 = 0x60;
    PWMA_CCER1 |= 0x01;
    PWMA_CR1 |= 0x01;
    // 温度 T1~T4、外部模拟量 1/2 使用 adc_sensor 模块内部 ADC.
    adc_sensor_init();
    // 直流电压、直流电流使用 uart_adc 模块经串口3/串口4接收.
    uart_adc_init();
}
/*============================================================================
 * PWM 占空比更新函数
 *============================================================================*/
void Set_PWM_Duty(uint16_t duty) // duty: 0~1000 对应 0.0%~100.0%
{
    uint16_t ccr_value;
    if (duty > 1000)
        duty = 1000;
    g_pwm_duty = duty;
    // 写入 PWM 占空比寄存器 (CCR1)，重载值500，所以除以2
    ccr_value = duty / 2;
    PWMA_CCR1H = (uint8_t)(ccr_value >> 8);
    PWMA_CCR1L = (uint8_t)(ccr_value & 0xFF);
}
/*============================================================================
 * 校准换算：将原始 ADC 采集值按"校零"和"满量程"参数换算为实际值
 * 公式: actual = (raw - zero) * 1000 / full
 *       zero = HLD_XXX_ZERO (校零寄存器原始值)
 *       full = HLD_XXX_FULL  (满量程寄存器原始值)
 * 输入: raw   - 原始采集值 (uint16)
 *       zero  - 校零值 (HOLDING_REG[HLD_XXX_ZERO_OFFSET])
 *       full  - 满量程值 (HOLDING_REG[HLD_XXX_FULL_OFFSET])
 * 返回: 换算后实际值 (放大100倍，HMI再除以100显示)
 *============================================================================*/
static uint16_t Apply_Calibration(uint16_t raw, uint16_t zero, uint16_t full)
{
    int32_t cal;
    if (full == 0) {
        // 未标定时，直接返回原始值
        return raw;
    }
    cal = ((int32_t)raw - (int32_t)zero) * 1000 / (int32_t)full;
    if (cal < 0) cal = 0;
    if (cal > 65535) cal = 65535;
    return (uint16_t)cal;
}
/*============================================================================
 * 输入寄存器刷新：将原始 ADC 值写入 g_input_regs，并应用校准公式
 * 30000~30009: 已校准值 (已换算，HMI直接显示)
 * 30020~30027: 原始采集值 (用于诊断/二次校准)
 *============================================================================*/
void Modbus_Input_Reg_Update(void)
{
    // 读取原始 ADC 值 (内部ADC先采样，再读UART ADC)
    adc_sensor_read_all();
    // 30020~30027: 原始采集值 (不做任何换算)
    g_input_regs[RAW_IR_VOLT_OFFSET]  = uart_adc_get_voltage();
    g_input_regs[RAW_IR_CURR_OFFSET]  = uart_adc_get_current();
    g_input_regs[RAW_IR_TEMP1_OFFSET] = adc_sensor_get_temp1();
    g_input_regs[RAW_IR_TEMP2_OFFSET] = adc_sensor_get_temp2();
    g_input_regs[RAW_IR_TEMP3_OFFSET] = adc_sensor_get_temp3();
    g_input_regs[RAW_IR_TEMP4_OFFSET] = adc_sensor_get_temp4();
    g_input_regs[RAW_IR_EXT1_OFFSET]  = adc_sensor_get_ext1();
    g_input_regs[RAW_IR_EXT2_OFFSET] = adc_sensor_get_ext2();
    // 30000~30009: 已校准值 (经校零/满量程换算，公式: (raw-zero)/full*100)
    g_input_regs[REG_DC_VOLT_OFFSET]  = Apply_Calibration(g_input_regs[RAW_IR_VOLT_OFFSET],
        g_holding_regs[HLD_VOLT_ZERO_OFFSET], g_holding_regs[HLD_VOLT_FULL_OFFSET]);
    g_input_regs[REG_DC_CURR_OFFSET]  = Apply_Calibration(g_input_regs[RAW_IR_CURR_OFFSET],
        g_holding_regs[HLD_CURR_ZERO_OFFSET], g_holding_regs[HLD_CURR_FULL_OFFSET]);
    g_input_regs[REG_TEMP_T1_OFFSET] = Apply_Calibration(g_input_regs[RAW_IR_TEMP1_OFFSET],
        g_holding_regs[HLD_TEMP1_ZERO_OFFSET], g_holding_regs[HLD_TEMP1_FULL_OFFSET]);
    g_input_regs[REG_TEMP_T2_OFFSET] = Apply_Calibration(g_input_regs[RAW_IR_TEMP2_OFFSET],
        g_holding_regs[HLD_TEMP2_ZERO_OFFSET], g_holding_regs[HLD_TEMP2_FULL_OFFSET]);
    g_input_regs[REG_TEMP_T3_OFFSET] = Apply_Calibration(g_input_regs[RAW_IR_TEMP3_OFFSET],
        g_holding_regs[HLD_TEMP3_ZERO_OFFSET], g_holding_regs[HLD_TEMP3_FULL_OFFSET]);
    g_input_regs[REG_TEMP_T4_OFFSET] = Apply_Calibration(g_input_regs[RAW_IR_TEMP4_OFFSET],
        g_holding_regs[HLD_TEMP4_ZERO_OFFSET], g_holding_regs[HLD_TEMP4_FULL_OFFSET]);
    g_input_regs[REG_EXT_ADC1_OFFSET] = Apply_Calibration(g_input_regs[RAW_IR_EXT1_OFFSET],
        g_holding_regs[HLD_EXT1_ZERO_OFFSET], g_holding_regs[HLD_EXT1_FULL_OFFSET]);
    g_input_regs[REG_EXT_ADC2_OFFSET] = Apply_Calibration(g_input_regs[RAW_IR_EXT2_OFFSET],
        g_holding_regs[HLD_EXT2_ZERO_OFFSET], g_holding_regs[HLD_EXT2_FULL_OFFSET]);
    // REG_WORK_FREQ_OFFSET(0) 由 main.c 中 Freq_Measure_Update() 维护
    // REG_REAL_POWER_OFFSET(3): 实时功率 P = U * I
     g_holding_regs[HLD_CHARGE_SET_OFFSET] = 0;
    {
        uint32_t power = (uint32_t)g_input_regs[REG_DC_VOLT_OFFSET] * (uint32_t)g_input_regs[REG_DC_CURR_OFFSET];
        if (power > 65535) power = 65535;
        g_input_regs[REG_REAL_POWER_OFFSET] = (uint16_t)power;
    }
}
/*============================================================================
 * 数据获取函数（从 Modbus 缓存或硬件读取）
 *============================================================================*/
int32_t Get_Temperature(void)
{
    // 从 30004 获取温度，单位放大100倍，转为Q10格式
    return (int32_t)g_input_regs[REG_TEMP_T1_OFFSET] * 1024 / 100;
}
int32_t Get_Power(void)
{
    // 从 30003 获取功率，单位放大100倍，转为Q10格式
    return (int32_t)g_input_regs[REG_REAL_POWER_OFFSET] * 1024 / 100;
}
// 获取目标温度
int32_t Get_Target_Temp(void)
{
    // 保持寄存器里的温度放大100倍，转为Q10格式
    return (int32_t)g_holding_regs[HLD_TARGET_TEMP_OFFSET] * 1024 / 100;
}
// 获取功率设定值 (40011)
int32_t Get_Power_Setpoint(void)
{
    // 保持寄存器里的功率放大100倍，转为Q10格式
    return (int32_t)g_holding_regs[HLD_POWER_SETPOINT_OFFSET] * 1024 / 100;
}







