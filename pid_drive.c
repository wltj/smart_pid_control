#include "pid_drive.h"
#include "adc_sensor.h"
#include "uart_adc.h"
#include "modbus_reg_config.h"
#include "modbus.h"

/* 部分工程头文件未暴露 PWMA 寄存器时，在这里补齐寄存器地址。 */
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

/* UART ADC 已经把 0~5V 输入转换成电压量，这里用 5 作为模拟量满刻度基准。 */
#define ANALOG_INPUT_FULL_SCALE 5U

/* PID 增益寄存器按百分数录入：100 表示 1.00 倍。 */
#define PID_GAIN_REG_DEFAULT 100U
#define PID_GAIN_REG_MAX 10000U

/* 最大上升率寄存器按 0.1 单位录入：100 表示 10.0 单位/秒。 */
#define PID_MAX_RISE_DEFAULT 100U
#define PID_MAX_RISE_MAX 1000U

/* PID 采样周期保护范围，避免误写 0 或异常大值导致控制失真。 */
#define PID_SAMPLE_DEFAULT_MS 100U
#define PID_SAMPLE_MIN_MS 20U
#define PID_SAMPLE_MAX_MS 5000U
#define I32_MAX_VALUE 2147483647L
#define I32_MIN_VALUE (-2147483647L - 1L)
#define U32_MAX_VALUE 0xFFFFFFFFUL

PID_Handle_t xdata g_temp_pid;
PID_Handle_t xdata g_power_pid;

/* 当前 PWM 输出，0~1000 对应 0.0%~100.0%，用于调试和状态保持。 */
static uint16_t xdata g_pwm_duty = 0;

static int32_t Clamp_I32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static uint32_t Abs_I32_To_U32(int32_t value)
{
    if (value == I32_MIN_VALUE) return ((uint32_t)I32_MAX_VALUE + 1UL);
    if (value < 0) return (uint32_t)(-value);
    return (uint32_t)value;
}

static int32_t Signed_From_U32(uint32_t value, uint8_t negative)
{
    if (negative) {
        if (value >= ((uint32_t)I32_MAX_VALUE + 1UL)) return I32_MIN_VALUE;
        return -(int32_t)value;
    }
    if (value > (uint32_t)I32_MAX_VALUE) return I32_MAX_VALUE;
    return (int32_t)value;
}

static uint32_t Mul_Div_U32_Sat(uint32_t value, uint32_t mul, uint32_t div)
{
    uint32_t high;
    uint32_t rem;
    uint32_t result;
    uint32_t part;

    if (div == 0) return U32_MAX_VALUE;

    high = value / div;
    rem = value % div;

    if (high != 0 && mul > (U32_MAX_VALUE / high)) return U32_MAX_VALUE;
    result = high * mul;

    if (rem != 0 && mul > (U32_MAX_VALUE / rem)) return U32_MAX_VALUE;
    part = (rem * mul) / div;

    if (result > U32_MAX_VALUE - part) return U32_MAX_VALUE;
    return result + part;
}

static int32_t PID_Mul_Div(int32_t value, uint32_t mul, uint32_t div)
{
    uint8_t negative;
    uint32_t scaled;

    negative = (value < 0) ? 1 : 0;
    scaled = Mul_Div_U32_Sat(Abs_I32_To_U32(value), mul, div);
    return Signed_From_U32(scaled, negative);
}

static int32_t PID_MulQ(int32_t a, int32_t b)
{
    uint8_t negative;
    uint32_t ua;
    uint32_t ub;
    uint32_t high;
    uint32_t rem;
    uint32_t result;
    uint32_t part;

    negative = 0;
    if (a < 0) negative ^= 1;
    if (b < 0) negative ^= 1;

    /* Q10 定点乘法拆成商和余数，降低 32 位乘法溢出的风险。 */
    ua = Abs_I32_To_U32(a);
    ub = Abs_I32_To_U32(b);
    high = ua / (uint32_t)Q_SCALE;
    rem = ua % (uint32_t)Q_SCALE;

    if (high != 0 && ub > (U32_MAX_VALUE / high)) {
        return negative ? I32_MIN_VALUE : I32_MAX_VALUE;
    }
    result = high * ub;

    if (rem != 0 && ub > (U32_MAX_VALUE / rem)) {
        return negative ? I32_MIN_VALUE : I32_MAX_VALUE;
    }
    part = (rem * ub) / (uint32_t)Q_SCALE;

    if (result > U32_MAX_VALUE - part) {
        return negative ? I32_MIN_VALUE : I32_MAX_VALUE;
    }
    return Signed_From_U32(result + part, negative);
}

static int32_t PID_Add_Sat(int32_t a, int32_t b)
{
    if (b > 0 && a > I32_MAX_VALUE - b) return I32_MAX_VALUE;
    if (b < 0 && a < I32_MIN_VALUE - b) return I32_MIN_VALUE;
    return a + b;
}

static int32_t PID_Apply_Rise_Limit(PID_Handle_t xdata *pid, int32_t output, uint16_t dt)
{
    int32_t delta;
    int32_t delta_max;

    /* 只限制上升速度，下降时立即响应，便于过温或过功率时快速回落。 */
    delta = output - pid->prev_output;
    if (delta <= 0) return output;

    delta_max = PID_Mul_Div(pid->max_rise, dt, 1000UL);
    if (delta > delta_max) return PID_Add_Sat(pid->prev_output, delta_max);
    return output;
}

static void PID_Set_Output_Limits(PID_Handle_t xdata *pid, int32_t out_min, int32_t out_max, int32_t integral_limit)
{
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_limit = integral_limit;
    pid->prev_output = Clamp_I32(pid->prev_output, out_min, out_max);
    pid->integral = Clamp_I32(pid->integral, -integral_limit, integral_limit);
}

static uint16_t Normalize_Sample_Time(uint16_t sample_ms)
{
    if (sample_ms == 0) return PID_SAMPLE_DEFAULT_MS;
    if (sample_ms < PID_SAMPLE_MIN_MS) return PID_SAMPLE_MIN_MS;
    if (sample_ms > PID_SAMPLE_MAX_MS) return PID_SAMPLE_MAX_MS;
    return sample_ms;
}

static int32_t Gain_Reg_To_Q10(uint16_t reg_value, uint8_t zero_uses_default)
{
    /* Kp 的 0 使用默认 1.00，Ki/Kd 的 0 表示关闭对应环节。 */
    if (reg_value == 0 && zero_uses_default) reg_value = PID_GAIN_REG_DEFAULT;
    if (reg_value > PID_GAIN_REG_MAX) reg_value = PID_GAIN_REG_MAX;
    return (int32_t)reg_value * Q_SCALE / 100L;
}

static void PID_Load_One(PID_Handle_t xdata *pid,
                         uint16_t kp_offset,
                         uint16_t ki_offset,
                         uint16_t kd_offset,
                         uint16_t max_rise_offset,
                         uint16_t filter_offset)
{
    uint16_t max_rise_reg;

    /* HMI 写入的 PID 参数统一转换成 Q10，PID_Calc 内部不再碰寄存器。 */
    pid->Kp = Gain_Reg_To_Q10(g_holding_regs[kp_offset], 1);
    pid->Ki = Gain_Reg_To_Q10(g_holding_regs[ki_offset], 0);
    pid->Kd = Gain_Reg_To_Q10(g_holding_regs[kd_offset], 0);

    max_rise_reg = g_holding_regs[max_rise_offset];
    if (max_rise_reg == 0 || max_rise_reg > PID_MAX_RISE_MAX) {
        max_rise_reg = PID_MAX_RISE_DEFAULT;
    }
    pid->max_rise = (int32_t)max_rise_reg * Q_SCALE / 10L;
    pid->filter_coeff = (uint8_t)(g_holding_regs[filter_offset] & 0xFF);
}

static uint16_t Apply_Temperature_Calibration(uint16_t raw, int16_t zero)
{
    int32_t cal;

    /* 温度 ADC 驱动已经换算成实际温度，这里只做零点修正，不再乘满量程。 */
    cal = (int32_t)raw - (int32_t)zero;
    if (cal < 0) cal = 0;
    if (cal > 65535L) cal = 65535L;
    return (uint16_t)cal;
}

static uint16_t Apply_Analog_Calibration(uint16_t raw, int16_t zero, uint16_t full)
{
    int32_t cal;

    if (full == 0) return raw;

    /* 其他 0~5V 模拟量：实际值 = (输入物理量 - 零点) * 满量程 / 5V。 */
    cal = ((int32_t)raw - (int32_t)zero) * (int32_t)full / (int32_t)ANALOG_INPUT_FULL_SCALE;
    if (cal < 0) cal = 0;
    if (cal > 65535L) cal = 65535L;
    return (uint16_t)cal;
}

static uint16_t Real_Power_From_VI(uint16_t voltage, uint16_t current)
{
    uint32_t power;

    /* 30003 是真实运行功率，不是百分比；电压、电流已经各自完成满量程换算。 */
    power = (uint32_t)voltage * (uint32_t)current;
    if (power > 65535UL) power = 65535UL;
    return (uint16_t)power;
}

uint16_t Get_Power_Limit_Value(void)
{
    uint16_t limit;

    /* 40027 单位与 30003/40011 一致；写 0 表示不限制目标功率。 */
    limit = g_holding_regs[HLD_POWER_LIMIT_OFFSET];
    if (limit == 0) return 65535U;
    return limit;
}

static int32_t Get_Power_Limit_Q10(void)
{
    return (int32_t)Get_Power_Limit_Value() * Q_SCALE;
}

uint16_t PID_Get_Temp_Sample_Time(void)
{
    return Normalize_Sample_Time(g_holding_regs[HLD_TEMP_PID_SAMPLE_TIME_OFFSET]);
}

uint16_t PID_Get_Power_Sample_Time(void)
{
    return Normalize_Sample_Time(g_holding_regs[HLD_POWER_PID_SAMPLE_TIME_OFFSET]);
}

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
    pid->max_rise = 10L * Q_SCALE;
    pid->filter_coeff = 0;
    PID_Reset(pid);
}

void PID_Reset(PID_Handle_t xdata *pid)
{
    pid->setpoint = 0;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_prev_error = 0;
    pid->prev_output = 0;
    pid->prev_measured = 0;
    pid->measured_valid = 0;
}

int32_t PID_Calc(PID_Handle_t xdata *pid, int32_t measurement, uint16_t dt)
{
    int32_t error;
    int32_t output;
    int32_t p_term;
    int32_t i_term;
    int32_t d_term;
    int32_t delta_error;
    int32_t second_error;
    int32_t old_integral;
    int32_t new_integral;
    int32_t derivative;

    if (dt == 0) dt = 1;

    if (pid->filter_coeff > 0 && pid->measured_valid) {
        measurement = (measurement * (256 - pid->filter_coeff) + pid->prev_measured * pid->filter_coeff) / 256;
    }
    pid->prev_measured = measurement;
    pid->measured_valid = 1;
    error = pid->setpoint - measurement;

    if (pid->is_incremental) {
        delta_error = error - pid->prev_error;
        second_error = error - 2L * pid->prev_error + pid->prev_prev_error;

        p_term = PID_MulQ(pid->Kp, delta_error);
        i_term = PID_MulQ(pid->Ki, PID_Mul_Div(error, dt, 1000UL));
        d_term = PID_MulQ(pid->Kd, PID_Mul_Div(second_error, 1000UL, dt));
        output = PID_Add_Sat(pid->prev_output, PID_Add_Sat(PID_Add_Sat(p_term, i_term), d_term));
    } else {
        old_integral = pid->integral;
        new_integral = PID_Add_Sat(pid->integral, PID_Mul_Div(error, dt, 1000UL));
        new_integral = Clamp_I32(new_integral, -pid->integral_limit, pid->integral_limit);
        derivative = PID_Mul_Div(error - pid->prev_error, 1000UL, dt);

        p_term = PID_MulQ(pid->Kp, error);
        i_term = PID_MulQ(pid->Ki, new_integral);
        d_term = PID_MulQ(pid->Kd, derivative);
        output = PID_Add_Sat(PID_Add_Sat(p_term, i_term), d_term);

        if ((output > pid->output_max && error > 0) ||
            (output < pid->output_min && error < 0)) {
            new_integral = old_integral;
            i_term = PID_MulQ(pid->Ki, new_integral);
            output = PID_Add_Sat(PID_Add_Sat(p_term, i_term), d_term);
        }
        pid->integral = new_integral;
    }

    output = Clamp_I32(output, pid->output_min, pid->output_max);
    output = PID_Apply_Rise_Limit(pid, output, dt);
    output = Clamp_I32(output, pid->output_min, pid->output_max);

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_output = output;
    return output;
}

void PID_LoadParams(void)
{
    /*
     * 级联控制单位约定：
     * - 温度 PID：输入/目标为温度，输出为目标功率；
     * - 功率 PID：输入/目标为真实功率，输出为 PWM 百分比。
     */
    PID_Load_One(&g_temp_pid,
        HLD_TEMP_PID_PROP_GAIN_OFFSET,
        HLD_TEMP_PID_INT_GAIN_OFFSET,
        HLD_TEMP_PID_DER_GAIN_OFFSET,
        HLD_TEMP_PID_MAX_RISE_OFFSET,
        HLD_TEMP_PID_FILTER_OFFSET);
    PID_Set_Output_Limits(&g_temp_pid, 0, Get_Power_Limit_Q10(), Get_Power_Limit_Q10());

    PID_Load_One(&g_power_pid,
        HLD_POWER_PID_PROP_GAIN_OFFSET,
        HLD_POWER_PID_INT_GAIN_OFFSET,
        HLD_POWER_PID_DER_GAIN_OFFSET,
        HLD_POWER_PID_MAX_RISE_OFFSET,
        HLD_POWER_PID_FILTER_OFFSET);
    PID_Set_Output_Limits(&g_power_pid, 0, PID_PERCENT_MAX, PID_PERCENT_MAX);
}

void PID_Hardware_Init(void)
{
    /* PWM 输出引脚配置为推挽输出。 */
    P2M0 |= BIT(IO_PWM_OUT_PIN);
    P2M1 &= ~BIT(IO_PWM_OUT_PIN);

    /* 初始化 PWMA1：ARR=500，CCR=duty/2，duty=1000 时对应 100.0%。 */
    PWMA_CR1 = 0x00;
    PWMA_CCER1 &= ~0x01;
    PWMA_PSCRH = 0x00;
    PWMA_PSCRL = 0x00;
    PWMA_ARRH = 0x01;
    PWMA_ARRL = 0xF4;
    PWMA_CCR1H = 0x00;
    PWMA_CCR1L = 0x00;
    g_pwm_duty = 0;
    PWMA_CCMR1 = 0x60;
    PWMA_CCER1 |= 0x01;
    PWMA_CR1 |= 0x01;

    adc_sensor_init();
    uart_adc_init();
}

void Set_PWM_Duty(uint16_t duty)
{
    uint16_t ccr_value;

    /* duty 使用 0.1% 精度，超过 1000 直接限幅，保护硬件输出。 */
    if (duty > 1000U) duty = 1000U;
    g_pwm_duty = duty;
    ccr_value = duty / 2U;
    PWMA_CCR1H = (uint8_t)(ccr_value >> 8);
    PWMA_CCR1L = (uint8_t)(ccr_value & 0xFF);
}

void Modbus_Input_Reg_Update(void)
{
    uint16_t temp1;
    uint16_t temp2;
    uint16_t temp3;
    uint16_t temp4;

    /* 先刷新本地 ADC 温度，再读取 UART ADC 的电压、电流和外部模拟量。 */
    adc_sensor_read_all();
    temp1 = adc_sensor_get_temp1();
    temp2 = adc_sensor_get_temp2();
    temp3 = adc_sensor_get_temp3();
    temp4 = adc_sensor_get_temp4();

    /* 30020~30027 保存未校准值，便于现场查看传感器和校准前状态。 */
    g_input_regs[RAW_IR_VOLT_OFFSET]  = uart_adc_get_voltage();
    g_input_regs[RAW_IR_CURR_OFFSET]  = uart_adc_get_current();
    g_input_regs[RAW_IR_TEMP1_OFFSET] = adc_sensor_get_temp1_raw();
    g_input_regs[RAW_IR_TEMP2_OFFSET] = adc_sensor_get_temp2_raw();
    g_input_regs[RAW_IR_TEMP3_OFFSET] = adc_sensor_get_temp3_raw();
    g_input_regs[RAW_IR_TEMP4_OFFSET] = adc_sensor_get_temp4_raw();
    g_input_regs[RAW_IR_EXT1_OFFSET]  = adc_sensor_get_ext1();
    g_input_regs[RAW_IR_EXT2_OFFSET]  = adc_sensor_get_ext2();

    /* 30001~30009 保存校准后的实际物理量。 */
    g_input_regs[REG_DC_VOLT_OFFSET] = Apply_Analog_Calibration(g_input_regs[RAW_IR_VOLT_OFFSET],
        (int16_t)g_holding_regs[HLD_VOLT_ZERO_OFFSET], g_holding_regs[HLD_VOLT_FULL_OFFSET]);
    g_input_regs[REG_DC_CURR_OFFSET] = Apply_Analog_Calibration(g_input_regs[RAW_IR_CURR_OFFSET],
        (int16_t)g_holding_regs[HLD_CURR_ZERO_OFFSET], g_holding_regs[HLD_CURR_FULL_OFFSET]);
    g_input_regs[REG_TEMP_T1_OFFSET] = Apply_Temperature_Calibration(temp1,
        (int16_t)g_holding_regs[HLD_TEMP1_ZERO_OFFSET]);
    g_input_regs[REG_TEMP_T2_OFFSET] = Apply_Temperature_Calibration(temp2,
        (int16_t)g_holding_regs[HLD_TEMP2_ZERO_OFFSET]);
    g_input_regs[REG_TEMP_T3_OFFSET] = Apply_Temperature_Calibration(temp3,
        (int16_t)g_holding_regs[HLD_TEMP3_ZERO_OFFSET]);
    g_input_regs[REG_TEMP_T4_OFFSET] = Apply_Temperature_Calibration(temp4,
        (int16_t)g_holding_regs[HLD_TEMP4_ZERO_OFFSET]);
    g_input_regs[REG_EXT_ADC1_OFFSET] = Apply_Analog_Calibration(g_input_regs[RAW_IR_EXT1_OFFSET],
        (int16_t)g_holding_regs[HLD_EXT1_ZERO_OFFSET], g_holding_regs[HLD_EXT1_FULL_OFFSET]);
    g_input_regs[REG_EXT_ADC2_OFFSET] = Apply_Analog_Calibration(g_input_regs[RAW_IR_EXT2_OFFSET],
        (int16_t)g_holding_regs[HLD_EXT2_ZERO_OFFSET], g_holding_regs[HLD_EXT2_FULL_OFFSET]);

    /* 30003 由真实电压和真实电流直接相乘得到，不使用 40032/40034 做百分比换算。 */
    g_input_regs[REG_REAL_POWER_OFFSET] = Real_Power_From_VI(
        g_input_regs[REG_DC_VOLT_OFFSET],
        g_input_regs[REG_DC_CURR_OFFSET]);
}

int32_t Get_Temperature(void)
{
    return (int32_t)g_input_regs[REG_TEMP_T1_OFFSET] * Q_SCALE;
}

int32_t Get_Power(void)
{
    /* 功率 PID 反馈使用 30003 真实功率，不能再按 0~100% 处理。 */
    return (int32_t)g_input_regs[REG_REAL_POWER_OFFSET] * Q_SCALE;
}

int32_t Get_Target_Temp(void)
{
    return (int32_t)g_holding_regs[HLD_TARGET_TEMP_OFFSET] * Q_SCALE;
}

int32_t Get_Power_Setpoint(void)
{
    uint16_t setpoint;
    uint16_t limit;

    /* 40011 是真实目标功率，最多受 40027 总功率限制约束。 */
    setpoint = g_holding_regs[HLD_POWER_SETPOINT_OFFSET];
    limit = Get_Power_Limit_Value();
    if (setpoint > limit) setpoint = limit;
    return (int32_t)setpoint * Q_SCALE;
}
