#include "run_control.h"
#include "pid_drive.h"
#include "modbus_reg_config.h"

/* 40003 控制模式：0=功率恒定，1=分段加热，2=温度闭环。 */
#define MODE_POWER   0U
#define MODE_SEGMENT 1U
#define MODE_TEMP    2U
#define CONTROL_DEFAULT_MS 100U

/* 按键和远程端子运行状态由其他扫描逻辑置位，控制循环统一汇总。 */
volatile uint8_t g_key_running = 0;
volatile uint8_t g_remote_running = 0;

/* 分段加热运行状态：每段时间单位为 100ms，系统节拍为 5ms。 */
static uint8_t  xdata g_segment_running = 0;
static uint8_t  xdata g_segment_index = 0;
static uint16_t xdata g_segment_elapsed_100ms = 0;
static uint16_t xdata g_segment_last_tick_5ms = 0;

static uint8_t  xdata g_control_prev_active = 0;
static uint16_t xdata g_control_prev_mode = 0xFFFFU;
static uint16_t xdata g_temp_elapsed_ms = 0;
static uint16_t xdata g_power_elapsed_ms = 0;

static uint8_t Control_Is_Active(void)
{
    /* 触屏启动、按键启动、远程端子启动三者任一有效即认为系统运行。 */
    return ((g_holding_regs[HLD_CTRL_MODE_OFFSET] == 1) && (g_coils[COIL_START_STOP_OFFSET] != 0))
        || (g_key_running != 0)
        || (g_remote_running != 0);
}

static void Control_Stop_All(void)
{
    /* 内部停止统一清除所有启动源，避免某一路状态残留导致再次启动。 */
    g_coils[COIL_START_STOP_OFFSET] = 0;
    g_key_running = 0;
    g_remote_running = 0;
}

static void Segment_Heat_Reset(void)
{
    g_segment_running = 0;
    g_segment_index = 0;
    g_segment_elapsed_100ms = 0;
    g_segment_last_tick_5ms = g_system_tick_5ms;
}

static int32_t Clamp_Power_Target_Q10(int32_t target)
{
    int32_t limit;

    /* 目标功率限幅：单位与 30003/40011 一致，内部乘 Q_SCALE。 */
    limit = (int32_t)Get_Power_Limit_Value() * Q_SCALE;
    if (target < 0) return 0;
    if (target > limit) return limit;
    return target;
}

static int32_t Clamp_Output_Percent_Q10(int32_t output)
{
    /* 功率 PID 的输出才是 PWM 百分比，固定限制在 0~100%。 */
    if (output < 0) return 0;
    if (output > PID_PERCENT_MAX) return PID_PERCENT_MAX;
    return output;
}

static uint16_t Accumulate_Elapsed(uint16_t current, uint16_t add, uint16_t cap)
{
    uint32_t sum;

    /* 累计采样时间时不超过采样周期，避免长时间阻塞后一次性超大 dt。 */
    sum = (uint32_t)current + (uint32_t)add;
    if (sum > (uint32_t)cap) return cap;
    return (uint16_t)sum;
}

static void Reset_Unused_Pids(uint16_t mode)
{
    /* 非当前模式的 PID/分段状态要复位，切回时从干净状态开始。 */
    if (mode != MODE_TEMP) {
        PID_Reset(&g_temp_pid);
        g_temp_elapsed_ms = PID_Get_Temp_Sample_Time();
    }
    if (mode != MODE_SEGMENT) {
        Segment_Heat_Reset();
    }
}

static void Reset_Control_State(uint16_t mode)
{
    PID_Reset(&g_temp_pid);
    PID_Reset(&g_power_pid);
    Segment_Heat_Reset();
    g_temp_elapsed_ms = PID_Get_Temp_Sample_Time();
    g_power_elapsed_ms = PID_Get_Power_Sample_Time();
    g_control_prev_mode = mode;
}

static int32_t Segment_Heat_Update(void)
{
    uint8_t work_no;
    uint16_t now_tick;
    uint16_t diff_tick;
    uint16_t power;
    uint16_t time_100ms;

    /* 分段模式读取当前工件号，每个工件最多 5 段。 */
    work_no = (uint8_t)g_holding_regs[HLD_CUR_WORK_NO_OFFSET];
    now_tick = g_system_tick_5ms;
    diff_tick = now_tick - g_segment_last_tick_5ms;

    if (work_no == 0 || work_no > 5U) {
        Segment_Heat_Reset();
        return 0;
    }

    if (g_segment_running == 0) {
        g_segment_running = 1;
        g_segment_index = 1;
        g_segment_elapsed_100ms = 0;
        g_segment_last_tick_5ms = now_tick;
    }

    /* 20 个 5ms tick = 100ms，和工艺时间寄存器单位一致。 */
    while (diff_tick >= 20U) {
        g_segment_elapsed_100ms++;
        g_segment_last_tick_5ms += 20U;
        diff_tick -= 20U;
    }

    /* 时间为 0 的段直接跳过；所有段结束后停止运行。 */
    while (g_segment_index <= 5U) {
        time_100ms = g_holding_regs[WORK_TIME_OFFSET(work_no, g_segment_index)];
        if (time_100ms == 0 || g_segment_elapsed_100ms >= time_100ms) {
            g_segment_index++;
            g_segment_elapsed_100ms = 0;
            g_segment_last_tick_5ms = now_tick;
            continue;
        }
        break;
    }

    if (g_segment_index > 5U) {
        Control_Stop_All();
        Segment_Heat_Reset();
        return 0;
    }

    /* 段功率是目标真实功率，不是 PWM 百分比。 */
    power = g_holding_regs[WORK_POWER_OFFSET(work_no, g_segment_index)];
    return Clamp_Power_Target_Q10((int32_t)power * Q_SCALE);
}

void Run_Control_Loop(uint16_t elapsed_ms)
{
    uint16_t mode;
    uint16_t temp_sample_ms;
    uint16_t power_sample_ms;
    int32_t current_temp;
    int32_t current_power;
    int32_t final_power_target;
    int32_t pwm_output;
    uint16_t duty;
    uint8_t control_active;

    if (elapsed_ms == 0) elapsed_ms = CONTROL_DEFAULT_MS;

    /* 未运行时强制关闭 PWM，并清空 PID 和显示调节量。 */
    control_active = Control_Is_Active();
    if (!control_active) {
        Set_PWM_Duty(0);
        PID_Reset(&g_temp_pid);
        PID_Reset(&g_power_pid);
        Segment_Heat_Reset();
        g_temp_elapsed_ms = 0;
        g_power_elapsed_ms = 0;
        g_control_prev_active = 0;
        g_holding_regs[HLD_TEMP_PID_ADJUST_OFFSET] = 0;
        g_holding_regs[HLD_POWER_PID_ADJUST_OFFSET] = 0;
        return;
    }

    /* 允许 HMI 在线修改 PID 参数，每次控制循环重新装载并限幅。 */
    PID_LoadParams();

    mode = g_holding_regs[HLD_CHANGE_MODE_OFFSET];
    if (mode > MODE_TEMP) mode = MODE_POWER;

    if (!g_control_prev_active || mode != g_control_prev_mode) {
        Reset_Control_State(mode);
    }
    g_control_prev_active = 1;
    g_control_prev_mode = mode;

    temp_sample_ms = PID_Get_Temp_Sample_Time();
    power_sample_ms = PID_Get_Power_Sample_Time();
    g_temp_elapsed_ms = Accumulate_Elapsed(g_temp_elapsed_ms, elapsed_ms, temp_sample_ms);
    g_power_elapsed_ms = Accumulate_Elapsed(g_power_elapsed_ms, elapsed_ms, power_sample_ms);

    Reset_Unused_Pids(mode);

    /* 测量值已经在 Modbus_Input_Reg_Update 中完成校准和物理量换算。 */
    current_temp = Get_Temperature();
    current_power = Get_Power();

    /*
     * 三种模式最终都产生一个“目标功率”：
     * - 功率模式：40011；
     * - 分段模式：当前段功率；
     * - 温控模式：温度 PID 输出。
     */
    if (mode == MODE_SEGMENT) {
        final_power_target = Segment_Heat_Update();
    } else if (mode == MODE_TEMP) {
        g_temp_pid.setpoint = Get_Target_Temp();
        if (g_temp_elapsed_ms >= temp_sample_ms) {
            final_power_target = PID_Calc(&g_temp_pid, current_temp, g_temp_elapsed_ms);
            g_temp_elapsed_ms = 0;
        } else {
            final_power_target = g_temp_pid.prev_output;
        }
        final_power_target = Clamp_Power_Target_Q10(final_power_target);
    } else {
        final_power_target = Get_Power_Setpoint();
    }

    /* 功率 PID 根据真实功率误差调节 PWM 百分比。 */
    g_power_pid.setpoint = Clamp_Power_Target_Q10(final_power_target);
    if (g_power_elapsed_ms >= power_sample_ms) {
        pwm_output = PID_Calc(&g_power_pid, current_power, g_power_elapsed_ms);
        g_power_elapsed_ms = 0;
    } else {
        pwm_output = g_power_pid.prev_output;
    }
    pwm_output = Clamp_Output_Percent_Q10(pwm_output);

    /* pwm_output 为 0~100%，Set_PWM_Duty 使用 0~1000 的 0.1% 精度。 */
    duty = (uint16_t)(pwm_output * 10L / Q_SCALE);
    Set_PWM_Duty(duty);

    /* 40108 显示温控外环输出目标功率；40116 显示功控内环 PWM 输出。 */
    if (mode == MODE_TEMP) {
        g_holding_regs[HLD_TEMP_PID_ADJUST_OFFSET] =
            (uint16_t)((uint32_t)final_power_target / (uint32_t)Q_SCALE);
    } else {
        g_holding_regs[HLD_TEMP_PID_ADJUST_OFFSET] = 0;
    }
    g_holding_regs[HLD_POWER_PID_ADJUST_OFFSET] =
        (uint16_t)(((uint32_t)pwm_output * 10UL) / (uint32_t)Q_SCALE);
}

#define REMOTE_DEBOUNCE_TICKS   10U
#define REMOTE_LONG_PRESS_TICKS 400U

static uint8_t  xdata g_remote_state = 0;
static uint16_t xdata g_remote_last_tick = 0;
static uint16_t xdata g_remote_press_tick = 0;
static uint8_t  xdata g_remote_was_running = 0;

void Remote_Control_Scan(void)
{
    uint16_t now;
    uint8_t pressed;

    /* 远程输入低电平有效，带 50ms 去抖和 2s 长按停止。 */
    now = g_system_tick_5ms;
    if ((uint16_t)(now - g_remote_last_tick) < REMOTE_DEBOUNCE_TICKS) {
        return;
    }
    g_remote_last_tick = now;

    pressed = REMOTE_READ() ? 0 : 1;
    if (pressed == g_remote_state) return;

    g_remote_state = pressed;
    if (pressed) {
        g_remote_was_running = Control_Is_Active();
        g_remote_press_tick = now;
        if (!g_remote_was_running) g_remote_running = 1;
    } else {
        if ((uint16_t)(now - g_remote_press_tick) >= REMOTE_LONG_PRESS_TICKS) {
            Control_Stop_All();
        } else {
            if (g_remote_was_running) Control_Stop_All();
        }
    }
}
