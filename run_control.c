#include "run_control.h"
#include "pid_drive.h"
#include "modbus_reg_config.h"

// 按键直接启动标志（定义）：物理按键置位，无视 HLD_CTRL_MODE_OFFSET
volatile uint8_t g_key_running = 0;

/*============================================================================
 * 分段加热状态
 *============================================================================*/
static uint8_t  xdata g_segment_running = 0;
static uint8_t  xdata g_segment_index = 0;
static uint16_t xdata g_segment_elapsed_100ms = 0;
static uint16_t xdata g_segment_last_tick_5ms = 0;

static void Segment_Heat_Reset(void)
{
    g_segment_running = 0;
    g_segment_index = 0;
    g_segment_elapsed_100ms = 0;
    g_segment_last_tick_5ms = g_system_tick_5ms;
}

/*============================================================================
 * 分段加热更新：根据当前工件编号和段索引返回目标功率(Q10格式)
 * 工件号 HLD_CUR_WORK_NO_OFFSET(1~5)，每工件5段，每段含[功率, 时间(×100ms)]
 * 所有段完成后自动停止加热
 *============================================================================*/
static int32_t Segment_Heat_Update(void)
{
    uint8_t  work_no = (uint8_t)g_holding_regs[HLD_CUR_WORK_NO_OFFSET];
    uint16_t now_tick = g_system_tick_5ms;
    uint16_t diff_tick = now_tick - g_segment_last_tick_5ms;
    uint16_t power;
    uint16_t time_100ms;

    if (work_no == 0 || work_no > 5)
    {
        Segment_Heat_Reset();
        return 0;
    }
    if (g_segment_running == 0)
    {
        g_segment_running = 1;
        g_segment_index = 1;
        g_segment_elapsed_100ms = 0;
        g_segment_last_tick_5ms = now_tick;
    }
    // 累计100ms为单位的时间（20 × 5ms = 100ms）
    while (diff_tick >= 20)
    {
        g_segment_elapsed_100ms++;
        g_segment_last_tick_5ms += 20;
        diff_tick -= 20;
    }
    // 推进到当前应执行的段
    while (g_segment_index <= 5)
    {
        time_100ms = g_holding_regs[WORK_TIME_OFFSET(work_no, g_segment_index)];
        if (time_100ms == 0 || g_segment_elapsed_100ms >= time_100ms)
        {
            g_segment_index++;
            g_segment_elapsed_100ms = 0;
            g_segment_last_tick_5ms = now_tick;
            continue;
        }
        break;
    }
    if (g_segment_index > 5)
    {
        // 所有段执行完毕，停止加热
        g_coils[COIL_START_STOP_OFFSET] = 0;
        g_key_running = 0;
        Segment_Heat_Reset();
        return 0;
    }
    power = g_holding_regs[WORK_POWER_OFFSET(work_no, g_segment_index)];
    if (power > 100)
        power = 100;
    // 返回功率Q10格式
    return ((int32_t)power) * 1024L;
}

/*============================================================================
 * 控制逻辑主循环
 *============================================================================*/
void Run_Control_Loop(void)
{
    uint16_t dt = 100; // 默认100ms周期
    uint16_t mode;
    int32_t current_temp;
    int32_t current_power;
    int32_t final_power_target;
    int32_t pwm_output;
    uint16_t duty;
    uint8_t control_active;

    // 控制启动条件：
    //   触屏模式(HLD_CTRL_MODE_OFFSET=1)由启动线圈COIL_START_STOP_OFFSET控制
    //   按键启动(g_key_running)无视HLD_CTRL_MODE_OFFSET，直接按HLD_CHANGE_MODE_OFFSET控制
    control_active = ((g_holding_regs[HLD_CTRL_MODE_OFFSET] == 1) && (g_coils[COIL_START_STOP_OFFSET] != 0))
                   || (g_key_running != 0);
    if (!control_active)
    {
        Set_PWM_Duty(0);
        PID_Reset(&g_temp_pid);
        PID_Reset(&g_power_pid);
        Segment_Heat_Reset();
        return;
    }

    // 加载最新PID参数
    PID_LoadParams();

    // 读取当前测量值
    current_temp = Get_Temperature();
    current_power = Get_Power();

    mode = g_holding_regs[HLD_CHANGE_MODE_OFFSET];

    switch (mode)
    {
        case 1: // 分段加热
            final_power_target = Segment_Heat_Update();
            break;
        case 2: // 温度加热（恒温控制）：温度PID输出作为目标功率
            g_temp_pid.setpoint = Get_Target_Temp();
            final_power_target = PID_Calc(&g_temp_pid, current_temp, dt);
            if (final_power_target < 0)
                final_power_target = 0;
            if (final_power_target > (int32_t)100 * Q_SCALE)
                final_power_target = 100 * Q_SCALE;
            break;
        default: // 0 = 默认，按目标功率PID控制
            final_power_target = Get_Power_Setpoint();
            break;
    }

    // 功率PID（内环）
    g_power_pid.setpoint = final_power_target;
    pwm_output = PID_Calc(&g_power_pid, current_power, dt);
    if (pwm_output < 0)
        pwm_output = 0;
    if (pwm_output > (int32_t)100 * Q_SCALE)
        pwm_output = 100 * Q_SCALE;

    // Q10格式转0~1000占空比
    duty = (uint16_t)(pwm_output * 10 / Q_SCALE);
    Set_PWM_Duty(duty);

    // 写入调节量到保持寄存器（只读，用于HMI显示，放大10倍）
    g_holding_regs[HLD_TEMP_PID_ADJUST_OFFSET] = (uint16_t)(((uint32_t)final_power_target * 10UL) / 1024UL);
    g_holding_regs[HLD_POWER_PID_ADJUST_OFFSET] = (uint16_t)(((uint32_t)pwm_output * 10UL) / 1024UL);
}
