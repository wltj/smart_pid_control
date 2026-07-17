#include "alarm_manager.h"
#include "board.h"
#include "modbus_reg_config.h"
#include "run_control.h"
#include "debugUart.h"

volatile unsigned char xdata g_coils[COIL_COUNT] = {0};
volatile unsigned char xdata g_discrete_inputs[DI_COUNT] = {0};
volatile unsigned short xdata g_input_regs[INPUT_REG_COUNT] = {0};
volatile unsigned short xdata g_holding_regs[HOLDING_REG_COUNT] = {0};

#define ALARM_OVER_LIMIT(value, low, high) (((low) != 0 && (value) < (low)) || ((high) != 0 && (value) > (high)))
#ifndef ALARM_IO_DEBUG_ENABLE
#define ALARM_IO_DEBUG_ENABLE 1
#endif

static unsigned int xdata g_run_blink_tick = 0;
static unsigned char xdata g_alarm_any_fault = 0;

extern volatile unsigned int xdata g_system_tick_5ms;

#if ALARM_IO_DEBUG_ENABLE
static unsigned char xdata g_alarm_io_debug_inited = 0;
static unsigned char xdata g_alarm_io_debug_over_current = 0;
static unsigned char xdata g_alarm_io_debug_water_lack = 0;
static unsigned char xdata g_alarm_io_debug_phase_loss = 0;
static unsigned char xdata g_alarm_io_debug_maintenance = 0;
static unsigned char xdata g_alarm_io_debug_running = 0;

static void alarm_debug_print_io(char *name, unsigned char value)
{
    debug_out("IO ");
    debug_out(name);
    debug_out(value ? "=1\r\n" : "=0\r\n");
}

static void alarm_debug_log_io_changes(void)
{
    unsigned char value;

    value = OVER_CURRENT_IN_READ() ? 1 : 0;
    if (!g_alarm_io_debug_inited || g_alarm_io_debug_over_current != value) {
        g_alarm_io_debug_over_current = value;
        alarm_debug_print_io("I-GL/P5.0", value);
    }

    value = WATER_LACK_READ() ? 1 : 0;
    if (!g_alarm_io_debug_inited || g_alarm_io_debug_water_lack != value) {
        g_alarm_io_debug_water_lack = value;
        alarm_debug_print_io("WATER/P4.3", value);
    }

    value = PHASE_LOSS_DET_READ() ? 1 : 0;
    if (!g_alarm_io_debug_inited || g_alarm_io_debug_phase_loss != value) {
        g_alarm_io_debug_phase_loss = value;
        alarm_debug_print_io("PHASE/P0.6", value);
    }

    value = MAINTENANCE_READ() ? 1 : 0;
    if (!g_alarm_io_debug_inited || g_alarm_io_debug_maintenance != value) {
        g_alarm_io_debug_maintenance = value;
        alarm_debug_print_io("MAINT/P4.1", value);
    }

    value = DEVICE_RUNNING_READ() ? 1 : 0;
    if (!g_alarm_io_debug_inited || g_alarm_io_debug_running != value) {
        g_alarm_io_debug_running = value;
        alarm_debug_print_io("RUN/P5.1", value);
    }

    g_alarm_io_debug_inited = 1;
}
#else
#define alarm_debug_log_io_changes() do { } while (0)
#endif

/*============================================================================
 * 报警相关 GPIO 引脚模式初始化
 *============================================================================*/
void alarm_gpio_init(void)
{
    /* 故障输出推挽：P2.2~P2.7 六路报警输出 */
    P2M0 |= BIT(IO_PHASE_LOSS_ALARM_PIN) | BIT(IO_WATER_PRESS_ALARM_PIN) | BIT(IO_WATER_TEMP_ALARM_PIN) | BIT(IO_GRID_ALARM_PIN) | BIT(IO_OVER_CURRENT_ALARM_PIN) | BIT(IO_FREQ_ALARM_PIN);
    P2M1 &= ~(BIT(IO_PHASE_LOSS_ALARM_PIN) | BIT(IO_WATER_PRESS_ALARM_PIN) | BIT(IO_WATER_TEMP_ALARM_PIN) | BIT(IO_GRID_ALARM_PIN) | BIT(IO_OVER_CURRENT_ALARM_PIN) | BIT(IO_FREQ_ALARM_PIN));
    P0M0 |= BIT(IO_TOTAL_FAULT_PIN) | BIT(IO_RUN_IND_PIN);
    P0M1 &= ~(BIT(IO_TOTAL_FAULT_PIN) | BIT(IO_RUN_IND_PIN));

    /* 故障/状态输入配置 */
    P5M0 &= ~BIT(IO_WORK_STATUS_IN_PIN);
    P5M1 |= BIT(IO_WORK_STATUS_IN_PIN);
    /* I-GL 是光耦开集电极输出：默认上拉为1，故障时光耦导通拉低为0。 */
    P5M0 &= ~BIT(IO_OVER_CURRENT_IN_PIN);
    P5M1 &= ~BIT(IO_OVER_CURRENT_IN_PIN);
    SET_PIN(IO_OVER_CURRENT_IN_PORT, IO_OVER_CURRENT_IN_PIN);
    P4M0 &= ~(BIT(IO_WATER_LACK_PIN) | BIT(IO_MAINTENANCE_PIN));
    P4M1 |= BIT(IO_WATER_LACK_PIN) | BIT(IO_MAINTENANCE_PIN);
    P0M0 &= ~BIT(IO_PHASE_LOSS_DET_PIN);
    P0M1 |= BIT(IO_PHASE_LOSS_DET_PIN);

    /* 初始全部清零 */
    CLR_PIN(IO_OVER_CURRENT_ALARM_PORT, IO_OVER_CURRENT_ALARM_PIN);
    CLR_PIN(IO_WATER_PRESS_ALARM_PORT, IO_WATER_PRESS_ALARM_PIN);
    CLR_PIN(IO_WATER_TEMP_ALARM_PORT, IO_WATER_TEMP_ALARM_PIN);
    CLR_PIN(IO_PHASE_LOSS_ALARM_PORT, IO_PHASE_LOSS_ALARM_PIN);
    CLR_PIN(IO_GRID_ALARM_PORT, IO_GRID_ALARM_PIN);
    CLR_PIN(IO_FREQ_ALARM_PORT, IO_FREQ_ALARM_PIN);
    CLR_PIN(IO_TOTAL_FAULT_PORT, IO_TOTAL_FAULT_PIN);
    CLR_PIN(IO_RUN_IND_PORT, IO_RUN_IND_PIN);
}

static void alarm_clear_latch(void)
{
    g_discrete_inputs[DI_WATER_TEMP_FAULT_OFFSET] = 0;
    g_discrete_inputs[DI_WATER_PRESS_FAULT_OFFSET] = 0;
    g_discrete_inputs[DI_OVER_CURRENT_FAULT_OFFSET] = 0;
    g_discrete_inputs[DI_PHASE_LOSS_FAULT_OFFSET] = 0;
    g_discrete_inputs[DI_VOLT_OVER_FAULT_OFFSET] = 0;
    g_discrete_inputs[DI_FREQ_OVER_FAULT_OFFSET] = 0;
}

static void alarm_update_outputs(void)
{
    if (g_discrete_inputs[DI_OVER_CURRENT_FAULT_OFFSET]) SET_PIN(IO_OVER_CURRENT_ALARM_PORT, IO_OVER_CURRENT_ALARM_PIN);
    else CLR_PIN(IO_OVER_CURRENT_ALARM_PORT, IO_OVER_CURRENT_ALARM_PIN);

    if (g_discrete_inputs[DI_WATER_PRESS_FAULT_OFFSET]) SET_PIN(IO_WATER_PRESS_ALARM_PORT, IO_WATER_PRESS_ALARM_PIN);
    else CLR_PIN(IO_WATER_PRESS_ALARM_PORT, IO_WATER_PRESS_ALARM_PIN);

    if (g_discrete_inputs[DI_WATER_TEMP_FAULT_OFFSET]) SET_PIN(IO_WATER_TEMP_ALARM_PORT, IO_WATER_TEMP_ALARM_PIN);
    else CLR_PIN(IO_WATER_TEMP_ALARM_PORT, IO_WATER_TEMP_ALARM_PIN);

    if (g_discrete_inputs[DI_PHASE_LOSS_FAULT_OFFSET]) SET_PIN(IO_PHASE_LOSS_ALARM_PORT, IO_PHASE_LOSS_ALARM_PIN);
    else CLR_PIN(IO_PHASE_LOSS_ALARM_PORT, IO_PHASE_LOSS_ALARM_PIN);

    if (g_discrete_inputs[DI_VOLT_OVER_FAULT_OFFSET]) SET_PIN(IO_GRID_ALARM_PORT, IO_GRID_ALARM_PIN);
    else CLR_PIN(IO_GRID_ALARM_PORT, IO_GRID_ALARM_PIN);

    if (g_discrete_inputs[DI_FREQ_OVER_FAULT_OFFSET]) SET_PIN(IO_FREQ_ALARM_PORT, IO_FREQ_ALARM_PIN);
    else CLR_PIN(IO_FREQ_ALARM_PORT, IO_FREQ_ALARM_PIN);

    if (g_alarm_any_fault) SET_PIN(IO_TOTAL_FAULT_PORT, IO_TOTAL_FAULT_PIN);
    else CLR_PIN(IO_TOTAL_FAULT_PORT, IO_TOTAL_FAULT_PIN);
}

static void alarm_update_run_indicator(void)
{
    unsigned int interval = g_alarm_any_fault ? 100 : 200;

    if ((unsigned int)(g_system_tick_5ms - g_run_blink_tick) >= interval) {
        g_run_blink_tick = g_system_tick_5ms;
        TOGGLE_PIN(IO_RUN_IND_PORT, IO_RUN_IND_PIN);
    }
}

void Alarm_Process(void)
{
    static unsigned char xdata last_start_state = 0;
    static unsigned int xdata start_tick = 0;
    unsigned char start_state;
    unsigned char delay_ok;

    /* 调试模式：屏蔽所有故障，不清除以便观察 */
    alarm_debug_log_io_changes(); /* log raw IO before early exits */
    if (IS_DEBUG_MODE()) {
        alarm_clear_latch();
        g_alarm_any_fault = 0;
        alarm_update_outputs();
        alarm_update_run_indicator();
        return;
    }

    /* 维修模式高电平有效：屏蔽所有故障，复位输出和地址 */
    if (MAINTENANCE_READ()) {
        alarm_clear_latch();
        g_alarm_any_fault = 0;
        alarm_update_outputs();
        alarm_update_run_indicator();
        return;
    }

    /* 故障复位按钮（0x0006 锁存复位） */
    if (g_coils[COIL_FAULT_RESET_OFFSET]) {
        alarm_clear_latch();
        g_coils[COIL_FAULT_RESET_OFFSET] = 0;
    }

    /* 工作状态由外部引脚P5.1透传 */
    g_discrete_inputs[DI_DEVICE_RUNNING_OFFSET] = DEVICE_RUNNING_READ();

    /* 启动状态检测（用于电压/频率 2s 延迟判断）：触屏线圈/按键/远控任一启动均算启动 */
    start_state = (g_coils[COIL_START_STOP_OFFSET] || g_key_running || g_remote_running) ? 1 : 0;
    if (start_state && !last_start_state) {
        start_tick = g_system_tick_5ms;
    }
    last_start_state = start_state;
    delay_ok = start_state && ((unsigned int)(g_system_tick_5ms - start_tick) >= 400);

    /* 电压超限（启动2s后判断） */
    if (delay_ok && ALARM_OVER_LIMIT(g_input_regs[REG_DC_VOLT_OFFSET],
        g_holding_regs[HLD_VOLT_LOW_OFFSET], g_holding_regs[HLD_VOLT_HIGH_OFFSET])) {
        g_discrete_inputs[DI_VOLT_OVER_FAULT_OFFSET] = 1;
    }

    /* 频率超限（启动2s后判断） */
    if (delay_ok && ALARM_OVER_LIMIT(g_input_regs[REG_WORK_FREQ_OFFSET],
        g_holding_regs[HLD_FREQ_LOW_OFFSET], g_holding_regs[HLD_FREQ_HIGH_OFFSET])) {
        g_discrete_inputs[DI_FREQ_OVER_FAULT_OFFSET] = 1;
    }

    /* 水温故障（四路温度任一超限） */
    if (ALARM_OVER_LIMIT(g_input_regs[REG_TEMP_T1_OFFSET], g_holding_regs[HLD_TEMP_LOW_OFFSET], g_holding_regs[HLD_TEMP_HIGH_OFFSET]) ||
        ALARM_OVER_LIMIT(g_input_regs[REG_TEMP_T2_OFFSET], g_holding_regs[HLD_TEMP_LOW_OFFSET], g_holding_regs[HLD_TEMP_HIGH_OFFSET]) ||
        ALARM_OVER_LIMIT(g_input_regs[REG_TEMP_T3_OFFSET], g_holding_regs[HLD_TEMP_LOW_OFFSET], g_holding_regs[HLD_TEMP_HIGH_OFFSET]) ||
        ALARM_OVER_LIMIT(g_input_regs[REG_TEMP_T4_OFFSET], g_holding_regs[HLD_TEMP_LOW_OFFSET], g_holding_regs[HLD_TEMP_HIGH_OFFSET])) {
        g_discrete_inputs[DI_WATER_TEMP_FAULT_OFFSET] = 1;
    }

    /* 过流故障（P5.0 低电平，光耦导通拉低） */
    if (!OVER_CURRENT_IN_READ()) g_discrete_inputs[DI_OVER_CURRENT_FAULT_OFFSET] = 1;

    /* 水压故障/缺水（P4.3 高电平） */
    if (WATER_LACK_READ()) g_discrete_inputs[DI_WATER_PRESS_FAULT_OFFSET] = 1;

    /* 缺相故障（P0.6 高电平） */
    if (PHASE_LOSS_DET_READ()) g_discrete_inputs[DI_PHASE_LOSS_FAULT_OFFSET] = 1;

    /* 总故障判断 */
    g_alarm_any_fault = g_discrete_inputs[DI_WATER_TEMP_FAULT_OFFSET] ||
        g_discrete_inputs[DI_WATER_PRESS_FAULT_OFFSET] ||
        g_discrete_inputs[DI_OVER_CURRENT_FAULT_OFFSET] ||
        g_discrete_inputs[DI_PHASE_LOSS_FAULT_OFFSET] ||
        g_discrete_inputs[DI_VOLT_OVER_FAULT_OFFSET] ||
        g_discrete_inputs[DI_FREQ_OVER_FAULT_OFFSET];

    alarm_update_outputs();
    alarm_update_run_indicator();
}
