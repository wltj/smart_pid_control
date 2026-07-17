#include "alarm_manager.h"
#include "board.h"
#include "modbus_reg_config.h"
#include "run_control.h"

volatile unsigned char xdata g_coils[COIL_COUNT] = {0};
volatile unsigned char xdata g_discrete_inputs[DI_COUNT] = {0};
volatile unsigned short xdata g_input_regs[INPUT_REG_COUNT] = {0};
volatile unsigned short xdata g_holding_regs[HOLDING_REG_COUNT] = {0};

#define ALARM_OVER_LIMIT(value, low, high) (((low) != 0 && (value) < (low)) || ((high) != 0 && (value) > (high)))

static unsigned int xdata g_run_blink_tick = 0;
static unsigned char xdata g_alarm_any_fault = 0;

extern volatile unsigned int xdata g_system_tick_5ms;

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

    // ========== 输入引脚 ==========
    // 1. 过流输入 P5.0：由高阻改为准双向口，并写1
    P5M0 &= ~BIT(IO_OVER_CURRENT_IN_PIN);                     // 00 准双向
    P5M1 &= ~BIT(IO_OVER_CURRENT_IN_PIN);                     // 00 准双向
    PIN_SET(IO_OVER_CURRENT_IN_PORT, IO_OVER_CURRENT_IN_PIN); // P5.0 = 1

    // 2. 缺水输入 P4.3：由高阻改为准双向口，并写1
    P4M0 &= ~BIT(IO_WATER_LACK_PIN);
    P4M1 &= ~BIT(IO_WATER_LACK_PIN);
    PIN_SET(IO_WATER_LACK_PORT, IO_WATER_LACK_PIN);

    // 3. 维修输入 P4.1：由高阻改为准双向口，并写1
    P4M0 &= ~BIT(IO_MAINTENANCE_PIN);
    P4M1 &= ~BIT(IO_MAINTENANCE_PIN);
    PIN_SET(IO_MAINTENANCE_PORT, IO_MAINTENANCE_PIN);

    // 4. 缺相输入 P0.6：高阻，外部已经上拉
    P0M0 &= ~BIT(IO_PHASE_LOSS_DET_PIN);
    P0M1 |= BIT(IO_PHASE_LOSS_DET_PIN);

    /* 注意：P2.1 工作状态输入原本就是准双向口（默认），但同样需要写1，确保弱上拉有效 */
    PIN_SET(IO_WORK_STATUS_IND_PORT, IO_WORK_STATUS_IND_PIN); // P2.1 = 1

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

/*============================================================================
 * 外部数字输入消抖
 * 每 5ms 采样一次，连续 10 次相同读数（50ms）才更新稳定状态，
 * 过滤继电器/接触器抖动和干扰毛刺。
 * 注意：缺相检测见 phase_loss_scan()，因需捕获 10ms 短脉冲。
 *============================================================================*/
#define DI_SAMPLE_TICKS   1U    /* 采样间隔 1 × 5ms = 5ms */
#define DI_STABLE_COUNT   10U   /* 稳定阈值 10 × 5ms = 50ms */

/* 消抖通道索引（不含缺相，缺相走专用脉冲捕获） */
#define DI_CH_MAINTENANCE   0
#define DI_CH_DEV_RUNNING   1
#define DI_CH_OVER_CURRENT  2
#define DI_CH_WATER_LACK    3
#define DI_CH_COUNT         4

static uint8_t  xdata g_di_stable[DI_CH_COUNT] = {0};
static uint8_t  xdata g_di_counter[DI_CH_COUNT] = {0};
static unsigned int xdata g_di_last_tick = 0;
static uint8_t  xdata g_di_inited = 0;

static void di_debounce_scan(void)
{
    uint8_t raw[DI_CH_COUNT];
    uint8_t i;
    unsigned int now;

    now = g_system_tick_5ms;
    if ((unsigned int)(now - g_di_last_tick) < DI_SAMPLE_TICKS)
        return;
    g_di_last_tick = now;

    raw[DI_CH_MAINTENANCE]  = MAINTENANCE_READ()     ? 1 : 0;
    raw[DI_CH_DEV_RUNNING]  = DEVICE_RUNNING_READ()  ? 1 : 0;
    raw[DI_CH_OVER_CURRENT] = OVER_CURRENT_IN_READ() ? 1 : 0;
    raw[DI_CH_WATER_LACK]   = WATER_LACK_READ()      ? 1 : 0;

    /* 首次调用：用当前读数直接初始化稳定状态，避免上电误判 */
    if (!g_di_inited) {
        for (i = 0; i < DI_CH_COUNT; i++) {
            g_di_stable[i] = raw[i];
            g_di_counter[i] = 0;
        }
        g_di_inited = 1;
        return;
    }

    for (i = 0; i < DI_CH_COUNT; i++) {
        if (raw[i] == g_di_stable[i]) {
            g_di_counter[i] = 0;
        } else {
            if (++g_di_counter[i] >= DI_STABLE_COUNT) {
                g_di_stable[i] = raw[i];
                g_di_counter[i] = 0;
            }
        }
    }
}

/*============================================================================
 * 缺相检测（专用脉冲捕获逻辑）
 * PHASE_LOSS_DET_READ 引脚特性：
 *   - 持续高电平：至少两相同时缺相
 *   - 输出~10ms 高电平脉冲：某一相接触不良、时通时断
 *   - 稳定常低：三相供电正常
 * 检测策略：每 5ms 采样，连续 2 次高（≥10ms）即判定缺相；
 *           连续 40 次低（200ms）才判定恢复正常。
 *           既捕获 10ms 短脉冲，又过滤 <5ms 干扰毛刺。
 *============================================================================*/
#define PHASE_LOSS_HIGH_THRESH  2U    /* 连续高 2×5ms=10ms 触发缺相 */
#define PHASE_LOSS_LOW_CLEAR    40U   /* 连续低 40×5ms=200ms 判定恢复 */

static uint8_t  xdata g_phase_high_cnt = 0;
static uint8_t  xdata g_phase_low_cnt = 0;
static uint8_t  xdata g_phase_loss_active = 0;
static unsigned int xdata g_phase_last_tick = 0;
static uint8_t  xdata g_phase_inited = 0;

static void phase_loss_scan(void)
{
    unsigned int now;
    uint8_t pin;

    now = g_system_tick_5ms;
    if ((unsigned int)(now - g_phase_last_tick) < 1U)
        return;
    g_phase_last_tick = now;

    pin = PHASE_LOSS_DET_READ() ? 1 : 0;

    /* 首次调用：用当前读数初始化，避免上电误判 */
    if (!g_phase_inited) {
        g_phase_loss_active = pin;
        g_phase_inited = 1;
        return;
    }

    if (pin) {
        /* 高电平：累计连续高采样，达到阈值即锁存缺相 */
        if (++g_phase_high_cnt >= PHASE_LOSS_HIGH_THRESH) {
            g_phase_loss_active = 1;
        }
        g_phase_low_cnt = 0;
    } else {
        /* 低电平：累计连续低采样，稳定 200ms 后清除缺相状态 */
        if (++g_phase_low_cnt >= PHASE_LOSS_LOW_CLEAR) {
            g_phase_loss_active = 0;
        }
        g_phase_high_cnt = 0;
    }
}

void Alarm_Process(void)
{
    static unsigned char xdata last_start_state = 0;
    static unsigned int xdata start_tick = 0;
    unsigned char start_state;
    unsigned char delay_ok;

    /* 数字输入消抖采样（5ms 一次，50ms 稳定确认） */
    di_debounce_scan();
    /* 缺相脉冲捕获（5ms 采样，捕获 10ms 短脉冲） */
    phase_loss_scan();

    /* 调试模式：屏蔽所有故障，不清除以便观察 */
    if (IS_DEBUG_MODE()) {
        alarm_clear_latch();
        g_alarm_any_fault = 0;
        alarm_update_outputs();
        alarm_update_run_indicator();
        return;
    }

    /* 维修信号：屏蔽所有故障，复位输出和地址（消抖后） */
    if (!g_di_stable[DI_CH_MAINTENANCE]) {
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

    /* 工作状态由外部引脚P2.1决定：待机=1,加热=0，DI取反（消抖后） */
    g_discrete_inputs[DI_DEVICE_RUNNING_OFFSET] = g_di_stable[DI_CH_DEV_RUNNING];

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

    /* 过流故障（P5.0 高电平，消抖后） */
    if (g_di_stable[DI_CH_OVER_CURRENT]) g_discrete_inputs[DI_OVER_CURRENT_FAULT_OFFSET] = 1;

    /* 水压故障/缺水（P4.3 高电平，消抖后） */
    if (g_di_stable[DI_CH_WATER_LACK]) g_discrete_inputs[DI_WATER_PRESS_FAULT_OFFSET] = 1;

    /* 缺相故障（P0.6：持续高=两相缺相，10ms脉冲=接触不良，脉冲捕获） */
    if (g_phase_loss_active) g_discrete_inputs[DI_PHASE_LOSS_FAULT_OFFSET] = 1;

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
