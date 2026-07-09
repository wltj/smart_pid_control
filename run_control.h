#ifndef _RUN_CONTROL_H_
#define _RUN_CONTROL_H_
#include <board.h>

// 按键直接启动标志：物理启停按键置位，无视 HLD_CTRL_MODE_OFFSET
// 直接按 HLD_CHANGE_MODE_OFFSET 控制方式运行（等同触屏模式控制）
extern volatile uint8_t g_key_running;

/*============================================================================
 * 控制逻辑主循环（需周期性调用，默认每100ms）
 *
 * 触屏控制(HLD_CTRL_MODE_OFFSET=1)时才输出PWM并启动PID，
 * 否则只刷新状态数据、不启动PID。
 * 启停由 COIL_START_STOP_OFFSET 控制（启动=1, 停止=0）。
 * 控制方式由 HLD_CHANGE_MODE_OFFSET 选择：
 *   0 = 默认，按目标功率(HLD_POWER_SETPOINT_OFFSET) PID控制
 *   1 = 分段加热，按选中工件(HLD_CUR_WORK_NO_OFFSET)的功率/时间曲线加热
 *   2 = 温度加热，按目标温度(HLD_TARGET_TEMP_OFFSET) 恒温PID控制
 *============================================================================*/
void Run_Control_Loop(void);

#endif /* _RUN_CONTROL_H_ */
