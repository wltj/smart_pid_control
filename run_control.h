#ifndef _RUN_CONTROL_H_
#define _RUN_CONTROL_H_

#include <board.h>

/* 外部启动状态，由按键扫描和远程端子扫描维护。 */
extern volatile uint8_t g_key_running;
extern volatile uint8_t g_remote_running;

/* 扫描远程启停输入：短按切换，长按停止。 */
void Remote_Control_Scan(void);

/*
 * 主运行控制循环。
 * elapsed_ms 为距离上次调用的实际时间，用于 PID 采样周期和分段计时。
 */
void Run_Control_Loop(uint16_t elapsed_ms);

#endif
