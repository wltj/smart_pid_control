#ifndef _ALARM_MANAGER_H_
#define _ALARM_MANAGER_H_

/* 初始化报警相关 GPIO 引脚模式 */
void alarm_gpio_init(void);

/* 报警主处理函数（需在主循环中周期性调用） */
void Alarm_Process(void);

#endif /* _ALARM_MANAGER_H_ */
