#ifndef _MODBUS_COMM_H_
#define _MODBUS_COMM_H_

#include "board.h"
#include "modbus_reg_config.h"

/*=========================================================================
  RS485 全局变量
=========================================================================*/
extern unsigned char xdata g_recv_buffer[64];
extern int xdata g_recv_buffer_index;
extern int xdata g_need_times;            //超时定时时间 10 x 5ms
extern int xdata g_current_need_times;     //超时实际时间
extern volatile unsigned char xdata g_need_process_datas;   //需要处理数据标志

/*=========================================================================
  函数声明
=========================================================================*/
/* 初始化串口2, 9600,8,n,1 (RS485 Modbus) */
void init_485_uart(void);

/* 485发送数据 */
void send_buffer(unsigned char *buf, int len);

/* 保持寄存器掉电保存（周期性写入EEPROM） */
void modbus_holding_save_process(void);

#endif /* _MODBUS_COMM_H_ */
