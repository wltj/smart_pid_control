#include "modbus_comm.h"
#include "eeprom.h"
#include <STC8A8K64D4.H>

#define S2RI  0x01          //S2CON.0
#define S2TI  0x02          //S2CON.1
#define ES2   0x01          //IE2.0

/*=========================================================================
  RS485 全局变量定义
=========================================================================*/
unsigned char xdata g_recv_buffer[64];
int xdata g_recv_buffer_index = 0;
int xdata g_need_times = 10;            //超时定时时间 10 x 5ms
int xdata g_current_need_times = 0;     //超时实际时间
volatile unsigned char xdata g_need_process_datas = 0;   //需要处理数据标志

/* main.c 中定义 */
extern void Delay5ms(void);
extern volatile unsigned int xdata g_system_tick_5ms;

/*=========================================================================
  初始化串口2, 9600,8,n,1 (RS485 Modbus)
=========================================================================*/
void init_485_uart(void)
{
	S2CON = 0x50;
	AUXR |= 0x04;
	T2L = 0xE0;
	T2H = 0xFE;
	AUXR |= 0x10;

	IE2 |= ES2;
	EA = 1;    //开中断
}

/*=========================================================================
  UART2 中断服务：RS485 接收 + 超时判断
=========================================================================*/
void UART2_Isr() interrupt 8
{
	if (S2CON & S2RI)
	{
		S2CON &= ~S2RI;
		if (g_need_process_datas) return;
		if (g_recv_buffer_index >= 64) g_recv_buffer_index = 0;
		g_recv_buffer[g_recv_buffer_index++] = S2BUF;
		g_current_need_times = g_need_times;
	}
	if (S2CON & S2TI)
	{
		S2CON &= ~S2TI;
	}
}

/*=========================================================================
  485发送数据
=========================================================================*/
void send_buffer(unsigned char *buf, int len)
{
	unsigned char ie2_backup;

	ie2_backup = IE2;
	IE2 &= ~ES2;
	RS485_SEND_EN();
	Delay5ms();
	while (len--) {
		S2CON &= ~S2TI;
		S2BUF = *buf++;
		while ((S2CON & S2TI) == 0);
	}
	S2CON &= ~S2TI;
	Delay5ms();
	RS485_RECV_EN();
	IE2 = ie2_backup;
}

/*=========================================================================
  保持寄存器掉电保存（周期性写入EEPROM）
=========================================================================*/
void modbus_holding_save_process(void)
{
	static unsigned int xdata last_save_tick = 0;
	/* 每60秒保存一次保持寄存器到EEPROM */
	if ((unsigned int)(g_system_tick_5ms - last_save_tick) >= 12000) {
		last_save_tick = g_system_tick_5ms;
		save_params((unsigned char xdata *)g_holding_regs, HOLDING_REG_COUNT * 2);
	}
}
