#include "modbus_comm.h"
#include "eeprom.h"
#include <STC8A8K64D4.H>

#define S2RI  0x01
#define S2TI  0x02
#define ES2   0x01

unsigned char xdata g_recv_buffer[64];
int xdata g_recv_buffer_index = 0;
int xdata g_need_times = 10;
int xdata g_current_need_times = 0;
volatile unsigned char xdata g_need_process_datas = 0;
volatile unsigned char xdata g_holding_regs_dirty = 0;

extern void Delay5ms(void);
extern volatile unsigned int xdata g_system_tick_5ms;

void init_485_uart(void)
{
	S2CON = 0x50;
	AUXR |= 0x04;
	T2L = 0xE0;
	T2H = 0xFE;
	AUXR |= 0x10;

	IE2 |= ES2;
	EA = 1;
}

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

void modbus_holding_save_process(void)
{
	static unsigned int xdata last_save_tick = 0;
	unsigned char need_save;

	need_save = 0;
	if (g_holding_regs_dirty) {
		need_save = 1;
	} else if ((unsigned int)(g_system_tick_5ms - last_save_tick) >= 1200) {
		need_save = 1;
	}

	if (need_save) {
		last_save_tick = g_system_tick_5ms;
		g_holding_regs[HLD_PARAM_MAGIC_OFFSET] = HLD_PARAM_MAGIC_VALUE;
		if (save_params((unsigned char xdata *)g_holding_regs, HOLDING_REG_COUNT * 2) == 1) {
			g_holding_regs_dirty = 0;
		}
	}
}
