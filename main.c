
#include <stdio.h>
#include <stdlib.h>
#include "modbus.h"
#include "debugUart.h"
#include "eeprom.h"
#include "board.h"
#include <intrins.h>
#include <STC8A8K64D4.H>

#define S2RI  0x01          //S2CON.0
#define S2TI  0x02          //S2CON.1
#define ES2   0x01          //IE2.0

#define PCA_5MS_RELOAD_H  0xEE
#define PCA_5MS_RELOAD_L  0x00

#if UART_RS485_NUM != 2
#error "init_485_uart only supports UART2"
#endif

//485接收缓冲区
unsigned char xdata g_recv_buffer[64];
int xdata g_recv_buffer_index = 0;
int xdata g_need_times = 10;            //超时定时时间 10 x 5ms
int xdata g_current_need_times = 0;     //超时实际时间
volatile unsigned char xdata g_need_process_datas = 0;   //需要处理数据标志
extern unsigned char dev_address;          //设备地址，默认为1

volatile unsigned int xdata g_system_tick_5ms = 0;

extern int xdata modbus_relay_delay_time[16];
extern unsigned char xdata modbus_relay_delay_time_eeprom[16];

extern char xdata msg[64];

unsigned char key_press[16];

volatile unsigned char flag_1s = 0;         //定时器计时标志

void Delay5ms()		//@11.0592MHz
{
	unsigned char i, j;

	_nop_();
	_nop_();
	i = 72;
	j = 205;
	do
	{
		while (--j);
	} while (--i);
}


void Delay10ms()		//@11.0592MHz
{
	unsigned char i, j;

	_nop_();
	_nop_();
	i = 144;
	j = 157;
	do
	{
		while (--j);
	} while (--i);
}


void Delay20ms()		//@11.0592MHz
{
	unsigned char i, j, k;

	i = 2;
	j = 32;
	k = 60;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}


void init_io()
{
	//设置io为 准双向口
	P0M1 = 0x0;    	P0M0 = 0x0;
	P1M1 = 0xc8;   	P1M0 = 0x0;
	P2M1 = 0x0;   	P2M0 = 0x0;
	P3M1 = 0x0;   	P3M0 = 0x0;
	P4M1 = 0x0;     P4M0 = 0x0;
	P5M1 = 0x0;   	P5M0 = 0x0;

	RS485_RECV_EN();   //设置485为接收状态
	
}

void init_485_uart()   //9600@11.0592  //初始化串口2, 9600,8,n,1
{
	S2CON = 0x50;
	AUXR |= 0x04;			
	T2L = 0xE0;				
	T2H = 0xFE;				
	AUXR |= 0x10;			
	
	IE2 |= ES2;
	EA = 1;    //开中断
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

void send_buffer(unsigned char *buf,int len)
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

static void Pca5msTimerInit(void)						//5毫秒@11.0592MHz
{
	CR = 0;
	CCON = 0;
	CMOD = 0x01;									//PCA时钟Fosc/12，允许PCA溢出中断
	CL = PCA_5MS_RELOAD_L;
	CH = PCA_5MS_RELOAD_H;
	CR = 1;
}

static void Timer0_Counter_Init(void)
{
	TR0 = 0;
	ET0 = 0;
	P3M0 &= ~BIT(IO_PULSE_CNT_PIN);
	P3M1 |= BIT(IO_PULSE_CNT_PIN);  // P3.4/T0 高阻输入，硬件下降沿计数

	TMOD &= 0xF0;
	TMOD |= 0x05;  // T0: 16位外部计数器模式，P3.4/T0 下降沿计数
	TL0 = 0x00;
	TH0 = 0x00;
	TF0 = 0;
	TR0 = 1;
}

void pca_isr() interrupt 7
{
	static int count = 0;

	if (!CF)
	{
		return;
	}

	CF = 0;
	CL = PCA_5MS_RELOAD_L;
	CH = PCA_5MS_RELOAD_H;
	g_system_tick_5ms++;
	
	if (g_recv_buffer_index > 0 && g_need_process_datas == 0) 
	{
    if (g_current_need_times-- == 0) 		//定时时间到了
		{      
			g_need_process_datas = 1;         //需要处理数据		
		}		
	}	
	
	if (count++ >= 200) 
	{
		count = 0;
		flag_1s = 1;              				//设置1秒定时标志位
	}
}

//系统工作于11.0592MHZ，使用内部晶振
void init_system()														//系统初始化
{
	unsigned char temp_dev = 0;
	int i = 0;
	
	g_recv_buffer_index = 0;              			//设置接收缓冲区为空
	g_current_need_times = g_need_times;  			//间隔超时时间设置
	g_need_process_datas = 0;             			//设置 需要处理数据 标志位 为0
	Delay20ms();                          			//延时20ms
	init_io();   							 									//初始化io
	init_debug_uart();         									//初始化调试串口1,9600,8，n,1
	init_485_uart();  				 									//初始化串口2, 9600,8,n,1
	adc_sensor_init(); 														//初始化ADC
	uart_adc_init(); 																//初始化ADC串口
	Timer0_Counter_Init();              						//定时器0用于P3.4/T0外部脉冲计数
	Pca5msTimerInit();              							//PCA用于485串口接收5ms超时判断

//	send_buffer("run...\r\n",8);   						//调试用
//	debug_out("485 system run ... \r\n\0");  	//调试串口输出 调试信息
	WDT_CONTR = 0x26;          								//开启看门狗   
}


void loop_system()																						//系统运行处理
{
	if (g_need_process_datas) 																	//有数据需要处理，则处理
	{       											         
		//debug_out("process msg\r\n\0");
		if (parse_recv_buffer(g_recv_buffer,g_recv_buffer_index)) //处理数据
		{    
			//send_buffer(g_recv_buffer,g_recv_buffer_index);
	//		debug_out("parse_recv_buffer() ok\r\n\0");
		}else
		{
			adc_sensor_read_all();  //读取ADC数据
			// uart_adc_get_voltage();  //读取ADC电压数据
			// uart_adc_get_current();  //读取ADC电流数据
//			debug_out("parse_recv_buffer() error\r\n\0");
		}

		g_recv_buffer_index = 0;
		g_need_process_datas = 0;                                	 //服务标志清空				
	}
	WDT_CONTR = 0x36;                                            //喂狗
}

int main(void)
{
	init_system();    //初始化系统
	while (1) 
	{
		loop_system();  //系统运行处理
	}
  return 0;	
}
