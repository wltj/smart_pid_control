
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
	
	IE2 = 0x1;
	EA = 1;    //开中断
}

void UART2_Isr() interrupt 8 
{
	if (S2CON & 0x01) 
	{
		S2CON &= ~0x01;                         
    if (g_need_process_datas) return;              						//如果正在处理数据，则不接收
		if (g_recv_buffer_index >= 64) g_recv_buffer_index = 0;  	//溢出处理
		g_recv_buffer[g_recv_buffer_index++] = S2BUF;   					//将接收到的数据保存到 接收缓冲区
		g_current_need_times = g_need_times;           						//收到数据，则重新计算超时时间			
	}
}

void send_buffer(unsigned char *buf,int len)									//发送数据
{
	RS485_SEND_EN();					//设置485为发送状态
	Delay5ms();
	while (len--) {
		S2CON &= ~S2TI;     					//Clear transmit interrupt flag
		S2BUF = *buf++;
		while ((S2CON & S2TI) == 0);
	}
	Delay5ms();
	RS485_RECV_EN();   			//设置485为接收状态
}

void Timer0Init(void)						//5毫秒@11.0592MHz
{
	AUXR |= 0x80;									//定时器时钟1T模式
	TMOD &= 0xF0;									//设置定时器模式
	TL0 = 0x00;		 
	TH0 = 0x28;		 
	TF0 = 0;											//清除TF0标志
	TR0 = 1;											//定时器0开始计时
	
	ET0 = 1;
}

void tm0_isr() interrupt 1
{
	static int count = 0;
	
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
	
	Timer0Init();              									//定时器0用于485串口接收超时判断

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
