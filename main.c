
#include <stdio.h>
#include <stdlib.h>
#include "modbus.h"
#include "modbus_comm.h"
#include "debugUart.h"
#include "eeprom.h"
#include "board.h"
#include "modbus_reg_config.h"
#include "pid_drive.h"
#include "alarm_manager.h"
#include <intrins.h>
#include <STC8A8K64D4.H>

#define PCA_5MS_RELOAD_H  0xEE
#define PCA_5MS_RELOAD_L  0x00

#if UART_RS485_NUM != 2
#error "init_485_uart only supports UART2"
#endif

/*=========================================================================
  全局变量
=========================================================================*/
volatile unsigned int xdata g_system_tick_5ms = 0;

extern unsigned char dev_address;          //设备地址，默认为1
extern char xdata msg[64];

unsigned char key_press[16];
volatile unsigned char flag_1s = 0;         //定时器计时标志

/*=========================================================================
  按键与调试相关定义
=========================================================================*/
#define KEY_DEBOUNCE_TICKS 4  // 4 × 5ms = 20ms 消抖

/* 调试功能开关：使用未使用的线圈0控制，1=开启调试模式，0=正常模式 */
#define DEBUG_MODE_COIL 0
#define IS_DEBUG_MODE() (DEBUG_MODE_COIL || (g_coils[DEBUG_MODE_COIL] == 1))

static unsigned int xdata g_last_key_tick = 0;
static unsigned char xdata g_start_key_state = 1;
static unsigned char xdata g_stop_key_state = 1;

/* pid_drive.c 中定义，用于刷新输入寄存器 */
extern void Modbus_Input_Reg_Update(void);

static unsigned char holding_regs_is_erased(void)
{
	unsigned int i;

	for (i = 0; i < HOLDING_REG_COUNT; i++) {
		if (g_holding_regs[i] != 0xFFFF) {
			return 0;
		}
	}

	return 1;
}

static void holding_regs_default_init(void)
{
	unsigned int i;

	for (i = 0; i < HOLDING_REG_COUNT; i++) {
		g_holding_regs[i] = 0;
	}
	g_holding_regs[HLD_PARAM_MAGIC_OFFSET] = HLD_PARAM_MAGIC_VALUE;
}

static void holding_regs_load(void)
{
	read_params((unsigned char xdata *)g_holding_regs, HOLDING_REG_COUNT * 2);

	if (g_holding_regs[HLD_PARAM_MAGIC_OFFSET] == HLD_PARAM_MAGIC_VALUE) {
		return;
	}

	if (holding_regs_is_erased()) {
		holding_regs_default_init();
	} else {
		g_holding_regs[HLD_PARAM_MAGIC_OFFSET] = HLD_PARAM_MAGIC_VALUE;
	}
}

/*=========================================================================
  延时函数
=========================================================================*/
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

/*=========================================================================
  调试串口输出（UART1）
=========================================================================*/
static void debug_uart_puts(char *str)
{
	while (*str != '\0') {
		TI = 0;
		SBUF = *str++;
		while (TI == 0);
	}
}

/*=========================================================================
  地址测试初始化：给每个寄存器写入特征值，方便屏幕核对地址
=========================================================================*/
static void debug_address_test_init(void)
{
	uint8_t i;
	// 输入寄存器：偏移x写入值为 1000 + x
	for (i = 0; i < INPUT_REG_COUNT; i++) {
		g_input_regs[i] = 1000 + i;
	}
	// 保持寄存器：偏移x写入值为 2000 + x（不覆盖已设置的参数）
	for (i = 0; i < HOLDING_REG_COUNT; i++) {
		if (g_holding_regs[i] == 0 || g_holding_regs[i] == 0xFFFF) {
			g_holding_regs[i] = 2000 + i;
		}
	}
	// 离散输入：偏移x写入1
	for (i = 0; i < DI_COUNT; i++) {
		g_discrete_inputs[i] = 1;
	}
	for (i = 0; i < HOLDING_REG_COUNT; i++) {
		if (g_holding_regs[i] == 0 || g_holding_regs[i] == 0xFFFF) {
			g_holding_regs[i] = 2000 + i;
		}
	}
	debug_uart_puts("Debug: Address test data init done\r\n");
}

/*=========================================================================
  模拟数据更新：生成动态传感器数据，不需要外设就能测试屏幕显示
=========================================================================*/
static void debug_simulate_data_update(void)
{
	static uint16_t cnt = 0;
	cnt++;
	// 模拟工作频率：30~50Hz动态变化
	g_input_regs[REG_WORK_FREQ_OFFSET] = 30 + (cnt % 20);
	// 模拟直流电压：300~320V动态变化
	g_input_regs[REG_DC_VOLT_OFFSET] = 3000 + (cnt % 200);
	// 模拟直流电流：5~15A动态变化
	g_input_regs[REG_DC_CURR_OFFSET] = 50 + (cnt % 100);
	// 模拟实时功率：30%~80%动态变化
	g_input_regs[REG_REAL_POWER_OFFSET] = 30 + (cnt % 50);
	// 模拟温度T1-T4：25~100℃动态变化
	g_input_regs[REG_TEMP_T1_OFFSET] = 250 + (cnt % 750);
	g_input_regs[REG_TEMP_T2_OFFSET] = 300 + (cnt % 700);
	g_input_regs[REG_TEMP_T3_OFFSET] = 350 + (cnt % 650);
	g_input_regs[REG_TEMP_T4_OFFSET] = 400 + (cnt % 600);
	// 模拟外部ADC输入
	g_input_regs[REG_EXT_ADC1_OFFSET] = 100 + (cnt % 400);
	g_input_regs[REG_EXT_ADC2_OFFSET] = 200 + (cnt % 300);
	g_input_regs[REG_WORK_FREQ_OFFSET] = 30 + (cnt % 20);
	g_input_regs[REG_DC_VOLT_OFFSET] = 3000 + (cnt % 200);
	g_input_regs[REG_DC_CURR_OFFSET] = 50 + (cnt % 100);
	g_input_regs[REG_REAL_POWER_OFFSET] = 30 + (cnt % 50);
	g_input_regs[REG_TEMP_T1_OFFSET] = 250 + (cnt % 750);
}

/*=========================================================================
  I/O口模式初始化
=========================================================================*/
void gpio_init(void)
{
	/* 默认所有端口为准双向口 */
	P0M0 = 0x00; P0M1 = 0x00;
	P1M0 = 0x00; P1M1 = 0x00;
	P2M0 = 0x00; P2M1 = 0x00;
	P3M0 = 0x00; P3M1 = 0x00;
	P4M0 = 0x00; P4M1 = 0x00;
	P5M0 = 0x00; P5M1 = 0x00;

	/* 串口接收引脚高阻输入：P0.0(UART3 RX), P0.2(UART4 RX) */
	P0M0 &= ~(BIT(0) | BIT(2));
	P0M1 |= (BIT(0) | BIT(2));

	/* 启动/停止按钮引脚高阻输入 */
	P3M0 &= ~(BIT(IO_START_IN_PIN) | BIT(IO_STOP_IN_PIN));
	P3M1 |= (BIT(IO_START_IN_PIN) | BIT(IO_STOP_IN_PIN));

	/* 报警相关引脚模式 */
	alarm_gpio_init();

	RS485_RECV_EN();   //设置485为接收状态
}

/*=========================================================================
  PCA 5ms定时器初始化
=========================================================================*/
static void Pca5msTimerInit(void)						//5毫秒@11.0592MHz
{
	CR = 0;
	CCON = 0;
	CMOD = 0x01;									//PCA时钟Fosc/12，允许PCA溢出中断
	CL = PCA_5MS_RELOAD_L;
	CH = PCA_5MS_RELOAD_H;
	CR = 1;
}

/*=========================================================================
  Timer0 外部脉冲计数初始化（P3.4下降沿计数，用于频率测量）
=========================================================================*/
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

/*=========================================================================
  频率测量更新：1秒闸门读取Timer0计数值，结果即为频率(Hz)
  量程 0~50000Hz（16位计数器最大65535，留有余量）
=========================================================================*/
#define FREQ_GATE_TICKS  40   // 1秒闸门 = 200 × 5ms

static void Freq_Measure_Update(void)
{
	static unsigned int xdata last_tick = 0;
	unsigned int now_tick;
	unsigned int count;

	now_tick = g_system_tick_5ms;
	if ((unsigned int)(now_tick - last_tick) < FREQ_GATE_TICKS) {
		return;
	}
	last_tick += FREQ_GATE_TICKS;

	TR0 = 0;                                    //暂停计数，读取计数值
	count = ((unsigned int)TH0 << 8) | TL0;
	TL0 = 0x00;                                 //清零计数器，开始下一周期
	TH0 = 0x00;
	TF0 = 0;
	TR0 = 1;                                    //恢复计数

	g_input_regs[REG_WORK_FREQ_OFFSET] = count*5; //1秒闸门，计数值即为频率(Hz)
}

/*=========================================================================
  PCA中断服务：5ms系统滴答 + 485超时判断 + 1s标志
=========================================================================*/
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

/*=========================================================================
  按钮扫描（含消抖，与 Modbus 0x0001 功能一致）
=========================================================================*/
static void key_scan(void)
{
	unsigned char raw;

	if ((unsigned int)(g_system_tick_5ms - g_last_key_tick) < KEY_DEBOUNCE_TICKS)
		return;
	g_last_key_tick = g_system_tick_5ms;

	/* 启动按钮 P3.7，低电平有效 */
	raw = START_IN_READ() ? 1 : 0;
	if (raw != g_start_key_state) {
		g_start_key_state = raw;
		if (raw == 0) {
			g_coils[COIL_START_STOP_OFFSET] = 1;  /* 启动 */
		}
	}

	/* 停止按钮 P3.6，低电平有效 */
	raw = STOP_IN_READ() ? 1 : 0;
	if (raw != g_stop_key_state) {
		g_stop_key_state = raw;
		if (raw == 0) {
			g_coils[COIL_START_STOP_OFFSET] = 0;  /* 停止 */
		}
	}
}

/*=========================================================================
  系统初始化
=========================================================================*/
void system_init(void)
{
	g_recv_buffer_index = 0;              			//设置接收缓冲区为空
	g_current_need_times = g_need_times;  			//间隔超时时间设置
	g_need_process_datas = 0;             			//设置 需要处理数据 标志位 为0
	Delay20ms();                          			//延时20ms

	gpio_init();   							 	//初始化IO（含报警引脚）
	init_debug_uart();         						//初始化调试串口1,9600,8，n,1
	debug_uart_puts("System Init\r\n");
	holding_regs_load();

	init_485_uart();  				 				//初始化串口2, 9600,8,n,1 (RS485 Modbus)
	Timer0_Counter_Init();              			//定时器0用于P3.4/T0外部脉冲计数
	Pca5msTimerInit();              				//PCA用于5ms系统滴答和485超时判断

	PID_Hardware_Init();							//初始化PWM、ADC、串口ADC
	PID_Init(&g_temp_pid, 1024, 0, 0, 0, 102400, 102400, 0);
	PID_Init(&g_power_pid, 1024, 0, 0, 0, 102400, 102400, 0);

	if (IS_DEBUG_MODE()) {
		debug_address_test_init(); 					//上电默认初始化地址测试数据
	}

	WDT_CONTR = 0x26;          						//开启看门狗
}

/*=========================================================================
  系统主循环
=========================================================================*/
void system_loop(void)
{
	static unsigned int xdata last_pid_tick = 0;
	static uint8_t debug_init_flag = 0;
	unsigned int now_tick;

	now_tick = g_system_tick_5ms;

	/* 调试模式切换处理 */
	if (IS_DEBUG_MODE() && !debug_init_flag) {
		debug_address_test_init();
		debug_init_flag = 1;
		debug_uart_puts("Debug: Mode enabled\r\n");
	} else if (!IS_DEBUG_MODE() && debug_init_flag) {
		debug_init_flag = 0;
		debug_uart_puts("Debug: Mode disabled\r\n");
	}

	/* Modbus通信处理 */
	if (g_need_process_datas) {
		parse_recv_buffer(g_recv_buffer, g_recv_buffer_index);
		g_recv_buffer_index = 0;
		g_need_process_datas = 0;
	}

	/* 保持寄存器掉电保存 */
	modbus_holding_save_process();

	/* 频率测量（1秒闸门，P3.4/T0下降沿计数） */
	Freq_Measure_Update();

	/* 输入寄存器刷新（调试模式用模拟数据） */
	if (IS_DEBUG_MODE()) {
		debug_simulate_data_update();
	} else {
		Modbus_Input_Reg_Update();
	}

	/* 按键扫描 */
	key_scan();

	/* 报警及外部输入处理 */
	Alarm_Process();

	/* PID运算（每100ms执行一次 = 20 × 5ms） */
	if ((unsigned int)(now_tick - last_pid_tick) >= 20)
	{
		last_pid_tick = now_tick;
		PID_RunLoop();
	}

	WDT_CONTR = 0x36;                            //喂狗
}

/*=========================================================================
  主函数
=========================================================================*/
void main(void)
{
	system_init();
	while (1) {
		system_loop();
	}
}
