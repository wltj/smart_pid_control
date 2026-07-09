#ifndef _BOARD_H_
#define _BOARD_H_

// 手动定义标准固定宽度类型，解决Keil C51不识别<stdint.h>报错
typedef unsigned char  uint8_t;
typedef signed char    int8_t;
typedef unsigned int   uint16_t;
typedef signed int     int16_t;
typedef unsigned long  uint32_t;
typedef signed long    int32_t;

// STC8A8K64D4 寄存器定义头文件
#include "stc8a8k64d4.h"

/*=========================================================================
  底层通用IO操作宏
  入参：PORT(P0/P1/P2/P3/P4/P5), PIN(0~7)，两个独立参数，无逗号拆分问题
=========================================================================*/
// 读取引脚电平：返回1=高电平，0=低电平
#define PIN_RD(PORT, PIN)    (((PORT) & (1U << (PIN))) != 0)
// 引脚置高输出
#define PIN_SET(PORT, PIN)   do{ (PORT) |=  (1U << (PIN)); }while(0)
// 引脚置低输出
#define PIN_CLR(PORT, PIN)   do{ (PORT) &= ~(1U << (PIN)); }while(0)
// 引脚电平翻转
#define PIN_TOG(PORT, PIN)   do{ (PORT) ^=  (1U << (PIN)); }while(0)

/*=========================================================================
  输出引脚定义：每组引脚 = PORT + PIN 两个宏，无逗号
=========================================================================*/
// 加热使能输出，待机输出高电平1，加热输出低0
#define IO_START_OUT_PORT    P5
#define IO_START_OUT_PIN     5

// 充电完成信号输出，达到设定值输出高1
#define IO_CHARGE_DONE_PORT  P5
#define IO_CHARGE_DONE_PIN   4

// RS485收发方向控制脚：0=接收，1=发送
#define IO_RS485_DIR_PORT    P5
#define IO_RS485_DIR_PIN     3

// 设备运行状态指示灯，待机1，加热0
#define IO_WORK_STATUS_IND_PORT P2
#define IO_WORK_STATUS_IND_PIN  1

// PID功率调节PWM输出引脚
#define IO_PWM_OUT_PORT      P2
#define IO_PWM_OUT_PIN       0

// 整机总故障报警输出，故障时输出高电平
#define IO_TOTAL_FAULT_PORT  P0
#define IO_TOTAL_FAULT_PIN   7

// 系统运行指示灯，正常运行常亮
#define IO_RUN_IND_PORT      P0
#define IO_RUN_IND_PIN       1

// 控制模式指示输出
#define IO_CTRL_MODE_PORT    P4
#define IO_CTRL_MODE_PIN     2

// 缺相故障报警输出
#define IO_PHASE_LOSS_ALARM_PORT P2
#define IO_PHASE_LOSS_ALARM_PIN  7

// 水压异常报警输出
#define IO_WATER_PRESS_ALARM_PORT P2
#define IO_WATER_PRESS_ALARM_PIN  6

// 水温超温报警输出
#define IO_WATER_TEMP_ALARM_PORT P2
#define IO_WATER_TEMP_ALARM_PIN  5

// 电网异常报警输出
#define IO_GRID_ALARM_PORT   P2
#define IO_GRID_ALARM_PIN    4

// 过流故障报警输出
#define IO_OVER_CURRENT_ALARM_PORT P2
#define IO_OVER_CURRENT_ALARM_PIN  3

// 频率异常报警输出
#define IO_FREQ_ALARM_PORT   P2
#define IO_FREQ_ALARM_PIN    2

/*=========================================================================
  输入引脚定义：每组引脚 = PORT + PIN 两个宏
=========================================================================*/
// 加热启动按键输入
#define IO_START_IN_PORT     P3
#define IO_START_IN_PIN      7

// 加热停止按键输入
#define IO_STOP_IN_PORT      P3
#define IO_STOP_IN_PIN       6

// 远程控制信号输入，低电平0有效
#define IO_REMOTE_PORT       P5
#define IO_REMOTE_PIN        2

// 外部设备工作状态透传输入
#define IO_WORK_STATUS_IN_PORT P5
#define IO_WORK_STATUS_IN_PIN  1

// 外部硬件过流故障输入
#define IO_OVER_CURRENT_IN_PORT P5
#define IO_OVER_CURRENT_IN_PIN  0

// 缺水/水压不足检测输入
#define IO_WATER_LACK_PORT   P4
#define IO_WATER_LACK_PIN    3

// 维修模式开关输入，高电平1有效
#define IO_MAINTENANCE_PORT  P4
#define IO_MAINTENANCE_PIN   1

// 外部缺相检测输入，高电平1有效
#define IO_PHASE_LOSS_DET_PORT P0
#define IO_PHASE_LOSS_DET_PIN  6

/*=========================================================================
  串口硬件配置（485、外部ADC采集串口）
=========================================================================*/
#define UART_RS485_NUM     2         // RS485 Modbus使用串口2：TXD2=P1.1，RXD2=P1.0
#define UART_VOLT_NUM      3         // 直流电压采集串口3，RXD3绑定P0.0
#define UART_CURR_NUM      4         // 直流电流采集串口4，RXD4绑定P0.2

/*=========================================================================
  片内ADC模拟量通道定义（STC8A内置ADC）
=========================================================================*/
#define ADC_EXT1_CH  7  // P1.7 外部模拟量信号1
#define ADC_EXT2_CH  6  // P1.6 外部模拟量信号2
#define ADC_TEMP4_CH 5  // P1.5 4号NTC温度传感器
#define ADC_TEMP3_CH 4  // P1.4 3号NTC温度传感器
#define ADC_TEMP2_CH 3  // P1.3 2号NTC温度传感器
#define ADC_TEMP1_CH 2  // P1.2 1号NTC温度传感器

/*=========================================================================
  辅助外设IO定义（SPI下载、脉冲计数）
=========================================================================*/
// SPI下载接收脚
#define IO_SPI_RX_PORT P3
#define IO_SPI_RX_PIN  1
// SPI下载发送脚
#define IO_SPI_TX_PORT P3
#define IO_SPI_TX_PIN  0
// T0定时器外部脉冲计数输入，下降沿触发计数
#define IO_PULSE_CNT_PORT P3
#define IO_PULSE_CNT_PIN  4

/*=========================================================================
  业务快捷操作宏（直接调用，无需传参，底层自动带入PORT/PIN）
  main.c 只用这一层，完全不会报参数数量错误
=========================================================================*/
// 加热输出控制
#define START_OUT_ON()      PIN_SET(IO_START_OUT_PORT, IO_START_OUT_PIN)
#define START_OUT_OFF()     PIN_CLR(IO_START_OUT_PORT, IO_START_OUT_PIN)
#define START_OUT_TOGGLE()  PIN_TOG(IO_START_OUT_PORT, IO_START_OUT_PIN)

// 按键输入读取
#define START_IN_READ()     PIN_RD(IO_START_IN_PORT, IO_START_IN_PIN)
#define STOP_IN_READ()      PIN_RD(IO_STOP_IN_PORT, IO_STOP_IN_PIN)

// RS485收发切换（匹配硬件逻辑：P5.3=1发送，P5.3=0接收）
#define RS485_SEND_EN()     PIN_SET(IO_RS485_DIR_PORT, IO_RS485_DIR_PIN)
#define RS485_RECV_EN()     PIN_CLR(IO_RS485_DIR_PORT, IO_RS485_DIR_PIN)

// 常用外部输入读取快捷宏
#define REMOTE_READ()           PIN_RD(IO_REMOTE_PORT, IO_REMOTE_PIN)
#define OVER_CURRENT_IN_READ()  PIN_RD(IO_OVER_CURRENT_IN_PORT, IO_OVER_CURRENT_IN_PIN)
#define WATER_LACK_READ()       PIN_RD(IO_WATER_LACK_PORT, IO_WATER_LACK_PIN)
#define MAINTENANCE_READ()      PIN_RD(IO_MAINTENANCE_PORT, IO_MAINTENANCE_PIN)
#define PHASE_LOSS_DET_READ()   PIN_RD(IO_PHASE_LOSS_DET_PORT, IO_PHASE_LOSS_DET_PIN)
#define DEVICE_RUNNING_READ()   PIN_RD(IO_WORK_STATUS_IND_PORT, IO_WORK_STATUS_IND_PIN)

/*============================================================================
 * 位操作辅助宏（适用于所有I/O口）
 *============================================================================*/
#define BIT(n) (1 << (n))

#define READ_PIN(PORT, PIN) (((PORT) & BIT(PIN)) != 0)
#define SET_PIN(PORT, PIN)  \
    do                      \
    {                       \
        (PORT) |= BIT(PIN); \
    } while (0)
#define CLR_PIN(PORT, PIN)   \
    do                       \
    {                        \
        (PORT) &= ~BIT(PIN); \
    } while (0)
#define TOGGLE_PIN(PORT, PIN) \
    do                        \
    {                         \
        (PORT) ^= BIT(PIN);   \
    } while (0)

    /*=========================================================================
      外部串口采集直流电压/电流全局变量接口
    =========================================================================*/
    extern uint16_t g_dc_volt_raw; // 串口3接收直流电压原始数据
extern uint16_t g_dc_curr_raw; // 串口4接收直流电流原始数据

// 串口中断更新采样值
#define UPDATE_DC_VOLT(value) (g_dc_volt_raw = (value))
#define UPDATE_DC_CURR(value) (g_dc_curr_raw = (value))
// 业务层读取采样值
#define GET_DC_VOLT() (g_dc_volt_raw)
#define GET_DC_CURR() (g_dc_curr_raw)

#endif /* _BOARD_H_ */