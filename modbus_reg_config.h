#ifndef _MODBUS_REG_CONFIG_H_
#define _MODBUS_REG_CONFIG_H_

#include <stdint.h>

/*============================================================================
 * 1. 线圈 (DO / 0x 地址) —— 功能码: 01(读), 05(单写), 0F(批量写)
 *    HMI 写入控制命令，单片机只读此区域并执行相应动作
 *    地址范围: 0x0001 ~ 0x0006
 *    单片机偏移直接对应协议表中的 "单片机地址偏移"
 *============================================================================*/
#define COIL_START_STOP_OFFSET 1      // HMI 0x0001 : 启动=1, 停止=0
#define COIL_SEGMENT_HEAT_EN_OFFSET 2 // HMI 0x0002 : 分段加热 启停
#define COIL_TEMP_PID_EN_OFFSET 3     // HMI 0x0003 : 温度PID 启停
#define COIL_FAULT_RESET_OFFSET 6     // HMI 0x0006 : 故障复位按钮 (上升沿触发)

/*============================================================================
 * 2. 离散输入 (DI / 1x 地址) —— 功能码: 02(读)
 *    单片机采集/运行状态发送给 HMI，HMI 只读此区域
 *    地址范围: 10005 ~ 10011
 *    单片机偏移直接对应协议表中的 "单片机地址偏移"
 *============================================================================*/
#define DI_DEVICE_RUNNING_OFFSET 5     // HMI 10005 : 加热运行中 (加热=1, 待机=0)
#define DI_WATER_TEMP_FAULT_OFFSET 6   // HMI 10006 : 水温故障 (故障弹窗)
#define DI_WATER_PRESS_FAULT_OFFSET 7  // HMI 10007 : 水压故障
#define DI_OVER_CURRENT_FAULT_OFFSET 8 // HMI 10008 : 过流故障
#define DI_PHASE_LOSS_FAULT_OFFSET 9   // HMI 10009 : 缺相故障
#define DI_VOLT_OVER_FAULT_OFFSET 10   // HMI 10010 : 电网电压超限故障 (启动2s后判断)
#define DI_FREQ_OVER_FAULT_OFFSET 11   // HMI 10011 : 频率超限故障 (启动2s后判断)

/*============================================================================
 * 3. 输入寄存器 (3x 地址) —— 功能码: 04(读)
 *    实时采集数据，单片机刷新，HMI 只读
 *    地址范围: 30000 ~ 30009 (已校准/处理后值)
 *              30020 ~ 30027 (原始采集值，用于校准运算)
 *============================================================================*/
#define REG_WORK_FREQ_OFFSET 0  // 30000 : 工作频率 (Hz，P3.4下降沿计数)
#define REG_DC_VOLT_OFFSET 1    // 30001 : 直流电压 (已校准)
#define REG_DC_CURR_OFFSET 2    // 30002 : 直流电流 (已校准)
#define REG_REAL_POWER_OFFSET 3 // 30003 : 实时运行功率 (0~100%)
#define REG_TEMP_T1_OFFSET 4    // 30004 : 温度采集T1 (已校准)
#define REG_TEMP_T2_OFFSET 5    // 30005 : 温度采集T2 (已校准)
#define REG_TEMP_T3_OFFSET 6    // 30006 : 温度采集T3 (已校准)
#define REG_TEMP_T4_OFFSET 7    // 30007 : 温度采集T4 (已校准)
#define REG_EXT_ADC1_OFFSET 8   // 30008 : 外部0-5V采集1 (已校准)
#define REG_EXT_ADC2_OFFSET 9   // 30009 : 外部0-5V采集2 (已校准)

/* 30020 ~ 30027: 原始采集值 (未校准) */
#define RAW_IR_VOLT_OFFSET 20   // 30020 : 直流电压 - 原始采集值
#define RAW_IR_CURR_OFFSET 21   // 30021 : 直流电流 - 原始采集值
#define RAW_IR_TEMP1_OFFSET 22  // 30022 : 温度T1 - 原始采集值
#define RAW_IR_TEMP2_OFFSET 23  // 30023 : 温度T2 - 原始采集值
#define RAW_IR_TEMP3_OFFSET 24  // 30024 : 温度T3 - 原始采集值
#define RAW_IR_TEMP4_OFFSET 25  // 30025 : 温度T4 - 原始采集值
#define RAW_IR_EXT1_OFFSET 26   // 30026 : 外部0-5V采集1 - 原始采集值
#define RAW_IR_EXT2_OFFSET 27   // 30027 : 外部0-5V采集2 - 原始采集值

/*============================================================================
 * 4. 保持寄存器 (4x 地址) —— 功能码: 03(读), 06(单写), 10(批量写)
 *    HMI 与单片机双向读写，全部掉电保存
 *============================================================================*/

/* --- 分区2：主控设定区 (偏移 1~2) --- */
#define HLD_CTRL_MODE_OFFSET 1   // 40001 : 控制方式选择
#define HLD_TARGET_TEMP_OFFSET 2 // 40002 : 目标温度
#define HLD_POWER_SETPOINT_OFFSET 3 // 40011 : 目标功率 (0~100%)

/* --- 分区3：报警上下限 (偏移 21~28) --- */
#define HLD_VOLT_LOW_OFFSET 21    // 40021 : 电压报警下限
#define HLD_VOLT_HIGH_OFFSET 22   // 40022 : 电压报警上限
#define HLD_FREQ_LOW_OFFSET 23    // 40023 : 频率报警下限
#define HLD_FREQ_HIGH_OFFSET 24   // 40024 : 频率报警上限
#define HLD_TEMP_HIGH_OFFSET 25   // 40025 : 温度报警上限
#define HLD_TEMP_LOW_OFFSET 26    // 40026 : 温度报警下限
#define HLD_POWER_LIMIT_OFFSET 27 // 40027 : 总功率限制
#define HLD_CHARGE_SET_OFFSET 28  // 40028 : 充电设置

/* --- 分区4：模拟量校准参数 (偏移 31~46) --- */
#define HLD_VOLT_ZERO_OFFSET 31  // 40031 : 直流电压 - 校零
#define HLD_VOLT_FULL_OFFSET 32  // 40032 : 直流电压 - 满量程
#define HLD_CURR_ZERO_OFFSET 33  // 40033 : 直流电流 - 校零
#define HLD_CURR_FULL_OFFSET 34  // 40034 : 直流电流 - 满量程
#define HLD_TEMP1_ZERO_OFFSET 35 // 40035 : 温度T1 - 校零
#define HLD_TEMP1_FULL_OFFSET 36 // 40036 : 温度T1 - 满量程
#define HLD_TEMP2_ZERO_OFFSET 37 // 40037 : 温度T2 - 校零
#define HLD_TEMP2_FULL_OFFSET 38 // 40038 : 温度T2 - 满量程
#define HLD_TEMP3_ZERO_OFFSET 39 // 40039 : 温度T3 - 校零
#define HLD_TEMP3_FULL_OFFSET 40 // 40040 : 温度T3 - 满量程
#define HLD_TEMP4_ZERO_OFFSET 41 // 40041 : 温度T4 - 校零
#define HLD_TEMP4_FULL_OFFSET 42 // 40042 : 温度T4 - 满量程
#define HLD_EXT1_ZERO_OFFSET 43  // 40043 : 外部0-5V采集1 - 校零
#define HLD_EXT1_FULL_OFFSET 44  // 40044 : 外部0-5V采集1 - 满量程
#define HLD_EXT2_ZERO_OFFSET 45  // 40045 : 外部0-5V采集2 - 校零
#define HLD_EXT2_FULL_OFFSET 46  // 40046 : 外部0-5V采集2 - 满量程

/* --- 分区5：工艺加热曲线参数 (偏移 50~99) --- */
/* 5组工件，每组5段，每段2个寄存器: [功率(0~100), 时间(×10, 0.1s精度)] */
#define WORK_GROUP_START(group) (50 + ((group) - 1) * 10)                         // group = 1~5
#define WORK_POWER_OFFSET(group, seg) (WORK_GROUP_START(group) + ((seg) - 1) * 2) // seg = 1~5
#define WORK_TIME_OFFSET(group, seg) (WORK_POWER_OFFSET(group, seg) + 1)

/* 当前选中工件编号 (偏移 101 -> HMI 40101) */
#define HLD_CUR_WORK_NO_OFFSET 101 // 值范围 0~5 (0表示无选中)

/* ---- PID参数 (温度PID) 偏移 200~206 ---- */
#define HLD_TEMP_PID_SAMPLE_TIME_OFFSET 200 // 采样时间
#define HLD_TEMP_PID_MAX_RISE_OFFSET 201    // 最大上升率
#define HLD_TEMP_PID_PROP_GAIN_OFFSET 202   // 比例增益
#define HLD_TEMP_PID_INT_GAIN_OFFSET 203    // 积分增益
#define HLD_TEMP_PID_DER_GAIN_OFFSET 204    // 微分增益
#define HLD_TEMP_PID_FILTER_OFFSET 205      // 滤波
#define HLD_TEMP_PID_ADJUST_OFFSET 206      // 调节量

/* ---- PID参数 (功率PID) 偏移 210~216 ---- */
#define HLD_POWER_PID_SAMPLE_TIME_OFFSET 210 // 采样时间
#define HLD_POWER_PID_MAX_RISE_OFFSET 211    // 最大上升率
#define HLD_POWER_PID_PROP_GAIN_OFFSET 212   // 比例增益
#define HLD_POWER_PID_INT_GAIN_OFFSET 213    // 积分增益
#define HLD_POWER_PID_DER_GAIN_OFFSET 214    // 微分增益
#define HLD_POWER_PID_FILTER_OFFSET 215      // 滤波
#define HLD_POWER_PID_ADJUST_OFFSET 216      // 调节量

/*============================================================================
 * 辅助宏：将偏移转换为标准 Modbus PLC 地址 (用于调试或HMI侧核对)
 *============================================================================*/
#define MODBUS_ADDR_COIL(offset) (0x0000 + (offset)) // 线圈 (如 0x0001)
#define MODBUS_ADDR_DI(offset) (10000 + (offset))    // 离散输入 (如 10005)
#define MODBUS_ADDR_IR(offset) (30000 + (offset))    // 输入寄存器 (如 30000)
#define MODBUS_ADDR_HR(offset) (40000 + (offset))    // 保持寄存器 (如 40001)

/*============================================================================
 * 数组大小定义
 *============================================================================*/
#define COIL_COUNT          8     // 线圈数量 (偏移0~6，偏移0未使用)
#define DI_COUNT            16    // 离散输入数量 (偏移0~21)
#define INPUT_REG_COUNT     32    // 输入寄存器数量 (偏移0~27，0~9为已校准值，20~27为原始采集值)
#define HOLDING_REG_COUNT   217   // 保持寄存器数量 (偏移0~216)

#endif /* _MODBUS_REG_CONFIG_H_ */

