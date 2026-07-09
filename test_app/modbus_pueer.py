#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Modbus RTU 上位机 - 用于 STC8A8K64D4 控制板联调
支持线圈、离散输入、输入寄存器、保持寄存器的读写，并显示地址。
"""

import sys
import os
import threading
import time
from datetime import datetime

# 自动设置 Qt 平台插件路径（解决嵌入式 Python 找不到插件的问题）
os.environ.setdefault('QT_QPA_PLATFORM_PLUGIN_PATH',
                      os.path.join(os.path.dirname(__import__('PyQt5').__file__),
                                   'Qt5', 'plugins', 'platforms'))

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QTabWidget, QGroupBox, QLabel,
                             QLineEdit, QPushButton, QComboBox, QTableWidget,
                             QTableWidgetItem, QHeaderView, QSpinBox, QCheckBox,
                             QMessageBox, QTextEdit, QSplitter, QFrame, QInputDialog)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor

import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException

# =============================================================================
# 寄存器地址映射（依据 modbus_reg_config.h）
# =============================================================================
# 线圈 (Coils) 偏移 0~7
COILS = {
    '调试模式':     {'offset': 0, 'addr': 0x0000, 'desc': '调试=1,正常=0'},
    '启动/停止':    {'offset': 1, 'addr': 0x0001, 'desc': '启动=1,停止=0'},
    '分段加热使能': {'offset': 2, 'addr': 0x0002, 'desc': '分段加热启停'},
    '温度PID使能':  {'offset': 3, 'addr': 0x0003, 'desc': '温度PID启停'},
    '故障复位':     {'offset': 6, 'addr': 0x0006, 'desc': '上升沿触发'},
}
COIL_COUNT = 8  # 实际只用0~6

# 离散输入 (DI) 偏移 0~6
DISCRETE_INPUTS = {
    '工作状态':      {'offset': 0, 'addr': 10000, 'desc': '加热=1,待机=0'},
    '水温故障':      {'offset': 1, 'addr': 10001, 'desc': '故障弹窗'},
    '水压故障':      {'offset': 2, 'addr': 10002, 'desc': ''},
    '过流故障':      {'offset': 3, 'addr': 10003, 'desc': ''},
    '缺相故障':      {'offset': 4, 'addr': 10004, 'desc': ''},
    '电压超限故障':  {'offset': 5, 'addr': 10005, 'desc': '启动2s后判断'},
    '频率超限故障':  {'offset': 6, 'addr': 10006, 'desc': '启动2s后判断'},
}
DI_COUNT = 16

# 输入寄存器 (IR) 偏移 0~27
INPUT_REGS = {
    '工作频率':      {'offset': 0,  'addr': 30000, 'unit': 'Hz'},
    '直流电压':      {'offset': 1,  'addr': 30001, 'unit': 'V×100'},
    '直流电流':      {'offset': 2,  'addr': 30002, 'unit': 'A×100'},
    '实时功率':      {'offset': 3,  'addr': 30003, 'unit': '%'},
    '温度T1':        {'offset': 4,  'addr': 30004, 'unit': '℃×100'},
    '温度T2':        {'offset': 5,  'addr': 30005, 'unit': '℃×100'},
    '温度T3':        {'offset': 6,  'addr': 30006, 'unit': '℃×100'},
    '温度T4':        {'offset': 7,  'addr': 30007, 'unit': '℃×100'},
    '外部ADC1':      {'offset': 8,  'addr': 30008, 'unit': 'V×100'},
    '外部ADC2':      {'offset': 9,  'addr': 30009, 'unit': 'V×100'},
    # 原始采集值（偏移20~27）
    '原始电压':      {'offset': 20, 'addr': 30020, 'unit': 'raw'},
    '原始电流':      {'offset': 21, 'addr': 30021, 'unit': 'raw'},
    '原始T1':        {'offset': 22, 'addr': 30022, 'unit': 'raw'},
    '原始T2':        {'offset': 23, 'addr': 30023, 'unit': 'raw'},
    '原始T3':        {'offset': 24, 'addr': 30024, 'unit': 'raw'},
    '原始T4':        {'offset': 25, 'addr': 30025, 'unit': 'raw'},
    '原始EXT1':      {'offset': 26, 'addr': 30026, 'unit': 'raw'},
    '原始EXT2':      {'offset': 27, 'addr': 30027, 'unit': 'raw'},
}
IR_COUNT = 32

# 保持寄存器 (HR) 偏移 0~116
HR_COUNT = 117
HOLDING_REGS = {
    '参数魔术字':     {'offset': 0,   'addr': 40000, 'desc': '固定0xA55A'},
    '控制方式':       {'offset': 1,   'addr': 40001, 'desc': '1=触屏,0=远程'},
    '目标温度':       {'offset': 2,   'addr': 40002, 'unit': '℃×100'},
    '控制模式':       {'offset': 3,   'addr': 40003, 'desc': '0=功率,1=分段,2=温控'},
    '目标功率':       {'offset': 11,  'addr': 40011, 'unit': '%'},
    '电压下限':       {'offset': 21,  'addr': 40021, 'unit': 'V×100'},
    '电压上限':       {'offset': 22,  'addr': 40022, 'unit': 'V×100'},
    '频率下限':       {'offset': 23,  'addr': 40023, 'unit': 'Hz'},
    '频率上限':       {'offset': 24,  'addr': 40024, 'unit': 'Hz'},
    '温度上限':       {'offset': 25,  'addr': 40025, 'unit': '℃×100'},
    '温度下限':       {'offset': 26,  'addr': 40026, 'unit': '℃×100'},
    '功率限制':       {'offset': 27,  'addr': 40027, 'unit': '%'},
    '充电设置':       {'offset': 28,  'addr': 40028, 'unit': 'mV'},
    # 校准参数
    '电压校零':       {'offset': 31,  'addr': 40031},
    '电压满量程':     {'offset': 32,  'addr': 40032},
    '电流校零':       {'offset': 33,  'addr': 40033},
    '电流满量程':     {'offset': 34,  'addr': 40034},
    'T1校零':         {'offset': 35,  'addr': 40035},
    'T1满量程':       {'offset': 36,  'addr': 40036},
    'T2校零':         {'offset': 37,  'addr': 40037},
    'T2满量程':       {'offset': 38,  'addr': 40038},
    'T3校零':         {'offset': 39,  'addr': 40039},
    'T3满量程':       {'offset': 40,  'addr': 40040},
    'T4校零':         {'offset': 41,  'addr': 40041},
    'T4满量程':       {'offset': 42,  'addr': 40042},
    'EXT1校零':       {'offset': 43,  'addr': 40043},
    'EXT1满量程':     {'offset': 44,  'addr': 40044},
    'EXT2校零':       {'offset': 45,  'addr': 40045},
    'EXT2满量程':     {'offset': 46,  'addr': 40046},
    '当前工件编号':   {'offset': 101, 'addr': 40101, 'desc': '0~5'},
    # PID温度参数
    '温控采样时间':   {'offset': 102, 'addr': 40102, 'unit': 'ms'},
    '温控最大上升率': {'offset': 103, 'addr': 40103, 'unit': '%/s'},
    '温控比例增益':   {'offset': 104, 'addr': 40104},
    '温控积分增益':   {'offset': 105, 'addr': 40105},
    '温控微分增益':   {'offset': 106, 'addr': 40106},
    '温控滤波系数':   {'offset': 107, 'addr': 40107},
    '温控调节量':     {'offset': 108, 'addr': 40108, 'unit': '%'},
    # PID功率参数
    '功控采样时间':   {'offset': 110, 'addr': 40110, 'unit': 'ms'},
    '功控最大上升率': {'offset': 111, 'addr': 40111, 'unit': '%/s'},
    '功控比例增益':   {'offset': 112, 'addr': 40112},
    '功控积分增益':   {'offset': 113, 'addr': 40113},
    '功控微分增益':   {'offset': 114, 'addr': 40114},
    '功控滤波系数':   {'offset': 115, 'addr': 40115},
    '功控调节量':     {'offset': 116, 'addr': 40116, 'unit': '%'},
}
# 工艺曲线参数（5组，每组5段，每段2个寄存器：功率、时间）
WORK_GROUPS = {}
for g in range(1, 6):
    base = 50 + (g-1)*10
    for seg in range(1, 6):
        p_off = base + (seg-1)*2
        t_off = p_off + 1
        WORK_GROUPS[f'工件{g}段{seg}功率'] = {'offset': p_off, 'addr': 40000 + p_off, 'unit': '%'}
        WORK_GROUPS[f'工件{g}段{seg}时间'] = {'offset': t_off, 'addr': 40000 + t_off, 'unit': '×100ms'}

# =============================================================================
# 通信线程 / 客户端封装
# =============================================================================
class ModbusRtuClient(QObject):
    data_updated = pyqtSignal(dict)
    log_signal = pyqtSignal(str)

    def __init__(self, port, baudrate=9600, parity='N', stopbits=1, timeout=1, slave=1):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.slave = slave
        self.client = None
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        # 构建智能读取计划（基于实际使用的保持寄存器偏移）
        self.holding_plan = self._build_holding_read_plan()

    def _build_holding_read_plan(self):
        """收集所有实际使用的保持寄存器偏移，合并为连续的区间（每段≤125）"""
        all_offsets = set()
        for info in HOLDING_REGS.values():
            all_offsets.add(info['offset'])
        for info in WORK_GROUPS.values():
            all_offsets.add(info['offset'])
        sorted_offsets = sorted(all_offsets)
        if not sorted_offsets:
            return []
        plan = []
        start = sorted_offsets[0]
        prev = sorted_offsets[0]
        for off in sorted_offsets[1:]:
            # 如果从 start 到 off 的长度 ≤ 125，则扩展区间
            if off - start + 1 <= 125:
                prev = off
            else:
                plan.append((start, prev - start + 1))
                start = off
                prev = off
        plan.append((start, prev - start + 1))
        return plan

    def connect(self):
        try:
            self.client = ModbusClient(port=self.port,
                                       baudrate=self.baudrate,
                                       parity=self.parity,
                                       stopbits=self.stopbits,
                                       timeout=self.timeout)
            if self.client.connect():
                self.running = True
                self.log_signal.emit(f"[{datetime.now().strftime('%H:%M:%S')}] 连接成功: {self.port}")
                self.log_signal.emit(f"保持寄存器读取计划: {self.holding_plan}")
                return True
            else:
                self.log_signal.emit(f"[{datetime.now().strftime('%H:%M:%S')}] 连接失败: {self.port}")
                return False
        except Exception as e:
            self.log_signal.emit(f"[{datetime.now().strftime('%H:%M:%S')}] 连接异常: {str(e)}")
            return False

    def disconnect(self):
        self.running = False
        if self.client:
            self.client.close()
            self.log_signal.emit(f"[{datetime.now().strftime('%H:%M:%S')}] 已断开连接")
        if self.thread and self.thread.is_alive():
            self.thread.join(0.5)

    def start_polling(self, interval_ms=500):
        self.interval = interval_ms / 1000.0
        if not self.running or self.thread:
            return
        self.thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.thread.start()

    def _poll_loop(self):
        while self.running:
            data = self.read_all()
            if data:
                self.data_updated.emit(data)
            time.sleep(self.interval)

    # ----- 读写函数（新版 pymodbus 兼容） -----
    def read_holding_registers(self, offset, count):
        with self.lock:
            try:
                rr = self.client.read_holding_registers(offset, count=count, device_id=self.slave)
                if not rr.isError():
                    return rr.registers
                else:
                    self.log_signal.emit(f"[ERR] 读保持寄存器 0x{offset:04X} 错误: {rr}")
                    return None
            except ModbusException as e:
                self.log_signal.emit(f"[ERR] 读保持寄存器异常: {str(e)}")
                return None

    def write_holding_register(self, offset, value):
        with self.lock:
            try:
                wr = self.client.write_register(offset, value, device_id=self.slave)
                if not wr.isError():
                    self.log_signal.emit(f"[WRITE] 保持寄存器 0x{offset:04X} = {value}")
                    return True
                else:
                    self.log_signal.emit(f"[ERR] 写保持寄存器 0x{offset:04X} 错误: {wr}")
                    return False
            except ModbusException as e:
                self.log_signal.emit(f"[ERR] 写保持寄存器异常: {str(e)}")
                return False

    def read_coils(self, offset, count):
        with self.lock:
            try:
                rr = self.client.read_coils(offset, count=count, device_id=self.slave)
                if not rr.isError():
                    return rr.bits
                else:
                    self.log_signal.emit(f"[ERR] 读线圈 0x{offset:04X} 错误: {rr}")
                    return None
            except ModbusException as e:
                self.log_signal.emit(f"[ERR] 读线圈异常: {str(e)}")
                return None

    def write_coil(self, offset, value):
        with self.lock:
            try:
                wr = self.client.write_coil(offset, value, device_id=self.slave)
                if not wr.isError():
                    self.log_signal.emit(f"[WRITE] 线圈 0x{offset:04X} = {value}")
                    return True
                else:
                    self.log_signal.emit(f"[ERR] 写线圈 0x{offset:04X} 错误: {wr}")
                    return False
            except ModbusException as e:
                self.log_signal.emit(f"[ERR] 写线圈异常: {str(e)}")
                return False

    def read_discrete_inputs(self, offset, count):
        with self.lock:
            try:
                rr = self.client.read_discrete_inputs(offset, count=count, device_id=self.slave)
                if not rr.isError():
                    return rr.bits
                else:
                    self.log_signal.emit(f"[ERR] 读离散输入 0x{offset:04X} 错误: {rr}")
                    return None
            except ModbusException as e:
                self.log_signal.emit(f"[ERR] 读离散输入异常: {str(e)}")
                return None

    def read_input_registers(self, offset, count):
        with self.lock:
            try:
                rr = self.client.read_input_registers(offset, count=count, device_id=self.slave)
                if not rr.isError():
                    return rr.registers
                else:
                    self.log_signal.emit(f"[ERR] 读输入寄存器 0x{offset:04X} 错误: {rr}")
                    return None
            except ModbusException as e:
                self.log_signal.emit(f"[ERR] 读输入寄存器异常: {str(e)}")
                return None

    def read_all(self):
        """一次性读取所有数据，保持寄存器按智能计划分批读取"""
        data = {}
        # 线圈
        bits = self.read_coils(0, COIL_COUNT)
        if bits is not None:
            data['coils'] = bits
        # 离散输入
        bits = self.read_discrete_inputs(0, DI_COUNT)
        if bits is not None:
            data['discrete'] = bits
        # 输入寄存器
        regs = self.read_input_registers(0, IR_COUNT)
        if regs is not None:
            data['input_regs'] = regs
        # 保持寄存器 —— 按计划分批，并组装为完整 HR_COUNT 长度列表
        holding = [0] * HR_COUNT
        for start, count in self.holding_plan:
            regs = self.read_holding_registers(start, count)
            if regs is None:
                # 若任一区间读取失败，放弃本次更新
                self.log_signal.emit("[WARN] 保持寄存器读取失败，放弃本次更新")
                return None
            for i, val in enumerate(regs):
                holding[start + i] = val
        data['holding_regs'] = holding
        return data

# =============================================================================
# 主窗口
# =============================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Modbus 上位机 - STC8A8K64D4 联调")
        self.setGeometry(100, 100, 1200, 700)

        self.client = None
        self.poll_timer = QTimer()
        self.poll_timer.timeout.connect(self.on_poll_timer)

        self.init_ui()
        self.update_port_list()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ----- 顶部工具栏 -----
        tool_frame = QFrame()
        tool_frame.setFrameShape(QFrame.StyledPanel)
        tool_layout = QHBoxLayout(tool_frame)

        tool_layout.addWidget(QLabel("串口:"))
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(100)
        tool_layout.addWidget(self.combo_port)

        tool_layout.addWidget(QLabel("波特率:"))
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(['9600', '19200', '38400', '115200'])
        self.combo_baud.setCurrentText('9600')
        tool_layout.addWidget(self.combo_baud)

        tool_layout.addWidget(QLabel("从站ID:"))
        self.spin_slave = QSpinBox()
        self.spin_slave.setRange(1, 247)
        self.spin_slave.setValue(1)
        tool_layout.addWidget(self.spin_slave)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self.toggle_connect)
        tool_layout.addWidget(self.btn_connect)

        self.btn_refresh = QPushButton("手动刷新")
        self.btn_refresh.clicked.connect(self.manual_refresh)
        tool_layout.addWidget(self.btn_refresh)

        self.btn_save_test = QPushButton("掉电保存测试")
        self.btn_save_test.clicked.connect(self.power_loss_save_test)
        tool_layout.addWidget(self.btn_save_test)

        tool_layout.addStretch()
        tool_layout.addWidget(QLabel("刷新间隔(ms):"))
        self.spin_interval = QSpinBox()
        self.spin_interval.setRange(100, 5000)
        self.spin_interval.setValue(500)
        self.spin_interval.valueChanged.connect(self.set_poll_interval)
        tool_layout.addWidget(self.spin_interval)

        self.cb_auto_refresh = QCheckBox("自动刷新")
        self.cb_auto_refresh.setChecked(True)
        self.cb_auto_refresh.toggled.connect(self.toggle_auto_refresh)
        tool_layout.addWidget(self.cb_auto_refresh)

        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        tool_layout.addWidget(self.status_label)

        main_layout.addWidget(tool_frame)

        # ----- 主区域：标签页 -----
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)

        # 创建各个页面
        self.tab_main = self.create_main_tab()
        self.tab_coils = self.create_coils_tab()
        self.tab_di = self.create_di_tab()
        self.tab_ir = self.create_ir_tab()
        self.tab_hr = self.create_hr_tab()
        self.tab_work = self.create_work_tab()
        self.tab_pid = self.create_pid_tab()
        self.tab_log = self.create_log_tab()

        self.tab_widget.addTab(self.tab_main, "主控面板")
        self.tab_widget.addTab(self.tab_coils, "线圈控制")
        self.tab_widget.addTab(self.tab_di, "离散输入")
        self.tab_widget.addTab(self.tab_ir, "输入寄存器")
        self.tab_widget.addTab(self.tab_hr, "保持寄存器")
        self.tab_widget.addTab(self.tab_work, "工艺曲线")
        self.tab_widget.addTab(self.tab_pid, "PID参数")
        self.tab_widget.addTab(self.tab_log, "通信日志")

    # ---------- 创建各标签页 ----------
    def create_main_tab(self):
        """主控面板：显示核心参数，附带地址"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        grid = QVBoxLayout()
        # 第一行：频率、电压、电流、功率
        row1 = QHBoxLayout()
        self.lbl_freq = self._create_labeled_value("工作频率", "0", "Hz", "30000")
        self.lbl_volt = self._create_labeled_value("直流电压", "0", "V×100", "30001")
        self.lbl_curr = self._create_labeled_value("直流电流", "0", "A×100", "30002")
        self.lbl_power = self._create_labeled_value("实时功率", "0", "%", "30003")
        for lbl in [self.lbl_freq, self.lbl_volt, self.lbl_curr, self.lbl_power]:
            row1.addWidget(lbl)
        grid.addLayout(row1)

        # 第二行：温度 T1~T4
        row2 = QHBoxLayout()
        self.lbl_t1 = self._create_labeled_value("温度T1", "0", "℃×100", "30004")
        self.lbl_t2 = self._create_labeled_value("温度T2", "0", "℃×100", "30005")
        self.lbl_t3 = self._create_labeled_value("温度T3", "0", "℃×100", "30006")
        self.lbl_t4 = self._create_labeled_value("温度T4", "0", "℃×100", "30007")
        for lbl in [self.lbl_t1, self.lbl_t2, self.lbl_t3, self.lbl_t4]:
            row2.addWidget(lbl)
        grid.addLayout(row2)

        # 第三行：外部ADC1/2，以及工作状态、故障总览
        row3 = QHBoxLayout()
        self.lbl_ext1 = self._create_labeled_value("外部ADC1", "0", "V×100", "30008")
        self.lbl_ext2 = self._create_labeled_value("外部ADC2", "0", "V×100", "30009")
        self.lbl_run_state = self._create_labeled_value("工作状态", "待机", "", "10000")
        self.lbl_fault_sum = self._create_labeled_value("总故障", "无", "", "10001-10006")
        for lbl in [self.lbl_ext1, self.lbl_ext2, self.lbl_run_state, self.lbl_fault_sum]:
            row3.addWidget(lbl)
        grid.addLayout(row3)

        layout.addLayout(grid)
        layout.addStretch()
        return widget

    def _create_labeled_value(self, label, init_val, unit, addr):
        """生成一个带地址标签的显示控件"""
        group = QGroupBox(label)
        layout = QVBoxLayout(group)
        val_lbl = QLabel(init_val)
        val_lbl.setAlignment(Qt.AlignCenter)
        val_lbl.setStyleSheet("font-size: 25px; font-weight: bold;")
        layout.addWidget(val_lbl)
        info = QLabel(f"地址: {addr}  {unit}" if unit else f"地址: {addr}")
        info.setAlignment(Qt.AlignCenter)
        info.setStyleSheet("color: gray; font-size: 25px;")
        layout.addWidget(info)
        # 保存引用
        group.val_lbl = val_lbl
        group.addr = addr
        return group

    def create_coils_tab(self):
        """线圈控制"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.coil_widgets = {}
        for name, info in COILS.items():
            cb = QCheckBox(name)
            cb.setChecked(False)
            cb.stateChanged.connect(lambda state, off=info['offset']: self.on_coil_toggle(off, state))
            layout.addWidget(cb)
            # 显示地址
            addr_lbl = QLabel(f"地址: 0x{info['addr']:04X}  ({info['desc']})")
            addr_lbl.setStyleSheet("color: gray; font-size: 25px;")
            layout.addWidget(addr_lbl)
            self.coil_widgets[info['offset']] = cb
        layout.addStretch()
        return widget

    def on_coil_toggle(self, offset, state):
        if not self.client or not self.client.running:
            QMessageBox.warning(self, "提示", "请先连接设备")
            return
        value = 1 if state == Qt.Checked else 0
        self.client.write_coil(offset, value)

    def create_di_tab(self):
        """离散输入状态显示"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.di_labels = {}
        for name, info in DISCRETE_INPUTS.items():
            hbox = QHBoxLayout()
            lbl = QLabel(name)
            lbl.setFixedWidth(120)
            hbox.addWidget(lbl)
            val_lbl = QLabel("0")
            val_lbl.setFixedWidth(60)
            hbox.addWidget(val_lbl)
            addr_lbl = QLabel(f"地址: {info['addr']}")
            addr_lbl.setStyleSheet("color: gray; font-size: 25px;")
            hbox.addWidget(addr_lbl)
            layout.addLayout(hbox)
            self.di_labels[info['offset']] = val_lbl
        layout.addStretch()
        return widget

    def create_ir_tab(self):
        """输入寄存器表格"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.table_ir = QTableWidget()
        self.table_ir.setColumnCount(4)
        self.table_ir.setHorizontalHeaderLabels(['名称', '地址', '值', '单位'])
        self.table_ir.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_ir.setRowCount(len(INPUT_REGS))
        row = 0
        self.ir_cells = {}
        for name, info in INPUT_REGS.items():
            self.table_ir.setItem(row, 0, QTableWidgetItem(name))
            self.table_ir.setItem(row, 1, QTableWidgetItem(str(info['addr'])))
            val_item = QTableWidgetItem("0")
            val_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table_ir.setItem(row, 2, val_item)
            self.table_ir.setItem(row, 3, QTableWidgetItem(info.get('unit', '')))
            self.ir_cells[info['offset']] = (row, 2)
            row += 1
        layout.addWidget(self.table_ir)
        return widget

    def create_hr_tab(self):
        """保持寄存器表格（可编辑）"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.table_hr = QTableWidget()
        self.table_hr.setColumnCount(4)
        self.table_hr.setHorizontalHeaderLabels(['名称', '地址', '值', '单位'])
        self.table_hr.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # 过滤掉工艺曲线和PID参数（这些在专门页面）
        hr_list = [item for item in HOLDING_REGS.items() if not item[0].startswith('工件') and not item[0].startswith('温控') and not item[0].startswith('功控')]
        self.table_hr.setRowCount(len(hr_list))
        self.hr_cells = {}
        row = 0
        for name, info in hr_list:
            self.table_hr.setItem(row, 0, QTableWidgetItem(name))
            self.table_hr.setItem(row, 1, QTableWidgetItem(str(info['addr'])))
            val_item = QTableWidgetItem("0")
            val_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table_hr.setItem(row, 2, val_item)
            self.table_hr.setItem(row, 3, QTableWidgetItem(info.get('unit', '')))
            self.hr_cells[info['offset']] = (row, 2)
            row += 1
        self.table_hr.itemDoubleClicked.connect(self.on_hr_double_click)
        layout.addWidget(self.table_hr)
        return widget

    def on_hr_double_click(self, item):
        """双击编辑保持寄存器"""
        row = item.row()
        offset = None
        for off, (r, c) in self.hr_cells.items():
            if r == row and c == 2:
                offset = off
                break
        if offset is None:
            return
        if not self.client or not self.client.running:
            QMessageBox.warning(self, "提示", "请先连接设备")
            return
        # 弹出输入框
        current_val = int(self.table_hr.item(row, 2).text())
        new_val, ok = QInputDialog.getInt(self, "修改寄存器", f"输入新值 (地址 {offset:04X}):", current_val, 0, 65535)
        if ok:
            if self.client.write_holding_register(offset, new_val):
                self.table_hr.item(row, 2).setText(str(new_val))

    def create_work_tab(self):
        """工艺曲线参数设置"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.work_table = QTableWidget()
        self.work_table.setColumnCount(6)
        self.work_table.setHorizontalHeaderLabels(['组', '段', '功率地址', '功率值', '时间地址', '时间值'])
        self.work_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        rows = 25
        self.work_table.setRowCount(rows)
        self.work_cells = {}
        row = 0
        for g in range(1, 6):
            for seg in range(1, 6):
                p_name = f'工件{g}段{seg}功率'
                t_name = f'工件{g}段{seg}时间'
                p_info = WORK_GROUPS[p_name]
                t_info = WORK_GROUPS[t_name]
                self.work_table.setItem(row, 0, QTableWidgetItem(str(g)))
                self.work_table.setItem(row, 1, QTableWidgetItem(str(seg)))
                self.work_table.setItem(row, 2, QTableWidgetItem(str(p_info['addr'])))
                p_item = QTableWidgetItem("0")
                p_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.work_table.setItem(row, 3, p_item)
                self.work_table.setItem(row, 4, QTableWidgetItem(str(t_info['addr'])))
                t_item = QTableWidgetItem("0")
                t_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.work_table.setItem(row, 5, t_item)
                self.work_cells[p_info['offset']] = (row, 3)
                self.work_cells[t_info['offset']] = (row, 5)
                row += 1
        self.work_table.itemDoubleClicked.connect(self.on_work_double_click)
        layout.addWidget(self.work_table)
        return widget

    def on_work_double_click(self, item):
        row = item.row()
        col = item.column()
        if col not in (3, 5):
            return
        # 查找 offset
        offset = None
        for off, (r, c) in self.work_cells.items():
            if r == row and c == col:
                offset = off
                break
        if offset is None:
            return
        if not self.client or not self.client.running:
            QMessageBox.warning(self, "提示", "请先连接设备")
            return
        current_val = int(self.work_table.item(row, col).text())
        new_val, ok = QInputDialog.getInt(self, "修改工艺参数", f"输入新值 (地址 {offset:04X}):", current_val, 0, 65535)
        if ok:
            if self.client.write_holding_register(offset, new_val):
                self.work_table.item(row, col).setText(str(new_val))

    def create_pid_tab(self):
        """PID参数设置"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.pid_table = QTableWidget()
        self.pid_table.setColumnCount(3)
        self.pid_table.setHorizontalHeaderLabels(['参数名称', '地址', '值'])
        self.pid_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        pid_list = [(k, v) for k, v in HOLDING_REGS.items() if k.startswith('温控') or k.startswith('功控')]
        self.pid_table.setRowCount(len(pid_list))
        self.pid_cells = {}
        row = 0
        for name, info in pid_list:
            self.pid_table.setItem(row, 0, QTableWidgetItem(name))
            self.pid_table.setItem(row, 1, QTableWidgetItem(str(info['addr'])))
            val_item = QTableWidgetItem("0")
            val_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.pid_table.setItem(row, 2, val_item)
            self.pid_cells[info['offset']] = (row, 2)
            row += 1
        self.pid_table.itemDoubleClicked.connect(self.on_pid_double_click)
        layout.addWidget(self.pid_table)
        return widget

    def on_pid_double_click(self, item):
        row = item.row()
        if item.column() != 2:
            return
        offset = None
        for off, (r, c) in self.pid_cells.items():
            if r == row and c == 2:
                offset = off
                break
        if offset is None:
            return
        if not self.client or not self.client.running:
            QMessageBox.warning(self, "提示", "请先连接设备")
            return
        current_val = int(self.pid_table.item(row, 2).text())
        new_val, ok = QInputDialog.getInt(self, "修改PID参数", f"输入新值 (地址 {offset:04X}):", current_val, 0, 65535)
        if ok:
            if self.client.write_holding_register(offset, new_val):
                self.pid_table.item(row, 2).setText(str(new_val))

    def create_log_tab(self):
        """通信日志"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Courier New", 9))
        layout.addWidget(self.log_text)
        btn_clear = QPushButton("清空日志")
        btn_clear.clicked.connect(lambda: self.log_text.clear())
        layout.addWidget(btn_clear)
        return widget

    # ---------- 串口连接与刷新 ----------
    def update_port_list(self):
        ports = serial.tools.list_ports.comports()
        self.combo_port.clear()
        for p in ports:
            self.combo_port.addItem(p.device)

    def toggle_connect(self):
        if self.btn_connect.text() == "连接":
            self.do_connect()
        else:
            self.do_disconnect()

    def do_connect(self):
        port = self.combo_port.currentText()
        if not port:
            QMessageBox.warning(self, "提示", "请选择串口")
            return
        baud = int(self.combo_baud.currentText())
        slave = self.spin_slave.value()
        self.client = ModbusRtuClient(port, baudrate=baud, slave=slave)
        self.client.log_signal.connect(self.append_log)
        self.client.data_updated.connect(self.update_ui_data)
        if self.client.connect():
            self.btn_connect.setText("断开")
            self.status_label.setText("已连接")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            if self.cb_auto_refresh.isChecked():
                self.client.start_polling(self.spin_interval.value())
        else:
            self.client = None

    def do_disconnect(self):
        if self.client:
            self.client.disconnect()
            self.client = None
        self.btn_connect.setText("连接")
        self.status_label.setText("未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")

    def toggle_auto_refresh(self, checked):
        if checked and self.client and self.client.running:
            self.client.start_polling(self.spin_interval.value())
        elif self.client:
            self.client.running = False

    def set_poll_interval(self, val):
        if self.client and self.client.running and self.cb_auto_refresh.isChecked():
            self.client.interval = val / 1000.0

    def manual_refresh(self):
        if self.client and self.client.running:
            data = self.client.read_all()
            if data:
                self.update_ui_data(data)
        else:
            QMessageBox.warning(self, "提示", "设备未连接")

    def power_loss_save_test(self):
        """掉电保存测试：写入特征值到保持寄存器，读回验证，然后提示用户断电再上电"""
        if not self.client or not self.client.running:
            QMessageBox.warning(self, "提示", "请先连接设备")
            return

        # 使用偏移2（目标温度，40002）写入测试值
        test_offset = 2
        test_value = 12345

        reply = QMessageBox.question(
            self, "掉电保存测试",
            f"即将向保持寄存器 地址{test_offset} (40002) 写入测试值 {test_value}。\n\n"
            f"步骤：\n"
            f"1. 点击'是'写入值\n"
            f"2. 等待3秒让下位机保存到EEPROM\n"
            f"3. 读回验证写入成功\n"
            f"4. 断开下位机电源\n"
            f"5. 重新上电后连接，检查值是否保持\n\n"
            f"是否开始？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)

        if reply != QMessageBox.Yes:
            return

        # 步骤1：写入
        self.append_log(f"[掉电测试] 写入 保持寄存器[{test_offset}] = {test_value}")
        ok = self.client.write_holding_register(test_offset, test_value)
        if not ok:
            QMessageBox.critical(self, "错误", "写入失败！请检查通信。")
            return

        # 步骤2：等待3秒让下位机保存到EEPROM
        self.append_log("[掉电测试] 等待3秒，让下位机保存到EEPROM...")
        QApplication.processEvents()
        time.sleep(3)

        # 步骤3：读取EEPROM保存状态（输入寄存器偏移28）
        status_regs = self.client.read_input_registers(28, 1)
        if status_regs is not None and len(status_regs) > 0:
            save_status = status_regs[0]
            status_text = {0: "空闲", 1: "正在保存", 2: "保存成功", 3: "保存失败"}.get(save_status, f"未知({save_status})")
            self.append_log(f"[掉电测试] EEPROM保存状态: {status_text}")
        else:
            self.append_log("[掉电测试] 无法读取保存状态")
            save_status = -1

        # 步骤4：读回验证
        regs = self.client.read_holding_registers(test_offset, 1)
        if regs is not None and len(regs) > 0:
            readback = regs[0]
            self.append_log(f"[掉电测试] 读回值 = {readback}")
            if readback == test_value:
                if save_status == 2:
                    QMessageBox.information(self, "验证成功",
                        f"写入值 {test_value} 读回一致，EEPROM保存成功！\n\n"
                        f"现在可以断开下位机电源，等待5秒后重新上电。\n"
                        f"重新连接后，检查保持寄存器[{test_offset}]的值是否仍为 {test_value}。")
                elif save_status == 3:
                    QMessageBox.critical(self, "EEPROM保存失败",
                        f"RAM读回值正确({test_value})，但EEPROM保存失败！\n\n"
                        f"可能原因：\n"
                        f"1. STC-ISP下载时未启用EEPROM功能\n"
                        f"2. IAP地址不在EEPROM范围内\n"
                        f"3. EEPROM扇区损坏\n\n"
                        f"请检查STC-ISP下载设置中的EEPROM配置。")
                else:
                    QMessageBox.information(self, "验证成功",
                        f"写入值 {test_value} 读回一致！\n\n"
                        f"现在可以断开下位机电源，等待5秒后重新上电。\n"
                        f"重新连接后，检查保持寄存器[{test_offset}]的值是否仍为 {test_value}。")
            else:
                QMessageBox.warning(self, "验证失败",
                    f"写入值 {test_value}，但读回值 {readback}！\n"
                    f"通信或写入逻辑有问题。")
        else:
            QMessageBox.critical(self, "验证失败", "读取保持寄存器失败！")

    def on_poll_timer(self):
        self.manual_refresh()

    # ---------- 更新 UI ----------
    def update_ui_data(self, data):
        """更新所有界面控件"""
        # 主控面板
        if 'input_regs' in data:
            ir = data['input_regs']
            self._set_value(self.lbl_freq, str(ir[0] if len(ir)>0 else 0))
            self._set_value(self.lbl_volt, str(ir[1] if len(ir)>1 else 0))
            self._set_value(self.lbl_curr, str(ir[2] if len(ir)>2 else 0))
            self._set_value(self.lbl_power, str(ir[3] if len(ir)>3 else 0))
            self._set_value(self.lbl_t1, str(ir[4] if len(ir)>4 else 0))
            self._set_value(self.lbl_t2, str(ir[5] if len(ir)>5 else 0))
            self._set_value(self.lbl_t3, str(ir[6] if len(ir)>6 else 0))
            self._set_value(self.lbl_t4, str(ir[7] if len(ir)>7 else 0))
            self._set_value(self.lbl_ext1, str(ir[8] if len(ir)>8 else 0))
            self._set_value(self.lbl_ext2, str(ir[9] if len(ir)>9 else 0))
        # 离散输入
        if 'discrete' in data:
            di = data['discrete']
            for off, lbl in self.di_labels.items():
                val = di[off] if off < len(di) else 0
                lbl.setText(str(val))
                # 故障状态用红色高亮
                if off >= 1 and off <= 6:
                    lbl.setStyleSheet("color: red; font-weight: bold;" if val else "")
        # 线圈状态
        if 'coils' in data:
            coils = data['coils']
            for off, cb in self.coil_widgets.items():
                val = coils[off] if off < len(coils) else 0
                cb.blockSignals(True)
                cb.setChecked(bool(val))
                cb.blockSignals(False)
        # 输入寄存器表格
        if 'input_regs' in data:
            ir = data['input_regs']
            for off, (row, col) in self.ir_cells.items():
                val = ir[off] if off < len(ir) else 0
                self.table_ir.item(row, col).setText(str(val))
        # 保持寄存器表格（非工艺/PID）
        if 'holding_regs' in data:
            hr = data['holding_regs']
            for off, (row, col) in self.hr_cells.items():
                val = hr[off] if off < len(hr) else 0
                self.table_hr.item(row, col).setText(str(val))
            # 工艺曲线
            for off, (row, col) in self.work_cells.items():
                val = hr[off] if off < len(hr) else 0
                self.work_table.item(row, col).setText(str(val))
            # PID
            for off, (row, col) in self.pid_cells.items():
                val = hr[off] if off < len(hr) else 0
                self.pid_table.item(row, col).setText(str(val))

    def _set_value(self, group, text):
        if hasattr(group, 'val_lbl'):
            group.val_lbl.setText(text)

    def append_log(self, msg):
        self.log_text.append(msg)

    def closeEvent(self, event):
        if self.client:
            self.client.disconnect()
        event.accept()

# =============================================================================
# 程序入口  pip install PyQt5 pyserial pymodbus
# =============================================================================
if __name__ == '__main__':
    app = QApplication(sys.argv)
    # 全局字体放大（影响所有控件）
    app.setFont(QFont("Microsoft YaHei", 11))
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
