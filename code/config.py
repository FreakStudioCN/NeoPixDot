# Python env   : MicroPython v1.27
# -*- coding: utf-8 -*-
# @Time    : 2025/9/5 下午10:12
# @Author  : 李清水
# @File    : config.py
# @Description : 全局配置
# @License : CC BY-NC 4.0

__version__ = "0.1.0"
__author__ = "李清水"
__license__ = "CC BY-NC 4.0"
__platform__ = "MicroPython v1.27"

# ======================================== 导入相关模块 =========================================

# ======================================== 全局变量 ============================================

# 调试开关：True-输出日志，False-关闭所有打印
DEBUG_ENABLE = True
# 核心配置
BAUDRATE = 115200
RING_BUFFER_SIZE = 1024  # 固定环形缓冲区大小（实际可用size-1）
ISR_READ_BUF_SIZE = 64  # ISR预分配读取缓冲区
WDT_TIMEOUT = 5000  # 看门狗超时时间（毫秒），设置为5秒
WDT_FEED_PERIOD = 1000  # 喂狗定时器周期（毫秒），设置为1秒

# ====================== 电池电压&灯效核心配置 ======================

BATTERY_ADC_PIN = 26  # GP26（ADC0）采集电池电压（1/2分压）
BATTERY_TIMER_PERIOD = 100  # 电池电压采样定时器周期：100ms
ADC_MAX_VALUE = 65535  # ADC最大值
ADC_REF_VOLTAGE = 3.3  # ADC参考电压（V）
LOW_VOLTAGE_THRESHOLD = 3.4  # 低电压阈值（V）
POWER_ON_SAMPLE_DURATION = 1000  # 上电采样时长：1秒
POWER_ON_SAMPLE_COUNT = 10  # 1秒内采样次数（10次，每次100ms）
RAINBOW_LOOP_TIMES = 2  # 彩虹流动次数：2次
RAINBOW_TOTAL_DURATION = 100  # 彩虹总时长（越小越快）
battery_voltage = 0.0  # 单次采样电压值
battery_voltage_window = []  # 5次滑动窗口缓冲区
WINDOW_SIZE = 5  # 滑动滤波窗口大小（5次）
low_battery_flag = False  # 低电压标志
prev_low_battery = False  # 上一次电压状态（用于检测状态变化）

# ====================== 调度标志位配置  ======================

is_scheduled = False  # UART数据处理调度标志
wdt_print_scheduled = False  # 看门狗打印调度标志

# ====================== WS2812配置 ======================

WS2812_PIN = 2
WS2812_NUM = 16

# ======================================== 功能函数 ============================================

# ======================================== 自定义类 ============================================

# ======================================== 初始化配置 ==========================================

# ========================================  主程序  ===========================================
