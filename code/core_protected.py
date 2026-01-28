# Python env   : MicroPython v1.27
# -*- coding: utf-8 -*-
# @Time    : 2025/9/5 下午10:12
# @Author  : 李清水
# @File    : core_protected.py
# @Description : 核心功能
# @License : CC BY-NC 4.0

__version__ = "0.1.0"
__author__ = "李清水"
__license__ = "CC BY-NC 4.0"
__platform__ = "MicroPython v1.27"

# ======================================== 导入相关模块 =========================================

import neopixel
from machine import UART, Pin, disable_irq, enable_irq, ADC, Timer, WDT
from config import *
from utils import debug_print, timed_function
from ring_buffer import RingBuffer
import time
import micropython
from ring_buffer import RingBuffer

# ======================================== 全局变量 ============================================

# ======================================== 功能函数 ============================================

@timed_function
def set_ws2812_color(r, g, b):
    for i in range(WS2812_NUM):
        np[i] = (r, g, b)
    np.write()
    debug_print("WS2812 updated: 16 LEDs set to (R:%d, G:%d, B:%d)" % (r, g, b))


# HSV转RGB（颜色空间转换）
def hsv_to_rgb(h, s, v):
    if s == 0.0:
        return (int(v * 255), int(v * 255), int(v * 255))
    i = int(h * 6.0)
    f = (h * 6.0) - i
    p, q, t = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    i = i % 6
    if i == 0:
        r, g, b = v, t, p
    elif i == 1:
        r, g, b = q, v, p
    elif i == 2:
        r, g, b = p, v, t
    elif i == 3:
        r, g, b = p, q, v
    elif i == 4:
        r, g, b = t, p, v
    else:
        r, g, b = v, p, q
    return (int(r * 255), int(g * 255), int(b * 255))


# 彩虹流动效果（新增：关键长操作前手动喂狗）
def rainbow_flow():
    debug_print("=== Rainbow Flow Start (Times: %d, Duration: %dms) ===" % (RAINBOW_LOOP_TIMES, RAINBOW_TOTAL_DURATION))
    # 关键长操作前手动喂狗，避免超时重启
    wdt.feed()
    debug_print("🐶 WDT fed before rainbow flow (long operation)")

    step_delay = RAINBOW_TOTAL_DURATION / (WS2812_NUM * RAINBOW_LOOP_TIMES)
    for _ in range(RAINBOW_LOOP_TIMES):
        for hue in range(360):
            for i in range(WS2812_NUM):
                pixel_hue = (hue + i * 10) % 360
                r, g, b = hsv_to_rgb(pixel_hue / 360.0, 1.0, 1.0)
                np[i] = (r, g, b)
            np.write()
            time.sleep_ms(int(step_delay))
    wdt.feed()
    set_ws2812_color(0, 0, 0)
    debug_print("=== Rainbow Flow End ===")


# 上电电压检测（初始化时采集多次取平均，提高准确性）
def power_on_battery_check():
    debug_print("=== Power On Battery Check ===")
    # 长耗时采样前手动喂狗
    wdt.feed()
    debug_print("🐶 WDT fed before power-on battery check (long operation)")

    voltages = []
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < POWER_ON_SAMPLE_DURATION:
        adc_value = adc.read_u16()
        voltage = (adc_value / ADC_MAX_VALUE) * ADC_REF_VOLTAGE * 2
        voltages.append(voltage)
        time.sleep_ms(POWER_ON_SAMPLE_DURATION // POWER_ON_SAMPLE_COUNT)
    avg_voltage = round(sum(voltages) / len(voltages), 2)
    debug_print("Power On Average Voltage: %.2fV (Threshold: %.1fV)" % (avg_voltage, LOW_VOLTAGE_THRESHOLD))
    # 初始化滑动窗口：上电检测的平均值填充窗口
    global battery_voltage_window
    battery_voltage_window = [avg_voltage] * WINDOW_SIZE
    return avg_voltage


# ====================== 电池电压读取&滑动滤波函数 ======================
def read_battery_adc(timer):
    global battery_voltage, battery_voltage_window
    adc_value = adc.read_u16()
    # 计算实际电压（1/2分压，所以乘以2）
    voltage = (adc_value / ADC_MAX_VALUE) * ADC_REF_VOLTAGE * 2
    battery_voltage = round(voltage, 2)

    # 更新滑动窗口（保留最近5次采样值）
    battery_voltage_window.append(battery_voltage)
    if len(battery_voltage_window) > WINDOW_SIZE:
        battery_voltage_window.pop(0)  # 移除最旧的数值


# 计算滑动窗口的平均电压（防抖核心）
def get_battery_avg_voltage():
    if not battery_voltage_window:
        return 0.0
    avg_volt = round(sum(battery_voltage_window) / len(battery_voltage_window), 2)
    return avg_volt


# ====================== 看门狗打印调度函数 ======================
def wdt_feed_print(_):
    """看门狗喂狗打印的调度执行函数（非中断上下文）"""
    global wdt_print_scheduled
    debug_print("🐶 WDT fed (timer callback)")
    wdt_print_scheduled = False  # 执行完成后重置标志位


# ====================== 看门狗喂狗回调函数（软件定时器触发） ======================
def wdt_feed_callback(timer):
    """
    看门狗喂狗回调函数
    由1秒周期的软件定时器触发，执行喂狗操作并调度打印
    """
    global wdt_print_scheduled
    wdt.feed()  # 重置看门狗超时计数器（喂狗核心操作）

    # 调度打印操作，避免中断上下文直接print，且防止重复调度
    if not wdt_print_scheduled:
        try:
            micropython.schedule(wdt_feed_print, None)
            wdt_print_scheduled = True
        except RuntimeError as e:
            # 调度队列满时仅在调试模式输出错误
            debug_print("⚠️ WDT print schedule queue full: %s" % str(e))
            wdt_print_scheduled = False


# ====================== UART数据处理函数 ======================
@timed_function
def parse_rgb_data(data):
    if len(data) >= 3:
        r, g, b = data[0], data[1], data[2]
        debug_print("Parsed RGB data (hex): %s | (R,G,B): (%d, %d, %d)" % (data[:3].hex(), r, g, b))
        return (r, g, b)
    else:
        debug_print("Insufficient data (%d bytes), cannot parse RGB" % len(data))
        return None


@timed_function
def forward_remaining_data(data):
    if len(data) >= 3:
        forward_data = data[3:]
        if len(forward_data) > 0:
            debug_print("Forwarded data (hex): %s | Length: %d bytes" % (forward_data.hex(), len(forward_data)))
            uart_forward.write(forward_data)
        else:
            debug_print("No remaining data to forward")
    else:
        debug_print("No data to forward (total bytes: %d)" % len(data))


@timed_function
def process_received_data(_):
    global is_scheduled
    is_scheduled = False

    data = ring_buffer.read_all()
    if len(data) == 0:
        return

    debug_print("\n=== Received Data ===")
    debug_print("Raw data (hex): %s" % bytes(data).hex())
    debug_print("Total bytes received: %d" % len(data))

    rgb_values = parse_rgb_data(data)
    # 低电压时禁用UART控制LED
    if rgb_values and not low_battery_flag:
        set_ws2812_color(*rgb_values)
    forward_remaining_data(data)


# ====================== ISR中断回调 ======================
def uart_idle_callback(uart):
    global is_scheduled, isr_read_buf, ring_buffer

    read_len = uart.readinto(isr_read_buf)
    if read_len == 0:
        return

    # 将接收到的数据写入环形缓冲区
    ring_buffer.write(isr_read_buf, read_len)

    # 避免重复调度处理函数
    if not is_scheduled:
        try:
            micropython.schedule(process_received_data, None)
            is_scheduled = True
        except RuntimeError as e:
            debug_print("⚠️ Schedule queue full: %s" % str(e))
            is_scheduled = False

# ======================================== 自定义类 ============================================

# ======================================== 初始化配置 ==========================================

# 初始化核心组件
ring_buffer = RingBuffer(RING_BUFFER_SIZE)

# 初始化看门狗（Watch Dog Timer）
# 超时时间设置为5000ms（5秒），若超过5秒未喂狗则自动重启设备
wdt = WDT(timeout=WDT_TIMEOUT)
np = neopixel.NeoPixel(Pin(WS2812_PIN), WS2812_NUM)
# 初始化ADC（电池电压采集）
adc = ADC(Pin(BATTERY_ADC_PIN))
isr_read_buf = bytearray(ISR_READ_BUF_SIZE)
uart_forward = UART(1, baudrate=BAUDRATE, tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1)

# ========================================  主程序  ===========================================