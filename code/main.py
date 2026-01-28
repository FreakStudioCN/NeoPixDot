# Python env   : MicroPython v1.27
# -*- coding: utf-8 -*-
# @Time    : 2026/1/27 上午10:51
# @Author  : 李清水
# @File    : main.py
# @Description : 实现 UART 解析 RGB 控制 WS2812 并转发数据，
#                ADC 滑动滤波监测电池电压（低电告警禁 UART 控灯），集成 WDT 防卡死，通过环形缓冲区、中断调度保障运行稳定。

# ======================================== 导入相关模块 =========================================

from machine import UART, Pin, disable_irq, enable_irq, ADC, Timer, WDT
import time
import neopixel
import micropython
from config import *
from utils import debug_print, timed_function
from core_protected import *

# ======================================== 全局变量 ============================================

# ======================================== 功能函数 ============================================

# ======================================== 自定义类 ============================================

# ======================================== 初始化配置 ==========================================

# 分配紧急异常缓冲区（防止中断中出现异常时无法打印信息）
micropython.alloc_emergency_exception_buf(100)

# 初始化电池电压采集定时器（100ms一次）
battery_timer = Timer(-1)
battery_timer.init(period=BATTERY_TIMER_PERIOD, mode=Timer.PERIODIC, callback=read_battery_adc)

# 初始化UART接收和转发端口
uart_recv = UART(0, baudrate=BAUDRATE, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)
# 配置UART空闲中断（接收完成后触发）
uart_recv.irq(handler=uart_idle_callback, trigger=UART.IRQ_RXIDLE, hard=False)
debug_print("✅ WDT initialized with timeout: %d seconds" % (WDT_TIMEOUT / 1000))

# 初始化喂狗软件定时器（1秒周期自动喂狗）
wdt_feed_timer = Timer(-1)
wdt_feed_timer.init(period=WDT_FEED_PERIOD, mode=Timer.PERIODIC, callback=wdt_feed_callback)
debug_print("✅ WDT feed timer initialized with period: %d seconds" % (WDT_FEED_PERIOD / 1000))

# ========================================  主程序  ===========================================

if __name__ == "__main__":
    debug_print("=== UART+WS2812+Battery Monitor ===")
    debug_print("UART Baudrate: %d" % BAUDRATE)
    debug_print("WS2812: GP%d, %d LEDs" % (WS2812_PIN, WS2812_NUM))
    debug_print("Battery ADC: GP%d, Threshold: %.1fV, Sliding Window: %d samples" % (
    BATTERY_ADC_PIN, LOW_VOLTAGE_THRESHOLD, WINDOW_SIZE))
    debug_print("RingBuffer: Size=%d bytes, Usable=%d bytes (reserved 1 byte for full/empty distinguish)" % (
    RING_BUFFER_SIZE, RING_BUFFER_SIZE - 1))
    debug_print("Debug Mode: %s" % ("Enabled" if DEBUG_ENABLE else "Disabled"))

    # 上电电压检测
    avg_voltage = power_on_battery_check()
    if avg_voltage < LOW_VOLTAGE_THRESHOLD:
        low_battery_flag = True
        debug_print("⚠️ Low Battery! (Avg: %.2fV < %.1fV) → Red LED On" % (avg_voltage, LOW_VOLTAGE_THRESHOLD))
        set_ws2812_color(255, 0, 0)
    else:
        low_battery_flag = False
        debug_print("✅ Battery Normal (Avg: %.2fV) → Rainbow Flow" % avg_voltage)
        rainbow_flow()

    # 初始化上一次状态
    prev_low_battery = low_battery_flag
    flash_count = 0

    # 主循环
    debug_print("\n=== Battery Voltage Monitor (Sliding Filter) ===")
    while True:
        # 打印实时电压和5次平均电压（每500ms一次）
        if flash_count % 5 == 0:
            avg_volt = get_battery_avg_voltage()
            debug_print("Battery Voltage - Single: %.2fV | Avg(5): %.2fV | Low Battery: %s" % (
            battery_voltage, avg_volt, low_battery_flag))

        # 1. 检测当前电压状态（基于5次滑动平均值）
        avg_volt = get_battery_avg_voltage()
        current_low_battery = avg_volt < LOW_VOLTAGE_THRESHOLD

        # 2. 低电压→正常电压 恢复逻辑
        if prev_low_battery and not current_low_battery:
            debug_print("✅ Battery Recovered! (Avg: %.2fV ≥ %.1fV) → LED Off, Restore UART Control" % (
            avg_volt, LOW_VOLTAGE_THRESHOLD))
            low_battery_flag = False  # 清除低电压标志
            set_ws2812_color(0, 0, 0)  # 关闭红灯
        # 3. 正常→低电压 告警逻辑
        elif not prev_low_battery and current_low_battery:
            debug_print("⚠️ Battery Low! (Avg: %.2fV < %.1fV) → Red LED Flash" % (avg_volt, LOW_VOLTAGE_THRESHOLD))
            low_battery_flag = True

        # 4. 低电压时红灯闪烁
        if current_low_battery:
            if flash_count % 10 < 5:
                set_ws2812_color(255, 0, 0)
            else:
                set_ws2812_color(0, 0, 0)

        # 5. 更新上一次状态（用于下一次循环对比）
        prev_low_battery = current_low_battery

        # 计数器递增
        flash_count += 1

        # 关键修改：主循环末尾补充喂狗
        wdt.feed()
        debug_print("🐶 WDT fed (main loop end)")

        time.sleep_ms(100)