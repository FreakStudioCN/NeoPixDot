# Python env   : MicroPython v1.27.0
# -*- coding: utf-8 -*-        
# @Time    : 2026/1/27 上午10:51   
# @Author  : 李清水            
# @File    : main.py       
# @Description : 实现 UART 解析 RGB 控制 WS2812 并转发数据，
#                ADC 滑动滤波监测电池电压（低电告警禁 UART 控灯），集成 WDT 防卡死，通过环形缓冲区、中断调度保障运行稳定。

from machine import UART, Pin, disable_irq, enable_irq, ADC, Timer, WDT  # 导入看门狗(WDT)模块
import time
import neopixel
import micropython

# 分配紧急异常缓冲区（防止中断中出现异常时无法打印信息）
micropython.alloc_emergency_exception_buf(100)

# ====================== 全局配置 ======================
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

# ====================== 调度标志位配置 ======================
is_scheduled = False  # UART数据处理调度标志
wdt_print_scheduled = False  # 看门狗打印调度标志


# ====================== 调试打印函数（统一控制输出） ======================
def debug_print(*args, **kwargs):
    """调试打印函数：仅当DEBUG_ENABLE为True时输出"""
    if DEBUG_ENABLE:
        print(*args, **kwargs)


# ====================== 修复后的环形缓冲区（解决满/空歧义） ======================
class RingBuffer:
    def __init__(self, size: int):
        self.buf = bytearray(size)
        self.size = size  # 总容量
        self.head = 0  # 读指针（下一个要读取的位置）
        self.tail = 0  # 写指针（下一个要写入的位置）

    def is_empty(self) -> bool:
        """判断缓冲区是否为空（仅head==tail表示空）"""
        return self.head == self.tail

    def is_full(self) -> bool:
        """判断缓冲区是否为满（预留1字节，避免head==tail歧义）"""
        return (self.tail + 1) % self.size == self.head

    def write(self, data: bytearray, length: int) -> int:
        if length <= 0:
            return 0

        # 计算实际可用的空闲空间（预留1字节，避免满/空歧义）
        if self.is_full():
            debug_print("⚠️ Ring buffer full (usable: %d bytes), discarding data" % (self.size - 1))
            return 0

        # 计算空闲空间大小
        if self.tail >= self.head:
            free = (self.size - self.tail - 1) + self.head  # 减1是预留空间
        else:
            free = self.head - self.tail - 1

        write_len = min(length, free)
        part1_len = min(write_len, self.size - self.tail)
        self.buf[self.tail:self.tail + part1_len] = data[:part1_len]

        part2_len = write_len - part1_len
        if part2_len > 0:
            self.buf[0:part2_len] = data[part1_len:part1_len + part2_len]

        self.tail = (self.tail + write_len) % self.size
        return write_len

    def read_all(self) -> bytearray:
        # 禁用中断防止读写冲突（原子操作）
        irq_state = disable_irq()
        try:
            if self.is_empty():  # 仅用is_empty判断，避免歧义
                return bytearray()

            if self.tail > self.head:
                data = self.buf[self.head:self.tail]
            else:
                data = self.buf[self.head:] + self.buf[:self.tail]

            # 读取后重置指针（保持原有逻辑，也可改为逐字节读取）
            self.head = self.tail = 0
            return data
        finally:
            # 恢复中断状态
            enable_irq(irq_state)


# 初始化核心组件
ring_buffer = RingBuffer(RING_BUFFER_SIZE)
isr_read_buf = bytearray(ISR_READ_BUF_SIZE)


# ====================== 计时装饰器 ======================
def timed_function(f: callable, *args: tuple, **kwargs: dict) -> callable:
    myname = str(f).split(' ')[1]

    def new_func(*args: tuple, **kwargs: dict) -> any:
        t: int = time.ticks_us()
        result = f(*args, **kwargs)
        delta: int = time.ticks_diff(time.ticks_us(), t)
        debug_print('Function %s Time = %6.3fms' % (myname, delta / 1000))
        return result

    return new_func


# ====================== WS2812配置&灯效函数 ======================
WS2812_PIN = 2
WS2812_NUM = 16
np = neopixel.NeoPixel(Pin(WS2812_PIN), WS2812_NUM)


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

    if uart is not uart_recv:
        return

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


# ====================== 初始化组件 ======================
# 初始化ADC（电池电压采集）
adc = ADC(Pin(BATTERY_ADC_PIN))
# 初始化电池电压采集定时器（100ms一次）
battery_timer = Timer(-1)
battery_timer.init(period=BATTERY_TIMER_PERIOD, mode=Timer.PERIODIC, callback=read_battery_adc)

# 初始化UART接收和转发端口
uart_recv = UART(0, baudrate=BAUDRATE, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)
uart_forward = UART(1, baudrate=BAUDRATE, tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1)
# 配置UART空闲中断（接收完成后触发）
uart_recv.irq(handler=uart_idle_callback, trigger=UART.IRQ_RXIDLE, hard=False)

# 初始化看门狗（Watch Dog Timer）
# 超时时间设置为5000ms（5秒），若超过5秒未喂狗则自动重启设备
wdt = WDT(timeout=WDT_TIMEOUT)
debug_print("✅ WDT initialized with timeout: %d seconds" % (WDT_TIMEOUT / 1000))

# 初始化喂狗软件定时器（1秒周期自动喂狗）
wdt_feed_timer = Timer(-1)
wdt_feed_timer.init(period=WDT_FEED_PERIOD, mode=Timer.PERIODIC, callback=wdt_feed_callback)
debug_print("✅ WDT feed timer initialized with period: %d seconds" % (WDT_FEED_PERIOD / 1000))

# ====================== 主程序入口 ======================
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
