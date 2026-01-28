# Python env   : MicroPython v1.27
# -*- coding: utf-8 -*-
# @Time    : 2025/9/5 下午10:12
# @Author  : 李清水
# @File    : ring_buffer.py
# @Description : 环形缓冲区
# @License : CC BY-NC 4.0

__version__ = "0.1.0"
__author__ = "李清水"
__license__ = "CC BY-NC 4.0"
__platform__ = "MicroPython v1.27"

# ======================================== 导入相关模块 =========================================

from machine import disable_irq, enable_irq
# 导入配置（编译mpy后仍可正常导入明文配置）
from config import DEBUG_ENABLE

# ======================================== 全局变量 ============================================


# ======================================== 功能函数 ============================================

# ======================================== 自定义类 ============================================

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

# ======================================== 初始化配置 ==========================================

# ========================================  主程序  ===========================================
