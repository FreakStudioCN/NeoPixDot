# Python env   : MicroPython v1.27
# -*- coding: utf-8 -*-
# @Time    : 2025/9/5 下午10:12
# @Author  : 李清水
# @File    : utils.py
# @Description : utils，通用函数文件
# @License : CC BY-NC 4.0

__version__ = "0.1.0"
__author__ = "李清水"
__license__ = "CC BY-NC 4.0"
__platform__ = "MicroPython v1.27"

# ======================================== 导入相关模块 =========================================

import time
from config import DEBUG_ENABLE

# ======================================== 全局变量 ============================================

# ======================================== 功能函数 ============================================

# 调试打印函数（统一控制输出）
def debug_print(*args, **kwargs):
    """调试打印函数：仅当DEBUG_ENABLE为True时输出"""
    if DEBUG_ENABLE:
        print(*args, **kwargs)

# 计时装饰器
def timed_function(f: callable, *args: tuple, **kwargs: dict) -> callable:
    myname = str(f).split(' ')[1]

    def new_func(*args: tuple, **kwargs: dict) -> any:
        t: int = time.ticks_us()
        result = f(*args, **kwargs)
        delta: int = time.ticks_diff(time.ticks_us(), t)
        debug_print('Function %s Time = %6.3fms' % (myname, delta / 1000))
        return result

    return new_func

# ======================================== 自定义类 ============================================

# ======================================== 初始化配置 ==========================================

# ========================================  主程序  ===========================================