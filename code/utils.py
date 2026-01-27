# -*- coding: utf-8 -*-
import time
from config import DEBUG_ENABLE


# ====================== 调试打印函数（统一控制输出） ======================
def debug_print(*args, **kwargs):
    """调试打印函数：仅当DEBUG_ENABLE为True时输出"""
    if DEBUG_ENABLE:
        print(*args, **kwargs)


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
