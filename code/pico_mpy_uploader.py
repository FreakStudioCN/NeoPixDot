# Python env   : MicroPython v1.27
# -*- coding: utf-8 -*-
# @Time    : 2025/9/5 下午10:12
# @Author  : 李清水
# @File    : pico_mpy_uploader.py
# @Description : pico_mpy_uploader，编译mpy文件
# @License : CC BY-NC 4.0

__version__ = "0.1.0"
__author__ = "李清水"
__license__ = "CC BY-NC 4.0"
__platform__ = "MicroPython v1.27"

# ======================================== 导入相关模块 =========================================

import os
import subprocess

# ======================================== 全局变量 ============================================

# ======================================== 功能函数 ============================================

def main():
    # 1. 区分文件：main.py单独处理，其他py文件编译
    current_script = os.path.basename(__file__)
    all_py_files = [f for f in os.listdir(".") if f.endswith(".py") and f != current_script]
    main_file = "main.py" if "main.py" in all_py_files else None
    compile_py_files = [f for f in all_py_files if f != main_file]  # 排除main.py

    if not all_py_files:
        print("当前文件夹没有可处理的Python文件！")
        return

    # 2. 编译非main.py的py文件为mpy
    compiled_mpy = []
    if compile_py_files:
        print("=== 开始编译py文件为mpy（排除main.py）===")
        for py_file in compile_py_files:
            mpy_file = py_file.replace(".py", ".mpy")
            try:
                # 调用mpy-cross编译
                subprocess.run(
                    ["python", "-m", "mpy_cross", py_file, "-o", mpy_file],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                print(f"✅ 编译成功: {py_file} → {mpy_file}")
                compiled_mpy.append(mpy_file)
            except subprocess.CalledProcessError as e:
                print(f"❌ 编译失败 {py_file}: {e.stderr}")

    # 3. 上传文件到Pico
    print("\n=== 开始上传到树莓派Pico ===")
    # 先传编译后的mpy文件
    for mpy_file in compiled_mpy:
        try:
            subprocess.run(
                ["mpremote", "fs", "cp", mpy_file, ":/"],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            print(f"✅ 上传成功: {mpy_file}")
        except subprocess.CalledProcessError as e:
            print(f"❌ 上传失败 {mpy_file}: {e.stderr}")

    # 单独上传main.py（直接传原文件）
    if main_file:
        try:
            subprocess.run(
                ["mpremote", "fs", "cp", main_file, ":/"],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            print(f"✅ 上传成功（直接传py）: {main_file}")
        except subprocess.CalledProcessError as e:
            print(f"❌ 上传失败 {main_file}: {e.stderr}")

# ======================================== 自定义类 ============================================

# ======================================== 初始化配置 ==========================================

# ========================================  主程序  ===========================================

if __name__ == "__main__":
    # 检查依赖工具是否安装
    try:
        subprocess.run(["python", "-m", "mpy_cross", "--version"], check=True, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE)
        subprocess.run(["mpremote", "--version"], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("请先安装依赖工具：")
        print("pip install mpy-cross mpremote")
        exit(1)

    main()