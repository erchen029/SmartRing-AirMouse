#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SmartRing Air Mouse & BMI270 实时可视化调试上位机 (Tkinter)
功能:
  - 自动扫描串口 (如 COM4) 并一键连接
  - 实时显示 BMI270 陀螺仪 (Gx, Gy, Gz) 与加速度计 (Ax, Ay, Az)
  - 敲击强度 (Jerk) 与 手势状态实时高亮 (单击左键 / 双击右键)
  - 飞鼠相对位移 (dx, dy) 实时监视
  - 终端日志滚动显示与参数命令交互
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import time
import re
import sys

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    messagebox.showerror("缺少依赖", "请先在终端运行: pip install pyserial")
    sys.exit(1)


class AirMouseDebugger:
    def __init__(self, root):
        self.root = root
        self.root.title("SmartRing 飞鼠与 BMI270 调试助手")
        self.root.geometry("860x650")
        self.root.minsize(750, 550)

        self.ser = None
        self.is_running = False
        self.rx_thread = None

        self.setup_ui()
        self.refresh_ports()

    def setup_ui(self):
        # 整体布局: 顶部连接栏, 中部传感器看板, 底部终端日志
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        # 1. 顶部串口连接控制栏
        top_frame = ttk.LabelFrame(self.root, text=" 端口连接 ", padding=10)
        top_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)

        ttk.Label(top_frame, text="串口:").pack(side=tk.LEFT, padx=5)
        self.port_combo = ttk.Combobox(top_frame, width=18, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=5)

        btn_refresh = ttk.Button(top_frame, text="🔄 刷新", width=8, command=self.refresh_ports)
        btn_refresh.pack(side=tk.LEFT, padx=5)

        ttk.Label(top_frame, text="波特率:").pack(side=tk.LEFT, padx=(15, 5))
        self.baud_combo = ttk.Combobox(top_frame, values=["115200", "9600", "57600", "230400", "1000000"], width=10)
        self.baud_combo.set("115200")
        self.baud_combo.pack(side=tk.LEFT, padx=5)

        self.btn_connect = ttk.Button(top_frame, text="⚡ 连接设备", width=12, command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=15)

        self.status_lbl = tk.Label(top_frame, text="● 未连接", fg="gray", font=("Arial", 10, "bold"))
        self.status_lbl.pack(side=tk.RIGHT, padx=10)

        # 2. 中部: BMI270 实时数据与手势看板
        data_frame = ttk.LabelFrame(self.root, text=" BMI270 传感器实时状态 ", padding=10)
        data_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        data_frame.columnconfigure((0, 1, 2), weight=1)

        # 卡片 1: 陀螺仪角速度 (rad/s)
        gyro_card = ttk.LabelFrame(data_frame, text=" 陀螺仪 (角速度) ", padding=8)
        gyro_card.grid(row=0, column=0, sticky="nsew", padx=5)
        self.lbl_gx = ttk.Label(gyro_card, text="Gx (俯仰):  0.00 rad/s", font=("Consolas", 10))
        self.lbl_gx.pack(anchor="w", pady=2)
        self.lbl_gy = ttk.Label(gyro_card, text="Gy (翻滚):  0.00 rad/s", font=("Consolas", 10))
        self.lbl_gy.pack(anchor="w", pady=2)
        self.lbl_gz = ttk.Label(gyro_card, text="Gz (偏航):  0.00 rad/s", font=("Consolas", 10))
        self.lbl_gz.pack(anchor="w", pady=2)

        # 卡片 2: 加速度计 (m/s^2)
        accel_card = ttk.LabelFrame(data_frame, text=" 加速度计 (重力与线加速度) ", padding=8)
        accel_card.grid(row=0, column=1, sticky="nsew", padx=5)
        self.lbl_ax = ttk.Label(accel_card, text="Ax:  0.00 m/s²", font=("Consolas", 10))
        self.lbl_ax.pack(anchor="w", pady=2)
        self.lbl_ay = ttk.Label(accel_card, text="Ay:  0.00 m/s²", font=("Consolas", 10))
        self.lbl_ay.pack(anchor="w", pady=2)
        self.lbl_az = ttk.Label(accel_card, text="Az:  0.00 m/s²", font=("Consolas", 10))
        self.lbl_az.pack(anchor="w", pady=2)

        # 卡片 3: 飞鼠手势与敲击检测
        mouse_card = ttk.LabelFrame(data_frame, text=" 飞鼠与敲击手势识别 ", padding=8)
        mouse_card.grid(row=0, column=2, sticky="nsew", padx=5)

        self.lbl_jerk = ttk.Label(mouse_card, text="敲击冲击 (Jerk): 0.00", font=("Consolas", 10))
        self.lbl_jerk.pack(anchor="w", pady=2)

        self.lbl_mouse_delta = ttk.Label(mouse_card, text="光标位移: (dx=0, dy=0)", font=("Consolas", 10))
        self.lbl_mouse_delta.pack(anchor="w", pady=2)

        self.gesture_badge = tk.Label(mouse_card, text="静止待机中", bg="#e0e0e0", fg="#333",
                                      font=("微软雅黑", 10, "bold"), width=16, height=1, relief="groove")
        self.gesture_badge.pack(anchor="w", pady=4)

        # 3. 底部: 终端日志接收区
        log_frame = ttk.LabelFrame(self.root, text=" 实时调试输出 / 串口日志 ", padding=10)
        log_frame.grid(row=2, column=0, sticky="nsew", padx=10, pady=5)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.txt_log = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, font=("Consolas", 9), bg="#1e1e1e", fg="#d4d4d4")
        self.txt_log.grid(row=0, column=0, columnspan=3, sticky="nsew", pady=5)

        # 日志控制按键
        btn_clear = ttk.Button(log_frame, text="清屏", command=self.clear_log)
        btn_clear.grid(row=1, column=0, sticky="w", pady=2)

        self.auto_scroll_var = tk.BooleanVar(value=True)
        chk_scroll = ttk.Checkbutton(log_frame, text="自动滚屏", variable=self.auto_scroll_var)
        chk_scroll.grid(row=1, column=1, sticky="w", pady=2)

        # 发送调试命令栏
        cmd_frame = ttk.Frame(log_frame)
        cmd_frame.grid(row=1, column=2, sticky="e", pady=2)
        ttk.Label(cmd_frame, text="发送指令:").pack(side=tk.LEFT, padx=5)
        self.ent_cmd = ttk.Entry(cmd_frame, width=25)
        self.ent_cmd.pack(side=tk.LEFT, padx=5)
        self.ent_cmd.bind("<Return>", lambda e: self.send_cmd())
        btn_send = ttk.Button(cmd_frame, text="发送", command=self.send_cmd)
        btn_send.pack(side=tk.LEFT)

    def refresh_ports(self):
        """扫描当前可用串口"""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device} ({p.description.split('(')[0].strip()})" for p in ports]
        self.port_combo['values'] = port_list
        if port_list:
            # 优先自动选中 COM4 或首个串口
            selected = False
            for p in port_list:
                if "COM4" in p or "USB" in p or "nRF" in p:
                    self.port_combo.set(p)
                    selected = True
                    break
            if not selected:
                self.port_combo.current(0)
        else:
            self.port_combo.set("")

    def toggle_connection(self):
        if not self.ser or not self.ser.is_open:
            self.connect_serial()
        else:
            self.disconnect_serial()

    def connect_serial(self):
        port_entry = self.port_combo.get()
        if not port_entry:
            messagebox.showwarning("提示", "未选择可用串口！")
            return

        port = port_entry.split()[0]
        baud = int(self.baud_combo.get())

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.is_running = True
            self.btn_connect.config(text="断开连接")
            self.status_lbl.config(text=f"● 已连接 ({port})", fg="green")
            self.append_log(f"--- 成功连接到 {port} @ {baud} bps ---\n")

            self.rx_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.rx_thread.start()
        except Exception as e:
            messagebox.showerror("连接失败", f"无法打开端口 {port}:\n{str(e)}")

    def disconnect_serial(self):
        self.is_running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.btn_connect.config(text="⚡ 连接设备")
        self.status_lbl.config(text="● 未连接", fg="gray")
        self.append_log("--- 串口已断开 ---\n")

    def read_serial_loop(self):
        """后台线程读取串口数据并解析"""
        while self.is_running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                if line:
                    self.root.after(0, self.handle_line, line)
            except Exception:
                break
        self.root.after(0, self.disconnect_serial)

    def handle_line(self, line):
        """解析串口单行数据"""
        self.append_log(line)

        # 匹配结构化日志: [BMI] G:gx,gy,gz | A:ax,ay,az | J:jerk | M:dx,dy | EVT:evt
        pattern = r"G:([-\d\.]+),([-\d\.]+),([-\d\.]+).*?A:([-\d\.]+),([-\d\.]+),([-\d\.]+).*?J:([-\d\.]+)"
        m = re.search(pattern, line)
        if m:
            gx, gy, gz, ax, ay, az, jerk = map(float, m.groups())
            self.lbl_gx.config(text=f"Gx (俯仰): {gx:6.2f} rad/s")
            self.lbl_gy.config(text=f"Gy (翻滚): {gy:6.2f} rad/s")
            self.lbl_gz.config(text=f"Gz (偏航): {gz:6.2f} rad/s")

            self.lbl_ax.config(text=f"Ax: {ax:6.2f} m/s²")
            self.lbl_ay.config(text=f"Ay: {ay:6.2f} m/s²")
            self.lbl_az.config(text=f"Az: {az:6.2f} m/s²")

            self.lbl_jerk.config(text=f"敲击冲击 (Jerk): {jerk:6.2f}")

        # 匹配鼠标位移
        m_mouse = re.search(r"M:([-\d]+),([-\d]+)", line)
        if m_mouse:
            dx, dy = m_mouse.groups()
            self.lbl_mouse_delta.config(text=f"光标位移: (dx={dx}, dy={dy})")

        # 匹配敲击手势事件
        if "Single Tap" in line or "LEFT CLICK" in line:
            self.gesture_badge.config(text="🖱️ 左键单击 (Single)", bg="#4caf50", fg="white")
            self.root.after(800, self.reset_badge)
        elif "Double Tap" in line or "RIGHT CLICK" in line:
            self.gesture_badge.config(text="🖱️ 右键单击 (Double)", bg="#2196f3", fg="white")
            self.root.after(800, self.reset_badge)

    def reset_badge(self):
        self.gesture_badge.config(text="静止待机中", bg="#e0e0e0", fg="#333")

    def append_log(self, text):
        self.txt_log.insert(tk.END, text)
        if self.auto_scroll_var.get():
            self.txt_log.see(tk.END)

    def clear_log(self):
        self.txt_log.delete(1.0, tk.END)

    def send_cmd(self):
        cmd = self.ent_cmd.get().strip()
        if not cmd:
            return
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\r\n").encode('utf-8'))
            self.append_log(f">>> 发送: {cmd}\n")
            self.ent_cmd.delete(0, tk.END)
        else:
            messagebox.showwarning("提示", "请先连接串口设备！")


if __name__ == '__main__':
    root = tk.Tk()
    app = AirMouseDebugger(root)
    root.mainloop()
