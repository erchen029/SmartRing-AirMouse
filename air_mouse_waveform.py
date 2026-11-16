#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SmartRing Air Mouse 实时多通道波形示波器 (Real-time Motion Waveform Scope)
无需安装 matplotlib，纯 Tkinter 原生硬件级 60FPS 双缓冲平滑绘制。

功能:
  - 实时绘制 陀螺仪角速度 (Gx, Gy, Gz) 动态波形 (红/绿/蓝)
  - 实时绘制 敲击冲击度 (Jerk) 动态波形与触发红线 (黄/橙)
  - 实时绘制 加速度计 (Ax, Ay, Az) 动态波形
  - 敲击手势 (单击左键 / 双击右键) 脉冲视觉高亮与事件记录
  - 虚拟光标轨迹微缩视窗 (直观看到手指晃动如何转换为鼠标移动)
  - 支持【暂停/继续】冻结波形逐帧分析
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import collections
import time
import re
import sys

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误: 未找到 pyserial，请运行 pip install pyserial")
    sys.exit(1)


# 正则匹配固件输出格式:
# [BMI] G:0.17,0.91,1.22 | A:-2.55,2.03,-6.98 | J:0.47 | M:0,0
BMI_REGEX = re.compile(
    r"\[BMI\]\s*G:([-\d\.]+),([-\d\.]+),([-\d\.]+)\s*\|\s*A:([-\d\.]+),([-\d\.]+),([-\d\.]+)\s*\|\s*J:([-\d\.]+)\s*\|\s*M:([-\d]+),([-\d]+)"
)


class WaveformScope:
    def __init__(self, root):
        self.root = root
        self.root.title("SmartRing 飞鼠运动学实时波形分析仪 (Waveform Scope)")
        self.root.geometry("1100x780")
        self.root.minsize(900, 650)
        self.root.configure(bg="#1e1e1e")

        self.ser = None
        self.is_running = False
        self.is_paused = False
        self.rx_thread = None

        # 数据缓冲区 (保留最近 250 个采样点，约 3.75 秒)
        self.BUF_LEN = 250
        self.time_history = collections.deque(maxlen=self.BUF_LEN)
        self.gx_history = collections.deque(maxlen=self.BUF_LEN)
        self.gy_history = collections.deque(maxlen=self.BUF_LEN)
        self.gz_history = collections.deque(maxlen=self.BUF_LEN)
        self.ax_history = collections.deque(maxlen=self.BUF_LEN)
        self.ay_history = collections.deque(maxlen=self.BUF_LEN)
        self.az_history = collections.deque(maxlen=self.BUF_LEN)
        self.jerk_history = collections.deque(maxlen=self.BUF_LEN)
        self.dx_history = collections.deque(maxlen=self.BUF_LEN)
        self.dy_history = collections.deque(maxlen=self.BUF_LEN)

        # 敲击事件标记 [(index, "Single"/"Double", time)]
        self.tap_events = collections.deque(maxlen=20)
        self.left_click_count = 0
        self.right_click_count = 0

        # 最新瞬时值
        self.latest_data = {
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "jerk": 0.0, "dx": 0, "dy": 0
        }

        # 虚拟光标位置
        self.cur_x = 100.0
        self.cur_y = 100.0

        self.setup_ui()
        self.refresh_ports()

        # 启动 50Hz 绘图刷新循环
        self.root.after(20, self.render_loop)

    def setup_ui(self):
        # 顶部操作工具栏
        top_bar = tk.Frame(self.root, bg="#2d2d2d", pady=8, padx=12)
        top_bar.pack(side=tk.TOP, fill=tk.X)

        tk.Label(top_bar, text="端口:", fg="#ffffff", bg="#2d2d2d", font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=(0, 4))
        self.port_combo = ttk.Combobox(top_bar, width=16, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)

        btn_refresh = tk.Button(top_bar, text="🔄 刷新", bg="#3c3c3c", fg="#fff", relief="flat", command=self.refresh_ports)
        btn_refresh.pack(side=tk.LEFT, padx=4)

        tk.Label(top_bar, text="波特率:", fg="#ffffff", bg="#2d2d2d", font=("Segoe UI", 10)).pack(side=tk.LEFT, padx=(12, 4))
        self.baud_combo = ttk.Combobox(top_bar, values=["115200", "9600", "57600", "230400", "1000000"], width=8)
        self.baud_combo.set("115200")
        self.baud_combo.pack(side=tk.LEFT, padx=4)

        self.btn_connect = tk.Button(top_bar, text="⚡ 连接串口", bg="#007acc", fg="#fff", font=("Segoe UI", 10, "bold"),
                                     relief="flat", padx=10, command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=12)

        self.btn_pause = tk.Button(top_bar, text="⏸ 暂停波形", bg="#555555", fg="#fff", relief="flat", padx=8,
                                   command=self.toggle_pause)
        self.btn_pause.pack(side=tk.LEFT, padx=4)

        btn_clear = tk.Button(top_bar, text="🧹 清空", bg="#444444", fg="#fff", relief="flat", padx=8, command=self.clear_data)
        btn_clear.pack(side=tk.LEFT, padx=4)

        self.status_badge = tk.Label(top_bar, text="● 未连接", fg="#888888", bg="#2d2d2d", font=("Segoe UI", 10, "bold"))
        self.status_badge.pack(side=tk.RIGHT, padx=10)

        # 主体布局：左侧波形示波器，右侧仪表盘与虚拟光标
        main_pane = tk.Frame(self.root, bg="#1e1e1e")
        main_pane.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=8)

        # 左侧波形画布容器
        left_frame = tk.Frame(main_pane, bg="#1e1e1e")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 示波器 1: 陀螺仪角速度波形
        self.canvas_gyro = tk.Canvas(left_frame, bg="#121212", highlightthickness=1, highlightbackground="#333333")
        self.canvas_gyro.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(0, 4))

        # 示波器 2: 敲击冲击强度 Jerk 波形 (重点排查点击)
        self.canvas_jerk = tk.Canvas(left_frame, bg="#121212", highlightthickness=1, highlightbackground="#333333", height=180)
        self.canvas_jerk.pack(side=tk.TOP, fill=tk.X, pady=(0, 4))

        # 示波器 3: 加速度计波形
        self.canvas_accel = tk.Canvas(left_frame, bg="#121212", highlightthickness=1, highlightbackground="#333333", height=180)
        self.canvas_accel.pack(side=tk.TOP, fill=tk.X)

        # 右侧信息面板
        right_frame = tk.Frame(main_pane, bg="#252526", width=260, padx=10, pady=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(8, 0))
        right_frame.pack_propagate(False)

        # 右侧卡片 1: 敲击检测状态
        tap_card = tk.LabelFrame(right_frame, text=" 敲击手势识别 ", bg="#252526", fg="#00bcff", font=("Segoe UI", 10, "bold"), padx=8, pady=8)
        tap_card.pack(fill=tk.X, pady=(0, 10))

        self.tap_badge = tk.Label(tap_card, text="静 止 / 待 机", bg="#333333", fg="#aaaaaa", font=("Segoe UI", 13, "bold"), pady=8)
        self.tap_badge.pack(fill=tk.X, pady=(0, 6))

        self.lbl_click_stats = tk.Label(tap_card, text="左键点击: 0 次 | 右键点击: 0 次", bg="#252526", fg="#cccccc", font=("Segoe UI", 9))
        self.lbl_click_stats.pack(anchor="w")

        # 右侧卡片 2: 虚拟光标位移视窗
        cursor_card = tk.LabelFrame(right_frame, text=" 飞鼠虚拟光标视窗 ", bg="#252526", fg="#00bcff", font=("Segoe UI", 10, "bold"), padx=8, pady=8)
        cursor_card.pack(fill=tk.X, pady=(0, 10))

        self.cursor_canvas = tk.Canvas(cursor_card, bg="#0e0e0e", width=220, height=180, highlightthickness=1, highlightbackground="#3a3a3a")
        self.cursor_canvas.pack(pady=4)

        self.lbl_mouse_delta = tk.Label(cursor_card, text="步进: dx = 0, dy = 0", bg="#252526", fg="#00ffcc", font=("Consolas", 10, "bold"))
        self.lbl_mouse_delta.pack(anchor="w")

        # 右侧卡片 3: 实时瞬时数值监控
        inst_card = tk.LabelFrame(right_frame, text=" 瞬时数值监控 ", bg="#252526", fg="#00bcff", font=("Segoe UI", 10, "bold"), padx=8, pady=8)
        inst_card.pack(fill=tk.BOTH, expand=True)

        self.lbl_gyro_val = tk.Label(inst_card, text="Gx:  0.00\nGy:  0.00\nGz:  0.00", bg="#252526", fg="#ff7777",
                                     font=("Consolas", 10), justify=tk.LEFT)
        self.lbl_gyro_val.pack(anchor="w", pady=2)

        self.lbl_jerk_val = tk.Label(inst_card, text="Jerk: 0.00 (门限 13.0)", bg="#252526", fg="#ffdd00",
                                     font=("Consolas", 10, "bold"), justify=tk.LEFT)
        self.lbl_jerk_val.pack(anchor="w", pady=4)

        self.lbl_acc_val = tk.Label(inst_card, text="Ax:  0.00\nAy:  0.00\nAz:  0.00", bg="#252526", fg="#44ff88",
                                    font=("Consolas", 10), justify=tk.LEFT)
        self.lbl_acc_val.pack(anchor="w", pady=2)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            # 优先选择常用端口或最前一个
            default_port = next((p for p in ports if "COM4" in p), ports[0])
            self.port_combo.set(default_port)
        else:
            self.port_combo.set("")

    def toggle_connection(self):
        if not self.is_running:
            port = self.port_combo.get().strip()
            if not port:
                messagebox.showwarning("提示", "请选择有效的串口！")
                return
            baud = int(self.baud_combo.get().strip())
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.is_running = True
                self.btn_connect.config(text="⏹ 断开连接", bg="#cc3333")
                self.status_badge.config(text=f"● 已连接 {port}", fg="#44ff44")

                self.rx_thread = threading.Thread(target=self.serial_read_worker, daemon=True)
                self.rx_thread.start()
            except Exception as e:
                messagebox.showerror("连接失败", f"无法打开 {port}:\n{e}")
        else:
            self.is_running = False
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.btn_connect.config(text="⚡ 连接串口", bg="#007acc")
            self.status_badge.config(text="● 已断开", fg="#888888")

    def toggle_pause(self):
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.btn_pause.config(text="▶ 继续波形", bg="#d97706")
        else:
            self.btn_pause.config(text="⏸ 暂停波形", bg="#555555")

    def clear_data(self):
        self.gx_history.clear()
        self.gy_history.clear()
        self.gz_history.clear()
        self.ax_history.clear()
        self.ay_history.clear()
        self.az_history.clear()
        self.jerk_history.clear()
        self.dx_history.clear()
        self.dy_history.clear()
        self.tap_events.clear()
        self.left_click_count = 0
        self.right_click_count = 0
        self.cur_x, self.cur_y = 110.0, 90.0

    def serial_read_worker(self):
        while self.is_running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # 匹配传感器实时数据
                m = BMI_REGEX.search(line)
                if m:
                    gx, gy, gz = float(m.group(1)), float(m.group(2)), float(m.group(3))
                    ax, ay, az = float(m.group(4)), float(m.group(5)), float(m.group(6))
                    jerk = float(m.group(7))
                    dx, dy = int(m.group(8)), int(m.group(9))

                    self.latest_data = {
                        "gx": gx, "gy": gy, "gz": gz,
                        "ax": ax, "ay": ay, "az": az,
                        "jerk": jerk, "dx": dx, "dy": dy
                    }

                    if not self.is_paused:
                        self.gx_history.append(gx)
                        self.gy_history.append(gy)
                        self.gz_history.append(gz)
                        self.ax_history.append(ax)
                        self.ay_history.append(ay)
                        self.az_history.append(az)
                        self.jerk_history.append(jerk)
                        self.dx_history.append(dx)
                        self.dy_history.append(dy)

                        # 更新虚拟光标位置 (带边界限制)
                        self.cur_x += dx * 0.8
                        self.cur_y += dy * 0.8
                        self.cur_x = max(10, min(210, self.cur_x))
                        self.cur_y = max(10, min(170, self.cur_y))

                # 匹配敲击手势
                if "Single Tap" in line:
                    self.left_click_count += 1
                    self.trigger_tap_ui("🖱️ 单击左键", "#00e676")
                elif "Double Tap" in line:
                    self.right_click_count += 1
                    self.trigger_tap_ui("🖱️ 双击右键", "#ff9100")

            except Exception:
                break

    def trigger_tap_ui(self, text, color):
        def _anim():
            self.tap_badge.config(text=text, bg=color, fg="#000000")
            time.sleep(0.3)
            self.tap_badge.config(text="静 止 / 待 机", bg="#333333", fg="#aaaaaa")
        threading.Thread(target=_anim, daemon=True).start()

    def render_loop(self):
        try:
            self.draw_gyro_scope()
            self.draw_jerk_scope()
            self.draw_accel_scope()
            self.draw_virtual_cursor()
            self.update_labels()
        except Exception:
            pass
        self.root.after(20, self.render_loop)

    def draw_gyro_scope(self):
        cv = self.canvas_gyro
        w = cv.winfo_width()
        h = cv.winfo_height()
        if w < 10 or h < 10:
            return

        cv.delete("all")
        mid_y = h / 2.0

        # 背景网格与刻度
        cv.create_line(0, mid_y, w, mid_y, fill="#2a2a2a", dash=(4, 4))
        cv.create_line(0, mid_y - h * 0.35, w, mid_y - h * 0.35, fill="#1f1f1f", dash=(2, 4))
        cv.create_line(0, mid_y + h * 0.35, w, mid_y + h * 0.35, fill="#1f1f1f", dash=(2, 4))

        # 标题与图例
        cv.create_text(10, 14, anchor="w", text="陀螺仪角速度 Gyro (rad/s)", fill="#ffffff", font=("Segoe UI", 10, "bold"))
        cv.create_text(w - 220, 14, anchor="w", text="— Gx(俯仰)", fill="#ff4d4d", font=("Consolas", 9, "bold"))
        cv.create_text(w - 140, 14, anchor="w", text="— Gy(翻滚)", fill="#00e676", font=("Consolas", 9, "bold"))
        cv.create_text(w - 60, 14, anchor="w", text="— Gz(偏航)", fill="#40c4ff", font=("Consolas", 9, "bold"))

        n = len(self.gx_history)
        if n < 2:
            return

        # 映射量程 ±4.0 rad/s
        scale_y = (h * 0.42) / 4.0
        step_x = (w - 20) / (self.BUF_LEN - 1)

        pts_gx, pts_gy, pts_gz = [], [], []
        start_x = w - 10 - (n - 1) * step_x

        for i in range(n):
            x = start_x + i * step_x
            y_gx = mid_y - self.gx_history[i] * scale_y
            y_gy = mid_y - self.gy_history[i] * scale_y
            y_gz = mid_y - self.gz_history[i] * scale_y

            pts_gx.extend([x, y_gx])
            pts_gy.extend([x, y_gy])
            pts_gz.extend([x, y_gz])

        cv.create_line(pts_gx, fill="#ff4d4d", width=1.8, smooth=True)
        cv.create_line(pts_gy, fill="#00e676", width=1.8, smooth=True)
        cv.create_line(pts_gz, fill="#40c4ff", width=2.2, smooth=True)

    def draw_jerk_scope(self):
        cv = self.canvas_jerk
        w = cv.winfo_width()
        h = cv.winfo_height()
        if w < 10 or h < 10:
            return

        cv.delete("all")
        base_y = h - 25

        # 标题与门限
        cv.create_text(10, 14, anchor="w", text="敲击冲击强度 Jerk (加速度突变率)", fill="#ffbb00", font=("Segoe UI", 10, "bold"))

        # 量程 0 ~ 40 m/s^2
        max_jerk = 40.0
        scale_y = (h - 40) / max_jerk

        # 绘制敲击门限红线 (Threshold = 13.0)
        thresh_y = base_y - 13.0 * scale_y
        cv.create_line(0, thresh_y, w, thresh_y, fill="#ff3333", dash=(4, 3), width=1.5)
        cv.create_text(w - 10, thresh_y - 8, anchor="e", text="敲击触发门限 (13.0)", fill="#ff5555", font=("Segoe UI", 8))

        n = len(self.jerk_history)
        if n < 2:
            return

        step_x = (w - 20) / (self.BUF_LEN - 1)
        start_x = w - 10 - (n - 1) * step_x

        pts_jerk = []
        for i in range(n):
            x = start_x + i * step_x
            val = min(self.jerk_history[i], max_jerk)
            y = base_y - val * scale_y
            pts_jerk.extend([x, y])

            # 超过门限打高亮圆点
            if val >= 13.0:
                cv.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#ffea00", outline="#ff1744")

        cv.create_line(pts_jerk, fill="#ffbb00", width=2.0)

    def draw_accel_scope(self):
        cv = self.canvas_accel
        w = cv.winfo_width()
        h = cv.winfo_height()
        if w < 10 or h < 10:
            return

        cv.delete("all")
        mid_y = h / 2.0

        cv.create_line(0, mid_y, w, mid_y, fill="#2a2a2a", dash=(4, 4))
        cv.create_text(10, 14, anchor="w", text="三轴加速度计 Accel (m/s²)", fill="#ffffff", font=("Segoe UI", 10, "bold"))
        cv.create_text(w - 220, 14, anchor="w", text="— Ax", fill="#ff99bb", font=("Consolas", 9))
        cv.create_text(w - 140, 14, anchor="w", text="— Ay", fill="#aaffaa", font=("Consolas", 9))
        cv.create_text(w - 60, 14, anchor="w", text="— Az", fill="#ffff99", font=("Consolas", 9))

        n = len(self.ax_history)
        if n < 2:
            return

        # 映射量程 ±20 m/s^2
        scale_y = (h * 0.42) / 20.0
        step_x = (w - 20) / (self.BUF_LEN - 1)
        start_x = w - 10 - (n - 1) * step_x

        pts_ax, pts_ay, pts_az = [], [], []
        for i in range(n):
            x = start_x + i * step_x
            y_ax = mid_y - self.ax_history[i] * scale_y
            y_ay = mid_y - self.ay_history[i] * scale_y
            y_az = mid_y - self.az_history[i] * scale_y
            pts_ax.extend([x, y_ax])
            pts_ay.extend([x, y_ay])
            pts_az.extend([x, y_az])

        cv.create_line(pts_ax, fill="#ff99bb", width=1.5)
        cv.create_line(pts_ay, fill="#aaffaa", width=1.5)
        cv.create_line(pts_az, fill="#ffff99", width=1.5)

    def draw_virtual_cursor(self):
        cv = self.cursor_canvas
        cv.delete("all")

        # 绘制中心十字参考线
        cv.create_line(110, 0, 110, 180, fill="#222222", dash=(2, 2))
        cv.create_line(0, 90, 220, 90, fill="#222222", dash=(2, 2))

        # 绘制虚拟光标十字准星与圆圈
        x, y = self.cur_x, self.cur_y
        cv.create_oval(x - 8, y - 8, x + 8, y + 8, outline="#00ffcc", width=2)
        cv.create_line(x - 12, y, x + 12, y, fill="#00ffcc", width=1.5)
        cv.create_line(x, y - 12, x, y + 12, fill="#00ffcc", width=1.5)

    def update_labels(self):
        d = self.latest_data
        self.lbl_gyro_val.config(text=f"Gx: {d['gx']:+6.2f}\nGy: {d['gy']:+6.2f}\nGz: {d['gz']:+6.2f}")
        self.lbl_acc_val.config(text=f"Ax: {d['ax']:+6.2f}\nAy: {d['ay']:+6.2f}\nAz: {d['az']:+6.2f}")
        self.lbl_jerk_val.config(text=f"Jerk: {d['jerk']:5.2f} (门限 13.0)")
        self.lbl_mouse_delta.config(text=f"步进: dx={d['dx']:+3d}, dy={d['dy']:+3d}")
        self.lbl_click_stats.config(text=f"左键点击: {self.left_click_count} 次 | 右键点击: {self.right_click_count} 次")


def main():
    root = tk.Tk()
    app = WaveformScope(root)
    root.mainloop()


if __name__ == "__main__":
    main()
