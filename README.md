# SmartRing Air Mouse (智能戒指空中飞鼠)

基于 **nRF Connect SDK (NCS / Zephyr RTOS)** 开发的低功耗智能戒指空中飞鼠开源固件。

- **主控芯片**：Nordic nRF52840（Nice!Nano / XIAO BLE 形态），支持 Adafruit UF2 Bootloader 免仿真器刷写。
- **IMU 传感器**：Bosch BMI270 六轴传感器（I2C 地址 `0x69`）。
- **人机交互**：无外部机械按键，全靠手势与敲击识别（单击左键、双击右键、光标悬空自适应防抖跟踪）。

---

## 🌟 核心特性

1. **工业级 4 层高级飞鼠手感算法链**：
   - **动态零偏自学习（Zero-rate Calibrator）**：静止时动态补偿微小温漂与底噪，放桌上或手持悬停时绝对 0 漂移。
   - **自适应生理抗手抖平滑（Dynamic EMA Filter）**：慢速瞄准强平滑去手颤，甩动大幅滑移零延迟跟手。
   - **非线性动力加速度曲线（Ballistics）**：微动手腕实现 1:1 像素级精准定位，手腕一甩指数加速跨屏。
   - **亚像素微积分累加器（Sub-pixel Integrator）**：杜绝整数截断丢步，微小滑动丝般顺滑。
   - **敲击光标防抖锁定（Click Freeze Protection）**：检测到敲击瞬间自动冻结光标 120ms，彻底杜绝“点击变成拖拽”。

2. **敲击手势识别（Tap Gestures）**：
   - **轻敲一次（Single Tap）**：触发**鼠标左键**。
   - **快速轻敲两次（Double Tap）**：触发**鼠标右键**。

3. **标准低功耗蓝牙 (BLE HID Mouse)**：
   - 采用标准 HID Mouse Profile，即连即用，免驱支持 Windows / macOS / Android / iOS / Linux。
   - 内置 **Just Works & 数值比对** 双重全自动安全连接协议，掉线断开后无缝自动重播。

4. **配套 PC 端实时可视化调试器**：
   - 提供基于 Python Tkinter 的图形上位机 (`air_mouse_debugger.py`)，可视化显示 6 轴加速度、角速度、敲击瞬时 Jerk 冲击波形与鼠标相对位移。

---

## 🔌 硬件接线说明

| BMI270 引脚 | Nice!Nano (ProMicro) 丝印 | nRF52840 GPIO | 功能说明 |
| :--- | :--- | :--- | :--- |
| **VCC** | **VCC / 3.3V** | 3.3V | 核心供电 (3.3V) |
| **GND** | **GND** | GND | 系统共地 |
| **SCL** | **D0** | `P0.20` | I2C0 SCL 时钟线 (内部上拉) |
| **SDA** | **D1** | `P0.17` | I2C0 SDA 数据线 (内部上拉) |
| **CS** | **3.3V** | - | 接高电平，使能 I2C 模式 |
| **SDO** | **3.3V** | - | 接高电平，I2C 物理从机地址为 `0x69` |

> **状态指示灯**：`P0.15` 为板载指示灯。未连蓝牙时 500ms 慢闪，蓝牙配对成功后常亮。

---

## 🛠️ 编译与烧录

### 环境要求
- **nRF Connect SDK**：v2.7.0
- **Zephyr SDK**：0.16.5
- **Toolchain**：NCS Toolchain (`ce3b5ff664`)

### 命令行编译
在工程根目录执行：
```powershell
west build -b xiao_ble
```

### UF2 一键烧录
1. 双击板子上的复位键（Reset），板载指示灯常亮，电脑弹出 `NICENANO` 虚拟 U 盘。
2. 双击运行 `flash_to_nicenano.bat` 脚本（或将 `build/zephyr/zephyr.uf2` 拖入 `NICENANO` 盘）。
3. 烧录完成后板子自动重启，开始广播 `SmartRing Mouse`。

---

## 💻 上位机调试器运行

双击运行 `run_debugger.bat`，或在命令行执行：
```powershell
python air_mouse_debugger.py
```
选择板子对应的串口（如 `COM4`，波特率 `115200`），点击连接即可实时监控传感器数据与敲击波形。

---

## 📄 License
MIT License
