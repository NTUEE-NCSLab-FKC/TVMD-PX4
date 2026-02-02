# TVMD SITL 遥控器手动控制指南

本指南介绍如何使用各种遥控输入方式手动控制 TVMD SITL。

## 🎯 可用的控制方式

| 方式 | 硬件需求 | 难度 | 真实感 | 适用场景 |
|------|---------|------|--------|---------|
| **QGC 虚拟摇杆** | 无 | ⭐ | ⭐⭐ | 日常测试、演示 |
| **游戏手柄** | USB 手柄 | ⭐⭐ | ⭐⭐⭐⭐ | 真实操作感、长时间测试 |
| **实际遥控器** | RC 遥控器 | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 专业测试、真实飞行准备 |
| **键盘模拟** | 无 | ⭐ | ⭐ | 临时测试 |

---

## 🎮 方案 A：QGroundControl 虚拟摇杆（推荐）

**最简单的方案，无需额外硬件。**

### 优点
- ✅ 无需额外硬件
- ✅ 设置简单
- ✅ 跨平台（Windows/Mac/Linux）
- ✅ 完整的飞行控制

### 缺点
- ⚠️ 触摸屏/鼠标操作不如实体摇杆精确
- ⚠️ 长时间操作较累

### 配置步骤

#### 1. 安装 QGroundControl（在 Windows 主机）

下载地址：https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

#### 2. 启动 TVMD SITL（在 VirtualBox Linux）

```bash
cd ~/TVMD-PX4
./launch_tvmd_headless.sh
```

#### 3. 连接 QGroundControl

QGC 应该自动检测到 PX4 SITL。如果没有：

1. 点击顶部 **Q** 图标
2. **Application Settings** → **Comm Links**
3. 确认有 UDP 连接：
   - Type: UDP
   - Listening Port: 14550
   - Target Hosts: 127.0.0.1

#### 4. 启用虚拟摇杆

1. **Q** → **Application Settings** → **General**
2. 找到 **Virtual Joystick** 部分
3. 勾选 **✅ Enable Virtual Joystick**
4. 选择 **Joystick Mode**：
   - **Mode 2**（推荐）：左油门+偏航，右俯仰+横滚
   - **Mode 1**：左俯仰+偏航，右油门+横滚

#### 5. 手动飞行

返回飞行视图，屏幕上会显示两个虚拟摇杆：

**左摇杆**（油门 + 偏航）：
- 上推/下拉：增加/减少油门（升降）
- 左推/右推：逆时针/顺时针偏航（旋转）

**右摇杆**（俯仰 + 横滚）：
- 上推/下拉：向前/向后飞行（俯仰）
- 左推/右推：向左/向右飞行（横滚）

**基本操作流程**：
```
1. 点击左侧 "ARM" 按钮 → 电机怠速
2. 左摇杆缓慢上推 → 增加油门，起飞
3. 右摇杆控制方向
4. 左摇杆缓慢下拉 → 降低油门，降落
5. 点击 "DISARM" → 解除武装
```

### 虚拟摇杆设置选项

在 **Application Settings** → **General** → **Virtual Joystick**：

- **Centered throttle**: 油门是否居中（一般选择 No）
- **Spring loaded throttle**: 松开摇杆后油门是否回中（一般选择 No）
- **Allow negative thrust**: 允许负推力（一般选择 No）

---

## 🕹️ 方案 B：游戏手柄/游戏控制器

**推荐用于长时间测试和真实操作感。**

### 支持的设备

- ✅ Xbox 360/One/Series 手柄
- ✅ PlayStation 3/4/5 DualShock 手柄
- ✅ Logitech F310/F710 游戏手柄
- ✅ Steam Controller
- ✅ 任何标准 USB/蓝牙游戏手柄

### 配置步骤

#### 1. 连接手柄到 VirtualBox

**Windows 主机**：
1. 插入 USB 手柄（或连接蓝牙）
2. VirtualBox → **设备** → **USB** → 选择您的手柄
3. 手柄设备传递到 Linux 虚拟机

#### 2. 安装 joystick 工具（Linux）

```bash
# 安装 joystick 支持
sudo apt-get install joystick jstest-gtk

# 验证手柄识别
ls /dev/input/js*
# 应该看到：/dev/input/js0

# 测试手柄输入
jstest /dev/input/js0
# 移动摇杆和按按钮，观察数值变化
```

#### 3. 校准手柄（可选）

```bash
# 使用图形化工具校准
jstest-gtk

# 或使用命令行
jscal /dev/input/js0
```

#### 4. 启动 SITL with Joystick

```bash
cd ~/TVMD-PX4

# 使用 joystick 支持的启动脚本
./launch_tvmd_with_joystick.sh
```

#### 5. 验证手柄连接

在 PX4 shell 中：

```bash
# 查看 RC 输入
pxh> listener manual_control_setpoint

# 移动手柄摇杆，应该看到数值变化：
# - x: 横滚 (-1.0 到 1.0)
# - y: 俯仰 (-1.0 到 1.0)
# - z: 油门 (0.0 到 1.0)
# - r: 偏航 (-1.0 到 1.0)
```

### 默认按键映射

**摇杆**（根据手柄类型可能不同）：
- **左摇杆 上/下**：油门
- **左摇杆 左/右**：偏航
- **右摇杆 上/下**：俯仰
- **右摇杆 左/右**：横滚

**按钮**（Xbox 手柄示例）：
- **A (下)**：Arm/Disarm 切换
- **B (右)**：返航（RTL）
- **X (左)**：悬停模式
- **Y (上)**：任务模式
- **LB/RB**：模式切换
- **Start**：起飞
- **Back**：降落

### 自定义按键映射

编辑 PX4 参数：

```bash
pxh> param set RC_MAP_THROTTLE 1    # 通道 1 = 油门
pxh> param set RC_MAP_ROLL 2        # 通道 2 = 横滚
pxh> param set RC_MAP_PITCH 3       # 通道 3 = 俯仰
pxh> param set RC_MAP_YAW 4         # 通道 4 = 偏航
pxh> param set RC_MAP_MODE_SW 5     # 通道 5 = 模式开关
pxh> param set RC_MAP_ARM_SW 6      # 通道 6 = Arm 开关
pxh> param save
```

---

## 🎛️ 方案 C：实际遥控器

**最真实的控制体验，适合专业测试。**

### 支持的遥控器

| 品牌 | 型号 | 连接方式 | 兼容性 |
|------|------|---------|--------|
| **FrSky** | Taranis X9D/X9D+/X10/X12S | USB | ✅ 优秀 |
| **Spektrum** | DX6/DX8/DX9 | USB + 适配器 | ✅ 良好 |
| **FlySky** | FS-i6/FS-i6X/FS-i6S | USB | ✅ 良好 |
| **Jumper** | T16/T18 | USB | ✅ 优秀 |
| **RadioMaster** | TX16S/TX12 | USB | ✅ 优秀 |

### FrSky Taranis 配置示例

#### 1. 硬件连接

1. Taranis 通过 **Mini-USB** 连接到 Windows
2. Taranis 进入 **USB Joystick Mode**：
   - 长按电源键开机
   - 选择 **USB Joystick (HID)**
3. VirtualBox → **设备** → **USB** → 选择 FrSky 遥控器

#### 2. 在 Linux 中验证

```bash
# 查看 USB 设备
lsusb | grep -i frsky
# 应该看到：Bus 001 Device XXX: ID 0483:5740 STMicroelectronics Virtual COM Port

# 查看 joystick 设备
ls -l /dev/input/js0
# 应该存在

# 测试遥控器输入
jstest /dev/input/js0
# 移动摇杆，观察 8-16 个通道的数值变化
```

#### 3. 配置 PX4

在启动脚本中或 PX4 shell：

```bash
# 启动 joystick 驱动
linux_joystick start /dev/input/js0

# 配置通道映射（FrSky Taranis 默认）
param set RC_MAP_THROTTLE 3    # 通道 3 = 油门（左摇杆上下）
param set RC_MAP_ROLL 1         # 通道 1 = 横滚（右摇杆左右）
param set RC_MAP_PITCH 2        # 通道 2 = 俯仰（右摇杆上下）
param set RC_MAP_YAW 4          # 通道 4 = 偏航（左摇杆左右）
param set RC_MAP_MODE_SW 5      # 通道 5 = 飞行模式开关（SA）
param set RC_MAP_ARM_SW 6       # 通道 6 = Arm 开关（SB）

# 设置 RC 输入模式
param set COM_RC_IN_MODE 1

# 保存参数
param save
```

#### 4. Taranis 模型配置

在 Taranis 中创建 **PX4 SITL** 模型：

```
通道 1: 副翼（右摇杆左右）
通道 2: 升降舵（右摇杆上下）
通道 3: 油门（左摇杆上下）
通道 4: 方向舵（左摇杆左右）
通道 5: SA 开关（3 位，飞行模式）
通道 6: SB 开关（2 位，Arm/Disarm）
通道 7: SC 开关（3 位，备用）
通道 8: SD 开关（2 位，备用）
```

#### 5. 飞行模式配置

配置 SA 开关（3 位）对应 3 种飞行模式：

```bash
# 位置 1（上）：手动模式
param set COM_FLTMODE1 0

# 位置 2（中）：定高模式
param set COM_FLTMODE2 2

# 位置 3（下）：位置模式
param set COM_FLTMODE3 3

param save
```

### Spektrum 遥控器配置

Spektrum 使用 **DSM/DSM2/DSMX** 协议，需要 USB 适配器：

1. 购买 **Spektrum USB Programming Cable**
2. 连接遥控器到 Windows
3. 遥控器进入 **Trainer Mode** 或 **Simulator Mode**
4. VirtualBox 传递 USB 设备到 Linux
5. 配置与 FrSky 类似

---

## ⌨️ 方案 D：键盘模拟（临时测试）

**仅用于快速测试，控制精度差。**

### 使用 MAVLink 命令

在 PX4 shell 或通过 MAVLink：

```bash
# Arm
pxh> commander arm

# 起飞到 2.5m
pxh> commander takeoff

# 切换到位置模式
pxh> commander mode posctl

# 降落
pxh> commander land

# Disarm
pxh> commander disarm
```

### 使用 QGC 键盘快捷键

在 QGroundControl 飞行视图：

- **A**：Arm/Disarm
- **T**：Takeoff
- **L**：Land
- **R**：RTL (Return to Launch)
- **H**：Hold (悬停)
- **Space**：暂停/继续任务

---

## 🔧 故障排除

### 问题 1：手柄/遥控器不识别

**症状**：`ls /dev/input/js*` 没有输出

**解决**：
```bash
# 检查 USB 设备
lsusb

# 检查内核日志
dmesg | tail -30

# 确保 VirtualBox USB 传递正确
# VirtualBox → 设备 → USB → 勾选设备
```

### 问题 2：摇杆不响应

**症状**：移动摇杆，PX4 没有反应

**解决**：
```bash
# 1. 检查 joystick 驱动是否启动
pxh> linux_joystick status

# 2. 如果未启动，启动驱动
pxh> linux_joystick start /dev/input/js0

# 3. 检查 RC 输入
pxh> listener manual_control_setpoint

# 4. 确认 RC 输入模式
pxh> param get COM_RC_IN_MODE
# 应该是 1（Joystick）或 2（RC）

# 5. 重启 PX4
pxh> reboot
```

### 问题 3：摇杆方向反了

**症状**：推摇杆向前，飞行器向后飞

**解决**：
```bash
# 反转通道
pxh> param set RC_MAP_PITCH -3   # 负号表示反转
pxh> param save
pxh> reboot
```

### 问题 4：QGC 虚拟摇杆不显示

**症状**：启用了虚拟摇杆但屏幕上没有

**解决**：
1. 确认在 **飞行视图**（不是设置页面）
2. **Q** → **Application Settings** → **General**
3. **Virtual Joystick** 确认勾选
4. 重启 QGroundControl
5. 确认 PX4 已连接（左上角显示绿色）

### 问题 5：摇杆控制不精确

**症状**：摇杆飘移、中点不准

**解决**：
```bash
# 校准 joystick
jstest-gtk

# 或在 PX4 中校准 RC
pxh> commander calibrate rc

# 调整死区
pxh> param set RC_DEADZ 0.05    # 5% 死区
pxh> param save
```

---

## 📊 控制模式说明

### 飞行模式

| 模式 | 名称 | 摇杆控制 | 稳定性 | 适用场景 |
|------|------|---------|--------|---------|
| **Manual** | 手动模式 | 直接控制姿态 | 低 | 特技飞行 |
| **Stabilized** | 稳定模式 | 控制姿态角度 | 中 | 一般飞行 |
| **Altitude** | 定高模式 | 自动保持高度 | 高 | 悬停练习 |
| **Position** | 位置模式 | 自动保持位置 | 最高 | 精确控制 |

### 摇杆响应曲线

调整摇杆灵敏度：

```bash
# 横滚/俯仰响应曲线（0=线性，1=指数）
pxh> param set MC_MAN_TILT_TAU 0.5

# 偏航响应率
pxh> param set MPC_MAN_Y_MAX 120   # 最大偏航率 120°/s

# 最大倾斜角度
pxh> param set MPC_MAN_TILT_MAX 35  # 最大倾斜 35°

pxh> param save
```

---

## 🎯 推荐配置总结

### 日常开发测试
→ **QGC 虚拟摇杆**（无需额外硬件）

### 算法验证和长时间测试
→ **USB 游戏手柄**（操作舒适，价格便宜）

### 专业飞行测试和操作员训练
→ **实际 RC 遥控器**（最真实，准备实飞）

---

## 📚 相关文档

- **[PX4 Joystick 设置](https://docs.px4.io/main/en/simulation/)**
- **[QGC 虚拟摇杆](https://docs.qgroundcontrol.com/master/en/SettingsView/VirtualJoystick.html)**
- **[RC 通道映射](https://docs.px4.io/main/en/config/radio.html)**
- **[飞行模式](https://docs.px4.io/main/en/flight_modes/)**

---

**总结：QGroundControl 虚拟摇杆是最简单、最快速的手动控制方式，推荐首先尝试！**
