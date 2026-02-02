# TVMD SITL 油门相关参数完整指南

本文档列出所有与油门控制相关的 PX4 参数及其配置。

## 🎯 油门参数分类

### 1. 基本油门设置

| 参数 | 默认值 | TVMD 设置 | 说明 |
|------|-------|----------|------|
| **MPC_THR_MIN** | 0.12 | 0.10 | **最小油门**（arm 后怠速） |
| **MPC_THR_MAX** | 1.0 | 1.0 | 最大油门 |
| **MPC_THR_HOVER** | 0.5 | 0.50 | **预期悬停油门**（用于估算） |
| **MPC_MANTHR_MIN** | 0.0 | 0.05 | **手动模式最小油门** |
| **MPC_MANTHR_MAX** | 1.0 | 1.0 | 手动模式最大油门 |

### 2. Arming 油门检查（高油门检查）

| 参数 | 默认值 | 推荐值 | 说明 |
|------|-------|--------|------|
| **COM_RC_ARM_HYST** | 1000 | 1000 | **Arm 油门检查滞后**（阈值） |
| **COM_PREARM_MODE** | 1 | 1 | 预 arm 模式（0=禁用，1=安全检查） |
| **COM_ARM_CHK_ESCS** | 0 | 0 | 是否检查 ESC（SITL 禁用） |

### 3. 油门曲线和响应

| 参数 | 默认值 | 说明 |
|------|-------|------|
| **MPC_THR_CURVE** | 0 | 油门曲线类型（0=自动，1=救援） |
| **MPC_Z_VEL_MAX_UP** | 3.0 | 最大上升速度 (m/s) |
| **MPC_Z_VEL_MAX_DN** | 1.5 | 最大下降速度 (m/s) |
| **MPC_LAND_SPEED** | 0.7 | 降落速度 (m/s) |

### 4. 电机输出映射（Gazebo 仿真）

| 参数 | 值 | 说明 |
|------|-----|------|
| **SIM_GZ_EC_MIN1-4** | 0 | **电机最小输出**（disarm 时） |
| **SIM_GZ_EC_MAX1-4** | 1000 | 电机最大输出 |
| **SIM_GZ_EC_FUNC1-4** | 101-104 | 电机功能映射 |

### 5. 失效保护油门

| 参数 | 默认值 | 说明 |
|------|-------|------|
| **COM_FAIL_ACT_T** | 0.5 | 失效保护触发时间 (s) |
| **NAV_RCL_ACT** | 2 | RC 丢失动作（2=降落） |
| **NAV_DLL_ACT** | 0 | 数据链路丢失动作 |

---

## 🔧 解决 "Arming denied: high throttle" 的方法

### 方法 1：确保油门在最低位置（推荐）

**使用 QGC 虚拟摇杆**：
1. 左摇杆**完全向下拉到底**
2. 然后 `commander arm`

**使用游戏手柄**：
1. 确保左摇杆垂直方向**完全向下**
2. 然后 `commander arm`

**使用实际遥控器**：
1. 油门杆**拉到最低**（最下方）
2. 然后 `commander arm`

### 方法 2：放宽高油门检查阈值

如果油门确实在底部但仍然报错，可以放宽阈值：

```bash
# 在 PX4 shell 中
pxh> param set COM_RC_ARM_HYST 5000    # 默认 1000，增加到 5000
pxh> param save
pxh> commander arm
```

### 方法 3：临时禁用高油门检查（不推荐，仅测试）

**警告：** 这会降低安全性！

```bash
# 禁用预 arm 模式（不推荐！）
pxh> param set COM_PREARM_MODE 0
pxh> param save
pxh> reboot
```

### 方法 4：检查油门通道映射

确认油门通道正确映射：

```bash
# 查看当前映射
pxh> param get RC_MAP_THROTTLE
# 应该是 3（通道 3 = 油门）

# 监听 RC 输入
pxh> listener manual_control_setpoint
# 查看 z 字段（油门）
# - 油门最低：z ≈ 0.0
# - 油门最高：z ≈ 1.0
# - 油门中位：z ≈ 0.5

# 如果 z 值不对，检查摇杆/手柄是否正常
```

### 方法 5：校准遥控器

如果使用游戏手柄或实际遥控器：

```bash
# 启动 RC 校准
pxh> commander calibrate rc

# 按照提示操作：
# 1. 移动所有摇杆到极限位置
# 2. 确认油门最低位置被正确识别
```

---

## 🎮 不同控制方式的油门设置

### QGroundControl 虚拟摇杆

**虚拟摇杆设置**：
1. **Q** → **Application Settings** → **General** → **Virtual Joystick**
2. **Centered throttle**: **❌ No**（油门不居中）
3. **Spring loaded throttle**: **❌ No**（松开不回中）
4. **Allow negative thrust**: **❌ No**（不允许负推力）

**Arm 前检查**：
- 左虚拟摇杆**完全向下**
- 屏幕显示油门为 0%
- 然后点击 ARM 或执行 `commander arm`

### USB 游戏手柄

**手柄校准**：
```bash
# 测试手柄输入
jstest /dev/input/js0

# 查看轴 1（通常是左摇杆 Y 轴 = 油门）
# 向下推：应该接近 -32767
# 向上推：应该接近 +32767
```

**死区设置**：
```bash
# 设置油门死区（防止飘移）
pxh> param set RC_DEADZ 0.05    # 5% 死区
pxh> param save
```

### 实际遥控器

**通道映射**（FrSky/Spektrum 等）：
```bash
# 确认油门通道
pxh> param get RC_MAP_THROTTLE
# 应该是 3（大多数遥控器）

# 如果不对，重新映射
pxh> param set RC_MAP_THROTTLE 3
pxh> param save
```

**油门反转**（如果方向错误）：
```bash
# 检查油门是否反转
pxh> listener manual_control_setpoint

# 如果推油门杆 z 值下降（应该上升），则反转
pxh> param set RC_MAP_THROTTLE -3    # 负号反转
pxh> param save
```

---

## 📊 油门值范围说明

### PX4 内部油门表示

| 油门位置 | z 值 | PWM (SIM_GZ_EC) | 电机状态 |
|---------|------|-----------------|----------|
| **最低** | 0.0 | 0 | 停转（disarm）或怠速（armed） |
| **10%** | 0.1 | 100 | 怠速（MPC_THR_MIN） |
| **50%** | 0.5 | 500 | 悬停（MPC_THR_HOVER） |
| **100%** | 1.0 | 1000 | 最大推力 |

### Arm 检查阈值

```
COM_RC_ARM_HYST = 1000（默认）

油门 PWM 值必须 < COM_RC_ARM_HYST 才能 arm
换算：z < (1000 / 1000) = 1.0

实际上，必须 z < 0.1 左右才能通过检查
```

---

## 🔧 TVMD 当前油门配置

根据 `4007_gz_tvmd` 文件，当前设置：

```bash
# 基本油门（已配置）
param set-default MPC_THR_MIN 0.10       # Arm 后 10% 怠速
param set-default MPC_THR_HOVER 0.50     # 预期 50% 悬停
param set-default MPC_MANTHR_MIN 0.05    # 手动模式最低 5%

# 电机最小输出（已配置）
param set-default SIM_GZ_EC_MIN1 0       # Disarm 时 0 输出
param set-default SIM_GZ_EC_MIN2 0
param set-default SIM_GZ_EC_MIN3 0
param set-default SIM_GZ_EC_MIN4 0

# 高油门检查（使用默认值）
# COM_RC_ARM_HYST = 1000（默认，未显式设置）
# COM_PREARM_MODE = 1（默认，启用安全检查）
```

---

## 🆘 故障排除检查清单

如果仍然无法 arm，按以下步骤排查：

### 1. 检查 RC 输入

```bash
pxh> listener manual_control_setpoint
# 观察输出：
# - x: 横滚（-1.0 到 1.0）
# - y: 俯仰（-1.0 到 1.0）
# - z: 油门（0.0 到 1.0）← 重点！
# - r: 偏航（-1.0 到 1.0）
```

**预期结果**：
- 油门杆最低时：`z: 0.000000` 或接近 0
- 油门杆中位时：`z: 0.500000`
- 油门杆最高时：`z: 1.000000`

**如果不对**：
- `z` 一直很高 → 油门杆没有放到底
- `z` 没有变化 → RC 输入未连接或通道映射错误
- `z` 反向变化 → 需要反转通道

### 2. 检查 Arming 条件

```bash
pxh> commander status
# 查看 arming 状态和阻止原因
```

### 3. 检查参数

```bash
# 油门相关参数
pxh> param get MPC_THR_MIN
pxh> param get COM_RC_ARM_HYST
pxh> param get RC_MAP_THROTTLE

# 显示所有油门参数
pxh> param show | grep THR
pxh> param show | grep MANTHR
```

### 4. 查看实时日志

```bash
# 实时查看 commander 日志
pxh> dmesg -f

# 尝试 arm 并观察输出
pxh> commander arm
# 应该显示具体拒绝原因
```

---

## 💡 推荐解决方案（优先级排序）

### 优先级 1：确保油门在最低位置 ⭐⭐⭐⭐⭐

- **QGC 虚拟摇杆**：左摇杆完全向下
- **游戏手柄**：左摇杆完全向下
- **实际遥控器**：油门杆最低
- **验证**：`listener manual_control_setpoint` 显示 `z: 0.0`

### 优先级 2：检查 RC 连接和通道映射 ⭐⭐⭐⭐

```bash
pxh> param get RC_MAP_THROTTLE    # 应该是 3
pxh> param get COM_RC_IN_MODE     # 应该是 1（Joystick）或 2（RC）
```

### 优先级 3：放宽高油门检查阈值 ⭐⭐⭐

```bash
pxh> param set COM_RC_ARM_HYST 5000
pxh> param save
```

### 优先级 4：校准 RC 输入 ⭐⭐

```bash
pxh> commander calibrate rc
# 按照提示完成校准
```

### 优先级 5：临时使用命令行 Arm（测试） ⭐

如果只是想快速测试，可以：

```bash
# 在没有 RC 输入的情况下强制 arm
pxh> param set COM_RC_IN_MODE 0    # 禁用 RC 输入要求
pxh> param save
pxh> commander arm -f              # 强制 arm（-f = force）
```

**警告**：这只是临时测试方法，不适合长期使用！

---

## 📚 相关 PX4 参数文档

- **[MPC_THR_* 参数](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#MPC_THR_MIN)**
- **[COM_RC_* 参数](https://docs.px4.io/main/en/advanced_config/parameter_reference.html#COM_RC_ARM_HYST)**
- **[Arming 检查](https://docs.px4.io/main/en/advanced_config/prearm_arm_disarm.html)**
- **[RC 校准](https://docs.px4.io/main/en/config/radio.html)**

---

## 🎯 快速参考卡

### Arm 前检查清单

- [ ] SITL 已启动超过 30 秒（EKF2 收敛）
- [ ] QGC 或 RC 输入已连接
- [ ] **油门杆在最低位置** ← 最重要！
- [ ] 飞行器在地面
- [ ] 没有预飞错误（`commander status`）

### 常用油门命令

```bash
# 查看当前油门值
listener manual_control_setpoint

# 查看油门参数
param get MPC_THR_MIN
param get MPC_THR_HOVER

# 调整最小油门
param set MPC_THR_MIN 0.08    # 降低怠速到 8%

# 放宽 arm 检查
param set COM_RC_ARM_HYST 5000

# 保存参数
param save
```

---

**总结：99% 的 "high throttle" 错误是因为油门杆没有放到最低位置！** 🎮⬇️
