# 实际 TVMD 硬件参数 vs 当前 SITL 配置对比

## 🎯 您提供的实际机体参数

从您的 model.sdf 文件中提取的关键物理参数：

### 1. 总体质量和惯性

```xml
<inertial>
  <pose>0.020647 0.000208 0.023472 0 0 0</pose>
  <mass>1.883</mass>  <!-- ⚠️ 比当前 SITL 轻 61%！ -->
  <inertia>
    <ixx>0.092624</ixx>  <!-- X 轴转动惯量 -->
    <ixy>0.000278</ixy>
    <ixz>-0.000508</ixz>
    <iyy>0.041137</iyy>  <!-- Y 轴转动惯量 -->
    <iyz>-0.000213</iyz>
    <izz>0.114626</izz>  <!-- Z 轴转动惯量 -->
  </inertia>
</inertial>
```

### 2. 模块位置（Module Positions）

| 模块 | X (m) | Y (m) | Z (m) |
|------|-------|-------|-------|
| **Module 1** (前左) | 0.1675 | 0.1825 | -0.02 |
| **Module 2** (前右) | 0.1675 | -0.172 | -0.02 |
| **Module 3** (后左) | -0.15 | 0.1825 | -0.02 |
| **Module 4** (后右) | -0.15 | -0.175 | -0.02 |

**几何分析**：
```
前后轴距：L = 0.1675 - (-0.15) = 0.3175 m
左右轴距：W ≈ 0.1825 (不完全对称)
配置：非对称 H 形
```

### 3. 电机旋转方向

```xml
<!-- 所有电机都是 CCW（逆时针）！ -->
<turningDirection>ccw</turningDirection>  <!-- Motor 0 -->
<turningDirection>ccw</turningDirection>  <!-- Motor 1 -->
<turningDirection>ccw</turningDirection>  <!-- Motor 2 -->
<turningDirection>ccw</turningDirection>  <!-- Motor 3 -->
```

⚠️ **警告：全部 CCW 会导致无法平衡反扭矩！**

### 4. 电机参数（与当前 SITL 相同）

```xml
<motorConstant>2e-05</motorConstant>
<momentConstant>0.06</momentConstant>
<maxRotVelocity>1500</maxRotVelocity>
<timeConstantUp>0.0125</timeConstantUp>
<timeConstantDown>0.025</timeConstantDown>
```

### 5. 关节限制（与当前 SITL 相同）

```xml
<limit>
  <lower>-0.7854</lower>  <!-- -45° -->
  <upper>0.7854</upper>   <!-- +45° -->
</limit>
```

---

## 📊 关键参数对比表

### 质量和惯性对比

| 参数 | 当前 SITL | 实际硬件 | 差异 | 影响 |
|------|----------|---------|------|------|
| **总质量 (kg)** | 4.872 | 1.883 | **-61.3%** | 🔴 重大差异！ |
| **Ixx (kg·m²)** | 0.1468 | 0.0926 | -36.9% | 🟡 中等差异 |
| **Iyy (kg·m²)** | 0.1538 | 0.0411 | **-73.3%** | 🔴 重大差异！ |
| **Izz (kg·m²)** | 0.2804 | 0.1146 | -59.1% | 🔴 重大差异！ |
| **质心 X (m)** | 0.0139 | 0.0207 | +48.9% | 🟢 可接受 |
| **质心 Y (m)** | -0.0006 | 0.0002 | ~ | 🟢 可接受 |
| **质心 Z (m)** | 0.0205 | 0.0235 | +14.6% | 🟢 可接受 |

### 模块位置对比

| 模块 | 当前 SITL (X, Y) | 实际硬件 (X, Y) | 差异 |
|------|-----------------|----------------|------|
| **Module 1** (前左) | (0.185, 0.16) | (0.1675, 0.1825) | 🟡 不同 |
| **Module 2** (前右) | (0.185, -0.16) | (0.1675, -0.172) | 🟡 不同 |
| **Module 3** (后左) | (-0.185, 0.16) | (-0.15, 0.1825) | 🟡 不同 |
| **Module 4** (后右) | (-0.185, -0.16) | (-0.15, -0.175) | 🟡 不同 |

**几何对比**：
```
当前 SITL：
- 前后距离：0.37 m
- 左右距离：0.32 m
- 对称 H 形

实际硬件：
- 前后距离：0.3175 m (-14.2%)
- 左右距离：~0.1825 m (不对称)
- 非对称 H 形
```

### 电机旋转方向对比

| 电机 | 当前 SITL | 实际硬件 | 状态 |
|------|----------|---------|------|
| **Motor 0** (模块1) | CW | CCW | 🔴 **不同！** |
| **Motor 1** (模块2) | CCW | CCW | 🟢 相同 |
| **Motor 2** (模块3) | CCW | CCW | 🟢 相同 |
| **Motor 3** (模块4) | CW | CCW | 🔴 **不同！** |

⚠️ **严重问题：实际硬件全部 CCW 无法平衡反扭矩！**

正常配置应该是：CW, CCW, CCW, CW（或其他平衡组合）

---

## 🚨 主要差异和影响分析

### 1. 质量减少 61% (4.87 kg → 1.88 kg)

**影响**：
- ✅ **更敏捷**：加速度增加 2.6 倍
- ✅ **悬停油门更低**：从 50% 降低到约 20-25%
- ⚠️ **更容易受风干扰**：质量轻，惯性小
- ⚠️ **需要重新调整所有油门参数**

**需要修改的 PX4 参数**：
```bash
MPC_THR_HOVER: 0.50 → 0.25 (悬停油门)
MPC_THR_MIN: 0.10 → 0.05 (最小油门)
CA_ROTOR_CT: 0.012 → 0.006 (推力系数)
```

### 2. Y 轴惯性减少 73% (0.154 → 0.041)

**影响**：
- ✅ **俯仰响应更快**：旋转加速度增加 3.7 倍
- ⚠️ **稳定性降低**：更容易震荡
- ⚠️ **需要降低 PID 增益**

**需要修改的 PX4 参数**：
```bash
MC_PITCH_P: 降低 30-40%
MC_PITCHRATE_P: 降低 30-40%
MC_PITCHRATE_D: 可能需要增加以提高阻尼
```

### 3. Z 轴惯性减少 59% (0.280 → 0.115)

**影响**：
- ✅ **偏航响应更快**
- ⚠️ **容易偏航漂移**

**需要修改的 PX4 参数**：
```bash
MC_YAW_P: 降低 30-40%
MC_YAWRATE_P: 降低 30-40%
```

### 4. 模块位置不对称

**影响**：
- ⚠️ **控制分配需要调整**
- ⚠️ **可能导致不对称飞行特性**
- ⚠️ **质心可能不在几何中心**

**需要修改的 PX4 参数**：
```bash
CA_MD0_PX: 0.185 → 0.1675
CA_MD0_PY: 0.16 → 0.1825
CA_MD1_PX: 0.185 → 0.1675
CA_MD1_PY: -0.16 → -0.172
CA_MD2_PX: -0.185 → -0.15
CA_MD2_PY: 0.16 → 0.1825
CA_MD3_PX: -0.185 → -0.15
CA_MD3_PY: -0.16 → -0.175
```

### 5. 🔴 全部 CCW 电机（严重问题！）

**问题**：
- ❌ **所有电机同向旋转会产生巨大的净反扭矩**
- ❌ **飞行器会持续自旋**
- ❌ **无法实现稳定悬停**

**可能原因**：
1. SDF 文件错误（应该是混合 CW/CCW）
2. 实际硬件设计确实全部同向（需要通过倾转补偿）

**解决方案**：

**方案 A：修正 SDF 文件（推荐）**
```xml
<!-- Motor 0 (Module 1): CW -->
<turningDirection>cw</turningDirection>

<!-- Motor 1 (Module 2): CCW -->
<turningDirection>ccw</turningDirection>

<!-- Motor 2 (Module 3): CCW -->
<turningDirection>ccw</turningDirection>

<!-- Motor 3 (Module 4): CW -->
<turningDirection>cw</turningDirection>
```

**方案 B：实际硬件全部同向（特殊设计）**

如果硬件确实全部同向，需要：
```bash
# 在 PX4 中调整力矩系数为 0（依靠倾转控制偏航）
CA_ROTOR0_KM: 0
CA_ROTOR1_KM: 0
CA_ROTOR2_KM: 0
CA_ROTOR3_KM: 0
```

---

## 🔧 完整 PX4 参数更新建议

如果使用您提供的实际硬件参数，需要在 `4007_gz_tvmd` 中修改：

```bash
# 1. 油门和推力参数（质量减少）
param set-default MPC_THR_MIN 0.05       # 10% → 5%
param set-default MPC_THR_HOVER 0.25     # 50% → 25%
param set-default MPC_MANTHR_MIN 0.02    # 5% → 2%

# 2. 推力系数（匹配轻质量）
param set-default CA_ROTOR0_CT 0.006     # 0.012 → 0.006
param set-default CA_ROTOR1_CT 0.006
param set-default CA_ROTOR2_CT 0.006
param set-default CA_ROTOR3_CT 0.006

# 3. 模块位置（匹配实际位置）
param set-default CA_MD0_PX 0.1675       # 0.185 → 0.1675
param set-default CA_MD0_PY 0.1825       # 0.16 → 0.1825
param set-default CA_MD1_PX 0.1675
param set-default CA_MD1_PY -0.172       # -0.16 → -0.172
param set-default CA_MD2_PX -0.15        # -0.185 → -0.15
param set-default CA_MD2_PY 0.1825
param set-default CA_MD3_PX -0.15
param set-default CA_MD3_PY -0.175       # -0.16 → -0.175

# 4. 姿态控制参数（惯性减少）
param set-default MC_ROLL_P 4.0          # 5.0 → 4.0 (-20%)
param set-default MC_ROLLRATE_P 0.10     # 降低 30%
param set-default MC_PITCH_P 4.0         # 降低 20%
param set-default MC_PITCHRATE_P 0.10    # 降低 30%
param set-default MC_YAW_P 1.5           # 2.0 → 1.5 (-25%)
param set-default MC_YAWRATE_P 0.15      # 降低 25%

# 5. 速度和加速度限制（更轻更敏捷）
param set-default MPC_ACC_HOR_MAX 5.0    # 水平加速度增加
param set-default MPC_ACC_UP_MAX 6.0     # 上升加速度增加
param set-default MPC_Z_VEL_MAX_UP 4.0   # 最大上升速度
param set-default MPC_Z_VEL_MAX_DN 2.0   # 最大下降速度

# 6. 电机旋转方向修正（如果需要）
# 如果硬件确实全部 CCW，设置扭矩系数为 0
# param set-default CA_ROTOR0_KM 0
# param set-default CA_ROTOR1_KM 0
# param set-default CA_ROTOR2_KM 0
# param set-default CA_ROTOR3_KM 0
```

---

## 📋 参数快速对比表

| 类别 | 参数 | 当前值 | 建议值 | 变化 |
|------|------|-------|--------|------|
| **质量** | mass | 4.872 kg | 1.883 kg | -61% |
| **惯性** | Ixx | 0.147 | 0.093 | -37% |
|  | Iyy | 0.154 | 0.041 | -73% |
|  | Izz | 0.280 | 0.115 | -59% |
| **油门** | MPC_THR_HOVER | 0.50 | 0.25 | -50% |
|  | MPC_THR_MIN | 0.10 | 0.05 | -50% |
| **推力系数** | CA_ROTOR_CT | 0.012 | 0.006 | -50% |
| **模块 X** | CA_MD0_PX | 0.185 | 0.1675 | -9% |
|  | CA_MD2_PX | -0.185 | -0.15 | -19% |
| **模块 Y** | CA_MD0_PY | 0.16 | 0.1825 | +14% |
|  | CA_MD1_PY | -0.16 | -0.172 | +8% |
| **PID** | MC_ROLL_P | 5.0 | 4.0 | -20% |
|  | MC_YAW_P | 2.0 | 1.5 | -25% |

---

## 🚀 实施步骤

### 方案 A：更新 model.sdf（推荐）

1. **备份当前文件**：
```bash
cd ~/TVMD-PX4/Tools/simulation/gz/models/tvmd/
cp model.sdf model.sdf.backup
```

2. **替换为您的实际硬件参数**：
```bash
# 将您提供的 model.sdf 内容替换当前文件
# 但务必修正电机旋转方向！
```

3. **修正电机旋转方向**：

在 model.sdf 中找到每个 MulticopterMotorModel 插件，修改：
```xml
<!-- Motor 0: CW -->
<turningDirection>cw</turningDirection>

<!-- Motor 1: CCW -->
<turningDirection>ccw</turningDirection>

<!-- Motor 2: CCW -->
<turningDirection>ccw</turningDirection>

<!-- Motor 3: CW -->
<turningDirection>cw</turningDirection>
```

4. **更新 4007_gz_tvmd 参数**：
```bash
# 编辑 ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_tvmd
# 按照上面的"完整 PX4 参数更新建议"修改
```

5. **重新编译和测试**：
```bash
make clean
make px4_sitl_default
./launch_tvmd_headless.sh
```

### 方案 B：创建新的 airframe 文件

创建 `4008_gz_tvmd_lightweight` 专门用于轻量化硬件：

```bash
cp ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_tvmd \
   ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_tvmd_lightweight
```

---

## ⚠️ 关键警告

### 1. 电机旋转方向问题

**您提供的 SDF 中所有电机都是 CCW！** 这会导致：
- ❌ 飞行器持续自旋
- ❌ 无法稳定悬停
- ❌ 偏航控制失效

**请确认**：
- 实际硬件电机旋转方向是什么？
- 是否真的全部同向？
- 还是 SDF 文件有误？

### 2. 质量差异巨大

质量减少 61% 意味着：
- ✅ 更敏捷、更省电
- ⚠️ 更容易受风干扰
- ⚠️ 所有调参都需要重新来过

### 3. 几何不对称

模块位置不对称可能导致：
- 不对称的飞行特性
- 某些方向飞行更敏捷/稳定
- 需要仔细调整控制分配

---

## 🎯 总结

**您的实际机体参数主要特点**：

1. ✅ **轻量化设计**：1.88 kg（比当前 SITL 轻 61%）
2. ⚠️ **低惯性**：特别是 Y 轴惯性减少 73%
3. ⚠️ **非对称布局**：模块位置不完全对称
4. 🔴 **电机旋转问题**：全部 CCW 需要修正

**最重要的修改**：
1. 🔴 **立即修正电机旋转方向**（CW, CCW, CCW, CW）
2. 🟡 **调整悬停油门**（50% → 25%）
3. 🟡 **降低 PID 增益**（-20% 到 -30%）
4. 🟡 **更新模块位置**（匹配实际几何）

**建议**：先修正电机旋转方向，然后逐步测试和调整其他参数！
