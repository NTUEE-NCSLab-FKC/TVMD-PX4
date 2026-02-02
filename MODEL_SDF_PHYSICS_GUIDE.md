# model.sdf 对 TVMD 物理动态的影响详解

`Tools/simulation/gz/models/tvmd/model.sdf` 文件**完全定义了 TVMD 在 Gazebo 中的物理行为**。

## 🎯 是的，它会直接影响物理动态！

这个文件包含了所有影响飞行动力学的参数，包括：
- 质量和惯性
- 电机推力特性
- 气动阻力
- 关节运动学
- 碰撞属性

---

## 📊 model.sdf 的关键物理参数

### 1. 质量和惯性特性（Mass & Inertia）

**位置**：`<link name='base_link'><inertial>` (第 4-15 行)

```xml
<inertial>
  <pose>0.0139 -0.0006 0.0205 0 0 0</pose>  <!-- 质心位置 -->
  <mass>4.8718</mass>                        <!-- 总质量：4.87 kg -->
  <inertia>
    <ixx>0.1468</ixx>  <!-- X 轴转动惯量 -->
    <ixy>0.0004</ixy>  <!-- 惯性积 -->
    <ixz>-0.0012</ixz>
    <iyy>0.1538</iyy>  <!-- Y 轴转动惯量 -->
    <iyz>-0.0002</iyz>
    <izz>0.2804</izz>  <!-- Z 轴转动惯量 -->
  </inertia>
</inertial>
```

**对飞行的影响**：
- ✈️ **mass**：影响加速度、惯性、悬停油门
  - 增加质量 → 需要更大推力悬停
  - 减少质量 → 更敏捷但更容易受风影响
- 🔄 **ixx, iyy, izz**：影响旋转响应速度
  - 增加 → 旋转更慢、更稳定
  - 减少 → 旋转更快、更敏捷
- 📍 **pose (质心)**：影响稳定性
  - 质心偏移 → 可能导致不对称飞行

---

### 2. 电机推力特性（Motor Model）

**位置**：`<plugin name='gz::sim::systems::MulticopterMotorModel'>` (多处)

```xml
<plugin name='gz::sim::systems::MulticopterMotorModel'>
  <jointName>module1_actuator_prop1_joint</jointName>
  <linkName>module1_prop1_link</linkName>
  <turningDirection>cw</turningDirection>        <!-- 旋转方向 -->

  <!-- 🚀 电机响应特性 -->
  <timeConstantUp>0.0125</timeConstantUp>       <!-- 加速时间常数 12.5ms -->
  <timeConstantDown>0.025</timeConstantDown>    <!-- 减速时间常数 25ms -->
  <maxRotVelocity>1500</maxRotVelocity>        <!-- 最大转速 1500 rad/s -->

  <!-- ⚡ 推力和扭矩系数 -->
  <motorConstant>2e-05</motorConstant>          <!-- 电机常数（推力系数）-->
  <momentConstant>0.06</momentConstant>         <!-- 力矩常数 -->

  <!-- 🌪️ 空气动力学 -->
  <rotorDragCoefficient>0.000106428</rotorDragCoefficient>    <!-- 旋翼阻力 -->
  <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>  <!-- 滚动力矩 -->

  <rotorVelocitySlowdownSim>20</rotorVelocitySlowdownSim>     <!-- 仿真减速因子 -->
  <motorType>velocity</motorType>                              <!-- 电机类型 -->
  <motorNumber>0</motorNumber>                                 <!-- 电机编号 -->
</plugin>
```

**对飞行的影响**：

| 参数 | 作用 | 增加后的效果 |
|------|------|------------|
| **timeConstantUp** | 加速响应时间 | 加速更慢，响应迟钝 |
| **timeConstantDown** | 减速响应时间 | 减速更慢，制动距离增加 |
| **maxRotVelocity** | 最大转速 | 最大推力增加 |
| **motorConstant** | 推力系数 | 相同转速下推力更大 |
| **momentConstant** | 扭矩系数 | 反扭矩增加（影响偏航） |
| **rotorDragCoefficient** | 旋翼阻力 | 减速更快，但效率降低 |

**实际意义**：
```
推力 = motorConstant × (转速)²
扭矩 = momentConstant × (转速)²

例如：
- motorConstant = 2e-05
- 转速 = 1000 rad/s
- 推力 = 2e-05 × 1000² = 20 N
```

---

### 3. 模块质量和惯性（Module Inertia）

**每个倾转模块的质量**：

```xml
<!-- 万向节 (Gimbal) -->
<link name='module1_gimbal_link'>
  <inertial>
    <mass>0.0340</mass>  <!-- 34 g -->
    <inertia>
      <ixx>2.55e-06</ixx>
      <iyy>2.19e-06</iyy>
      <izz>3.91e-06</izz>
    </inertia>
  </inertial>
</link>

<!-- 执行器 (Actuator) -->
<link name='module1_actuator_link'>
  <inertial>
    <mass>0.1254</mass>  <!-- 125 g -->
    <inertia>
      <ixx>0.000111</ixx>
      <iyy>0.000111</iyy>
      <izz>0.000125</izz>
    </inertia>
  </inertial>
</link>

<!-- 螺旋桨 (Propeller) -->
<link name='module1_prop1_link'>
  <inertial>
    <mass>0.0025</mass>  <!-- 2.5 g -->
    <inertia>
      <ixx>9.75e-07</ixx>
      <iyy>5.31e-07</iyy>
      <izz>9.75e-07</izz>
    </inertia>
  </inertial>
</link>
```

**对飞行的影响**：
- 🔄 **模块质量**：影响倾转速度和力矩需求
- ⚖️ **质量分布**：影响整体稳定性
- 💨 **轻量化模块** → 更快的倾转响应

---

### 4. 关节运动学（Joint Kinematics）

**万向节限制**：

```xml
<joint name='module1_gimbal_actuator_joint' type='revolute'>
  <axis>
    <xyz>1 0 0</xyz>  <!-- X 轴旋转 -->
    <limit>
      <lower>-0.7854</lower>  <!-- -45° = -π/4 rad -->
      <upper>0.7854</upper>   <!-- +45° = +π/4 rad -->
    </limit>
    <dynamics>
      <damping>0</damping>    <!-- 阻尼系数 -->
    </dynamics>
  </axis>
</joint>

<joint name='module1_body_gimbal_joint' type='revolute'>
  <axis>
    <xyz>0 1 0</xyz>  <!-- Y 轴旋转 -->
    <limit>
      <lower>-0.7854</lower>  <!-- -45° -->
      <upper>0.7854</upper>   <!-- +45° -->
    </limit>
    <dynamics>
      <damping>0</damping>
    </dynamics>
  </axis>
</joint>
```

**对飞行的影响**：
- 📐 **limit (上下限)**：倾转角度范围
  - ±45° → 圆锥形 LAFS（力轴可自由对齐）
  - 增加限制 → 更大倾转范围，更快机动
  - 减少限制 → 更稳定但机动性降低
- 🛑 **damping (阻尼)**：倾转阻力
  - damping=0 → 无阻力，快速响应
  - 增加阻尼 → 减慢倾转，更平滑

---

### 5. 关节控制器（Joint Controllers）

**伺服电机控制**：

```xml
<plugin name='gz::sim::systems::JointPositionController'>
  <joint_name>module1_body_gimbal_joint</joint_name>
  <sub_topic>servo_1</sub_topic>
  <!-- PID 控制参数（可选，使用默认值）-->
  <p_gain>10</p_gain>    <!-- 比例增益 -->
  <i_gain>0</i_gain>      <!-- 积分增益 -->
  <d_gain>0.1</d_gain>    <!-- 微分增益 -->
</plugin>
```

**对飞行的影响**：
- 🎯 **p_gain (比例增益)**：位置跟踪速度
  - 增加 → 更快响应，但可能震荡
  - 减少 → 更平滑，但响应慢
- 🔄 **i_gain (积分增益)**：消除稳态误差
- 🛡️ **d_gain (微分增益)**：阻尼，防止超调

---

### 6. 碰撞特性（Collision Properties）

```xml
<collision name='base_link_collision'>
  <geometry>
    <box>
      <size>0.3 0.3 0.1</size>  <!-- 碰撞盒尺寸 -->
    </box>
  </geometry>
  <surface>
    <contact>
      <ode>
        <kp>1e6</kp>         <!-- 接触刚度 -->
        <kd>100</kd>         <!-- 接触阻尼 -->
        <max_vel>0.01</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>0.5</mu>         <!-- 摩擦系数 -->
        <mu2>0.5</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

**对飞行的影响**：
- 🛬 **着陆反弹**：kp/kd 影响着陆时的缓冲
- 🌍 **地面摩擦**：mu 影响在地面滑动

---

## 🔧 如何修改 model.sdf 以改变物理行为

### 场景 1：增加推力（更强劲）

```xml
<!-- 方法 1：增加电机常数 -->
<motorConstant>4e-05</motorConstant>  <!-- 2e-05 → 4e-05，推力增加 2 倍 -->

<!-- 方法 2：增加最大转速 -->
<maxRotVelocity>3000</maxRotVelocity>  <!-- 1500 → 3000，推力增加 4 倍 -->
```

### 场景 2：更快的响应（更敏捷）

```xml
<!-- 减少电机时间常数 -->
<timeConstantUp>0.005</timeConstantUp>    <!-- 12.5ms → 5ms -->
<timeConstantDown>0.010</timeConstantDown> <!-- 25ms → 10ms -->

<!-- 增加伺服 PID 增益 -->
<p_gain>20</p_gain>  <!-- 10 → 20 -->
```

### 场景 3：更稳定（降低敏感度）

```xml
<!-- 增加惯性 -->
<ixx>0.25</ixx>  <!-- 0.1468 → 0.25 -->
<iyy>0.25</iyy>  <!-- 0.1538 → 0.25 -->
<izz>0.40</izz>  <!-- 0.2804 → 0.40 -->

<!-- 增加阻尼 -->
<damping>0.1</damping>  <!-- 0 → 0.1 -->
```

### 场景 4：增加载重能力

```xml
<!-- 增加总质量 -->
<mass>6.0</mass>  <!-- 4.87 → 6.0 kg -->

<!-- 同时需要增加推力 -->
<motorConstant>3e-05</motorConstant>  <!-- 2e-05 → 3e-05 -->
```

### 场景 5：更大的倾转范围

```xml
<!-- 增加关节限制 -->
<limit>
  <lower>-1.0472</lower>  <!-- -45° → -60° = -π/3 rad -->
  <upper>1.0472</upper>   <!-- +45° → +60° -->
</limit>
```

---

## ⚠️ 修改后的重要步骤

### 1. 重新启动 SITL

```bash
# 修改 model.sdf 后，必须重启
pkill -9 gz px4
./launch_tvmd_headless.sh
```

### 2. 重新调整 PX4 参数

如果修改了推力特性，需要调整 PX4 参数：

```bash
# 重新估计悬停油门
pxh> param set MPC_THR_HOVER 0.60  # 如果推力减小

# 重新调整推力曲线
pxh> param set CA_ROTOR0_CT 0.015  # 匹配新的电机常数

pxh> param save
```

### 3. 测试和验证

```bash
# 1. 基本悬停测试
pxh> commander arm
pxh> commander takeoff

# 2. 观察悬停稳定性
# 在 QGC 中查看高度保持性能

# 3. 手动控制测试
# 使用虚拟摇杆测试响应速度

# 4. 检查油门使用
# 悬停应该在 40-60% 油门范围
```

---

## 📊 关键参数对照表

| model.sdf 参数 | 单位 | TVMD 当前值 | 影响 | 对应 PX4 参数 |
|---------------|------|------------|------|--------------|
| **mass** | kg | 4.87 | 惯性、悬停油门 | MPC_THR_HOVER |
| **ixx/iyy/izz** | kg·m² | 0.15/0.15/0.28 | 旋转速度 | MC_ROLLRATE_MAX |
| **motorConstant** | N·s²/rad² | 2e-05 | 推力 | CA_ROTOR_CT |
| **momentConstant** | N·m·s²/rad² | 0.06 | 扭矩 | CA_ROTOR_KM |
| **maxRotVelocity** | rad/s | 1500 | 最大推力 | - |
| **timeConstantUp** | s | 0.0125 | 加速响应 | - |
| **timeConstantDown** | s | 0.025 | 减速响应 | - |
| **joint limit** | rad | ±0.7854 (±45°) | 倾转范围 | CA_SV_TL_MIN/MAX |

---

## 🧪 实验：测试 model.sdf 的影响

### 实验 1：推力测试

**修改前**（原始值）：
```xml
<motorConstant>2e-05</motorConstant>
```

**修改后**（增加推力）：
```xml
<motorConstant>4e-05</motorConstant>
```

**预期结果**：
- ✅ 相同油门下，升力增加 2 倍
- ✅ 悬停油门从 ~50% 降低到 ~35%
- ✅ 爬升更快
- ⚠️ 需要重新调整 MPC_THR_HOVER

### 实验 2：惯性测试

**修改前**：
```xml
<izz>0.2804</izz>
```

**修改后**（增加 Z 轴惯性）：
```xml
<izz>0.5</izz>
```

**预期结果**：
- ✅ 偏航响应变慢
- ✅ 偏航更稳定，不易飘移
- ⚠️ 机动性降低

### 实验 3：电机响应测试

**修改前**：
```xml
<timeConstantUp>0.0125</timeConstantUp>
```

**修改后**（更快响应）：
```xml
<timeConstantUp>0.005</timeConstantUp>
```

**预期结果**：
- ✅ 油门响应更快
- ✅ 姿态控制更敏捷
- ⚠️ 可能需要重新调整 PID

---

## 🔗 model.sdf 与 PX4 的关系

```
model.sdf (Gazebo)          →    PX4 感知到的效果
─────────────────────────────────────────────────────────
motorConstant = 2e-05       →    需要特定油门才能悬停
mass = 4.87 kg              →    加速度响应
ixx/iyy/izz                 →    角速度响应
joint limits ±45°           →    伺服有效范围
timeConstant                →    推力延迟
```

**关键点**：
1. model.sdf 定义**物理真实性**
2. PX4 参数定义**控制策略**
3. 两者必须匹配才能获得最佳性能

---

## 📚 相关文件

| 文件 | 作用 | 关系 |
|------|------|------|
| **model.sdf** | Gazebo 物理模型 | 定义"飞行器是什么" |
| **4007_gz_tvmd** | PX4 参数配置 | 定义"如何控制飞行器" |
| **model.config** | Gazebo 模型元数据 | 模型名称、作者等 |
| **meshes/** | 3D 视觉模型 | 仅影响外观，不影响物理 |

---

## 🎯 总结

**是的，`model.sdf` 会直接影响物理动态！**

它定义了：
1. ✅ **质量和惯性** → 影响加速度和旋转
2. ✅ **电机推力特性** → 影响推力和响应
3. ✅ **关节运动学** → 影响倾转范围和速度
4. ✅ **空气动力学** → 影响阻力和稳定性
5. ✅ **碰撞特性** → 影响着陆行为

**修改建议**：
- 🔬 **实验性修改**：备份原文件，逐步测试
- 📊 **数据驱动**：基于实际飞行数据调整参数
- 🔄 **迭代优化**：修改 → 测试 → 调整 PX4 → 重复

**当前 TVMD model.sdf 配置已经针对硬件进行了优化，建议先使用默认值，熟悉后再进行调整！**
