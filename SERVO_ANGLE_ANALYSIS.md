# TVMD 舵機角度映射問題分析報告

## 問題概述
在初始推動馬達時，各舵機的角度不正確，需要檢查座標轉換、映射的推力向量數值與 PWM 關係。

---

## 一、座標系統定義

### 1. 機體座標系 (Body Frame)
從配置文件 `13300_generic_vtol_tvmd` 定義：

```
模組 0 (前左): position = ( 0.185,  0.16, 0)  [m]
模組 1 (前右): position = ( 0.185, -0.16, 0)  [m]
模組 2 (後左): position = (-0.185,  0.16, 0)  [m]
模組 3 (後右): position = (-0.185, -0.16, 0)  [m]
```

**H-frame 配置圖**:
```
        前方 (X+)
          ↑
     0 ●━━━━━● 1
       ┃     ┃
       ┃  +  ┃  → 右側 (Y-)
       ┃     ┃
     2 ●━━━━━● 3

● = 推力模組
+ = 機體重心
```

**座標軸定義**:
- X 軸：指向前方（機頭方向）
- Y 軸：指向左側
- Z 軸：指向上方（垂直向上）

### 2. 模組方向參數 (ax_psi)
所有模組的 `CA_MD0~3_AZ = 0`，表示所有模組的局部座標系與機體座標系對齊。

旋轉矩陣 Rz 定義 (ActuatorEffectivenessVTOL_TVMD.cpp:201-207):
```cpp
Rz = [cos(psi), -sin(psi), 0]
     [sin(psi),  cos(psi), 0]
     [0,         0,        1]

當 psi = 0 時，Rz = I（單位矩陣）
```

---

## 二、偽力向量到舵機角度的轉換

### 1. 轉換公式 (ControlAllocationModularBundled.cpp:230-249)

**inverse_transform()** - 偽力 → 舵機角度:
```cpp
Tf = sqrt(fx² + fy² + fz²)    // 推力大小
eta_x = asin(-fy / Tf)        // X軸舵機角度 (pitch)
eta_y = atan2(fx, fz)         // Y軸舵機角度 (roll)
```

**forward_transform()** - 舵機角度 → 偽力:
```cpp
fx = cos(eta_x) * sin(eta_y) * Tf
fy = -sin(eta_x) * Tf
fz = cos(eta_x) * cos(eta_y) * Tf
```

### 2. 符號約定分析

| 偽力方向 | 舵機角度 | 物理意義 |
|---------|---------|---------|
| fx > 0 | eta_y > 0 | 推力向前傾斜 |
| fy > 0 | eta_x < 0 | 推力向左傾斜（注意負號！）|
| fz > 0 | 推力向上 | 垂直分量 |

**關鍵觀察**:
- `eta_x = asin(-fy / Tf)` 中有負號
- 這意味著 **fy 正向 → eta_x 負向**

### 3. 初始化檢查

初始偽力向量 (ControlAllocationModularBundled.cpp:77-80):
```cpp
_f(i*3 + 0) = 0       // fx = 0
_f(i*3 + 1) = 0       // fy = 0
_f(i*3 + 2) = 0.6N    // fz = f_min + 0.1
```

計算初始舵機角度:
```
Tf = sqrt(0 + 0 + 0.36) = 0.6 N
eta_x = asin(0 / 0.6) = 0 rad = 0°  ✓
eta_y = atan2(0, 0.6) = 0 rad = 0°  ✓
```

**結論**: 理論上初始化應該產生零舵機角度。

---

## 三、舵機角度到 PWM 的映射鏈路

### 1. 角度標準化 (ActuatorEffectivenessVTOL_TVMD.cpp:278-283)

```cpp
scale = (1.0 - (-1.0)) * gear_ratio / (servo_max - servo_min)
normalized_value = angle_rad * scale
```

**參數值**:
- `CA_SV_TL0_MINA = -45° = -0.7854 rad`
- `CA_SV_TL0_MAXA = +45° = +0.7854 rad`
- `gear_ratio = 1.0`

**計算**:
```
scale = 2 * 1.0 / (0.7854 - (-0.7854))
      = 2 / 1.5708
      = 1.2732
```

**映射關係**:
```
-45° (-0.7854 rad) → -0.7854 * 1.2732 = -1.0 (normalized)
  0° (0 rad)       →  0 * 1.2732      =  0   (normalized)
+45° (+0.7854 rad) → +0.7854 * 1.2732 = +1.0 (normalized)
```

### 2. 標準化值到 PWM (mixer_module.cpp:520-535)

```cpp
PWM = normalized_value * (PWM_MAX - PWM_MIN) / 2 + (PWM_MAX + PWM_MIN) / 2
```

**參數值**:
- `PWM_AUX_MIN = 1000 µs`
- `PWM_AUX_MAX = 2000 µs`

**映射關係**:
```
normalized = -1.0 → PWM = 1000 µs (對應 -45°)
normalized =  0   → PWM = 1500 µs (對應   0°)
normalized = +1.0 → PWM = 2000 µs (對應 +45°)
```

---

## 四、發現的問題

### ⚠️ **問題 1: 舵機 TRIM 未設置** (高優先級)

**位置**: `ActuatorEffectivenessVTOL_TVMD.cpp:221`

```cpp
// configuration.trim?  ← 這行被註解掉了！
```

**問題描述**:
1. TVMD 的代碼中**完全沒有設置 `configuration.trim`**
2. 這意味著系統假設所有舵機的物理中位點都在 PWM 1500 µs
3. **實際上大部分舵機的中位點不是 1500 µs！**

**影響**:
- 即使控制算法輸出 0° 角度
- PWM 會輸出 1500 µs
- 但舵機的物理零位可能在 1450 µs 或 1550 µs
- 導致舵機出現初始偏移

**解決方案**:
需要為每個舵機設置 trim 值，將物理零位映射到 PWM 1500。

### ⚠️ **問題 2: PWM_AUX_DIS 參數的作用** (高優先級)

**當前配置** (13300_generic_vtol_tvmd:330-337):
```bash
param set-default PWM_AUX_DIS1 1500    # Center position (0°)
param set-default PWM_AUX_DIS2 1500
...
param set-default PWM_AUX_DIS8 1500
```

**問題**:
- `PWM_AUX_DIS` 是解除武裝 (disarmed) 時的 PWM 值
- 這個參數**應該設置為舵機的物理中位點**
- 但這不會影響武裝 (armed) 狀態下的映射

**需要校準的參數**:
每個舵機需要測量其物理中位點的 PWM 值，例如：
```bash
# 假設測量結果：
PWM_AUX_DIS1 1520  # 舵機 1 的物理零位
PWM_AUX_DIS2 1480  # 舵機 2 的物理零位
PWM_AUX_DIS3 1505  # 舵機 3 的物理零位
...
```

### ⚠️ **問題 3: 舵機索引映射** (中等優先級)

**索引計算** (ActuatorEffectivenessVTOL_TVMD.hpp:169-170):
```cpp
get_motor_idx(module_id, offset) = module_id
get_servo_idx(module_id, offset) = 2*module_id + offset + num_agents
```

**映射關係**:
```
馬達索引:
  Module 0 → Motor 0 (PWM_MAIN_FUNC1 = 101)
  Module 1 → Motor 1 (PWM_MAIN_FUNC2 = 102)
  Module 2 → Motor 2 (PWM_MAIN_FUNC3 = 103)
  Module 3 → Motor 3 (PWM_MAIN_FUNC4 = 104)

舵機索引 (num_agents = 4):
  Module 0, X-axis → Servo 0 = 2*0 + 0 + 4 = 4  (PWM_AUX_FUNC1 = 201)
  Module 0, Y-axis → Servo 1 = 2*0 + 1 + 4 = 5  (PWM_AUX_FUNC2 = 202)
  Module 1, X-axis → Servo 2 = 2*1 + 0 + 4 = 6  (PWM_AUX_FUNC3 = 203)
  Module 1, Y-axis → Servo 3 = 2*1 + 1 + 4 = 7  (PWM_AUX_FUNC4 = 204)
  Module 2, X-axis → Servo 4 = 2*2 + 0 + 4 = 8  (PWM_AUX_FUNC5 = 205)
  Module 2, Y-axis → Servo 5 = 2*2 + 1 + 4 = 9  (PWM_AUX_FUNC6 = 206)
  Module 3, X-axis → Servo 6 = 2*3 + 0 + 4 = 10 (PWM_AUX_FUNC7 = 207)
  Module 3, Y-axis → Servo 7 = 2*3 + 1 + 4 = 11 (PWM_AUX_FUNC8 = 208)
```

**需要驗證**:
- X-axis servo 控制哪個物理舵機？
- Y-axis servo 控制哪個物理舵機？
- 物理接線是否與這個映射一致？

### ⚠️ **問題 4: 舵機旋轉方向約定** (中等優先級)

**符號約定檢查**:
```
當 fy > 0 時，eta_x = asin(-fy/Tf) < 0
```

這意味著：
- **正的 Y 方向偽力** → **負的 X 軸舵機角度**

**需要驗證**:
1. 手動設置 `fy = 0.5N, fx = 0, fz = 5N`
2. 計算期望角度: `eta_x = asin(-0.5/5.025) ≈ -5.7°`
3. 觀察實際舵機是否向預期方向旋轉
4. 如果方向相反，需要在代碼中調整符號或使用參數反轉舵機

---

## 五、建議的檢查和校準步驟

### Step 1: 測量舵機物理零位 (最重要！)

**目的**: 找到每個舵機的物理中位點對應的 PWM 值

**操作步驟**:
1. 解除系統武裝
2. 手動調整每個舵機到機械中位點（推力向上，無傾斜）
3. 使用示波器或 PWM 測試工具測量當前 PWM 值
4. 記錄每個舵機的實際零位 PWM 值

**預期結果**:
```
Servo 1 (Module 0, X): 實測零位 = ____ µs
Servo 2 (Module 0, Y): 實測零位 = ____ µs
Servo 3 (Module 1, X): 實測零位 = ____ µs
Servo 4 (Module 1, Y): 實測零位 = ____ µs
Servo 5 (Module 2, X): 實測零位 = ____ µs
Servo 6 (Module 2, Y): 實測零位 = ____ µs
Servo 7 (Module 3, X): 實測零位 = ____ µs
Servo 8 (Module 3, Y): 實測零位 = ____ µs
```

### Step 2: 更新 PWM_AUX_DIS 參數

將測量結果寫入配置文件:
```bash
param set-default PWM_AUX_DIS1 <測量值>
param set-default PWM_AUX_DIS2 <測量值>
...
```

### Step 3: 添加調試輸出

在 `generate_actuator_sp()` 中添加 (ControlAllocationModularBundled.cpp:275):
```cpp
void ControlAllocationModularBundled::generate_actuator_sp(const PseudoForceVector &pseudo_force)
{
    for (ActiveAgent i = 0; i < NUM_MODULES; i++) {
        matrix::Vector3f raw;
        const matrix::Vector3f f_i( pseudo_force.slice<3, 1>(3*i, 0) );
        inverse_transform(raw, f_i);

        // 添加調試輸出
        printf("[Module %d] f=(%.3f, %.3f, %.3f) -> Tf=%.3f, eta_x=%.2f°, eta_y=%.2f°\n",
            i, f_i(0), f_i(1), f_i(2),
            raw(2), raw(0)*180.0f/M_PI_F, raw(1)*180.0f/M_PI_F);

        const uint8_t motor_idx = 2*i;
        const uint8_t eta_idx = _actuator_idx_offset + 2*i;
        _actuator_sp(motor_idx  ) = raw(2);
        _actuator_sp(motor_idx+1) = 0;
        _actuator_sp(eta_idx  )   = raw(0);
        _actuator_sp(eta_idx+1)   = raw(1);
    }
    _prev_actuator_sp = _actuator_sp;
}
```

### Step 4: 驗證舵機方向

**測試程序**:
1. 武裝系統，油門保持最低
2. 手動給定測試偽力向量:
   ```
   Module 0: fx=0, fy=1.0, fz=5.0 → 期望 eta_x ≈ -11.5°
   Module 1: fx=1.0, fy=0, fz=5.0 → 期望 eta_y ≈ +11.3°
   ```
3. 觀察舵機實際旋轉方向
4. 如果方向錯誤，需要調整符號或使用 `PWM_AUX_REV` 參數

### Step 5: 實施 TRIM 功能（長期解決方案）

**需要修改的代碼**:

在 `ActuatorEffectivenessVTOL_TVMD.cpp` 中添加:
```cpp
// 在 getEffectivenessMatrix() 中添加
for (int i = 0; i < 2 * _geometry.num_agents; ++i) {
    // 計算 trim: 將舵機的物理零位映射到標準化值 0
    const float servo_min = _geometry.module_geometry[i/2].servo_conf[i%2](1);
    const float servo_max = _geometry.module_geometry[i/2].servo_conf[i%2](2);

    // 如果物理零位不在角度範圍中間，需要 trim
    // trim = -(物理零位角度) * scale
    configuration.trim[0](NUM_MOTORS + i) = 0.0f;  // 暫時先用 0
}
```

---

## 六、關鍵代碼位置總結

| 功能 | 文件 | 行號 | 說明 |
|------|------|------|------|
| 模組位置定義 | `13300_generic_vtol_tvmd` | 82-110 | CA_MD0~3_PX/PY/PZ |
| 舵機角度範圍 | `13300_generic_vtol_tvmd` | 187-232 | CA_SV_TL0~7_MINA/MAXA |
| PWM 解除武裝值 | `13300_generic_vtol_tvmd` | 330-337 | PWM_AUX_DIS1~8 |
| 偽力→角度轉換 | `ControlAllocationModularBundled.cpp` | 230-249 | inverse_transform() |
| 角度標準化 | `ActuatorEffectivenessVTOL_TVMD.cpp` | 278-283 | 縮放計算 |
| 標準化→PWM | `mixer_module.cpp` | 520-535 | output_limit_calc_single() |
| Trim 設置位置 | `ActuatorEffectivenessVTOL_TVMD.cpp` | 221 | ← 目前未實現！|
| 舵機索引映射 | `ActuatorEffectivenessVTOL_TVMD.hpp` | 169-170 | get_servo_idx() |

---

## 七、總結

**最可能的問題根源**:

1. **舵機物理零位 ≠ PWM 1500 µs** (最可能)
   - 每個舵機的機械零位可能在 1450~1550 µs 之間
   - 需要逐個測量並校準 `PWM_AUX_DIS` 參數

2. **TRIM 功能未實現** (系統性問題)
   - 代碼中完全沒有設置 `configuration.trim`
   - 需要添加代碼來處理舵機零位偏移

3. **舵機接線或方向定義不一致** (需驗證)
   - X-axis / Y-axis 的定義可能與實際硬體不符
   - 舵機旋轉方向可能需要反轉

**建議立即執行**:
- Step 1: 測量所有舵機的物理零位 PWM 值
- Step 3: 添加調試輸出查看實際角度計算
- Step 4: 驗證舵機旋轉方向是否符合預期

**長期改進**:
- 實施完整的 TRIM 功能
- 添加舵機校準工具或程序
