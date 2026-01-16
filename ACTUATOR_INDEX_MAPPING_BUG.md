# TVMD 馬達與舵機配置映射檢查報告

## 問題背景
在 QGC 的 Actuator Testing 中，舵機滑桿在中間位置時，舵機物理上位於中點。
但解鎖起飛待命時，舵機角度會改變，不在中點位置。

---

## 一、致動器索引映射關係

### 1. **ActuatorEffectivenessVTOL_TVMD 的索引函數**

**位置**: `ActuatorEffectivenessVTOL_TVMD.hpp:169-170`

```cpp
// 馬達索引
inline uint8_t get_motor_idx(uint8_t module_id, uint8_t offset) {
    return module_id;
}

// 舵機索引
inline uint8_t get_servo_idx(uint8_t module_id, uint8_t offset) {
    return 2 * module_id + offset + _geometry.num_agents;
}
```

### 2. **映射計算 (num_agents = 4)**

#### 馬達映射:
```
get_motor_idx(0, 0) = 0  → Motor 0 → PWM_MAIN_FUNC1 = 101
get_motor_idx(1, 0) = 1  → Motor 1 → PWM_MAIN_FUNC2 = 102
get_motor_idx(2, 0) = 2  → Motor 2 → PWM_MAIN_FUNC3 = 103
get_motor_idx(3, 0) = 3  → Motor 3 → PWM_MAIN_FUNC4 = 104
```

#### 舵機映射:
```
Module 0, X-axis: get_servo_idx(0, 0) = 2*0 + 0 + 4 = 4  → Servo 0 → PWM_AUX_FUNC1 = 201
Module 0, Y-axis: get_servo_idx(0, 1) = 2*0 + 1 + 4 = 5  → Servo 1 → PWM_AUX_FUNC2 = 202
Module 1, X-axis: get_servo_idx(1, 0) = 2*1 + 0 + 4 = 6  → Servo 2 → PWM_AUX_FUNC3 = 203
Module 1, Y-axis: get_servo_idx(1, 1) = 2*1 + 1 + 4 = 7  → Servo 3 → PWM_AUX_FUNC4 = 204
Module 2, X-axis: get_servo_idx(2, 0) = 2*2 + 0 + 4 = 8  → Servo 4 → PWM_AUX_FUNC5 = 205
Module 2, Y-axis: get_servo_idx(2, 1) = 2*2 + 1 + 4 = 9  → Servo 5 → PWM_AUX_FUNC6 = 206
Module 3, X-axis: get_servo_idx(3, 0) = 2*3 + 0 + 4 = 10 → Servo 6 → PWM_AUX_FUNC7 = 207
Module 3, Y-axis: get_servo_idx(3, 1) = 2*3 + 1 + 4 = 11 → Servo 7 → PWM_AUX_FUNC8 = 208
```

---

## 二、ControlAllocationModularBundled 的致動器設定

### 1. **_actuator_idx_offset 定義**

**位置**: `ControlAllocationModularBundled.hpp:171`

```cpp
const uint8_t _actuator_idx_offset{NUM_MODULES * 2};
// NUM_MODULES = 4 → _actuator_idx_offset = 8
```

### 2. **generate_actuator_sp() 函數**

**位置**: `ControlAllocationModularBundled.cpp:275-296`

```cpp
void ControlAllocationModularBundled::generate_actuator_sp(const PseudoForceVector &pseudo_force)
{
    for (ActiveAgent i = 0; i < NUM_MODULES; i++) {
        matrix::Vector3f raw;
        const matrix::Vector3f f_i( pseudo_force.slice<3, 1>(3*i, 0) );
        inverse_transform(raw, f_i);

        const uint8_t motor_idx = 2*i;           // ← 這裡！
        const uint8_t eta_idx = _actuator_idx_offset + 2*i;  // ← 這裡！

        _actuator_sp(motor_idx  ) = raw(2);  // Tf    (N)
        _actuator_sp(motor_idx+1) = 0;       // Td    (Nm) - 未使用
        _actuator_sp(eta_idx  )   = raw(0);  // eta_x (rad)
        _actuator_sp(eta_idx+1)   = raw(1);  // eta_y (rad)
    }
    _prev_actuator_sp = _actuator_sp;
}
```

### 3. **實際索引計算**

```
Module 0:
  motor_idx = 2*0 = 0      → _actuator_sp(0) = Tf
  motor_idx+1 = 1          → _actuator_sp(1) = 0 (Td)
  eta_idx = 8 + 2*0 = 8    → _actuator_sp(8) = eta_x
  eta_idx+1 = 9            → _actuator_sp(9) = eta_y

Module 1:
  motor_idx = 2*1 = 2      → _actuator_sp(2) = Tf
  motor_idx+1 = 3          → _actuator_sp(3) = 0 (Td)
  eta_idx = 8 + 2*1 = 10   → _actuator_sp(10) = eta_x
  eta_idx+1 = 11           → _actuator_sp(11) = eta_y

Module 2:
  motor_idx = 2*2 = 4      → _actuator_sp(4) = Tf
  motor_idx+1 = 5          → _actuator_sp(5) = 0 (Td)
  eta_idx = 8 + 2*2 = 12   → _actuator_sp(12) = eta_x
  eta_idx+1 = 13           → _actuator_sp(13) = eta_y

Module 3:
  motor_idx = 2*3 = 6      → _actuator_sp(6) = Tf
  motor_idx+1 = 7          → _actuator_sp(7) = 0 (Td)
  eta_idx = 8 + 2*3 = 14   → _actuator_sp(14) = eta_x
  eta_idx+1 = 15           → _actuator_sp(15) = eta_y
```

---

## 三、⚠️ 發現嚴重的索引映射錯誤！

### **問題 1: 馬達索引不連續**

ControlAllocationModularBundled 使用:
```cpp
motor_idx = 2*i  // 產生 0, 2, 4, 6
```

ActuatorEffectivenessVTOL_TVMD 期望:
```cpp
get_motor_idx(i, 0) = i  // 產生 0, 1, 2, 3
```

**結果**:
- Module 0 馬達 → _actuator_sp(0) ✓ 正確
- Module 1 馬達 → _actuator_sp(2) ✗ **應該是 _actuator_sp(1)**
- Module 2 馬達 → _actuator_sp(4) ✗ **應該是 _actuator_sp(2)**
- Module 3 馬達 → _actuator_sp(6) ✗ **應該是 _actuator_sp(3)**

### **問題 2: 舵機索引不匹配**

ControlAllocationModularBundled 使用:
```cpp
eta_idx = _actuator_idx_offset + 2*i = 8 + 2*i
// 產生 8, 10, 12, 14
```

ActuatorEffectivenessVTOL_TVMD 期望:
```cpp
get_servo_idx(i, 0) = 2*i + 0 + num_agents = 2*i + 4
// 產生 4, 6, 8, 10
```

**結果**:
- Module 0, X-axis → _actuator_sp(8) ✗ **應該是 _actuator_sp(4)**
- Module 0, Y-axis → _actuator_sp(9) ✗ **應該是 _actuator_sp(5)**
- Module 1, X-axis → _actuator_sp(10) ✗ **應該是 _actuator_sp(6)**
- Module 1, Y-axis → _actuator_sp(11) ✗ **應該是 _actuator_sp(7)**
- Module 2, X-axis → _actuator_sp(12) ✗ **應該是 _actuator_sp(8)**
- Module 2, Y-axis → _actuator_sp(13) ✗ **應該是 _actuator_sp(9)**
- Module 3, X-axis → _actuator_sp(14) ✗ **應該是 _actuator_sp(10)**
- Module 3, Y-axis → _actuator_sp(15) ✗ **應該是 _actuator_sp(11)**

### **問題 3: _actuator_sp 中間有未使用的索引**

```
_actuator_sp(0)  = Module 0 Tf     ✓
_actuator_sp(1)  = 0 (未使用的 Td) ← 浪費
_actuator_sp(2)  = Module 1 Tf     ✗ 錯誤位置
_actuator_sp(3)  = 0 (未使用的 Td) ← 浪費
_actuator_sp(4)  = Module 2 Tf     ✗ 錯誤位置
_actuator_sp(5)  = 0 (未使用的 Td) ← 浪費
_actuator_sp(6)  = Module 3 Tf     ✗ 錯誤位置
_actuator_sp(7)  = 0 (未使用的 Td) ← 浪費
_actuator_sp(8)  = Module 0 eta_x  ✗ 錯誤位置
_actuator_sp(9)  = Module 0 eta_y  ✗ 錯誤位置
_actuator_sp(10) = Module 1 eta_x  ✗ 錯誤位置
_actuator_sp(11) = Module 1 eta_y  ✗ 錯誤位置
_actuator_sp(12) = Module 2 eta_x  ✗ 錯誤位置
_actuator_sp(13) = Module 2 eta_y  ✗ 錯誤位置
_actuator_sp(14) = Module 3 eta_x  ✗ 錯誤位置
_actuator_sp(15) = Module 3 eta_y  ✗ 錯誤位置
```

---

## 四、正確的致動器布局

### **應該是這樣**:

```
_actuator_sp(0)  = Module 0 Motor (Tf)
_actuator_sp(1)  = Module 1 Motor (Tf)
_actuator_sp(2)  = Module 2 Motor (Tf)
_actuator_sp(3)  = Module 3 Motor (Tf)
_actuator_sp(4)  = Module 0 Servo X-axis (eta_x)
_actuator_sp(5)  = Module 0 Servo Y-axis (eta_y)
_actuator_sp(6)  = Module 1 Servo X-axis (eta_x)
_actuator_sp(7)  = Module 1 Servo Y-axis (eta_y)
_actuator_sp(8)  = Module 2 Servo X-axis (eta_x)
_actuator_sp(9)  = Module 2 Servo Y-axis (eta_y)
_actuator_sp(10) = Module 3 Servo X-axis (eta_x)
_actuator_sp(11) = Module 3 Servo Y-axis (eta_y)
```

---

## 五、QGC Actuator Testing vs 解鎖待命的差異

### 1. **QGC Actuator Testing 模式**

**位置**: `actuator_test.cpp:86-95`

```cpp
// handle servos: add trim
if ((int)OutputFunction::Servo1 <= actuator_test.function &&
    actuator_test.function <= (int)OutputFunction::ServoMax) {

    actuator_servos_trim_s trim{};
    _actuator_servos_trim_sub.copy(&trim);
    int idx = actuator_test.function - (int)OutputFunction::Servo1;

    if (idx < actuator_servos_trim_s::NUM_CONTROLS) {
        value += trim.trim[idx];  // ← 在測試模式下會加上 trim！
    }
}
```

**特點**:
- 滑桿值直接轉換為標準化值 (-1 ~ 1)
- **會加上 trim 值**
- 滑桿在中間 → value = 0 → PWM = 1500 µs (如果 trim = 0)

### 2. **解鎖待命模式**

**流程**:
1. 控制分配算法計算偽力向量 `_f`
2. `inverse_transform()` 轉換為舵機角度 (eta_x, eta_y)
3. `generate_actuator_sp()` 設置 `_actuator_sp`
4. **但索引錯誤，導致值被寫到錯誤位置**
5. PWM 輸出讀取錯誤位置的值

**問題**:
- 即使控制算法計算出正確的 eta_x = 0, eta_y = 0
- 由於索引錯誤，實際的舵機可能讀取到其他模組的值
- 或者讀取到未初始化的內存

---

## 六、為什麼 QGC 測試正常但飛行異常？

### **QGC Actuator Testing**:
```
用戶滑桿 → actuator_test 消息 → 直接設置 PWM
(繞過控制分配，直接控制硬體)
```

### **解鎖飛行**:
```
控制器 → 控制分配 (錯誤索引) → 錯誤的 _actuator_sp → 錯誤的 PWM
```

因此：
- **QGC 測試**：直接控制 PWM，索引正確，舵機在中點 ✓
- **解鎖待命**：經過控制分配，索引錯誤，舵機角度錯誤 ✗

---

## 七、需要修復的代碼

### **修復方案**: 修改 `ControlAllocationModularBundled.cpp:275-290`

#### **當前代碼** (錯誤):
```cpp
void ControlAllocationModularBundled::generate_actuator_sp(const PseudoForceVector &pseudo_force)
{
    for (ActiveAgent i = 0; i < NUM_MODULES; i++) {
        matrix::Vector3f raw;
        const matrix::Vector3f f_i( pseudo_force.slice<3, 1>(3*i, 0) );
        inverse_transform(raw, f_i);

        const uint8_t motor_idx = 2*i;                        // ✗ 錯誤
        const uint8_t eta_idx = _actuator_idx_offset + 2*i;  // ✗ 錯誤

        _actuator_sp(motor_idx  ) = raw(2);  // Tf
        _actuator_sp(motor_idx+1) = 0;       // Td (不需要)
        _actuator_sp(eta_idx  )   = raw(0);  // eta_x
        _actuator_sp(eta_idx+1)   = raw(1);  // eta_y
    }
    _prev_actuator_sp = _actuator_sp;
}
```

#### **修正後代碼** (正確):
```cpp
void ControlAllocationModularBundled::generate_actuator_sp(const PseudoForceVector &pseudo_force)
{
    for (ActiveAgent i = 0; i < NUM_MODULES; i++) {
        matrix::Vector3f raw;
        const matrix::Vector3f f_i( pseudo_force.slice<3, 1>(3*i, 0) );
        inverse_transform(raw, f_i);

        // 修正索引計算
        const uint8_t motor_idx = i;              // ✓ 正確: 0, 1, 2, 3
        const uint8_t eta_idx = NUM_MODULES + 2*i; // ✓ 正確: 4, 6, 8, 10

        _actuator_sp(motor_idx) = raw(2);     // Tf    → _actuator_sp(0,1,2,3)
        _actuator_sp(eta_idx  ) = raw(0);     // eta_x → _actuator_sp(4,6,8,10)
        _actuator_sp(eta_idx+1) = raw(1);     // eta_y → _actuator_sp(5,7,9,11)
    }
    _prev_actuator_sp = _actuator_sp;
}
```

同時需要修改 `_actuator_idx_offset`:
```cpp
// ControlAllocationModularBundled.hpp:171
const uint8_t _actuator_idx_offset{NUM_MODULES};  // 改為 4，不是 8
```

---

## 八、驗證修正

### **修正前**:
```
Module 0: Tf→sp(0), eta_x→sp(8),  eta_y→sp(9)   ✗
Module 1: Tf→sp(2), eta_x→sp(10), eta_y→sp(11)  ✗
Module 2: Tf→sp(4), eta_x→sp(12), eta_y→sp(13)  ✗
Module 3: Tf→sp(6), eta_x→sp(14), eta_y→sp(15)  ✗
```

### **修正後**:
```
Module 0: Tf→sp(0), eta_x→sp(4),  eta_y→sp(5)   ✓
Module 1: Tf→sp(1), eta_x→sp(6),  eta_y→sp(7)   ✓
Module 2: Tf→sp(2), eta_x→sp(8),  eta_y→sp(9)   ✓
Module 3: Tf→sp(3), eta_x→sp(10), eta_y→sp(11)  ✓
```

這樣就與 `ActuatorEffectivenessVTOL_TVMD` 的期望一致了！

---

## 九、總結

### **根本原因**:
`ControlAllocationModularBundled::generate_actuator_sp()` 中的致動器索引計算錯誤，
導致控制分配的輸出被寫入錯誤的 `_actuator_sp` 位置。

### **症狀**:
- QGC Actuator Testing 正常（直接控制 PWM，繞過控制分配）
- 解鎖待命時舵機角度錯誤（經過控制分配，索引錯誤）

### **修復建議**:
1. 修改 `ControlAllocationModularBundled.cpp:284-289` 的索引計算
2. 修改 `ControlAllocationModularBundled.hpp:171` 的 `_actuator_idx_offset` 值
3. 移除不必要的 Td (扭矩) 索引

### **影響**:
- 馬達可能接收到錯誤模組的推力指令
- 舵機接收到錯誤模組的角度指令
- 導致飛行控制完全混亂

**這是一個嚴重的 Bug，必須立即修復！**
