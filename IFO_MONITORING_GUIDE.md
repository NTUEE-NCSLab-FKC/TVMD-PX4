# Internal Force Optimizer (IFO) 監控與參數調整指南

## 一、如何在 QGC 查看 IFO 是否成功應用

### 1. **通過 uORB Topic 監控**

IFO 的運作資訊會發布到 `control_allocation_meta_data` topic。

#### **在系統控制台查看**：

```bash
# SSH 連接到飛控後執行
listener control_allocation_meta_data
```

#### **Topic 資料結構** (`ControlAllocationMetaData.msg`):

```
uint64 timestamp                    # 時間戳記

float32[6] control_sp              # 控制輸入（期望的 wrench）
float32[6] allocated_control       # 分配後的控制輸出

float32[16] f_x                    # 每次迭代的 fx 值（4 次迭代 × 4 模組）
float32[16] f_y                    # 每次迭代的 fy 值
float32[16] f_z                    # 每次迭代的 fz 值

float32[16] t_x                    # 每次迭代的約束檢查值
float32[16] t_y
float32[16] t_z
float32[16] t_min                  # 最小約束距離

int8[4] saturated_idx              # 飽和的模組索引
float32[4] increment               # 每次迭代的增量
```

### 2. **通過 MAVLink Inspector 監控**

在 QGC 中：

1. **打開 MAVLink Inspector**：
   - 點擊 QGC 左上角的 Q 圖標
   - 選擇 "Analyze Tools" → "MAVLink Inspector"

2. **找到 CONTROL_ALLOCATION_META_DATA 訊息**：
   - 在訊息列表中搜索 `control_allocation_meta_data`
   - 展開查看各個欄位

3. **關鍵監控欄位**：
   ```
   control_sp[0~5]        # 期望控制輸入
   allocated_control[0~5] # 實際分配輸出
   f_x[12~15]            # 最後一次迭代的 fx（IFO 應用後）
   f_y[12~15]            # 最後一次迭代的 fy
   f_z[12~15]            # 最後一次迭代的 fz
   ```

### 3. **通過調試輸出監控**

如果啟用了 `CA_EBRCA_DEBUGGER`（在 `ControlAllocationEBRCA.hpp:53`），
系統會在串口輸出詳細的 IFO 應用資訊：

#### **啟用調試輸出**：

修改 `ControlAllocationEBRCA.hpp:53`:
```cpp
#define CA_EBRCA_DEBUGGER  // 取消註解
#define CA_EBRCA_ENABLE_PBP
```

重新編譯後，會在控制台看到：

```
========== Applying PTE (Nullspace Projection + Scaling) ==========
Before PTE - Wrench error: 0.000123
Module 0: pos=(0.185, 0.160), psi=0.0 deg, inward_thr=(-0.707, -0.707), alpha=-10.6 deg, beta=10.6 deg
Module 1: pos=(0.185, -0.160), psi=0.0 deg, inward_thr=(-0.707, 0.707), alpha=10.6 deg, beta=10.6 deg
Module 2: pos=(-0.185, 0.160), psi=0.0 deg, inward_thr=(0.707, -0.707), alpha=-10.6 deg, beta=-10.6 deg
Module 3: pos=(-0.185, -0.160), psi=0.0 deg, inward_thr=(0.707, 0.707), alpha=10.6 deg, beta=-10.6 deg
✓ PTE applied successfully (k=0.8234)
  After PTE - Wrench error: 0.000087
===================================================================
```

### 4. **驗證 IFO 是否運作的指標**

#### **IFO 啟動條件** (`ControlAllocationEBRCA.cpp:157`):

```cpp
if (full_rank && (c >= 0.0f)) {
    // 應用 IFO
}
```

條件：
- `full_rank = true`：分配矩陣仍為滿秩（沒有太多飽和）
- `c >= 0.0f`：分配完成度 ≥ 0（c=0 表示全時啟動，c=1 表示完全分配）

#### **IFO 成功應用的標誌**:

1. **Wrench Error 變小**：
   - `Before PTE - Wrench error` vs `After PTE - Wrench error`
   - 如果 `After < Before`，表示 IFO 改善了控制精度

2. **Scaling Factor > 0.01**：
   - `k_scaling > 0.01` 才會應用
   - k 值越大，IFO 的影響越明顯

3. **Wrench Error < 1e-3**：
   - 只有當誤差非常小時才接受 IFO 結果
   - 確保不會破壞控制精度

---

## 二、target_tilt_angle_deg 的影響

`target_tilt_angle_deg` 定義在 `ControlAllocationEBRCA.cpp:169`：

```cpp
const float target_tilt_angle_deg = 15.0f;  // 15 degrees inward tilt
```

### 1. **物理意義**

這個參數控制 IFO 試圖將推力器傾斜多少角度**朝向機體中心**。

#### **傾斜方向定義**：

對於 H-frame 配置：
```
        前方 (X+)
          ↑
     0 ↖      ↗ 1      ← 向內傾斜 15°
       ┃     ┃
       ┃  +  ┃
       ┃     ┃
     2 ↖      ↗ 3      ← 向內傾斜 15°

● = 推力模組
↖/↗ = 傾斜方向（指向中心）
```

### 2. **不同 target_tilt_angle_deg 值的效果**

#### **✅ target_tilt_angle_deg = 15°**（預設值）

**效果**：
- 推力器向內傾斜 15°
- 產生向內的水平分力
- 增加系統的內部張力

**優點**：
```
+ 提高結構穩定性（類似張拉結構）
+ 增加對抗外部擾動的能力
+ 減少致動器飽和風險
+ 改善姿態控制響應
```

**缺點**：
```
- 垂直推力減少約 3.4%（cos(15°) ≈ 0.966）
- 需要更多總推力補償
```

**計算示例**（Module 0，位於前左）：
```
位置: (0.185, 0.16) m
向內方向: (-0.707, -0.707) 單位向量

目標角度:
  alpha (pitch) = -10.6°  （向下傾斜，因為 y 分量為負）
  beta (roll)   = +10.6°  （向左傾斜，因為 x 分量為負）

實際效果: 推力器從垂直向上傾斜到指向機體中心
```

---

#### **target_tilt_angle_deg = 0°**

**效果**：
- IFO 試圖保持推力器**完全垂直**
- 不產生內部張力
- 退化為標準 EBRCA（無 IFO 增強）

**優點**：
```
+ 最大垂直推力（100%）
+ 最小能量消耗
+ 最簡單的控制
```

**缺點**：
```
- 沒有內部力優化
- 系統剛性降低
- 對擾動敏感
- 致動器更容易飽和
```

**使用場景**：
- 需要最大升力時（例如重載起飛）
- 低速懸停節能
- 測試基準對比

---

#### **target_tilt_angle_deg = -15°**（負值）

**效果**：
- 推力器向**外**傾斜 15°
- 產生向外的水平分力
- 形成擴張結構（不穩定）

**⚠️ 警告**：
```
✗ 結構不穩定（擴張而非收縮）
✗ 降低抗擾能力
✗ 增加致動器飽和風險
✗ 可能導致控制發散
```

**物理類比**：
```
正值 (+15°): 像帳篷支架向內撐（穩定）
負值 (-15°): 像支架向外推（容易倒塌）
```

**不建議使用負值！**

---

### 3. **不同角度的數值對比表**

| target_tilt_angle_deg | 垂直推力保留 | 水平力分量 | 穩定性 | 能耗 | 建議使用 |
|---------------------|------------|-----------|--------|------|---------|
| **-15°** (向外) | 96.6% | -25.9% | ✗✗✗ 極差 | 中 | ❌ 不建議 |
| **-5°** (向外)  | 99.6% | -8.7% | ✗ 差 | 低 | ❌ 不建議 |
| **0°** (垂直)    | 100% | 0% | ○ 普通 | 最低 | ⚠️ 基準 |
| **5°** (向內)    | 99.6% | 8.7% | ✓ 好 | 低 | ✅ 可用 |
| **10°** (向內)   | 98.5% | 17.4% | ✓✓ 很好 | 中低 | ✅ 推薦 |
| **15°** (向內)   | 96.6% | 25.9% | ✓✓✓ 極佳 | 中 | ✅ **預設** |
| **20°** (向內)   | 94.0% | 34.2% | ✓✓✓ 極佳 | 中高 | ⚠️ 激進 |
| **25°** (向內)   | 90.6% | 42.3% | ✓✓✓ 極佳 | 高 | ⚠️ 激進 |
| **30°** (向內)   | 86.6% | 50.0% | ✓✓ 很好 | 很高 | ❌ 過度 |

---

### 4. **如何選擇 target_tilt_angle_deg**

#### **保守設定**（5°~10°）：
```cpp
const float target_tilt_angle_deg = 10.0f;
```
- 適合初始測試
- 對升力影響小
- 能耗增加少

#### **標準設定**（10°~15°）：
```cpp
const float target_tilt_angle_deg = 15.0f;  // 當前預設
```
- 平衡性能與效率
- 顯著改善穩定性
- 適合大多數情況

#### **激進設定**（15°~25°）：
```cpp
const float target_tilt_angle_deg = 20.0f;
```
- 最大穩定性
- 犧牲部分升力和效率
- 適合高擾動環境（強風、快速機動）

#### **動態調整**（未實現，未來改進）：
```cpp
// 根據飛行狀態動態調整
if (hover_mode) {
    target_tilt_angle_deg = 5.0f;   // 節能
} else if (aggressive_mode) {
    target_tilt_angle_deg = 20.0f;  // 穩定
}
```

---

### 5. **修改 target_tilt_angle_deg 的方法**

#### **方法 1：修改源代碼**（永久修改）

編輯 `ControlAllocationEBRCA.cpp:169`：
```cpp
const float target_tilt_angle_deg = 10.0f;  // 改為你想要的值
```

然後重新編譯：
```bash
make px4_fmu-v5_default
```

#### **方法 2：參數化**（推薦，未來改進）

在 `ControlAllocationEBRCA.hpp` 中添加參數：
```cpp
class ControlAllocationEBRCA: public ControlAllocationModularBundled
{
private:
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::CA_IFO_TILT_DEG>) _param_ifo_tilt_deg
    )
};
```

在 `params.c` 中定義：
```c
/**
 * IFO target tilt angle
 *
 * @unit deg
 * @min -30
 * @max 30
 * @decimal 1
 * @increment 1
 * @reboot_required false
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_IFO_TILT_DEG, 15.0f);
```

這樣就可以在 QGC 中動態調整，無需重新編譯！

---

## 三、實驗建議

### **測試步驟**：

1. **基準測試**（target_tilt_angle_deg = 0°）
   - 記錄基本飛行性能
   - 測量懸停功耗

2. **保守測試**（target_tilt_angle_deg = 10°）
   - 觀察穩定性改善
   - 測量功耗增加

3. **標準測試**（target_tilt_angle_deg = 15°）
   - 驗證預設配置
   - 測試抗擾能力

4. **激進測試**（target_tilt_angle_deg = 20°）
   - 測試極限穩定性
   - 評估功耗代價

### **評估指標**：

| 指標 | 測量方法 | 期望值 |
|------|---------|-------|
| 懸停功耗 | 電流監控 | 越低越好 |
| 姿態響應 | Step 響應測試 | 越快越好 |
| 抗風能力 | 風速測試 | 越強越好 |
| 控制精度 | 位置保持測試 | 誤差越小越好 |
| 致動器飽和頻率 | Log 分析 | 越少越好 |

---

## 四、常見問題

### Q1: IFO 為什麼有時候不應用？

**可能原因**：
1. `full_rank = false`：太多致動器飽和
2. `wrench_error > 1e-3`：控制誤差太大
3. `k_scaling < 0.01`：沒有足夠的零空間自由度
4. 分配完成度 `c < 0.0f`

**解決方法**：
- 降低控制增益，減少飽和
- 增加致動器限制範圍
- 調整 `c >= 0.0f` 的條件到更低值

### Q2: target_tilt_angle_deg 越大越好嗎？

**不是！**
- 過大的角度會顯著降低垂直推力
- 增加能耗
- 可能導致推力不足

**建議範圍**：5°~20°

### Q3: 如何知道 IFO 在實際飛行中起作用？

**觀察指標**：
1. 舵機角度變化更平滑
2. 懸停時舵機輕微向內傾斜（約 10-15°）
3. 抗風性能提升
4. 姿態控制更穩定
5. MAVLink Inspector 中看到 wrench_error 降低

### Q4: 負的 target_tilt_angle_deg 有什麼用？

**幾乎沒有！**
- 物理上不穩定
- 只用於研究或對比實驗
- 實際飛行中應避免使用

---

## 五、總結

### **IFO 的核心理念**：

```
控制分配 → 滿足 Wrench 需求
     ↓
  零空間優化 → 在不影響 Wrench 的前提下
     ↓
  調整舵機角度 → 使推力器向內傾斜
     ↓
  形成內部張力 → 提高系統剛性和穩定性
```

### **最佳實踐**：

1. **預設使用 15°**：經過權衡的最佳值
2. **懸停節能用 5-10°**：減少功耗
3. **高機動用 15-20°**：最大穩定性
4. **永遠不要用負值**：物理上不合理

### **監控建議**：

1. 使用 MAVLink Inspector 實時監控
2. 啟用調試輸出進行詳細分析
3. 記錄飛行日誌進行後期評估
4. 對比不同 target_tilt_angle_deg 的性能
