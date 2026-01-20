# TVMD PX4 SITL 完整指南

本指南涵蓋 TVMD 的 PX4 Software-In-The-Loop (SITL) 模擬環境設定與啟動。

---

## 📋 系統需求

- Ubuntu 20.04 或更新版本
- Python 3.8+
- Gazebo Ignition (gz)
- 網路連接（首次安裝依賴時需要）

---

## 🔧 首次設定（必須執行一次）

### 步驟 1：安裝系統依賴

```bash
# 安裝 Gazebo Ignition
sudo apt-get update
sudo apt-get install -y gz-garden

# 確認安裝
gz sim --version
```

### 步驟 2：安裝 PX4 Python 依賴

#### 方法 A：使用 PX4 官方腳本（推薦）

```bash
cd /home/user/TVMD-PX4
bash ./Tools/setup/ubuntu.sh
```

#### 方法 B：手動安裝

```bash
pip3 install --user \
    kconfiglib \
    jsonschema \
    jinja2 \
    pyros-genmsg \
    packaging \
    toml \
    numpy \
    empy \
    pyyaml
```

#### 方法 C：使用 requirements.txt

```bash
cd /home/user/TVMD-PX4
pip3 install --user -r Tools/setup/requirements.txt
```

### 步驟 3：驗證安裝

```bash
# 檢查 Python 依賴
python3 -c "import kconfiglib; print('✅ kconfiglib 已安裝')"
python3 -c "import jinja2; print('✅ jinja2 已安裝')"

# 檢查 Gazebo
gz sim --version
```

---

## 🚀 啟動 TVMD SITL

### 方法 1：使用啟動腳本（最簡單）

```bash
cd /home/user/TVMD-PX4

# 基本啟動
./launch_tvmd_sitl.sh

# 使用不同的世界
./launch_tvmd_sitl.sh empty    # 空世界
./launch_tvmd_sitl.sh default  # 預設世界
```

### 方法 2：使用環境變數

```bash
cd /home/user/TVMD-PX4

# 一行命令
PX4_SIM_MODEL=tvmd make px4_sitl gz

# 或分開設定
export PX4_SIM_MODEL=tvmd
export PX4_GZ_WORLD=default
make px4_sitl gz
```

### 方法 3：手動啟動（進階）

#### 終端 1：編譯 PX4
```bash
cd /home/user/TVMD-PX4
make px4_sitl_default
```

#### 終端 2：啟動 Gazebo
```bash
cd /home/user/TVMD-PX4
gz sim -v 4 -r Tools/simulation/gz/worlds/default.sdf
```

#### 終端 3：啟動 PX4 與 TVMD Airframe
```bash
cd /home/user/TVMD-PX4/build/px4_sitl_default/bin

# 設定環境變數
export PX4_SIM_MODEL=tvmd

# 啟動 PX4
./px4 -d ../../../ROMFS/px4fmu_common

# 在 PX4 shell 中：
pxh> param set SYS_AUTOSTART 4007
pxh> param set SIM_GZ_EN 1
pxh> param save
pxh> reboot
```

---

## 🎮 驗證 SITL 是否正常運行

### 檢查清單

啟動後，應該看到：

- [ ] **Gazebo 視窗**：顯示 TVMD 模型
- [ ] **PX4 終端**：顯示 "INFO [commander] Ready for takeoff"
- [ ] **4 個馬達**：在 Gazebo 中可見（位於 4 個角落）
- [ ] **8 個伺服馬達**：gimbal 關節可動作

### 測試命令

在 PX4 shell 中執行：

```bash
# 檢查 airframe
param show SYS_AUTOSTART
# 應該顯示: 4007

# 檢查模型配置
param show CA_ROTOR_COUNT
# 應該顯示: 4

param show CA_SV_TL_COUNT
# 應該顯示: 8

# 檢查模組位置
param show CA_MD0_PX
# 應該顯示: 0.185000

param show CA_MD0_PY
# 應該顯示: 0.160000

# 測試 actuator 輸出
actuator_test -m 1 -v 0.5    # 測試馬達 1
actuator_test -m 2 -v 0.5    # 測試馬達 2
actuator_test -s 201 -v 0.5  # 測試伺服 1
```

### Gazebo 視覺檢查

1. **馬達旋轉**：arm 後應該看到螺旋槳旋轉
2. **伺服動作**：actuator_test 時應該看到 gimbal 移動
3. **模組位置**：4 個模組位於 (±0.185, ±0.16)

---

## 🔍 常見問題排除

### 問題 1：`ninja: error: unknown target 'gz_tvmd'`

**原因**：PX4 沒有自動建立 `gz_tvmd` make 目標。

**解決方案**：使用環境變數方式
```bash
PX4_SIM_MODEL=tvmd make px4_sitl gz
```

### 問題 2：`ModuleNotFoundError: No module named 'kconfiglib'`

**原因**：Python 依賴未安裝。

**解決方案**：
```bash
pip3 install --user kconfiglib
# 或安裝所有依賴
pip3 install --user -r Tools/setup/requirements.txt
```

### 問題 3：`CMake Error: build directory not found`

**原因**：編譯失敗或 build 目錄損壞。

**解決方案**：
```bash
# 清理並重新編譯
rm -rf build/px4_sitl_default
make px4_sitl_default
```

### 問題 4：Gazebo 視窗沒有顯示 TVMD

**可能原因 1**：模型路徑錯誤

**檢查**：
```bash
ls -la Tools/simulation/gz/models/tvmd/model.sdf
```

**可能原因 2**：環境變數設定錯誤

**檢查**：
```bash
echo $PX4_SIM_MODEL  # 應該是 "tvmd"
echo $GZ_SIM_RESOURCE_PATH  # 應該包含 models 路徑
```

**解決方案**：
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/Tools/simulation/gz/models
```

### 問題 5：馬達不旋轉

**原因**：未 arm 或 actuator 配置錯誤。

**解決方案**：
```bash
# 在 PX4 shell 中
commander arm
# 或使用 QGroundControl arm
```

---

## 📊 TVMD 配置摘要

| 參數 | 值 | 說明 |
|-----|---|------|
| **Airframe ID** | 4007 | SITL TVMD airframe |
| **馬達數量** | 4 | 單螺旋槳配置 |
| **伺服數量** | 8 | 每模組 2 個 (X/Y 軸) |
| **模組位置** | (±0.185, ±0.16) | 矩形 H-frame |
| **伺服範圍** | ±45° | 圓錐形 LAFS |
| **CT 係數** | 0.012 | 推力係數 |
| **旋轉方向** | CW, CCW, CCW, CW | 馬達 0-3 |

---

## 🎯 下一步

### 1. 連接 QGroundControl

```bash
# QGC 會自動偵測 UDP 連接 (14550)
# 或手動添加連接：
# - Type: UDP
# - Port: 14550
```

### 2. 測試基本飛行

在 QGroundControl 中：
1. Arm 飛機
2. 切換到 Position 模式
3. 使用 joystick 或手動命令測試

### 3. 進階測試

```bash
# 使用 MAVSDK 或 MAVROS 測試
# 或使用 PX4 內建 commander 測試
commander takeoff
commander land
```

---

## 📖 相關文檔

- **硬體配置**：`ROMFS/px4fmu_common/init.d/airframes/13300_generic_vtol_tvmd`
- **SITL 配置**：`ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_tvmd`
- **Gazebo 模型**：`Tools/simulation/gz/models/tvmd/model.sdf`
- **STP 整合**：`Tools/simulation/gz/models/tvmd/CUSTOM_MODEL_INTEGRATION.md`

---

## 💡 提示

1. **首次啟動較慢**：第一次編譯和啟動需要較長時間
2. **保持終端開啟**：PX4 和 Gazebo 需要在背景運行
3. **使用 tmux/screen**：方便管理多個終端
4. **查看日誌**：`tail -f build/px4_sitl_default/logs/debug_mavlink.log`

---

## 🛠️ 進階配置

### 自訂 Gazebo 世界

```bash
# 建立自訂世界檔案
cp Tools/simulation/gz/worlds/default.sdf Tools/simulation/gz/worlds/my_world.sdf

# 使用自訂世界
PX4_GZ_WORLD=my_world PX4_SIM_MODEL=tvmd make px4_sitl gz
```

### 多機模擬

```bash
# 終端 1：UAV 1
PX4_SIM_MODEL=tvmd make px4_sitl_instance_0 gz

# 終端 2：UAV 2
PX4_SIM_MODEL=tvmd make px4_sitl_instance_1 gz
```

### 自訂起始位置

修改 `Tools/simulation/gz/worlds/default.sdf`：
```xml
<pose>0 0 0.5 0 0 0</pose>  <!-- X Y Z Roll Pitch Yaw -->
```

---

需要協助？請參考 PX4 官方文檔或提交 issue。
