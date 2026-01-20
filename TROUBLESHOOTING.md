# TVMD PX4 SITL 故障排除指南

本文檔涵蓋 TVMD PX4 SITL 環境中常見問題的解決方案。

---

## 🔴 問題 1: `-bash: /usr/share/gazebo/setup.sh: No such file or directory`

### 原因
這個錯誤表示：
1. **Gazebo 未安裝**
2. 或安裝了錯誤版本的 Gazebo

PX4 現在使用 **Gazebo Ignition (Harmonic)**，而不是舊的 **Gazebo Classic**。

### 解決方案

#### 步驟 1：檢查當前 Gazebo 安裝狀態

```bash
# 檢查 Gazebo Ignition (新版本)
gz sim --version

# 檢查 Gazebo Classic (舊版本)
gazebo --version
```

#### 步驟 2：安裝 Gazebo Harmonic (推薦)

**方法 A：使用自動化腳本**

```bash
cd /home/user/TVMD-PX4

# 執行安裝腳本（需要 sudo 權限和網路連接）
sudo ./install_gazebo.sh
```

**方法 B：手動安裝**

```bash
# 1. 安裝必要工具
sudo apt-get update
sudo apt-get install -y lsb-release wget gnupg

# 2. 添加 Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. 安裝 Gazebo Harmonic
sudo apt-get update
sudo apt-get install -y gz-harmonic

# 4. 驗證安裝
gz sim --version
```

**方法 C：使用 PX4 官方腳本**

```bash
cd /home/user/TVMD-PX4
bash ./Tools/setup/ubuntu.sh
```

#### 步驟 3：設定環境變數

```bash
# 添加到 ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/usr/share/gz/gz-sim7/worlds' >> ~/.bashrc

# 重新載入
source ~/.bashrc
```

#### 步驟 4：測試 Gazebo

```bash
# 啟動空的 Gazebo 世界
gz sim -v 4

# 如果成功，應該看到 Gazebo GUI 視窗
```

---

## 🔴 問題 2: `ninja: error: unknown target 'gz_tvmd'`

### 原因
PX4 沒有自動建立 `gz_tvmd` make 目標。

### 解決方案

**使用環境變數指定模型：**

```bash
# ✅ 正確
PX4_SIM_MODEL=tvmd make px4_sitl gz

# ❌ 錯誤
make px4_sitl gz_tvmd  # 這個目標不存在
```

**或使用啟動腳本：**

```bash
cd /home/user/TVMD-PX4
./launch_tvmd_sitl.sh
```

---

## 🔴 問題 3: `ModuleNotFoundError: No module named 'kconfiglib'`

### 原因
Python 依賴未安裝。

### 解決方案

```bash
# 安裝所有必要的 Python 套件
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

# 或使用 PX4 提供的 requirements.txt
pip3 install --user -r Tools/setup/requirements.txt
```

---

## 🔴 問題 4: 網路連接失敗

### 症狀
```
Temporary failure resolving 'archive.ubuntu.com'
Could not find a version that satisfies the requirement
```

### 原因
- 無網路連接
- DNS 問題
- Proxy 設定問題

### 解決方案

#### 檢查網路連接

```bash
# 測試網路
ping -c 3 8.8.8.8

# 測試 DNS
ping -c 3 google.com
```

#### 配置 DNS（如果需要）

```bash
# 編輯 resolv.conf
sudo nano /etc/resolv.conf

# 添加 Google DNS
nameserver 8.8.8.8
nameserver 8.8.4.4
```

#### 清除 Proxy 設定

```bash
# 暫時清除 proxy
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY

# 然後重試安裝
pip3 install --user kconfiglib
```

---

## 🔴 問題 5: Gazebo 視窗沒有顯示 TVMD 模型

### 可能原因 1：模型路徑未設定

**檢查：**
```bash
echo $GZ_SIM_RESOURCE_PATH
```

**修復：**
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/Tools/simulation/gz/models
```

### 可能原因 2：model.sdf 語法錯誤

**檢查：**
```bash
gz sdf -k Tools/simulation/gz/models/tvmd/model.sdf
```

### 可能原因 3：環境變數設定錯誤

**檢查：**
```bash
echo $PX4_SIM_MODEL  # 應該是 "tvmd"
```

**修復：**
```bash
export PX4_SIM_MODEL=tvmd
```

---

## 🔴 問題 6: PX4 編譯失敗

### 常見錯誤 A：CMake 錯誤

```bash
# 清理 build 目錄
rm -rf build/px4_sitl_default

# 重新編譯
make px4_sitl_default
```

### 常見錯誤 B：權限問題

```bash
# 修復權限
sudo chown -R $USER:$USER .
```

### 常見錯誤 C：依賴缺失

```bash
# 安裝完整依賴
bash ./Tools/setup/ubuntu.sh
```

---

## 🔴 問題 7: 馬達不旋轉 / 飛機不動

### 檢查 1：飛機是否已 ARM

```bash
# 在 PX4 shell 中
commander arm
```

### 檢查 2：Actuator 配置是否正確

```bash
# 檢查馬達數量
param show CA_ROTOR_COUNT  # 應該是 4

# 檢查伺服數量
param show CA_SV_TL_COUNT  # 應該是 8

# 檢查 airframe
param show SYS_AUTOSTART  # 應該是 4007
```

### 檢查 3：測試單一 Actuator

```bash
# 測試馬達 1
actuator_test -m 1 -v 0.5

# 測試伺服 1
actuator_test -s 201 -v 0.5
```

---

## 🔴 問題 8: 模擬速度太慢

### 優化 1：簡化碰撞模型

確保 model.sdf 中使用簡單幾何體進行碰撞：

```xml
<!-- ✅ 好 -->
<collision>
  <geometry>
    <box><size>0.5 0.4 0.1</size></box>
  </geometry>
</collision>

<!-- ❌ 不好 -->
<collision>
  <geometry>
    <mesh><uri>complex_model.dae</uri></mesh>
  </geometry>
</collision>
```

### 優化 2：降低物理更新率

在 world 檔案中：

```xml
<physics type="ode">
  <max_step_size>0.01</max_step_size>  <!-- 從 0.001 增加到 0.01 -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- 從 1000 降低 -->
</physics>
```

### 優化 3：關閉不必要的 sensor

暫時註解掉不需要的感測器：

```xml
<!-- <sensor name='camera' type='camera'> -->
```

---

## 🔴 問題 9: QGroundControl 連接不上

### 檢查 1：確認 PX4 正在運行

```bash
# 查看 PX4 進程
ps aux | grep px4
```

### 檢查 2：檢查 MAVLink 連接

```bash
# 在 PX4 shell 中查看 MAVLink 狀態
mavlink status
```

### 檢查 3：確認端口沒有被佔用

```bash
# 檢查 UDP 端口 14550
sudo netstat -tulnp | grep 14550
```

### 解決方案：重啟 PX4 MAVLink

```bash
# 在 PX4 shell 中
mavlink stop-all
mavlink start -u 14550 -r 4000000
```

---

## 🔴 問題 10: 版本不相容

### 檢查版本

```bash
# Gazebo 版本
gz sim --version

# PX4 版本
git log -1 --oneline

# Ubuntu 版本
lsb_release -a
```

### 相容性表

| Ubuntu | Gazebo 版本 | PX4 分支 | 狀態 |
|--------|------------|----------|------|
| 24.04  | Harmonic   | main     | ✅ 推薦 |
| 22.04  | Garden     | main     | ✅ 穩定 |
| 20.04  | Citadel    | v1.14    | ⚠️ 舊版 |

---

## 📚 有用的除錯命令

### Gazebo 除錯

```bash
# 啟動 Gazebo 並顯示詳細日誌
gz sim -v 4

# 檢查可用的 Gazebo plugins
gz plugin -l

# 檢查模型
gz model --list
```

### PX4 除錯

```bash
# 啟動 PX4 並顯示詳細日誌
make px4_sitl gz VERBOSE=1

# 查看 PX4 日誌
tail -f build/px4_sitl_default/logs/debug_mavlink.log

# 在 PX4 shell 中查看系統狀態
top
```

### 系統除錯

```bash
# 檢查系統資源
htop

# 檢查 GPU 狀態（如果有）
nvidia-smi

# 檢查磁碟空間
df -h
```

---

## 📞 尋求幫助

如果以上方法都無法解決問題：

1. **檢查 PX4 官方論壇**
   - https://discuss.px4.io/

2. **查看 PX4 文檔**
   - https://docs.px4.io/main/en/simulation/

3. **GitHub Issues**
   - https://github.com/PX4/PX4-Autopilot/issues

4. **收集除錯資訊**
   ```bash
   # 系統資訊
   uname -a
   lsb_release -a

   # Gazebo 版本
   gz sim --version

   # PX4 版本
   git log -1

   # 錯誤日誌
   cat build/px4_sitl_default/logs/debug_mavlink.log
   ```

---

## ✅ 快速檢查清單

執行 SITL 前，確認：

- [ ] Gazebo Harmonic 已安裝 (`gz sim --version`)
- [ ] Python 依賴已安裝 (`python3 -c "import kconfiglib"`)
- [ ] 環境變數已設定 (`echo $PX4_SIM_MODEL`)
- [ ] PX4 已編譯 (`ls build/px4_sitl_default/bin/px4`)
- [ ] 網路連接正常（首次安裝時需要）
- [ ] 磁碟空間充足 (`df -h`)
- [ ] 沒有其他 Gazebo/PX4 進程在運行 (`pkill -9 px4; pkill -9 gz`)

---

**更新日期**: 2026-01-20
