# TVMD SITL WSL 設置指南

本指南專門針對 **Windows 10/11 WSL** 環境設置 TVMD PX4 SITL。

## 🔍 當前狀態分析

您目前的環境：
- ✅ Ubuntu 24.04.3 LTS (WSL)
- ✅ Gazebo Classic 11.10.2 已安裝並運行
- ✅ PX4 SITL 基本功能正常
- ❌ **Gazebo Ignition/Harmonic 未安裝** ← TVMD 需要這個！
- ❌ 網路 DNS/防火牆問題阻擋安裝

## ⚠️ 重要：為什麼需要 Gazebo Ignition？

| 模擬器 | TVMD 支援 | 命令 | 狀態 |
|--------|----------|------|------|
| **Gazebo Classic** | ❌ 只有 iris 等預設模型 | `make px4_sitl gazebo-classic` | 您目前使用的 |
| **Gazebo Ignition/Harmonic** | ✅ 完整 TVMD 支援 | `make px4_sitl gz` | 需要安裝 |

**TVMD 模型位置**：
- Gazebo Classic 模型：`Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/` （**無 tvmd**）
- Gazebo Ignition 模型：`Tools/simulation/gz/models/tvmd/` （**有 tvmd**）

---

## 🚀 解決方案

### 方案 A：修復 WSL 網路並安裝 Gazebo Ignition（推薦）

#### 步驟 1：修復 WSL DNS 問題

在 **Windows PowerShell (管理員)** 中執行：

```powershell
# 停止 WSL
wsl --shutdown

# 創建或編輯 .wslconfig
notepad "$env:USERPROFILE\.wslconfig"
```

在 `.wslconfig` 中添加：

```ini
[wsl2]
# 禁用自動生成 DNS
generateResolvConf = false

# 記憶體限制（可選）
memory=8GB
processors=4
```

保存後，重新啟動 WSL：

```powershell
wsl
```

#### 步驟 2：在 WSL 中手動配置 DNS

```bash
# 刪除自動生成的 resolv.conf
sudo rm -f /etc/resolv.conf

# 創建新的 DNS 配置
sudo bash -c 'cat > /etc/resolv.conf << EOF
nameserver 8.8.8.8
nameserver 8.8.4.4
nameserver 1.1.1.1
EOF'

# 鎖定文件防止被覆蓋
sudo chattr +i /etc/resolv.conf 2>/dev/null || true

# 測試 DNS
nslookup google.com
```

#### 步驟 3：如果公司/學校有代理，配置代理

```bash
# 在 ~/.bashrc 中添加（如果需要）
export http_proxy="http://proxy.example.com:8080"
export https_proxy="http://proxy.example.com:8080"
export no_proxy="localhost,127.0.0.1"

# 重新載入
source ~/.bashrc

# 為 apt 配置代理
sudo bash -c 'cat > /etc/apt/apt.conf.d/proxy.conf << EOF
Acquire::http::Proxy "http://proxy.example.com:8080";
Acquire::https::Proxy "http://proxy.example.com:8080";
EOF'
```

#### 步驟 4：安裝 Gazebo Harmonic

```bash
cd ~/TVMD-PX4

# 執行安裝腳本
sudo ./install_gazebo.sh

# 驗證安裝
gz sim --version
# 應該顯示：Gazebo Sim, version 8.x.x 或更高
```

---

### 方案 B：使用預編譯的 Gazebo 二進制文件（網路受限時）

如果網路問題無法解決，可以在 **Windows** 上下載，然後複製到 WSL：

#### 1. 在 Windows 下載 Gazebo

訪問（使用瀏覽器）：
- https://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/i/gz-harmonic/

下載所需的 `.deb` 文件到 Windows（例如 `C:\Downloads\gazebo\`）

#### 2. 從 WSL 訪問 Windows 文件

```bash
# Windows C: 盤在 WSL 中對應 /mnt/c/
cd /mnt/c/Users/<你的用戶名>/Downloads/gazebo/

# 安裝下載的 .deb 包
sudo dpkg -i *.deb

# 修復依賴關係
sudo apt-get install -f
```

---

### 方案 C：臨時使用 Docker（完全離線）

如果以上方法都不行，可以使用預配置的 Docker 容器：

```bash
# 拉取 PX4 官方 Docker 鏡像（包含 Gazebo）
docker pull px4io/px4-dev-simulation-focal

# 運行容器
docker run -it --rm \
    -v ~/TVMD-PX4:/workspace/TVMD-PX4 \
    px4io/px4-dev-simulation-focal \
    bash

# 在容器內
cd /workspace/TVMD-PX4
PX4_SIM_MODEL=tvmd make px4_sitl gz
```

---

## 🖥️ WSL 圖形界面設置

您看到的 X Window 錯誤是因為 WSL 沒有圖形界面。有三種解決方案：

### 選項 1：使用 WSLg（Windows 11 或 Windows 10 最新版本）

Windows 11 和 Windows 10 (Build 19044+) 已內建 WSLg：

```bash
# 直接運行，應該會自動顯示
PX4_SIM_MODEL=tvmd make px4_sitl gz
```

如果沒有顯示，更新 WSL：

```powershell
# 在 PowerShell (管理員) 中
wsl --update
wsl --shutdown
wsl
```

### 選項 2：使用 VcXsrv（Windows 10 舊版本）

1. 下載並安裝 [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. 啟動 XLaunch：
   - Display settings: Multiple windows
   - Start no client
   - ✅ **Disable access control** ← 重要！
3. 在 WSL 中設置 DISPLAY：

```bash
# 添加到 ~/.bashrc
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0

# 重新載入
source ~/.bashrc

# 測試
xeyes  # 應該會顯示一對眼睛
```

### 選項 3：無頭模式（Headless）

如果不需要圖形界面，只用命令行：

```bash
# 設置無頭模式
export HEADLESS=1
export LIBGL_ALWAYS_SOFTWARE=1

# 啟動 SITL（無 GUI）
PX4_SIM_MODEL=tvmd make px4_sitl gz_tvmd_headless
```

---

## 📋 完整安裝檢查清單

完成以下步驟後，TVMD SITL 應該可以正常運行：

- [ ] **網路修復**
  - [ ] 配置 WSL DNS（.wslconfig + resolv.conf）
  - [ ] 如有需要配置代理
  - [ ] 測試：`wget -q --spider http://google.com && echo "OK"`

- [ ] **Gazebo Ignition 安裝**
  - [ ] 執行 `sudo ./install_gazebo.sh`
  - [ ] 驗證：`gz sim --version` 顯示版本號

- [ ] **Python 依賴**
  - [ ] `pip3 install --user kconfiglib jsonschema jinja2 pyros-genmsg packaging toml numpy empy pyyaml`

- [ ] **圖形界面**
  - [ ] WSLg 已啟用 或
  - [ ] VcXsrv 已安裝並運行 或
  - [ ] 設置無頭模式

- [ ] **啟動 TVMD SITL**
  - [ ] 使用正確命令：`PX4_SIM_MODEL=tvmd make px4_sitl gz`
  - [ ] 或使用腳本：`./launch_tvmd_sitl.sh`

---

## 🔧 常見 WSL 問題

### 問題 1：DNS 在重啟後失效

**原因**：WSL 自動覆蓋 `/etc/resolv.conf`

**解決**：
```bash
# 鎖定文件
sudo chattr +i /etc/resolv.conf

# 或在 Windows .wslconfig 中設置 generateResolvConf=false
```

### 問題 2："Temporary failure resolving"

**原因**：防火牆或代理阻擋

**解決**：
1. 檢查 Windows 防火牆設置
2. 暫時關閉 VPN
3. 配置代理（見上文）
4. 使用方案 B（離線安裝）

### 問題 3："Can't open display" 或 "Qt platform plugin"

**原因**：沒有 X server

**解決**：
1. Windows 11：確保 WSLg 已更新
2. Windows 10：安裝並運行 VcXsrv
3. 設置 `export DISPLAY=:0`

### 問題 4：Gazebo 卡住或崩潰

**原因**：WSL 記憶體不足

**解決**：
```powershell
# 在 .wslconfig 中增加記憶體
[wsl2]
memory=8GB
swap=4GB
```

---

## 🎯 快速啟動（網路正常後）

```bash
# 1. 確保在正確目錄
cd ~/TVMD-PX4

# 2. 啟動 TVMD SITL
PX4_SIM_MODEL=tvmd make px4_sitl gz

# 預期輸出：
# - Gazebo Ignition/Harmonic 啟動
# - 載入 TVMD 模型（4 個馬達 + 8 個伺服馬達）
# - PX4 顯示 "Ready for takeoff!"
```

---

## 📊 命令對照表

| 目的 | ❌ 錯誤命令 | ✅ 正確命令 |
|------|-----------|-----------|
| 啟動 TVMD | `make px4_sitl gazebo` | `PX4_SIM_MODEL=tvmd make px4_sitl gz` |
| 啟動 TVMD | `make px4_sitl gz_tvmd` | `PX4_SIM_MODEL=tvmd make px4_sitl gz` |
| 啟動 TVMD | `make px4_sitl gazebo-classic` | `PX4_SIM_MODEL=tvmd make px4_sitl gz` |
| 檢查 Gazebo | `gazebo --version` | `gz sim --version` |
| 無頭模式 | （不適用） | `HEADLESS=1 make px4_sitl gz` |

---

## 🆘 需要幫助？

詳細故障排除請參考：
- `TROUBLESHOOTING.md`：一般性問題
- `TVMD_SITL_GUIDE.md`：TVMD SITL 完整指南
- [PX4 WSL 文檔](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl.html)

---

## 📝 摘要：您目前需要做的

1. **修復網路** → 參考「方案 A - 步驟 1-2」
2. **安裝 Gazebo Ignition** → `sudo ./install_gazebo.sh`
3. **設置圖形界面** → WSLg 或 VcXsrv
4. **使用正確命令** → `PX4_SIM_MODEL=tvmd make px4_sitl gz` （注意是 `gz` 不是 `gazebo`）

目前您運行的 `gazebo-classic` 不支援 TVMD 模型，這就是為什麼它使用了 iris 而不是 tvmd！
