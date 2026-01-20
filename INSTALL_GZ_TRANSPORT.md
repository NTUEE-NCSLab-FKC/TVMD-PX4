# 安裝 gz-transport 開發庫指南

## 🎯 當前狀態

您已經安裝了：
- ✅ Gazebo Sim 7.9.0 (Gazebo Garden)
- ✅ PX4 已成功編譯
- ❌ **gz-transport 開發庫缺失** ← 這導致 PX4 無法生成 gz_tvmd 目標

## 🔍 問題診斷

```bash
# Gazebo 已安裝
$ gz sim --version
Gazebo Sim, version 7.9.0

# 但 gz-transport 開發庫缺失
$ pkg-config --modversion gz-transport12
Package gz-transport12 was not found

# 導致 PX4 沒有 gz 目標
$ make px4_sitl gz
make[1]: *** 沒有規則可製作目標「gz」
```

---

## 🚀 解決方案

### 方法 A：添加 Gazebo 源並安裝開發庫（推薦）

#### 步驟 1：添加 Gazebo 官方 repository

```bash
# 下載並添加 GPG 密鑰
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# 添加 Gazebo 源（Ubuntu 24.04）
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 更新包列表
sudo apt-get update
```

#### 步驟 2：安裝 gz-transport 開發庫

Gazebo Garden (版本 7.x) 使用 gz-transport12：

```bash
# 安裝 gz-transport12 和相關開發庫
sudo apt-get install -y \
    libgz-transport12-dev \
    libgz-msgs9-dev \
    libgz-sim7-dev \
    libgz-plugin2-dev \
    libgz-math7-dev

# 驗證安裝
pkg-config --modversion gz-transport12
# 應該顯示：12.x.x
```

#### 步驟 3：重新編譯 PX4

```bash
cd ~/TVMD-PX4

# 清理舊的構建（重要！讓 CMake 重新檢測）
make distclean

# 重新編譯
make px4_sitl_default

# 在編譯輸出中查找 gz-transport 檢測信息
# 應該看到類似：
# -- Found gz-transport12
# -- GZ_TRANSPORT_VER: 12
```

#### 步驟 4：驗證 gz_tvmd 目標生成

```bash
# 檢查 gz_tvmd 目標是否存在
grep -c "gz_tvmd" build/px4_sitl_default/build.ninja
# 應該顯示一個非零數字

# 列出所有 gz 相關目標
make list_vmd_targets 2>&1 | grep gz_ | head -10
```

#### 步驟 5：啟動 TVMD SITL

```bash
# 方法 1：直接使用 gz_tvmd 目標
make px4_sitl gz_tvmd

# 方法 2：使用環境變數（兩個命令等價）
PX4_SIM_MODEL=gz_tvmd make px4_sitl
# 或
PX4_SIM_MODEL=tvmd make px4_sitl gz

# 方法 3：使用啟動腳本
./launch_tvmd_sitl.sh
```

---

### 方法 B：如果網絡問題無法解決

如果無法訪問 packages.osrfoundation.org，可以：

#### 選項 1：使用備用鏡像

```bash
# 使用清華大學鏡像（中國用戶）
echo "deb [arch=$(dpkg --print-architecture)] https://mirrors.tuna.tsinghua.edu.cn/osrf/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

# 更新並安裝
sudo apt-get update
sudo apt-get install -y libgz-transport12-dev libgz-msgs9-dev libgz-sim7-dev
```

#### 選項 2：手動下載 .deb 包

在能訪問互聯網的機器上下載以下包，然後複製到您的系統：

從 http://packages.osrfoundation.org/gazebo/ubuntu-stable/pool/main/ 下載：

1. `libgz-transport12_12.x.x_amd64.deb`
2. `libgz-transport12-dev_12.x.x_amd64.deb`
3. `libgz-msgs9_9.x.x_amd64.deb`
4. `libgz-msgs9-dev_9.x.x_amd64.deb`
5. `libgz-sim7-dev_7.x.x_amd64.deb`

然後安裝：

```bash
sudo dpkg -i *.deb
sudo apt-get install -f  # 修復依賴
```

---

## 🔧 故障排除

### 問題 1：CMake 仍然找不到 gz-transport

**症狀**：
```bash
make distclean && make px4_sitl_default
# 沒有顯示 "Found gz-transport12"
```

**解決**：
```bash
# 確認 pkg-config 能找到
pkg-config --modversion gz-transport12
pkg-config --cflags gz-transport12

# 如果找不到，檢查 .pc 文件位置
find /usr -name "gz-transport12.pc" 2>/dev/null

# 設置 PKG_CONFIG_PATH（如果需要）
export PKG_CONFIG_PATH=/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
```

### 問題 2：編譯時出現 gz-transport 相關錯誤

**症狀**：
```
fatal error: gz/transport.hpp: No such file or directory
```

**解決**：
```bash
# 確認頭文件已安裝
ls /usr/include/gz/transport12/

# 如果不存在，重新安裝開發包
sudo apt-get install --reinstall libgz-transport12-dev
```

### 問題 3：make px4_sitl gz 仍然失敗

**症狀**：
```
make: *** 沒有規則可製作目標「gz」
```

**檢查清單**：
1. ✅ gz-transport 已安裝：`pkg-config --modversion gz-transport12`
2. ✅ 已經執行過 `make distclean`
3. ✅ CMake 檢測到 gz-transport：`grep "gz-transport" build/px4_sitl_default/CMakeCache.txt`
4. ✅ gz_bridge 模組已編譯：`ls build/px4_sitl_default/src/modules/simulation/gz_bridge/`

如果以上都正確，查看構建日誌：
```bash
make px4_sitl_default 2>&1 | tee build.log
grep -i "gz-transport\|gazebo\|ignition" build.log
```

---

## 📊 完整檢查清單

執行以下命令驗證所有組件：

```bash
echo "=== Gazebo 安裝檢查 ==="
gz sim --version

echo -e "\n=== gz-transport 檢查 ==="
pkg-config --modversion gz-transport12

echo -e "\n=== 開發庫檢查 ==="
dpkg -l | grep -E "libgz-(transport|msgs|sim).*-dev"

echo -e "\n=== PX4 構建檢查 ==="
[ -d build/px4_sitl_default/src/modules/simulation/gz_bridge ] && echo "✅ gz_bridge 模組已構建" || echo "❌ gz_bridge 模組缺失"

echo -e "\n=== gz_tvmd 目標檢查 ==="
grep -q "gz_tvmd" build/px4_sitl_default/build.ninja && echo "✅ gz_tvmd 目標存在" || echo "❌ gz_tvmd 目標不存在"
```

預期輸出：
```
=== Gazebo 安裝檢查 ===
Gazebo Sim, version 7.9.0

=== gz-transport 檢查 ===
12.x.x

=== 開發庫檢查 ===
ii  libgz-msgs9-dev
ii  libgz-sim7-dev
ii  libgz-transport12-dev

=== PX4 構建檢查 ===
✅ gz_bridge 模組已構建

=== gz_tvmd 目標檢查 ===
✅ gz_tvmd 目標存在
```

---

## 🎯 快速啟動（所有步驟完成後）

```bash
# 一鍵啟動 TVMD SITL
cd ~/TVMD-PX4
make px4_sitl gz_tvmd

# 或使用啟動腳本
./launch_tvmd_sitl.sh

# 預期輸出：
# - Gazebo Garden 啟動
# - 載入 TVMD 模型（4 個馬達 + 8 個伺服馬達）
# - PX4 連接到 Gazebo
# - QGroundControl 可以連接（如果運行）
# - 顯示 "Ready for takeoff!"
```

---

## 📚 相關文檔

- **WSL_SETUP_GUIDE.md** - WSL 環境專用設置
- **TVMD_SITL_GUIDE.md** - TVMD SITL 完整指南
- **TROUBLESHOOTING.md** - 常見問題排除
- **install_gazebo.sh** - 自動安裝腳本（需要網絡）

---

## 🆘 仍然有問題？

執行完整的診斷：

```bash
# 生成診斷報告
cat > /tmp/px4_gz_diagnostic.sh << 'EOF'
#!/bin/bash
echo "========== PX4 Gazebo 診斷報告 =========="
echo "時間: $(date)"
echo ""

echo "=== 系統信息 ==="
lsb_release -a 2>/dev/null
uname -a
echo ""

echo "=== Gazebo 版本 ==="
gz sim --version 2>&1 || echo "Gazebo 未安裝"
echo ""

echo "=== gz-transport 檢測 ==="
pkg-config --modversion gz-transport13 2>/dev/null || \
pkg-config --modversion gz-transport12 2>/dev/null || \
echo "gz-transport 未找到"
echo ""

echo "=== 已安裝的 Gazebo 包 ==="
dpkg -l | grep -E "gz-|ignition-|libgz" | awk '{print $2, $3}'
echo ""

echo "=== PX4 版本 ==="
cd ~/TVMD-PX4
cat package.xml | grep version
git log --oneline -1
echo ""

echo "=== CMake 檢測結果 ==="
grep -i "gz-transport" build/px4_sitl_default/CMakeCache.txt 2>/dev/null || echo "無緩存或未找到"
echo ""

echo "=== 可用的 make 目標 ==="
make help 2>&1 | grep -i "gz\|gazebo" | head -10
echo ""

echo "========== 診斷完成 =========="
EOF

chmod +x /tmp/px4_gz_diagnostic.sh
/tmp/px4_gz_diagnostic.sh
```

將輸出發送給支持團隊或在 PX4 論壇提問。
