#!/bin/bash
###############################################################################
# Gazebo Installation Script for TVMD PX4 SITL
#
# 此腳本為 Ubuntu 24.04 安裝 Gazebo Harmonic (最新版本)
#
# 使用方法:
#   sudo ./install_gazebo.sh
#
# 注意: 需要網路連接
#
###############################################################################

set -e

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# 檢查是否為 root
if [ "$EUID" -ne 0 ]; then
    print_error "請使用 sudo 執行此腳本"
    echo "使用方法: sudo $0"
    exit 1
fi

print_info "==================== Gazebo Harmonic 安裝腳本 ===================="

# 檢查 Ubuntu 版本
. /etc/os-release
print_info "偵測到系統: $ID $VERSION_ID"

if [ "$ID" != "ubuntu" ]; then
    print_error "此腳本僅支援 Ubuntu"
    exit 1
fi

# 安裝必要工具
print_info "安裝必要工具..."
apt-get update
apt-get install -y lsb-release wget gnupg

# 添加 Gazebo repository
print_info "添加 Gazebo 官方 repository..."
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 更新套件列表
print_info "更新套件列表..."
apt-get update

# 安裝 Gazebo Harmonic
print_info "安裝 Gazebo Harmonic..."
print_warning "這可能需要幾分鐘時間..."

apt-get install -y gz-harmonic

# 驗證安裝
print_info "驗證 Gazebo 安裝..."
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version | head -1)
    print_success "Gazebo 安裝成功！"
    print_info "版本: $GZ_VERSION"
else
    print_error "Gazebo 安裝失敗"
    exit 1
fi

# 設定環境變數
print_info "設定環境變數..."

# 添加到 bashrc（如果還沒有）
BASHRC_FILE="/root/.bashrc"
if [ -n "$SUDO_USER" ]; then
    BASHRC_FILE="/home/$SUDO_USER/.bashrc"
fi

if ! grep -q "GZ_SIM_RESOURCE_PATH" "$BASHRC_FILE"; then
    echo "" >> "$BASHRC_FILE"
    echo "# Gazebo Harmonic environment" >> "$BASHRC_FILE"
    echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:/usr/share/gz/gz-sim7/worlds" >> "$BASHRC_FILE"
    print_success "環境變數已添加到 $BASHRC_FILE"
fi

# 安裝額外的 Gazebo 工具
print_info "安裝額外的 Gazebo 工具..."
apt-get install -y \
    gz-tools2 \
    python3-gz-math7 \
    python3-gz-transport12 \
    python3-gz-sim7

print_success "==================== 安裝完成 ===================="
echo ""
print_info "下一步："
echo "  1. 重新載入環境變數:"
echo "     source ~/.bashrc"
echo ""
echo "  2. 測試 Gazebo:"
echo "     gz sim -v 4"
echo ""
echo "  3. 啟動 TVMD SITL:"
echo "     cd /home/user/TVMD-PX4"
echo "     PX4_SIM_MODEL=tvmd make px4_sitl gz"
echo ""
print_warning "注意: 可能需要重新登入或重啟終端才能使環境變數生效"
