#!/bin/bash
###############################################################################
# TVMD SITL Launch Script
#
# 此腳本簡化 TVMD 的 PX4 SITL 啟動流程
#
# 使用方法:
#   ./launch_tvmd_sitl.sh [world_name]
#
# 參數:
#   world_name - Gazebo 世界檔案名稱 (預設: default)
#
# 範例:
#   ./launch_tvmd_sitl.sh          # 使用預設世界
#   ./launch_tvmd_sitl.sh empty    # 使用空世界
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

# 取得參數
WORLD_NAME="${1:-default}"

# 檢查是否在 PX4 目錄
if [ ! -f "CMakeLists.txt" ] || [ ! -d "ROMFS" ]; then
    print_error "請在 PX4 根目錄執行此腳本"
    exit 1
fi

print_info "==================== TVMD SITL 啟動腳本 ===================="
print_info "Gazebo 世界: $WORLD_NAME"
print_info "模型: tvmd"
print_info "Airframe: 4007_gz_tvmd"
print_info "============================================================"

# 設定環境變數
export PX4_SIM_MODEL=tvmd
export PX4_GZ_WORLD=$WORLD_NAME

# 檢查依賴
print_info "檢查 Python 依賴..."
if ! python3 -c "import kconfiglib" 2>/dev/null; then
    print_error "kconfiglib 未安裝"
    echo ""
    echo "請執行以下命令安裝依賴："
    echo "  pip3 install --user kconfiglib jsonschema jinja2 pyros-genmsg packaging toml numpy empy pyyaml"
    echo ""
    echo "或使用 PX4 官方腳本："
    echo "  bash ./Tools/setup/ubuntu.sh"
    exit 1
fi

print_success "依賴檢查通過"

# 清理舊的 build（可選）
# print_info "清理舊的 build 目錄..."
# rm -rf build/px4_sitl_default

# 啟動 SITL
print_info "啟動 PX4 SITL..."
print_info "執行命令: make px4_sitl gz"
echo ""

make px4_sitl gz

print_success "SITL 已啟動"
