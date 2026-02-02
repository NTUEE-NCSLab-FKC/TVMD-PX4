#!/bin/bash
# TVMD SITL 无头模式启动脚本（适用于 VirtualBox 环境）
# 此脚本在后台运行 Gazebo（无 GUI），避免 OpenGL 问题

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否在 TVMD-PX4 目录中
if [ ! -f "package.xml" ] || [ ! -d "Tools/simulation/gz" ]; then
    print_error "请在 TVMD-PX4 根目录运行此脚本"
    exit 1
fi

PX4_DIR=$(pwd)

print_info "==================== TVMD SITL 无头模式启动 ===================="
print_info "此模式适用于 VirtualBox 等不支持 OpenGL 3.3+ 的环境"
print_info "Gazebo 将在后台运行物理仿真，无 GUI 窗口"
echo ""

# 检查 Gazebo 是否安装
if ! command -v gz &> /dev/null; then
    print_error "Gazebo 未安装！请先安装 Gazebo Garden/Harmonic"
    print_info "参考: INSTALL_GZ_TRANSPORT.md"
    exit 1
fi

GZ_VERSION=$(gz sim --version 2>&1 | head -1)
print_info "检测到: $GZ_VERSION"

# 设置环境变量
export GZ_SIM_RESOURCE_PATH=${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds
export PX4_SIM_MODEL=gz_tvmd
export PX4_GZ_WORLD=${PX4_GZ_WORLD:-default}

print_info "环境变量设置:"
print_info "  GZ_SIM_RESOURCE_PATH: $GZ_SIM_RESOURCE_PATH"
print_info "  PX4_SIM_MODEL: $PX4_SIM_MODEL"
print_info "  PX4_GZ_WORLD: $PX4_GZ_WORLD"
echo ""

# 清理旧的 Gazebo 进程
print_info "清理旧的 Gazebo 进程..."
pkill -9 gz 2>/dev/null || true
pkill -9 ruby 2>/dev/null || true
sleep 2

# 查找世界文件
WORLD_FILE="${PX4_DIR}/Tools/simulation/gz/worlds/${PX4_GZ_WORLD}.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    print_warning "世界文件 $WORLD_FILE 不存在，将使用空世界"
    WORLD_FILE=""
fi

# 启动 Gazebo 无头模式
print_info "启动 Gazebo 无头模式（后台运行）..."

if [ -n "$WORLD_FILE" ]; then
    print_info "载入世界: $WORLD_FILE"
    gz sim -s -r -v 4 "$WORLD_FILE" &
else
    print_info "使用空世界"
    gz sim -s -r -v 4 &
fi

GZ_PID=$!
print_info "Gazebo PID: $GZ_PID"

# 等待 Gazebo 启动
print_info "等待 Gazebo 启动..."
sleep 5

# 检查 Gazebo 是否运行
if ! kill -0 $GZ_PID 2>/dev/null; then
    print_error "Gazebo 启动失败"
    exit 1
fi

# 检查 Gazebo topics
print_info "检查 Gazebo 连接..."
if gz topic -l &>/dev/null; then
    print_success "Gazebo 运行正常"
    print_info "可用的 topics:"
    gz topic -l | head -10
else
    print_error "无法连接到 Gazebo"
    kill $GZ_PID 2>/dev/null
    exit 1
fi

echo ""
print_info "==================== 启动 PX4 SITL ===================="

# 构建 PX4（如果需要）
if [ ! -f "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]; then
    print_info "PX4 未构建，开始构建..."
    make px4_sitl_default
fi

# 启动 PX4
print_info "启动 PX4 并连接到 Gazebo..."
cd "${PX4_DIR}"

# 使用 ROMFS 路径启动 PX4
PX4_SIM_MODEL=gz_tvmd "${PX4_DIR}/build/px4_sitl_default/bin/px4" \
    "${PX4_DIR}/ROMFS/px4fmu_common" \
    -s etc/init.d-posix/rcS &

PX4_PID=$!
print_info "PX4 PID: $PX4_PID"

# 等待并监控
sleep 3

if kill -0 $PX4_PID 2>/dev/null; then
    print_success "PX4 运行中"
else
    print_error "PX4 启动失败"
    kill $GZ_PID 2>/dev/null
    exit 1
fi

echo ""
print_success "==================== TVMD SITL 启动成功 ===================="
print_info "Gazebo PID: $GZ_PID (无头模式)"
print_info "PX4 PID: $PX4_PID"
echo ""
print_info "提示:"
print_info "  - Gazebo 在后台运行，无 GUI 窗口"
print_info "  - 可以使用 QGroundControl 连接到 PX4 (UDP 18570)"
print_info "  - 使用 'gz topic -l' 查看 Gazebo topics"
print_info "  - 使用 'gz topic -e -t /world/default/stats' 查看仿真统计"
print_info "  - 按 Ctrl+C 停止"
echo ""

# 清理函数
cleanup() {
    echo ""
    print_info "正在停止 TVMD SITL..."
    kill $PX4_PID 2>/dev/null || true
    kill $GZ_PID 2>/dev/null || true
    sleep 2
    pkill -9 gz 2>/dev/null || true
    pkill -9 px4 2>/dev/null || true
    print_success "已停止"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 保持脚本运行
print_info "TVMD SITL 运行中... (按 Ctrl+C 停止)"
wait $PX4_PID
