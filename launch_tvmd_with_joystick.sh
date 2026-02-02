#!/bin/bash
# TVMD SITL with Joystick/Gamepad support

set -e

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

print_info "==================== TVMD SITL with Joystick ===================="

# 检查 joystick 设备
if [ ! -e /dev/input/js0 ]; then
    print_error "未找到 joystick 设备 /dev/input/js0"
    print_info "请确保："
    print_info "  1. 游戏手柄已连接到 USB"
    print_info "  2. VirtualBox 已将 USB 设备传递给虚拟机"
    print_info "  3. 运行 'lsusb' 查看 USB 设备"
    exit 1
fi

print_success "找到 joystick 设备: /dev/input/js0"

# 显示 joystick 信息
print_info "Joystick 信息："
jstest --event /dev/input/js0 &
JSTEST_PID=$!
sleep 2
kill $JSTEST_PID 2>/dev/null || true

print_info ""
print_info "启动 TVMD SITL（无头模式 + Joystick）..."
print_info ""

# 设置环境变量
export PX4_SIM_MODEL=gz_tvmd
export HEADLESS=1

# 启动 PX4 with joystick support
cd $(dirname $0)

# 创建临时启动脚本
cat > /tmp/px4_joystick_rcS << 'EOF'
#!/bin/sh

# 标准 TVMD 初始化
. ${R}etc/init.d-posix/rcS

# 启动 joystick 驱动
if [ -e /dev/input/js0 ]
then
    # Linux joystick driver
    linux_joystick start /dev/input/js0

    # 配置 joystick 到 RC 的映射
    param set COM_RC_IN_MODE 1
    param set RC_MAP_THROTTLE 1
    param set RC_MAP_ROLL 2
    param set RC_MAP_PITCH 3
    param set RC_MAP_YAW 4
    param set RC_MAP_MODE_SW 5
    param set RC_MAP_ARM_SW 6

    echo "Joystick enabled on /dev/input/js0"
else
    echo "No joystick found"
fi
EOF

chmod +x /tmp/px4_joystick_rcS

print_info "使用自定义启动脚本（包含 joystick 支持）"
print_info ""

# 启动 Gazebo 无头模式
print_info "启动 Gazebo..."
export GZ_SIM_RESOURCE_PATH=${PWD}/Tools/simulation/gz/models:${PWD}/Tools/simulation/gz/worlds
gz sim -s -r -v 4 &
GZ_PID=$!
sleep 5

# 启动 PX4
print_info "启动 PX4 with joystick support..."
PX4_SIM_MODEL=gz_tvmd ./build/px4_sitl_default/bin/px4 \
    ./ROMFS/px4fmu_common \
    -s /tmp/px4_joystick_rcS &

PX4_PID=$!

print_success "==================== 启动成功 ===================="
print_info "Gazebo PID: $GZ_PID"
print_info "PX4 PID: $PX4_PID"
print_info ""
print_info "提示："
print_info "  - 使用游戏手柄控制 TVMD"
print_info "  - 左摇杆：油门 + 偏航"
print_info "  - 右摇杆：俯仰 + 横滚"
print_info "  - 按键配置："
print_info "    * 按钮 5/6: 模式切换"
print_info "    * 按钮 7/8: Arm/Disarm"
print_info "  - 按 Ctrl+C 停止"
print_info ""

cleanup() {
    print_info "停止 TVMD SITL..."
    kill $PX4_PID $GZ_PID 2>/dev/null || true
    sleep 2
    pkill -9 gz px4 2>/dev/null || true
    print_success "已停止"
    exit 0
}

trap cleanup SIGINT SIGTERM

wait $PX4_PID
