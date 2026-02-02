# VirtualBox 环境下的 TVMD SITL 设置指南

本指南专门针对在 **VirtualBox 虚拟机**中运行 TVMD PX4 SITL。

## 🔍 VirtualBox 特有问题

VirtualBox 虚拟机运行 Gazebo 时常见问题：

### 问题 1：OpenGL 3.3 不支持

**症状：**
```
[GUI] [Err] [Ogre2RenderEngine.cc:1140] Unable to create the rendering window:
OGRE EXCEPTION(3:RenderingAPIException): OpenGL 3.3 is not supported.
Please update your graphics card drivers.
```

**原因：** VirtualBox 的虚拟显卡驱动不完全支持 OpenGL 3.3+，而 Gazebo 的 OGRE2 渲染引擎需要 OpenGL 3.3+。

**解决方案：** 使用 Gazebo 无头模式（见下文）

### 问题 2：程序崩溃但 PX4 仍在运行

**症状：**
```
程式記憶體區段錯誤 (位址沒有映射到物件 [0x220])
INFO  [lockstep_scheduler] setting initial absolute time to 8000 us
INFO  [commander] LED: open /dev/led0 failed (22)
```

**原因：** Gazebo GUI 渲染引擎崩溃，但 PX4 进程仍在运行。

**解决方案：** 使用无头模式，避免 GUI 渲染。

---

## 🚀 解决方案 A：无头模式（推荐）

### 什么是无头模式？

无头模式（Headless Mode）让 Gazebo 在后台运行物理仿真，**不显示 GUI 窗口**，从而避免 OpenGL 问题。

- ✅ **优点**：不需要 OpenGL 3.3+，稳定可靠
- ✅ **适用于**：VirtualBox、SSH 远程连接、服务器环境
- ❌ **缺点**：无法实时查看仿真画面

### 使用无头模式启动脚本

我们为您准备了专用脚本：

```bash
cd ~/TVMD-PX4

# 启动 TVMD SITL（无头模式）
./launch_tvmd_headless.sh
```

**预期输出：**
```
[INFO] ==================== TVMD SITL 无头模式启动 ====================
[INFO] 检测到: Gazebo Sim, version 7.9.0
[INFO] 启动 Gazebo 无头模式（后台运行）...
[SUCCESS] Gazebo 运行正常
[INFO] 启动 PX4 并连接到 Gazebo...
[SUCCESS] PX4 运行中
[SUCCESS] ==================== TVMD SITL 启动成功 ====================
[INFO] Gazebo PID: 12345 (无头模式)
[INFO] PX4 PID: 12346
[INFO] TVMD SITL 运行中... (按 Ctrl+C 停止)
```

### 手动启动无头模式

如果您想手动控制：

**终端 1：启动 Gazebo 无头模式**
```bash
# -s = server only (无 GUI)
# -r = run simulation
# -v 4 = verbose level 4
gz sim -s -r -v 4
```

**终端 2：启动 PX4**
```bash
cd ~/TVMD-PX4
PX4_SIM_MODEL=gz_tvmd ./build/px4_sitl_default/bin/px4 ./ROMFS/px4fmu_common -s etc/init.d-posix/rcS
```

### 监控无头模式仿真

虽然没有 GUI，但您可以通过命令行监控：

```bash
# 查看所有 Gazebo topics
gz topic -l

# 实时查看仿真统计（帧率、时间等）
gz topic -e -t /world/default/stats

# 查看 TVMD 模型状态
gz topic -e -t /model/tvmd/pose

# 查看电机速度
gz topic -e -t /model/tvmd/joint/module1_actuator_prop1_joint/cmd_vel
```

---

## 🚀 解决方案 B：启用 VirtualBox 3D 加速（可能有效）

如果您**确实需要 GUI**，可以尝试启用 VirtualBox 3D 加速：

### 步骤 1：在 VirtualBox 中启用 3D 加速

1. 关闭虚拟机
2. 打开虚拟机设置
3. 进入 **显示** → **屏幕**
4. 勾选 **✅ 启用 3D 加速**
5. 显存设置为最大（建议 128MB+）
6. **图形控制器** 选择 **VMSVGA** 或 **VBoxVGA**
7. 保存并重新启动虚拟机

### 步骤 2：安装 VirtualBox Guest Additions

```bash
# 在虚拟机中
sudo apt-get update
sudo apt-get install -y virtualbox-guest-dkms virtualbox-guest-utils virtualbox-guest-x11

# 重启虚拟机
sudo reboot
```

### 步骤 3：验证 OpenGL 支持

```bash
# 安装 glxinfo
sudo apt-get install mesa-utils

# 检查 OpenGL 版本
glxinfo | grep "OpenGL version"
# 期望：OpenGL version string: 3.3 或更高

# 检查直接渲染
glxinfo | grep "direct rendering"
# 期望：direct rendering: Yes
```

### 步骤 4：尝试启动 Gazebo GUI

```bash
# 尝试启动 Gazebo 带 GUI
gz sim -v 4

# 如果成功，使用完整脚本
./launch_tvmd_sitl.sh
```

**注意：** 即使启用了 3D 加速，VirtualBox 的 OpenGL 支持仍可能不足。如果仍然崩溃，请使用无头模式。

---

## 🚀 解决方案 C：使用软件渲染（较慢但兼容）

强制 Gazebo 使用软件渲染（不依赖 GPU）：

```bash
# 设置环境变量
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# 启动 Gazebo
gz sim -v 4

# 或添加到启动脚本
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
echo 'export GALLIUM_DRIVER=llvmpipe' >> ~/.bashrc
source ~/.bashrc
```

**警告：** 软件渲染性能非常慢，仿真帧率可能很低。

---

## 🚀 解决方案 D：使用 Docker（隔离环境）

如果以上方法都不行，使用 Docker 容器：

```bash
# 安装 Docker
sudo apt-get install docker.io
sudo usermod -aG docker $USER
newgrp docker

# 使用 PX4 官方镜像
docker pull px4io/px4-dev-simulation-focal

# 运行容器（无头模式）
docker run -it --rm \
    -v ~/TVMD-PX4:/workspace/TVMD-PX4 \
    --network host \
    px4io/px4-dev-simulation-focal \
    bash

# 在容器内
cd /workspace/TVMD-PX4
make px4_sitl gz_tvmd
```

---

## 🎯 推荐方案对比

| 方案 | 难度 | 稳定性 | GUI | 性能 | 推荐度 |
|------|------|--------|-----|------|--------|
| **A. 无头模式** | ⭐ 简单 | ⭐⭐⭐⭐⭐ 极高 | ❌ 无 | ⭐⭐⭐⭐⭐ 最快 | **⭐⭐⭐⭐⭐ 强烈推荐** |
| B. 启用 3D 加速 | ⭐⭐ 中等 | ⭐⭐ 不稳定 | ✅ 有 | ⭐⭐⭐ 中等 | ⭐⭐ 可能失败 |
| C. 软件渲染 | ⭐ 简单 | ⭐⭐⭐ 较高 | ✅ 有 | ⭐ 很慢 | ⭐ 不推荐 |
| D. Docker | ⭐⭐⭐ 复杂 | ⭐⭐⭐⭐ 高 | ❌ 无 | ⭐⭐⭐⭐ 快 | ⭐⭐⭐ 备选 |

**VirtualBox 环境下，我们强烈推荐使用方案 A（无头模式）**。

---

## 🔧 使用 QGroundControl 监控

无头模式虽然没有 Gazebo GUI，但您可以使用 **QGroundControl** 监控和控制飞行器：

### 安装 QGroundControl（在 Windows 主机上）

1. 下载：https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html
2. 安装并运行 QGroundControl
3. PX4 会自动通过 UDP 18570 广播

### 连接到 PX4

QGroundControl 应该会自动检测到 PX4 SITL。如果没有：

1. 点击顶部 **Q** 图标
2. 进入 **Application Settings** → **Comm Links**
3. 添加连接：
   - Type: UDP
   - Listening Port: 18570
   - Server Address: 127.0.0.1 (或虚拟机 IP)
4. 连接

### 在 QGroundControl 中控制 TVMD

- 查看飞行器状态
- 切换飞行模式
- 执行起飞/降落
- 查看遥测数据
- 规划任务

**注意：** QGroundControl 提供的可视化可以部分替代 Gazebo GUI。

---

## 📋 完整启动流程（无头模式）

### 一键启动

```bash
cd ~/TVMD-PX4
./launch_tvmd_headless.sh
```

### 手动启动

```bash
# 步骤 1：设置环境
export GZ_SIM_RESOURCE_PATH=$HOME/TVMD-PX4/Tools/simulation/gz/models:$HOME/TVMD-PX4/Tools/simulation/gz/worlds

# 步骤 2：启动 Gazebo 无头模式（后台）
gz sim -s -r -v 4 &

# 步骤 3：等待 Gazebo 启动
sleep 5

# 步骤 4：验证 Gazebo 运行
gz topic -l

# 步骤 5：启动 PX4
cd ~/TVMD-PX4
PX4_SIM_MODEL=gz_tvmd ./build/px4_sitl_default/bin/px4 ./ROMFS/px4fmu_common -s etc/init.d-posix/rcS
```

### 验证成功

**Gazebo：**
```bash
gz topic -e -t /world/default/stats
# 应该看到实时更新的仿真统计
```

**PX4：**
```
INFO  [gz_bridge] connected to Gazebo instance
INFO  [commander] Ready for takeoff!
```

---

## 🆘 故障排除

### 问题 1：gz 命令找不到

**症状：**
```
bash: gz: command not found
```

**解决：**
```bash
# Gazebo 未安装
gz sim --version

# 如果未安装，参考 INSTALL_GZ_TRANSPORT.md
```

### 问题 2：Service call timed out

**症状：**
```
ERROR [gz_bridge] Service call timed out
```

**解决：**
```bash
# 检查 Gazebo 是否运行
ps aux | grep "gz sim"

# 检查 Gazebo topics
gz topic -l

# 如果没有输出，重启 Gazebo
pkill -9 gz
gz sim -s -r &
```

### 问题 3：PX4 找不到模型

**症状：**
```
ERROR [init] Unknown model gz_tvmd
```

**解决：**
```bash
# 检查 airframe 文件
ls ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_tvmd

# 确保使用正确的环境变量
PX4_SIM_MODEL=gz_tvmd  # 注意是 gz_tvmd 不是 tvmd
```

### 问题 4：仿真运行但没有响应

**解决：**
```bash
# 检查仿真时间是否前进
gz topic -e -t /world/default/clock

# 如果时间不变，Gazebo 可能暂停了
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: false'
```

---

## 📚 相关文档

- **launch_tvmd_headless.sh** - 无头模式一键启动脚本
- **WSL_SETUP_GUIDE.md** - WSL 环境设置
- **INSTALL_GZ_TRANSPORT.md** - gz-transport 安装指南
- **TVMD_SITL_GUIDE.md** - TVMD SITL 完整指南
- **TROUBLESHOOTING.md** - 常见问题排除

---

## 🎯 最佳实践建议

对于 **VirtualBox 环境**：

1. ✅ **使用无头模式**（`launch_tvmd_headless.sh`）
2. ✅ **在 Windows 主机运行 QGroundControl** 监控飞行器
3. ✅ **分配足够的资源**：至少 4GB RAM、2 CPU 核心
4. ✅ **使用命令行工具** 监控仿真状态
5. ❌ **不要依赖 Gazebo GUI**（VirtualBox OpenGL 支持不足）

---

## 📊 系统要求

### 最低配置（无头模式）

- **内存**：4GB RAM
- **CPU**：2 核心
- **硬盘**：20GB 可用空间
- **网络**：用于下载依赖

### 推荐配置（无头模式）

- **内存**：8GB RAM
- **CPU**：4 核心
- **硬盘**：40GB 可用空间

**注意：** 使用无头模式时，不需要强大的 GPU。

---

## 🔗 外部资源

- [Gazebo Headless 文档](https://gazebosim.org/docs/garden/headless_rendering)
- [PX4 SITL 文档](https://docs.px4.io/main/en/simulation/)
- [QGroundControl 下载](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
- [VirtualBox 3D 加速](https://www.virtualbox.org/manual/ch04.html#guestadd-3d)

---

**总结：在 VirtualBox 中运行 TVMD SITL，使用 `./launch_tvmd_headless.sh` 是最简单、最可靠的方法！**
