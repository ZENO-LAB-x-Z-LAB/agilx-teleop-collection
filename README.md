# NERO 遥操作示范采集

用主臂示教、从臂执行，把关节实测值、发给从臂的动作指令和所选相机图像录到同一份 ROS 2 MCAP。每段示范单独保存，训练用数据可以从 MCAP 离线导出。仓库包含 NERO 与夹爪的 CAN 驱动、机器人模型、单臂和双臂启动脚本、USB 相机选择器、可选触觉采集，以及键盘录制器。

**第一次使用可以先跑无硬件自测。已有遥操系统在发布 ROS 2 话题的用户，可以直接跳到“录一段示范”。**

## 安装与无硬件自测

交付的源码包可直接解压使用。收到离线 Git 仓库（`.bundle`）时，运行 `git clone <bundle文件路径> nero-teleop-collection`，再进入 `nero-teleop-collection` 目录。

验证环境是 Ubuntu 24.04、ROS 2 Jazzy、系统 Python 3.12。先安装 ROS 2 Jazzy Desktop，下面的命令在仓库根目录运行。采集与回读使用系统 Python，LeRobot 是导出时才需要的依赖。[ROS 2 Jazzy 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-numpy python3-typing-extensions python3-can python3-serial \
  python3-yaml python3-tk python3-pil.imagetk v4l-utils can-utils iproute2 \
  ros-jazzy-ros2bag ros-jazzy-rosbag2-storage-mcap

# 仅在这台机器首次配置 rosdep 时执行 sudo rosdep init
rosdep update --rosdistro jazzy
./scripts/build_ros2.sh
source scripts/env/pure_teleop_env.sh
/usr/bin/python3 tests/smoke_record.py
```

构建脚本使用 rosdep 安装包清单中的依赖（含 Pinocchio、xacro），再构建 5 个包。依赖已经齐备的离线机器可以运行 `./scripts/build_ros2.sh --build-only`。自测在本机独立话题下发布已知关节值、动作和 JPEG，通过正式录制入口模拟按键保存两个 episode，再逐条解码校验。成功后打印两行 `state=… action=… jpeg=…`。测试数据写在临时目录。

## 配置自己的设备

```bash
cp config/hardware.env.example config/local.env
ip -br link show type can
ip -details link show type can
/usr/bin/python3 scripts/show_connected_cameras.py --list --no-preview
```

编辑 `config/local.env`。各启动入口会读取它，指定其他文件可用 `NERO_CONFIG=/绝对路径/hardware.env ./pure-teleop-tui`。本机配置不进入 Git。[配置模板](config/hardware.env.example)

| 采集设备或设置 | 需要配置的内容 |
| --- | --- |
| 单侧主从臂 | 一侧需要两路可区分的 SocketCAN 接口。例如做右臂示教，把右主臂和右从臂对应的接口分别填入 `NERO_RIGHT_LEADER_CAN` 和 `NERO_RIGHT_FOLLOWER_CAN`。 |
| 双臂 | 配置左右共四个接口。模板中 `can_ml`、`can_sl`、`can_mr`、`can_sr` 是示例名称。驱动预设使用 1 Mbit/s，激活前按接线核对角色、接口状态和比特率。 |
| USB 相机 | 首次录制时在预览中依次选择左腕、右腕、顶部，或当前模式提示的角色。默认按序列号保存映射。相同或缺失序列号的设备可设 `CAMERA_ROLE_IDENTITY=usb-port` 并使用固定 USB 端口。更换设备时可用 `--force-camera-select` 重新选择。 |
| 数据目录与 ROS 域 | `PURE_TELEOP_ROSBAG_ROOT` 指向示范保存目录。主臂、从臂、相机和录制器使用相同的 `ROS_DOMAIN_ID`。 |
| 夹爪触觉（可选） | TUI 可以预览串口并选择传感器。脚本模式把触觉模板复制为同目录的 `tactile_sensors.local.yaml`，为目标填入实际端口，启用对应条目并设 `publish_raw_points: true`。串口用户需有设备读写权限。[触觉模板](agx_arm_ros/src/piper_nero_teleop/config/tactile_sensors.example.yaml) |

如已确认映射且对应 CAN 接口处于 DOWN，可逐个启用。下面以已配好的右从臂接口为例。

```bash
source scripts/env/pure_teleop_env.sh
sudo ip link set dev "$NERO_RIGHT_FOLLOWER_CAN" up type can bitrate 1000000
ip -details link show dev "$NERO_RIGHT_FOLLOWER_CAN"
```

## 录一段示范

在仓库根目录的三个终端分别运行 `./pure-teleop-tui`。前两个终端运行硬件控制，最后一个负责记录。激活前核对屏幕上显示的角色、左右侧和 CAN 端口，确认现场可使能机械臂。默认使用 NERO `v112` 固件控制预设。

| 终端 | 菜单操作 |
| --- | --- |
| 1 · 从臂 | 选 **5 → 1（follower）**，再选左侧、右侧或双臂。确认命令并保持运行。 |
| 2 · 主臂 | 选 **5 → 2（leader）**，选择同样的范围，确认并保持运行。 |
| 3 · 录制 | 选 **1**。子菜单 **1** 录双臂和选中的相机，**2** 录右臂、右腕和顶部，**6** 录右臂和右腕，**3–5** 提供右臂触觉模式。新建或继续一个 session，按提示选择相机。 |

在录制终端按一次 **空格**。看到 `[record] START episode_001` 后开始示教。完成动作后再按 **空格**，看到 `metadata=…` 表示已保存。继续按空格可录下一段，按 **Q** 退出录制。已有主从臂 ROS 节点运行时，只需打开第三个终端。

也可以用命令行启动录制：

```bash
# 双臂、夹爪、所选相机
./scripts/record_current_pure_teleop_rosbag2.sh --dual-arm --name demo_01

# 双臂和夹爪，跳过本机相机
./scripts/record_current_pure_teleop_rosbag2.sh --dual-arm --skip-camera --name arms_only

# 右臂、夹爪、右腕和顶部相机，不打开 RViz
./scripts/record_current_pure_teleop_rosbag2.sh --right-wrist-top --no-rviz --name right_demo
```

单臂腕部相机预设会检查指定相机是否有样本。双臂预设允许部分相机缺席。某路画面是训练必需时，把它列为必采话题，例如在双臂命令末尾加 `-- --required-topic /teleop_camera/top/image_raw/compressed`，缺帧时录制器会在开始前提示。已有其他 ROS 相机或传感器可用同样方式加入。开发板通过网络直接送出的 OWAC RAW10 数据需先发布成 ROS 话题才能走这条路径。

## 查看采到了什么

默认输出为 `data/pure_teleop/rosbag2/<session>/episode_001/`。每段里的 `episode/` 含 MCAP 和 ROS bag 元数据，`selected_topics.txt` 列出本段话题，`metadata.json` 保存采集参数、起止时间和录制前检查结果。

```bash
source scripts/env/pure_teleop_env.sh
bag_root="${PURE_TELEOP_ROSBAG_ROOT:-$PWD/data/pure_teleop/rosbag2}"
ros2 bag info "$bag_root/demo_01/episode_001/episode"
```

| 内容 | 典型话题与含义 |
| --- | --- |
| 机械臂观测 | `/slave_right/joint_states_single`，从臂实测关节和夹爪。需要力矩时可加 `--require-joint-effort`，录制前会检查对应每侧的 8 个 effort 值。 |
| 示教动作 | `/slave_right/joint_cmd_mux/output`，实际送往从臂的命令。夹爪状态另存为 `/slave_right/gripper_status`。左侧对应 `slave_left`。 |
| 相机 | `/teleop_camera/{left,right,top}/image_raw/compressed`，每个视角独立保存 JPEG 压缩图像。ROS 消息保留各自时间戳，MCAP 另存接收时间。 |
| 触觉 | 启用后，`/slave_right/tactile/gripper/joint_states` 的 `effort` 是 9 个触点分别按 Fx、Fy、Fz 排列的 27 个数。 |

## 可选：导出训练数据

录制完成后再装导出环境。下面用系统 Python 3.12 创建单独的虚拟环境，使导出器也能读取 Jazzy 的 ROS 消息。双臂导出的 `observation.state` 和 `action` 都按左 8 维、右 8 维排列，每侧为 7 个关节加夹爪。当前导出器处理位置、动作和指定图像，触觉与力矩保存在原始 MCAP 中。

```bash
sudo apt install python3-venv ffmpeg
source scripts/env/pure_teleop_env.sh
/usr/bin/python3 -m venv --system-site-packages .venv-export
.venv-export/bin/python -m pip install -r config/requirements-export.txt

bag_root="${PURE_TELEOP_ROSBAG_ROOT:-$PWD/data/pure_teleop/rosbag2}"
.venv-export/bin/python scripts/export_pure_teleop_mcap_lerobot_dual_arm.py \
  --session-dir "$bag_root/demo_01" --output-root data/lerobot \
  --dataset-id demo_01 --task '用双臂把物体放入盒中' \
  --camera-topic observation.images.left=/teleop_camera/left/image_raw/compressed \
  --camera-topic observation.images.right=/teleop_camera/right/image_raw/compressed \
  --camera-topic observation.images.top=/teleop_camera/top/image_raw/compressed
```

按实际录到的视角填写 `--camera-topic`。右臂单臂可换用 `scripts/export_pure_teleop_mcap_lerobot_v30.py`，默认使用右腕相机，左臂可用 `--state-topic` 和 `--action-topic` 指定对应话题。导出默认以 bag 时间对齐，输出 30 FPS，图像为 256 × 256。已验证的 LeRobot 0.3.3 会在 `meta/info.json` 写入 `codebase_version: v2.1`。实测该版本把各视角编码为 MP4/AV1，LeRobot 和系统 FFmpeg 均可回读。原始 MCAP 中仍是 JPEG。文件名中的 `v30` 是原脚本名称，实际数据版本以该元数据为准。

在 Jazzy 上已验证 5 个包能从源码构建，机器人模型可加载。无硬件测试完成了两段示范录制回读，并检查了双臂预设的 9 路话题（两侧各有状态、命令、夹爪，加三个相机视角）。用已有的真实双臂 MCAP 分别导出并回读了 20 帧的右臂单视角样本和双臂三视角样本。新设备上仍需现场核对主从臂接线、相机视角与夹爪响应。[代码来源与许可](NOTICE.md)
