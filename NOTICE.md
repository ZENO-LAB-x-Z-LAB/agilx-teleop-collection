# 来源与许可

本仓库从实验主机的 `nero-pure-teleop` 工作目录提取。原项目的基线提交为 `52dbb8cba9dba24b027709fc4719ae0aa39afadd`。提取时保留了工作目录中已经用于采集的机械臂控制节点、键盘录制器、相机与触觉启动脚本、TUI 的未提交改动，以及触觉预览 RViz 配置。

本次整理把 CAN 接口、相机选择、数据目录和本机设置从工作站环境中独立出来，增加了构建入口、复用说明和不接硬件的录制回读测试。压缩包只包含这个仓库的跟踪文件。

| 组件 | 已有许可信息 |
| --- | --- |
| `pyAgxArm/` | 随包 SDK 子集的 `setup.py` 声明 LGPL-3.0-only。许可证及其引用的 GPL 全文在仓库内。[AgileX 上游](https://github.com/agilexrobotics/pyAgxArm) · [LGPL](LICENSES/pyAgxArm-LGPL-3.0.txt) · [GPL](LICENSES/GPL-3.0.txt) |
| `agx_arm_urdf/` 与 `agx_arm_ros/src/` | 保留本机 ROS 包各自的 `package.xml` 许可声明（Apache-2.0）。AgileX 的 URDF 和 ROS 上游仓库另附 MIT 声明，随包保留对应文本。[Apache-2.0](LICENSES/Apache-2.0.txt) · [上游 MIT](LICENSES/AgileX-MIT.txt) · [URDF 来源](https://github.com/agilexrobotics/agx_arm_urdf) · [ROS 来源](https://github.com/agilexrobotics/agx_arm_ros) |
| 项目顶层的遥操、录制和导出脚本 | 原项目未随这些脚本提供统一的开源许可。对外开源发布这些文件的授权范围由其权利人确认。 |
