# piper_real

基于 ROS 2 的实物抓取框架，使用 AgileX Piper（`arm_type=piper`）、AGX 官方平行夹爪 `agx_gripper` 和安装在末端的 Vidu RGB-D 相机（眼在手上）。2026-11-08 已按用户更正的实际型号从 Piper X 调整为 Piper。

当前阶段主要完成相机与视觉基础链路。机器人控制、规划与执行模块已经分包，但完整实机抓取、手眼标定和精度验收尚未完成。

各测试文件、七个 `.sh` 和五个正式 launch 的用途、默认参数、配置来源、命令示例与结果判定，见 [可执行入口使用手册](docs/entrypoints_reference.md)。

## 目录与职责

| 目录 | 职责 |
| --- | --- |
| `src/agx_ros_control_moveit/` | AGX 机械臂驱动、机器人描述、MoveIt、控制器及 RViz 配置 |
| `src/vidu_camera_wrapper/` | Vidu ROS 2 消息接口与相机采集节点，依赖外部 Vidu SDK |
| `src/my_robot_perception/` | 原始 RGB 颜色分割、深度关联和 ArUco 检测节点 |
| `src/my_robot_interface/` | 上层模块共用的消息与 Action 定义 |
| `src/my_robot_commander/` | MoveIt 运动封装、抓放执行器和碰撞场景管理 |
| `src/my_robot_hardware/` | ros2_control 的官方 AGX 话题适配库，接收真实反馈并下发命令 |
| `src/my_robot_planner/` | 根据感知对象组织分拣任务，调用执行层 Action |
| `src/my_robot_bringup/` | 三个独立分层入口与总入口：控制、运动/行为规划、相机感知 |
| `test/` | 五个按职责划分的 Python 测试入口，`native/` 保留实际 C++ 转换验证源 |
| `docs/` | 方案、集成缺口、实施说明和调试记录 |

主要视觉数据流：

```text
Vidu SDK → vidu_ros2_wrapper_node
  ├─ 原始 RGB → color_detector → 完整颜色标签 ─┐
  ├─ 原始 Depth、内参、外参 ──────────────────┤
  │                                          ↓
  │                                   depth_localizer
  │                                     ├─ 表面代表点 → 同帧 RGB 毫米标注
  │                                     └─ 物体中心 → 任务规划/执行层
  └─ 原始 RGB、RGB 内参 → aruco_detector → 单码位姿 TF
```

颜色分割不依赖深度有效性。定位器将 ToF 测量投影到原始 RGB 区域，再检查深度关联；证据不足时保留二维框并显示 `-- mm`。物体中心输出还需要真实机器人/手眼 TF 和符合配置的目标几何。

## 环境与依赖

2026-11-07 本机环境记录：Ubuntu 22.04.5、ROS 2 Humble、Python 3.10.12、OpenCV 4.5.4、Vidu SDK 2.1。其他组合未在本阶段验证。

### ROS 与系统依赖

以下命令以已安装 ROS 2 Humble 的 Ubuntu 22.04 为前提，在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt install build-essential cmake python3-colcon-common-extensions python3-rosdep \
  python3-pip libopencv-dev libopencv-contrib-dev python3-opencv \
  python3-numpy python3-scipy python3-pytest
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

尚未初始化 rosdep 的机器需先执行一次 `sudo rosdep init`。各 ROS 包的依赖以其 `package.xml` 为准；完整机械臂环境还需 MoveIt、ros2_control 和相应控制器，参见 [AGX 配置说明](src/agx_ros_control_moveit/agx_arm_moveit/README.md)。

Python 依赖及本机版本记录在 [`requirement.txt`](requirement.txt)，按需补装：

```bash
/usr/bin/python3 -m pip install -r requirement.txt
```

该文件只列 Python 依赖，不替代 rosdep 或厂商 SDK 安装。`cv2`、`cv_bridge`、`rclpy` 使用系统/ROS 提供的版本，未在 pip 文件中重复引入 OpenCV 发行包。NumPy/SciPy/pytest 已通过上述 APT 安装且版本匹配时，无需重复安装。

### 外部 SDK

- **Vidu SDK 2.1**：SDK 二进制不在本仓库。安装厂商提供的对应系统安装包，例如在安装包所在目录执行 `sudo apt install ./Vidu_SDK_Api_2_1_amd64.deb`。构建需要找到 `Vidu_SDK_ApiConfig.cmake`；本机位于 `/usr/lib/Vidu_SDK_Api/cmake/`。非标准安装路径可在 colcon 的 `--cmake-args` 后传入 `-DVidu_SDK_Api_DIR=/实际路径/cmake`。
- **pyAgxArm**：机械臂 Python 驱动还依赖该 SDK。当前环境的分发版本为 `1.0.0`，来自本地 `/home/ltt/python_can_agx/pyAgxArm` 源码安装，并非已核实的 PyPI 发行版本。新机器须取得与当前驱动兼容的源码，再使用 `/usr/bin/python3 -m pip install /实际路径/pyAgxArm`；`requirement.txt` 不硬编码本机路径，也不代替此步骤。仅做视觉测试时不需要启动该驱动。

## 构建

仅构建当前视觉链路及其工作区依赖：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to my_robot_perception --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

需要构建完整工作区时使用 `colcon build`。当前验证覆盖视觉基础链路及机器人底层配置/mock 执行，不代表完整实机链路已验收。

## 分层启动

眼在手上标定使用独立入口：`bash script/handeye_calibration.sh`。脚本配置位于
[`config/handeye_calibration/`](config/handeye_calibration/)，样本和结果默认保存到工作区
`calibration/handeye/`；按同名结果是否存在自动选择采集或发布。
首次构建、实物操作、服务命令和结果验证见[手眼标定操作说明](docs/handeye_calibration.md)。
修改配置后重启即可，无需重新构建；脚本支持 `--dry-run` 和 `--recalibrate`。

构建并加载完整启动栈：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to my_robot_bringup --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

三个入口均在 `my_robot_bringup`，分三个已加载环境的终端按顺序启动：

```bash
# 1. 底层：真实驱动、ros2_control、MoveIt、robot_state_publisher、RViz
ros2 launch my_robot_bringup robot_control_moveit.launch.py control_mode:=real can_port:=can0

# 2. 中层：抓放执行器、碰撞环境、任务规划器
ros2 launch my_robot_bringup robot_motion_planning.launch.py control_mode:=real

# 3. 上层：Vidu 相机与视觉处理
ros2 launch my_robot_bringup robot_perception.launch.py perception_mode:=color show_window:=true
```

也可用总入口启动同一组合：

```bash
ros2 launch my_robot_bringup robot.launch.py control_mode:=real can_port:=can0 \
  perception_mode:=color show_window:=true
```

当前工作区底层和总入口的 `control_mode` 默认 `real`、`auto_enable` 默认 `true`，会连接并尝试使能真机；这是现有启动配置，本轮未更改。上层的 `start_camera` 和 `publish_handeye_tf` 默认均为 `true`。离线验证必须显式指定 mock、关闭相机及保存的手眼 TF 发布，例如：

```bash
ros2 launch my_robot_bringup robot.launch.py control_mode:=mock \
  use_rviz:=false start_camera:=false show_window:=false publish_handeye_tf:=false
```

底层可以独立用 RViz/MoveIt 控制机器人；视觉可以独立检测并输出相机坐标结果。中层按开关仅装配启用的组件，关闭执行器时不读取它的运动学配置；执行器缺底层时等待规划场景服务；启动本身不派发抓取任务。总入口只组合三层、汇总参数，每层参数作用域独立。查看当前完整参数：

```bash
ros2 launch my_robot_bringup robot.launch.py --show-args
```

| 参数 | 范围与含义 |
| --- | --- |
| `control_mode` | 底层与中层共用 `mock/real`；分别启动时两者应一致 |
| `can_port`、`auto_enable`、`tcp_offset`、`use_rviz` | 底层驱动、TCP 模型和显示配置 |
| `start_executor`、`start_planner` | 中层分别启用现有抓放执行器、任务规划器；替换组件时可独立关闭 |
| `manipulation_config`、`planner_config` | 抓放与任务配置 YAML，名称分开以免冲突 |
| `perception_mode`、`perception_config` | 上层检测模式与 YAML，由视觉入口直接装配节点 |
| `start_camera`、`enable_depth`、`show_window` | 上层采集、深度关联与窗口配置 |
| `use_sim_time` | 三层共享，默认 `false`；mock 模式本身不产生 `/clock` |

本组合面向根命名空间的单臂。官方 `/control/joint_states` 保持命令入口，`/feedback/joint_states` 保持实测反馈出口，未提供改名开关。MoveIt/RSP/执行器在 real 模式订阅反馈，在 mock 模式订阅模拟控制器状态。消费者的 `joint_states` 重映射不会改变官方驱动话题。

`MotionController` 是底层对上的同步 C++ 控制接口，作为现有 `my_robot_commander` 的独立库目标导出，没有新增转发节点或 ROS 包。中层通过它调用基本运动，库本身不管理视觉、任务策略或碰撞环境。中层从底层发布的 `/robot_description`、`/robot_description_semantic` 读取模型，不重复生成另一份 URDF/SRDF；外部 C++ 调用者须提供持续 spin 的节点和运动学配置，并将 `joint_states` 订阅接到对应状态源。

```cpp
motion.moveToNamedTarget("pose_1");
motion.moveGripperToWidth(0.04);                 // 绝对总开口 40 mm，单位 m
motion.moveGripperToNamedTarget("gripper_half"); // 总开口 50 mm
motion.openGripper();                           // 官方 gripper_open，100 mm
motion.closeGripper();                          // 官方 gripper_close，0 mm
```

夹爪命名状态只保留官方 `gripper_open/gripper_half/gripper_close`，已移除新增的 `open/close` 别名。数值宽度不依赖命名状态，抓放 YAML 的 `gripper_open_width` 为 0.095 m，并按模型限位检查。独立关节 `gripper` 表示总开口，`gripper_joint1/2` 由模型 mimic 派生，不使用旧 `joint7/joint8` 手指向量。arm 保留 `pose_1/pose_2`；真实支架、桌面和运动路径仍需实物确认。

接口长度为 m、角度为 rad；输入或规划/执行被拒时返回 false，配置及 ROS/MoveIt 异常可向调用者抛出，`lastError()` 提供最近的运动错误。位姿须带有效坐标系，笛卡尔目标须已转换到 `reference_frame`。同一实例串行执行；RViz 或其他进程的运动请求不受该实例的互斥保护，目前没有统一仲裁。返回 true 表示轨迹执行成功；real 模式按真实关节反馈及配置容差判断，仍不证明绝对 TCP 精度或夹持成功。执行器会将取消请求传到 ExecuteTrajectory；取消结果未知时阻止后续运动，不等同于急停。跨进程任务使用 `/sorting/sort_objects`、`/manipulation/pick_place`，已有 `/manipulation/move_to_named` 只控制 arm。

抓放执行器在闭合命令成功后才附着模型；闭合不确定、附着失败或搬运失败时，不自动开爪或假定空手。`/manipulation/release_held_object` 是显式恢复接口，可能真实开爪，须先确认承托和现场状态。状态不确定时拒绝新抓取与命名运动；已附着的对象在运动失败后仍保留碰撞模型。

视觉缺失可能是遮挡或相机移开，不能据此删除障碍。确认已移走本执行器管理的物体后，可调用 `/manipulation/clear_observed_objects`（std_srvs/Trigger）删除相应世界碰撞体；忙碌、待恢复或仍附着时拒绝。固定 `environment/` 环境不会删除；清理前旧帧不会重新回灌，新有效观测仍可加入物体。该接口只改规划场景，不移动机器人或开爪。

相机挂载 TF 留待手眼标定，未迁入参考项目的错误外参；缺机器人 TF 时，相机二维检测和相机系测量仍可独立工作，`base_link` 物体输出尚不完整。`tcp_offset` 默认零，执行器 `tool_to_tcp` 仍需实际核对。启动层不再管理 Octomap 开关；当前没有启用相机障碍输入，固定碰撞环境由中层管理。

real 模式已改用 `my_robot_hardware/AgxTopicSystem`：`/feedback/joint_states` 写入真实状态接口，`/control/joint_states` 仅由适配器下发命令；广播器状态改发 `/controller_joint_states`，不再混入命令话题。启动先同步实际关节，缺失、过期或非法反馈不能激活；运行中反馈失效会停止发布并请求停用本系统的运动控制器，不自动恢复。mock 保留 GenericSystem。相关参数、故障恢复边界及验证见[链路可靠性修复记录](docs/control_reliability_repair.md)。本轮尚未复验真机，关节容差不代表实际 TCP 精度；本机 MoveIt 2.5.10 退出阶段段错误仍未解决，不能用自动重启掩盖。

## 视觉启动与参数

连接相机后，在已 source 工作区的终端运行：

```bash
ros2 launch my_robot_bringup robot_perception.launch.py \
  perception_mode:=color show_window:=true
```

| `perception_mode` | 启动内容 | `show_window:=true` 时的窗口 |
| --- | --- | --- |
| `color`（默认） | 颜色检测，默认同时启动深度定位 | Color detector |
| `aruco` | ArUco 检测 | ArUco |
| `both` | 两条检测路径 | 两个窗口 |
| `none` | 仅原始 RGB 预览，无检测 | RGB preview |

其他参数：

- `start_camera`：默认 `true`。已有相机节点或播放录包时设为 false，避免重复打开设备。
- `show_window`：默认 `true`；false 关闭全部窗口。`perception_mode:=none show_window:=false` 不启动检测/预览节点，相机驱动是否启动仍由 `start_camera` 决定。
- `enable_depth`：默认 `true`，只影响 color/both 模式；false 保留纯 RGB 检测，不等待深度标注。
- `perception_config`：默认使用已安装的 `perception.yaml`。修改源码配置后可重建，或直接传入其绝对路径。
- `publish_handeye_tf`：当前默认 `true`，发布已保存的手眼标定；独立二维测试可显式关闭。二维显示、颜色检测和相机系 ArUco 位姿不要求手眼 TF；输出基座系物体位置需要完整 TF。该开关不代表 launch 会等待 TF 后才启动视觉节点。
- `use_sim_time`：默认 `false`；播放带 `/clock` 的录包时按需设置。

直接使用工作区源码配置的示例：

```bash
ros2 launch my_robot_bringup robot_perception.launch.py \
  perception_mode:=aruco show_window:=true \
  perception_config:="$(pwd)/src/my_robot_perception/config/perception.yaml"
```

视觉节点的显式配置参数为启动时只读，修改 YAML 后重新启动。当前 ArUco 配置为 `DICT_4X4_50`、`marker_size: 0.03` 米；边长指单个黑色码的外边长，含黑框、不含白边。更换标定板时必须同步确认这两项。

当前物体中心模型面向已知尺寸、平放且轴与 base_link 对齐的单色箱体：深度提供顶面平面，RGB 与已知箱体投影补偿斜视时可见侧面对 bbox 中心的影响。它保留 RGB 独立分割，不用稀疏深度质心替代横向定位；尚不支持任意遮挡与任意物体姿态的精度保证。

## 关键输出与显示

| 话题 / TF | 含义 |
| --- | --- |
| `/camera/color/origin` | 原始 RGB，`bgr8`，颜色检测及预览使用此图 |
| `/camera/color/aligned` | SDK 对齐到深度网格的 RGB，可能有空洞；不是颜色分割输入 |
| `/camera/depth` | `32FC1` 深度，单位米，无效值 NaN |
| `/camera/info/color`、`/camera/info/depth` | 两路图像内参 |
| `/camera/extrinsics/color_to_depth` | 沿用旧话题名；本机 SDK 约定实际用于 `P_rgb = R × P_tof + t` |
| `/camera/pointcloud` | SDK 测量的稠密场景点云，剔除无效点，与目标代表点话题不同 |
| `/perception/color_labels` | 原始 RGB 网格的 `mono8` 标签：0 背景、1 红、2 绿、3 蓝 |
| `/perception/color_debug` | RGB 上的框与 `red1: 150 mm` 等标注；深度表示 RGB 光学系轴向 Z |
| `/perception/object_points` | 表面代表点，默认 RGB 光学系；含 xyz/label/u/v/depth 字段 |
| `/perception/objects` | 已知尺寸、平放且轴向对齐物块的中心；默认在 `base_link`，缺真实 TF 时可能为空 |
| `aruco_marker_<ID>` | 每个识别到的单码位姿 TF，不是整板位姿或手眼标定结果 |

RViz 看图使用 Image 显示项。单独查看相机点云时可将 Fixed Frame 设为 `camera_depth_optical_frame`；接入机械臂坐标需要真实手眼 TF。窗口里的颜色区域编号每帧生成，不是稳定跟踪 ID。

## 验证与当前边界

测试源码集中在根目录 `test/`，保留各包的 CMake 测试注册：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select my_robot_bringup vidu_ros2_wrapper my_robot_perception \
  --event-handlers console_direct+
colcon test-result --verbose
```

也可直接运行集中 Python 功能测试（先完成带测试的构建）：

```bash
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/piper_test_logs \
  /usr/bin/python3 -m pytest -q test/
```

配置检查不启动节点；模拟联动使用隔离域和 mock 控制器，验证分层独立性、依赖等待、命名姿态和 40 mm 总开口。视觉与手眼回归使用合成数据，不连接硬件。测试目录保留五个按职责划分的 Python 入口；三物料前置检查已并入配置测试，重复的视觉模式检查与上游模板 lint 已移除。原生转换检查仍直接调用生产 C++ 实现。完整用法、各项判定与依赖见 [test/README.md](test/README.md)。

2026-11-08 分层验证：11 包构建通过；22 项模型/启动配置、2 项独立与总入口 mock 联动、1 项控制门恢复测试、15 项视觉回归及原生转换程序的 8 项检查通过。Piper mock 联动验证了无自动运动、底层模型读取、pose_1/pose_2 与 40 mm 总开口，未启动真机或相机。MoveIt 退出段错误仍作为明确警告记录，不能据此认定退出稳定性通过。

2026-11-10 链路可靠性修复：14 个依赖包构建成功，集中离线回归 `79 passed, 5 warnings`（129.00 s），原生转换入口含 9 项 C++ 检查。新增用例覆盖真实状态适配器的合成反馈、无反馈拒绝激活、反馈故障停用、运动中取消、执行超时和客户端等待期限。手眼 GUI 与非法配置退出也已回归；尚未复验真机。详细范围、命令与剩余限制见[修复记录](docs/control_reliability_repair.md)，Planner 新能力见[待审方案](docs/planner_upgrade_proposal.md)。

2026-11-07 用户实测反馈：已测试四个场景的实时检测，color 效果改善，部分物体识别较准确。本阶段接受保留以下问题，后续再处理：

| 待改进项 | 当前记录 |
| --- | --- |
| 检测区域与实际面积不符 | 尚需结合目标轮廓、HSV/形态学及遮挡场景定位；像素区域也不能直接当物理面积 |
| 近距深度失效 | 用户报告低于约 300 mm 时可能变成 `-- mm`；未确定根因，不能据此认定硬件最小测距为 300 mm |
| 噪声与标签稳定性 | 仍有噪声候选及区域编号变化，尚无跨帧目标跟踪 |

`-- mm` 表示当前关联不足，不发布猜测距离。现有前景边界检查依赖局部平面与外围有效深度；复杂遮挡、物体内部色块或缺少外围测量时可能拒绝三维关联。当前几何配置仍是 1 cm 小物块，不适用于任意红盒/杯盖的抓取中心估计。眼在手上的标定 TF、世界系稳定性与完整机器人执行闭环仍待后续集成。

详细记录：[相机实施说明](docs/camera_vision_implementation.md)、[联调报告](docs/camera_debugging_report.md)、[集成缺口、规划与底层合并记录](docs/real_grasp_integration_gaps.md)。已结合参考项目补齐底层配置，后续继续处理依赖退出缺陷、手眼标定、真实环境与执行反馈；视觉精度问题按计划后续处理。当前 `.gitignore` 忽略 `docs/`，这些本地文档尚不会自动随代码交付；因此主要使用方法和待办同时保留在本 README。

仓库含多个上游组件，许可请分别查看对应包及厂商 SDK 的声明。

### 三物料抓取与 ROS 域清理

整批分拣成功后，脚本通过现有命名运动接口回到 `post_grasp_target`，在 `config/three_material_grasp/startup.yaml` 中配置，默认 `home`，`""` 表示不回位，也可改为其他 SRDF arm 命名姿态。它与预定位/观察姿态 `pre_grasp_target` 分别配置。脚本等待结束回位成功才结束；分拣失败、取消或结果不明时不自动回位。

闭爪压缩量位于 `src/my_robot_commander/config/manipulation.yaml`：`gripper_compression: 0.004` 表示总压缩量 4 mm，比之前的闭合目标再小 2 mm。当前 50 mm 轴对齐方块、每侧 3 mm 垫片，对应机械开口目标 `50 + 6 - 4 = 52 mm`；预张开宽度保持 95 mm。脚本读取源 YAML，修改这两处配置后重新运行即可，无需为 YAML 改动构建。

脚本入口统一放在 `script/*.sh`；入口使用的 Python 流程集中在 `script/build/*.py`，按同名文件对应。该目录存放随项目维护的源码，不是 colcon 生成目录。可修改参数仍放在根目录 `config/`。`my_robot_bringup` 仅负责 launch 编排。

`script/three_material_grasp.sh` 启动控制、规划和视觉三层。`pre_grasp_target` 非空时，脚本先通过命名运动 Action 到该姿态（当前配置 `pose_1`），成功后才提交 `/sorting/sort_objects`，物料间也回该姿态观察；设为 `""` 则直接从当前位置开始，并关闭物料间固定回位。SortObjects 本身只接收策略和颜色列表，不检查是否到固定观察位，首次使用当前位置的新鲜观测；独立 Planner 默认也不回固定姿态。先编辑 [启动配置](config/three_material_grasp/startup.yaml)：`default_strategy`、`default_color_order`、`place_position_modes` 和 `place_positions` 都以该文件为本次任务的配置来源，覆盖基础 planner YAML 的对应项；基础文件不会被修改。Action 不再硬编码顺序，默认按 `[1, 2, 3]` 即红、绿、蓝每色选择一个目标。

`place_position_modes` 是三种颜色共用的 `[x, y, z]` 模式，缺省为 `[absolute, absolute, absolute]`；`place_positions` 保持每色一个三维数值数组，单位米。absolute 表示物体放置中心的绝对坐标，relative 表示相对该物体本次抓取前中心的偏移，方向均沿 `base_link`，不跟随物体自身旋转。例如模式 `[relative, absolute, absolute]` 配合 red 的 `[-0.24, -0.06, -0.12]`，目标为 `[红块初始 x - 0.24, -0.06, -0.12]`。Planner 在派发时解析为绝对 place_pose；Executor 和 Action 接口不因放置模式改变。dry-run 只能显示规则，Planner 运行日志才显示当前物体及最终目标坐标。

抓取区域筛选位于 Planner 内部：物体转换到 `reference_frame` 后、固定批次 ID 前，按物体中心的 XYZ 闭区间排除区域外目标；后续派发前也会复查。参数为 `task_planner.yaml` 的 `grasp_region_enabled/min/max`，默认关闭，零值上下界仅是占位，启用前须按现场填写。`startup.yaml` 的 `grasp_region: null` 或省略该项表示继承；映射中的 `enabled/min/max` 分别覆盖基础参数，`{enabled: false}` 显式关闭。修改 YAML 后重新启动脚本即可，不需重新构建。区域外目标本批不再加入，即使随后进入区域，也需新批次；日志和结果 message 列出跳过 ID、数量及完成数量。全部越界返回失败；其余候选流程均完成可返回成功，并保留跳过汇总。尚未观察到的指定颜色仍按原有期限等待。视觉话题和碰撞场景保持完整；该功能不验证物体整体包围盒、放置区域、逆解或路径可达性。

夹爪位置到位检查由底层 `agx_arm_moveit/launch/demo.launch.py` 函数内的固定值 `gripper_goal_tolerance = 0.0`（m）关闭，避免接触物体后因未达到计算开口而中止，不读取环境变量，也不对外提供 YAML、脚本或 launch 参数。该设置作用于夹爪的张开、闭合及定宽运动，完成仅表示命令流程完成，不保证实际开口到位或物体已夹住。机械最大开度、机械臂关节容差、反馈断流保护、超时和取消仍保留。修改该安装脚本后需构建 agx_arm_moveit 同步，再安全重启原控制栈。日志分别打印闭合目标计算与实际开口；当前几何输入仍是固定尺寸方块，更换物体须同步其尺寸与抓姿假设。

三个放置数组须根据现场填写；不要沿用 1 cm 仿真物块的 Z 高度。当前脚本 YAML 的 `launch.auto_enable` 为 true，会在真机启动时尝试使能；需要手动使能时改为 false。不要同时启动另一套相机或机械臂驱动。SortObjects 定义已简化，接口变更后须重建依赖程序并重启旧节点；当前位置启动仍要求有效 TF、当前观测和可规划的目标。

```bash
bash script/three_material_grasp.sh --dry-run  # 检查配置，不启动或运动
bash script/three_material_grasp.sh            # 启动后自动发送实际分拣任务
```

支持 `--config 路径`，相对路径从工作区解析。脚本检查标定文件与视觉配置，并等待与文件一致的手眼 TF、实时 `base_link` 到相机 TF 和分拣 Action；缺失时不发送抓取目标，不要求 TF 发布器使用固定节点名。启动就绪等待为 90 s，硬件首批反馈等待为 40 s，执行器等待 MoveIt 为 60 s。

Planner 派发前检查新观测，短暂失测最多等待 15 s 恢复，观测最大年龄仍为 2 s；单条非法记录不影响其他有效目标。感知在图像采集时刻转换坐标，Planner/Executor 仅在输入坐标系需要转换时查询对应 TF，已在基座系的目标不额外依赖当前相机 TF。执行开始后按固定目标完成，不要求物体持续可见。深度配对断流期限为 3 s，等待期间不重发旧目标；恢复后自动继续定位。无效场景观测不覆盖或删除已知障碍。

`PickPlace.Result.error_code` 区分目标不可用、轨迹执行失败和需要恢复。接触前的输入或规划失败只跳过该目标，本批不再重试，其他物体继续处理；执行故障、可能持物或结果不明时停止后续动作。批次末尾汇总未完成目标，存在跳过的失败目标时 `SortObjects.success=false`，完成数只计实际成功的运动流程。脚本报“抓取流程未完成”并保留控制栈，不能把这一任务结果当成节点崩溃。新增了 Action 结果字段，旧节点和终端须统一重新加载构建后的工作区再使用。

抬升先沿 Z 轴移动 lift_distance，再通过碰撞检查的搬运规划到达 transport_height；不执行残缺直线路径。脚本在执行期间监测启动进程、Action 期限与取消，请求响应、整批执行和取消确认受 `action_timeouts` 约束。**软件取消、TF 检查或停止发布命令均不保证真机立即停下**。TF 存在不代表精度合格，实际自动抓取前仍需验证目标中心、TCP 和标定。Action 成功只表示运动流程完成，夹持与放置仍需核实。客户端结束后控制栈保持运行，关闭前应确认现场状态。

```bash
bash script/clear.sh --domain-id 189 --list  # 只列出
bash script/clear.sh --domain-id 189         # 列出并确认后停止本地可识别的 ROS 进程
```

不传域时使用 `ROS_DOMAIN_ID`，环境变量也未设置时为 ROS 默认的 0。清理以当前用户、进程环境中的域和 ROS 进程特征为依据，可能无法识别自定义域上下文、其他用户或远程机器上的节点；ROS 节点也可能与其他节点共用一个进程。停止控制进程不等于机械臂安全停机或失能，清理前应确认现场状态。可用 `--yes` 跳过交互确认。
