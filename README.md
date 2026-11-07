# piper_real

基于 ROS 2 的实物抓取框架，使用 AgileX Piper X、AGX 官方平行夹爪 `agx_gripper` 和安装在末端的 Vidu RGB-D 相机（眼在手上）。

当前阶段主要完成相机与视觉基础链路。机器人控制、规划与执行模块已经分包，但完整实机抓取、手眼标定和精度验收尚未完成。

## 目录与职责

| 目录 | 职责 |
| --- | --- |
| `src/agx_ros_control_moveit/` | AGX 机械臂驱动、机器人描述、MoveIt、控制器及 RViz 配置 |
| `src/vidu_camera_wrapper/` | Vidu ROS 2 消息接口与相机采集节点，依赖外部 Vidu SDK |
| `src/my_robot_perception/` | 原始 RGB 颜色分割、深度关联、ArUco 检测和视觉测试入口 |
| `src/my_robot_interface/` | 上层模块共用的消息与 Action 定义 |
| `src/my_robot_commander/` | MoveIt 运动封装、抓放执行器和碰撞场景管理 |
| `src/my_robot_planner/` | 根据感知对象组织分拣任务，调用执行层 Action |
| `src/my_robot_bringup/` | 预留启动包，当前尚无统一实机启动入口 |
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

需要构建完整工作区时使用 `colcon build`。本阶段的构建与测试结论主要覆盖视觉相关包，不代表完整实机链路已验收。

## 视觉启动与参数

连接相机后，在已 source 工作区的终端运行：

```bash
ros2 launch my_robot_perception perception_test.launch.py \
  start_camera:=true mode:=color show_window:=true
```

| `mode` | 启动内容 | `show_window:=true` 时的窗口 |
| --- | --- | --- |
| `color`（默认） | 颜色检测，默认同时启动深度定位 | Color detector |
| `aruco` | ArUco 检测 | ArUco |
| `both` | 两条检测路径 | 两个窗口 |
| `none` | 仅原始 RGB 预览，无检测 | RGB preview |

其他参数：

- `start_camera`：默认 `false`。已有相机节点或播放录包时保持 false；设为 true 才打开实物相机。
- `show_window`：默认 `true`；false 关闭全部窗口。`mode:=none show_window:=false` 不启动视觉节点，相机驱动是否启动仍由 `start_camera` 决定。
- `enable_depth`：默认 `true`，只影响 color/both 模式；false 保留纯 RGB 检测，不等待深度标注。
- `config`：默认使用已安装的 `perception.yaml`。修改源码配置后可重建，或直接传入其绝对路径。
- `use_sim_time`：默认 `false`；播放带 `/clock` 的录包时按需设置。

直接使用工作区源码配置的示例：

```bash
ros2 launch my_robot_perception perception_test.launch.py \
  start_camera:=true mode:=aruco show_window:=true \
  config:="$(pwd)/src/my_robot_perception/config/perception.yaml"
```

视觉节点的显式配置参数为启动时只读，修改 YAML 后重新启动。当前 ArUco 配置为 `DICT_4X4_50`、`marker_size: 0.03` 米；边长指单个黑色码的外边长，含黑框、不含白边。更换标定板时必须同步确认这两项。

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

不连接相机也可运行现有合成回归：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select vidu_ros2_wrapper my_robot_perception \
  --event-handlers console_direct+
colcon test-result --verbose
```

最近一次代码回归：8 项 C++ 和 15 项 Python 功能用例通过；8 组 mode/窗口组合通过。完整两包检查为 0 failures，另有 8 项 cppcheck 检查因本机工具版本跳过。这些结果不等于实机抓取精度或长期稳定性验收。

2026-11-07 用户实测反馈：已测试四个场景的实时检测，color 效果改善，部分物体识别较准确。本阶段接受保留以下问题，后续再处理：

| 待改进项 | 当前记录 |
| --- | --- |
| 检测区域与实际面积不符 | 尚需结合目标轮廓、HSV/形态学及遮挡场景定位；像素区域也不能直接当物理面积 |
| 近距深度失效 | 用户报告低于约 300 mm 时可能变成 `-- mm`；未确定根因，不能据此认定硬件最小测距为 300 mm |
| 噪声与标签稳定性 | 仍有噪声候选及区域编号变化，尚无跨帧目标跟踪 |

`-- mm` 表示当前关联不足，不发布猜测距离。现有前景边界检查依赖局部平面与外围有效深度；复杂遮挡、物体内部色块或缺少外围测量时可能拒绝三维关联。当前几何配置仍是 1 cm 小物块，不适用于任意红盒/杯盖的抓取中心估计。眼在手上的标定 TF、世界系稳定性与完整机器人执行闭环仍待后续集成。

详细记录：[相机实施说明](docs/camera_vision_implementation.md)、[联调报告](docs/camera_debugging_report.md)、[抓取集成缺口](docs/real_grasp_integration_gaps.md)。当前 `.gitignore` 忽略 `docs/`，这些本地文档尚不会自动随代码交付；因此主要使用方法和待办同时保留在本 README。

仓库含多个上游组件，许可请分别查看对应包及厂商 SDK 的声明。
