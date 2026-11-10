# 测试使用说明

测试按职责保留五个 Python 入口。正式启动统一使用 `my_robot_bringup/launch/`，测试调用正式入口，不维护另一套节点装配逻辑。

```text
test/
├── README.md
├── test_robot_configuration.py  # 配置、模型、launch 与脚本前置检查
├── test_robot_launch.py         # 三层模拟联动与控制门恢复
├── test_visual_pipeline.py      # RGB、深度、ArUco 与异常输入回归
├── test_handeye.py              # 手眼求解、存储与合成 TF 标定
├── test_conversions.py          # SDK 转换检查的 Python 入口
└── native/
    └── test_conversions.cpp     # 直接测试项目实际使用的 C++ 转换实现
```

三物料启动检查已并入配置测试；视觉模式选择也只在配置测试中检查一次。已移除不属于本项目功能回归的三份上游模板 lint 脚本。上游 AGX 和 easy_handeye2 源码未因这次整理修改。

## 准备与运行

依赖 Ubuntu 22.04、ROS 2 Humble、系统 Python、pytest、PyYAML、NumPy、OpenCV ArUco、cv_bridge，以及工作区的消息、MoveIt、ros2_control 和相机转换程序。转换与 wrapper 用例仍需 Vidu SDK 运行库，但不会打开实物相机。完整环境安装见[项目 README](../README.md)。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to my_robot_bringup --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

只收集列表，不启动节点：

```bash
/usr/bin/python3 -m pytest --collect-only -q test/
```

运行全部功能回归：

```bash
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/piper_test_logs \
  /usr/bin/python3 -m pytest -v test/
```

当前共 **65 项 pytest 用例**：配置 38、规划及模拟联动 9、视觉 12、手眼 5、转换 1。转换入口内部还有 9 项 C++ GTest 检查，不重复计入 pytest 数量。正常结果为 `65 passed`；失败表示相应用例未满足要求。通过不代表实物位置精度、夹持质量或手眼结果已经验收。

这些测试不启用 CAN、真机驱动或实物相机。模拟联动强制使用域 175，控制门使用 177，手眼合成链路使用 178，放置坐标检查使用 179，夹持恢复使用 181；视觉和转换直接运行时使用命令指定的 176。不要在这些域运行实际应用，也不要同时运行两份测试或使用 pytest-xdist。它们会发布合成相机、物体、TF 和关节消息。

可按文件运行，也可用 `文件::用例名` 定位：

```bash
ROS_LOG_DIR=/tmp/piper_test_logs /usr/bin/python3 -m pytest -v test/test_robot_configuration.py
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/piper_test_logs \
  /usr/bin/python3 -m pytest -v -x test/test_visual_pipeline.py::test_live_synthetic_pipeline
```

## 配置与前置条件：`test_robot_configuration.py`

38 项，不启动 ROS 节点或真实硬件。依赖已安装的机器人模型、启动模块、xacro 和项目配置；TF 检查使用模拟数据。

| 用例 | 验证内容及通过标准 |
| --- | --- |
| `test_named_states_fit_actual_model`，4 项 | Piper、Piper X、Nero 的模型资源匹配型号；命名状态在关节限位内；Piper 保留 pose_1/pose_2；夹爪仅保留官方命名状态，总开口与 mimic 关系一致 |
| `test_follow_selects_official_state`，2 项 | 真机读取官方反馈，模拟读取控制状态；没有遗留的话题改名或 Octomap 参数 |
| `test_official_driver_endpoints_are_not_renamed`，1 项 | 官方控制和反馈话题保持原名 |
| `test_driver_compositions_keep_official_endpoints`，2 项 | AGX 的 MoveIt/RViz 组合入口沿用官方接口 |
| `test_control_layer_starts_only_control`，2 项 | 底层只组合控制，固定 Piper 与 AGX gripper；真机默认不自动使能 |
| `test_middle_consumes_control_without_rebuilding_model`，2 项 | 中层按真机/模拟模式选择状态来源，复用底层模型，只添加运动学参数 |
| `test_perception_layer_selects_nodes`，7 项 | color/aruco/both/none、深度、窗口及相机开关产生正确节点；配置文件与节点参数一致；纯 RGB 检测不等待深度标注 |
| `test_perception_rejects_invalid_configuration`，1 项 | 非法模式或不存在的视觉配置被明确拒绝 |
| `test_perception_can_publish_saved_handeye_without_loading_robot`，1 项 | 视觉入口可以附加保存的手眼 TF，转发目录和名称，不启动机械臂 |
| `test_middle_switches_load_only_enabled_components`，4 项 | 按开关启动执行器/规划器；关闭执行器时不加载其配置依赖 |
| `test_master_exposes_and_forwards_all_layer_arguments`，1 项 | 总入口组合三层，参数没有重复声明，且完整转发 |
| `test_startup_requires_calibration_and_real_place_positions`，1 项 | 三物料脚本拒绝缺失标定或未填写的放置点；生成的配置启用每色一个物体及相机 TF 检查 |
| `test_startup_task_parameters_override_only_the_generated_config`，1 项 | startup 中的观察姿态、策略、颜色顺序、轴模式和放置点覆盖基础 planner 的冲突值，但不修改基础文件；旧 startup 未设置轴模式时保持全 absolute |
| `test_invalid_place_position_modes_are_rejected`，3 项 | 启动脚本拒绝轴模式数量不等于三、未知模式以及布尔值写法 |
| `test_optional_pre_grasp_precedes_sorting`，5 项 | 模拟选择预定位时的成功、拒绝、失败和 TF 丢失；还验证空预定位直接提交分拣、不调用命名运动。分拣使用配置默认顺序，Action 不包含姿态到位标志 |
| `test_live_tf_checks_publisher_calibration_and_freshness`，1 项 | 用模拟 TF 验证：标定发布节点消失、变换不匹配、TF 过期均被拒绝；名称中的 live 不表示连接真实设备 |

失败时检查源文件、配置路径、单位、参数转发及是否加载了当前工作区的安装文件。此文件不判断真实手眼标定是否准确。

## 三层模拟联动：`test_robot_launch.py`

9 项。三层联动只使用 mock 硬件；放置坐标用例只运行实际 C++ Planner，测试执行器拒绝全部运动请求。联动依赖 MoveIt、ros2_control、controller_manager_msgs、moveit_msgs、控制器插件和已构建的 commander/planner/perception。

| 用例 | 使用结果判定 |
| --- | --- |
| `test_mock_layers[independent]` | 视觉可独立启动，中层在底层未就绪时等待；底层启动后接口接通；不出现真机驱动或相机节点；显式命令能到达 pose_1、pose_2 和 40 mm 总开口；在 pose_2 直接发送普通分拣 Action 不强制回 pose_1，仍可取消等待 |
| `test_mock_layers[master]` | 总 launch 与分层启动具有相同节点和模拟执行能力，启动本身不自动运动 |
| `test_control_gate_retries_and_synchronizes` | 模拟门控服务晚启动、拒绝、重启、延迟响应时，客户端会重试并恢复正确开关状态；手臂或夹爪仍在执行时不提前关门 |
| `test_planner_resolves_place_positions`，5 项 | 检查全 absolute、混合、全 relative，以及错误长度/未知模式；截获实际 Planner 的 place_pose，验证物体旋转不改变偏移轴、多次任务不会累加偏移。没有命名运动服务且配置了物料间观察位，首次分拣仍能直接派发。测试执行器始终拒绝请求，不执行抓取 |
| `test_grasp_recovery_and_scene_lifecycle`，1 项 | 启动真实 MoveIt/Executor 配合 mock 硬件，在闭合和抬升阶段停用模拟控制器注入失败；确认闭合失败不提前附着、抬升失败保留附着，未恢复时拒绝新抓取和清理。恢复后显式清理成功，固定桌面保留；观测缺失不删除障碍 |

测试只清理自己启动的进程。已知 MoveIt 在退出阶段可能出现 SIGSEGV；现有用例会将这一已知问题明确报告为警告，其他异常退出仍判失败。模拟轨迹成功不等于真机到位；门控服务测试也不等于硬件急停测试。

## 合成视觉回归：`test_visual_pipeline.py`

12 项，依赖系统 OpenCV、NumPy、cv_bridge、ROS 图像/点云/TF/参数接口及已构建的视觉节点。关闭窗口，使用合成 RGB、深度和 ArUco，因此不弹出画面是正常现象。

| 用例 | 使用结果判定 |
| --- | --- |
| `test_live_synthetic_pipeline`，1 项 | 通过正式视觉 launch 启动三个检测节点；深度空洞不改变 RGB 分割；跨分辨率投影、毫米标注、物体中心、畸变和 ArUco 位姿满足构造场景的期望；无效内参不会覆盖可用内参 |
| `test_wrapper_sigint_without_opening_device`，1 项 | 使用故意不合法的相机 frame 配置，在打开设备前拒绝启动；连续三次 SIGINT 均正常退出，无 terminate 错误 |
| `test_rgb_detection_and_preview`，3 项 | 无深度时仍能识别蓝色；分辨率变化后面积规则有效；纯预览不发布检测结果 |
| `test_startup_parameters_are_readonly`，3 项 | 三个视觉节点拒绝运行时修改只读启动参数，读取的实际值保持一致 |
| `test_annotation_validation_and_rgb_watchdog`，1 项 | 点云异常布局、字节序与行跨度不会使 RGB 节点崩溃；有效深度标注正确；RGB 停流后清除旧标注并标记过期 |
| `test_oblique_cube_center_with_depth_holes`，3 项 | 光线与立方体求交生成几何真值，覆盖 X/Y 斜视和复合斜视；分别使用完整深度和半幅深度缺失，中心误差不超过 2 mm。此阈值只适用于无噪声合成场景，不能当作实物精度承诺 |

相机内参、颜色标签、深度单位、TF 或点云布局错误，都可能导致该组失败。先查看 `nodes.log`、`rgb_only.log`、`readonly.log` 或 `annotation.log`，再根据断言定位。合成输入下的毫米误差阈值不能当作相机实测精度。

实物预览直接使用同一个正式入口，不另建测试 launch：

```bash
ros2 launch my_robot_bringup robot_perception.launch.py \
  perception_mode:=color show_window:=true publish_handeye_tf:=false
```

该命令会连接实物相机，与上述自动回归分开运行。改为 `perception_mode:=aruco` 可查看标记；已有相机驱动时传 `start_camera:=false`。二维检测不要求手眼 TF；输出 base_link 下的三维目标仍需有效 TF。

## 手眼标定：`test_handeye.py`

5 项。依赖 easy_handeye2、OpenCV、transforms3d、ROS TF/服务与项目 launch；结果只写入临时目录，不覆盖实际标定文件。

| 用例 | 使用结果判定 |
| --- | --- |
| `test_solver_and_workspace_storage` | 从已知变换生成的数据恢复正确手眼变换，保存和读取一致；非法四元数被拒绝 |
| `test_degenerate_samples_are_rejected` | 样本过少或旋转轴不足的数据不能被误判为有效标定 |
| `test_launch_uses_existing_visual_pipeline_without_dummy_tf` | 标定入口复用正式视觉层；发布模式只启动真实保存结果的发布器，不补造相机 TF |
| `test_script_auto_selection_and_overrides` | .calib 文件决定采集/发布模式，只有 .samples 不算完成标定；重标定、名称、marker 参数覆盖和非法参数处理符合约定；只执行 dry-run |
| `test_synthetic_tf_roundtrip` | 在独立域生成两棵合成 TF 树，经采样、求解、保存和重新发布恢复已知变换；标记停流后拒绝过期样本 |

这一组验证标定软件链路，不验证真实标定板尺寸、安装刚度、关节零点或机器人绝对精度。

## SDK 转换：`test_conversions.py` 与 `native/test_conversions.cpp`

Python 入口运行已构建的 `build/vidu_ros2_wrapper/conversion_probe`，因此必须以 `BUILD_TESTING=ON` 构建 wrapper，并安装 SDK、OpenCV、GTest 及 ROS 标定依赖。需要指定其他构建位置时可设置 `VIDU_CONVERSION_PROBE`。

9 项原生检查覆盖：RGB/BGR 通道和点云颜色、深度米制与无效值、内参与畸变模型、显式标定文件错误、三类恢复诊断、未知模型、SDK 字节序与像素布局、首帧超时预算，以及 RGB/ToF 外参方向。全部 C++ 检查通过且程序返回 0，Python 入口才通过。

保留 C++ 测试源是为了直接调用生产转换实现；不在 Python 中复制一套转换算法。此测试不打开相机，也不验证 SDK 的现场测量质量。

## 构建系统、日志与失败定位

各包仍注册根目录测试文件，可按包运行：

```bash
colcon test --packages-select my_robot_bringup my_robot_perception vidu_ros2_wrapper \
  --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
```

包级结果还包含 ament 的代码规范检查，数量与 65 项 pytest 不同；`test/` 中不再混入上游模板 lint。包级日志位于 `build/<包名>/Testing/Temporary/` 及 `log/latest_test/`。直接 pytest 的临时目录由 pytest 管理，失败输出会给出路径；`ROS_LOG_DIR` 保存 ROS 自身日志。

常见失败处理：

| 现象 | 检查方向 |
| --- | --- |
| 模块、接口或可执行程序找不到 | 使用系统 Python，加载 ROS 和当前工作区；确认相关包已构建 |
| 配置检查通过，运行仍使用旧行为 | 重建相关包并重新 source；核对安装目录是否残留已删除的 launch |
| 等待话题、Action、TF 超时 | 检查隔离域是否被占用、DDS 配置和对应子进程日志，不要连接真实驱动来绕过测试 |
| 只有退出阶段报错 | 区分已记录的 MoveIt SIGSEGV 与新的崩溃或无法结束的进程 |
| wrapper 测试加载动态库失败 | 检查 Vidu SDK 的安装与动态库搜索路径 |
