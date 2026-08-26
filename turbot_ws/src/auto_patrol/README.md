# TurtleBot3 `waffle` 自动巡检

> ROS 2 Humble | `turtlebot3_world` | Nav2 | 三目标点自主巡检

`auto_patrol` 是一个面向 TurtleBot3 的 ROS2 自动巡航节点。节点默认请求
Nav2 AMCL 进行全局重定位，在确认传感器和定位状态有效后，依次执行三个
waypoint。定位阶段会自动通过 `/cmd_vel_nav` 原地旋转，主动获取不同方向的激光
观测；第一次达到低协方差时不会立刻开始巡检，而是先检查前方 LaserScan 净空、
开始短距离平移前清空旧 AMCL 证据，保留平移期间的新观测；若样本尚不足，
仅进行安全原地旋转补足观测，再静止确认定位结果仍有效。
确认后程序会清理全局和局部代价地图，等待其根据当前扫描重新生成障碍与膨胀层。
默认流程不要求在 RViz 中手动点击 `2D Pose Estimate`，也不要求预先填写机器人
出生位置。本 README 面向作业验收与项目复现：先说明如何一条命令启动，再解释
组件、定位状态机和 Nav2 调参入口。

## 作业目标与已实现能力

作业统一环境为 ROS 2 Humble、TurtleBot3 `waffle` 与 `turtlebot3_world`。项目以
`NavigateToPose` Action 向 Nav2 提交三个 `map` 坐标系目标点，而不是绕过 Nav2
直接控制机器人速度；规划、避障、恢复和速度平滑仍由 Nav2 负责。

| 作业要求或加分方向 | 本项目实现 |
| --- | --- |
| 三个具有空间跨度的目标点 | `patrol.yaml` 中的 `waypoint_1/2/3`；三点均使用 `[x, y, yaw]`（m、m、rad）配置。 |
| 避开环境障碍物 | 目标通过 Nav2 全局规划、局部 DWB 控制器、代价地图和恢复行为执行，不由巡检节点硬编码路径。 |
| 无需 RViz 手动发送目标 | `PatrolController` 仅在上一个 `NavigateToPose` 返回 `SUCCEEDED` 后发送下一点。 |
| 无需手动设置初始位姿 | 默认调用 AMCL `/reinitialize_global_localization`，再进行旋转、短距离平移和二次确认。 |
| 少量命令启动 | `patrol_with_nav2.launch.py` 一次启动 Gazebo 世界、Nav2、RViz 和巡检节点。 |
| 导航失败 Recovery | Nav2 负责路径无效、局部障碍等恢复；巡检节点对 `ABORTED` 或 `CANCELED` 的目标按 `max_retries` 重试。 |
| 运动稳定性调节入口 | `nav2_waffle.yaml` 集中保存 DWB 与 `velocity_smoother` 参数。 |

开发过程曾得到三点均返回 `Goal reached`、最终输出
`Patrol completed successfully: all 3 goals reached.` 的运行结果。最终验收仍应按下文
“验收录制建议”从完整启动过程重新录制，并以实际运行画面确认无明显碰撞、卡死或
持续原地旋转。

## 组件职责

- `AutoPatrolNode`：组装组件、创建 ROS2 通信资源、驱动 Timer 和编排流程；
- `PatrolConfig`：声明、读取和校验参数；
- `pose_utils`：提供无副作用的角度、姿态和时间戳计算；
- `LocalizationMonitor`：检查 `/scan` 时间戳以及 AMCL 定位稳定性；
- `localization_safety`：提供前方激光安全判断和 odom 平面距离的纯计算，便于不启动
  Gazebo 的单元测试；
- `LocalizationBootstrapper`：调用 `/reinitialize_global_localization`，清除旧定位
  样本，管理旋转探索、激光安全检查、odom 平移验证、二次确认和超时状态机；
- `CostmapCleaner`：异步调用 Nav2 的全局、局部代价地图清理服务，并等待刷新窗口；
- `InitialPosePublisher`：仅在手动兼容模式下按配置重复发布 `/initialpose`；
- `PatrolController`：管理 Nav2 Action、waypoint 推进、失败重试和终态。

入口 Node 不直接承担参数校验、姿态计算、传感器稳定判断和 Action 重试，
这样可以在不启动真实机器人和 Nav2 的情况下测试核心逻辑。

## 配置与启动文件

本包按作业目录要求，将配置和 Launch 源文件放在 `src/` 下。地图源位于工作区根目录
的 `map/`；CMake 会将其安装到 `share/auto_patrol/map`，因此标准
`ros2 launch auto_patrol ...` 命令可以从任意工作目录启动。

```text
turbot_ws/
├── map/
│   ├── map.yaml                    # 当前地图描述；image 字段为 map.png
│   ├── map.png                     # 当前由 Nav2 map_server 实际加载的栅格图
│   └── map.pgm                     # 保留的备用图像，除非修改 map.yaml，否则不会加载
└── src/auto_patrol/
    ├── include/auto_patrol/        # 公共接口和组件职责声明
    ├── src/
    │   ├── config/
    │   │   ├── patrol.yaml         # waypoint、自动定位、超时和重试配置
    │   │   └── nav2_waffle.yaml    # 完整 Nav2 参数副本及速度平滑器配置
    │   ├── launch/
    │   │   ├── auto_patrol.launch.py
    │   │   └── patrol_with_nav2.launch.py
    │   ├── main.cpp                # 仅负责 ROS2 初始化和退出码
    │   └── *.cpp                   # 组件实现
    └── test/                       # 纯逻辑和 Launch 结构测试
```

`nav2_waffle.yaml` 基于当前 TurtleBot3 Humble 的
`turtlebot3_navigation2/param/humble/waffle.yaml` 完整复制。不能以一个只含少量
速度参数的 YAML 替代它，因为官方 Nav2 Launch 只接收一个 `params_file`；不完整
文件会遗漏 AMCL、costmap、planner 等节点的必需参数。

`patrol.yaml` 使用本项目已经成功完成三点巡检时的 waypoint 和定位等待时间。
它是推荐的启动配置；表中“默认值”仍表示不加载 YAML、直接 `ros2 run` 时 C++
节点内部声明的默认值。

## 地图配置

组合 Launch 的 `map` 参数默认使用构建后安装的
`install/auto_patrol/share/auto_patrol/map/map.yaml`。该文件来自工作区根目录的
`map/map.yaml`，当前内容为：

```yaml
image: map.png
resolution: 0.05
origin: [-2.37, -2.38, 0]
```

`image` 是相对 `map.yaml` 所在目录解析的，因此 `map.png` 必须和 `map.yaml` 一起
保留并提交。`map.pgm` 当前不被 YAML 引用；只有将 `image: map.png` 改为
`image: map.pgm` 后，Nav2 才会加载它。

不要在 `nav2_waffle.yaml` 中写个人电脑的绝对地图路径。组合 Launch 会把自身的
`map` 参数传给官方 `navigation2.launch.py`，由官方 Launch 覆盖 `map_server` 的
`yaml_filename`。这样既保留了手动覆盖地图的能力，也保证仓库克隆到其他目录后可用。

## 自动定位状态机

```text
等待 Action Server
        ↓
等待 /reinitialize_global_localization 服务
        ↓
请求 AMCL 全局重定位
        ↓
清除旧 AMCL 稳定样本
        ↓
低速原地旋转探索
        ↓
连续低协方差 AMCL 候选样本
        ↓
检查前方 LaserScan 扇区净空
        ↓
按 /odom 平移约 0.25 m
        ↓
平移期间保留新 AMCL 证据；不足时安全原地旋转补足样本
        ↓
发布零速度并静止确认
        ↓
清理 global_costmap 与 local_costmap，等待当前扫描刷新
        ↓
发送当前 waypoint
        ↓
成功：推进 waypoint
失败：按 max_retries 重试
        ↓
全部完成：成功退出
重试耗尽：失败退出
```

程序调用的是 AMCL 的全局重定位服务，而不是向 `/initialpose` 写入一个预先
配置的坐标。AMCL 会在地图的可行驶区域中初始化粒子，并结合 LaserScan、地图
和里程计逐渐收敛。由于全局重定位服务本身不会驱动机器人，程序会在定位阶段
通过 `/cmd_vel_nav` 旋转。探索过程中机器人自身的 yaw 会变化，因此自动全局
定位累计的是连续低协方差样本，而不是相邻 yaw 不变的样本。

第一次连续低协方差只表示“存在可信候选”，不能排除地图相似区域中的误定位。
因此程序起步时要求前方扇区每个有效量测都大于“最小安全距离 + 验证平移距离”；
当前配置为 `0.45 + 0.25 = 0.70 m`。平移中仍以 `0.45 m` 作为紧急停止下限；
若净空下降，机器人停止后会旋转寻找新的安全验证方向，而不是直接退出。平移距离
由 `/odom` 实际位置计算，不按速度乘时间估算；扫描无有效量测、里程计过期或平移
超时时，都会立即发布零速度并以失败终止，绝不带着未完成的验证去导航。
平移开始前会清除首次候选的 AMCL 样本；平移和后续安全旋转产生的连续低协方差
样本共同构成二次确认依据。停止后只确认这组结果仍然新鲜且低协方差，不假设
AMCL 会在机器人静止时继续发布位姿。过期或未来时间戳不会触发导航；收到过期
AMCL 后，稳定样本计数和置信度会被清零。

二次确认成功后，`CostmapCleaner` 会分别调用
`/global_costmap/clear_entirely_global_costmap` 和
`/local_costmap/clear_entirely_local_costmap`。这只清除定位阶段可能遗留的动态观测，
不会删除静态地图；刷新等待结束后，costmap 会根据当前 `/scan` 重新生成障碍和
膨胀层。任一服务不可用、异常或超过 `costmap_clear_timeout_sec` 时，巡检不会启动。

`/reinitialize_global_localization` 不能保证瞬间得到唯一位置。如果地图有对称
区域，或者机器人没有获得足够的扫描变化，AMCL 可能在超时时间内无法收敛；此时
程序会失败退出，而不会带着未知或低置信度的位置发送导航目标。

## 主要参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `goal_frame` | `map` | waypoint 和初始位姿使用的坐标系 |
| `amcl_pose_topic` | `/amcl_pose` | AMCL 位姿 Topic |
| `scan_topic` | `/scan` | LaserScan Topic |
| `odom_topic` | `/odom` | 平移验证使用的里程计 Topic |
| `automatic_global_localization` | `true` | 是否调用 AMCL 全局重定位服务 |
| `localization_timeout_sec` | `120.0` | AMCL 服务和定位收敛总超时时间，单位 s |
| `localization_position_variance_threshold` | `0.25` | x/y 方差上限，单位 m² |
| `localization_yaw_variance_threshold` | `0.10` | yaw 方差上限，单位 rad² |
| `localization_exploration_enabled` | `true` | 是否在定位阶段自动旋转探索 |
| `localization_cmd_vel_topic` | `/cmd_vel_nav` | 定位探索速度输入 Topic |
| `localization_exploration_angular_speed` | `0.20` | 探索角速度，单位 rad/s，不能为 0 |
| `localization_exploration_max_duration_sec` | `40.0` | 自动探索最大持续时间，单位 s |
| `localization_exploration_linear_speed` | `0.06` | 平移验证线速度，单位 m/s，必须大于 0 |
| `localization_exploration_translation_distance` | `0.25` | 使用 odom 测量的验证距离，单位 m |
| `localization_exploration_translation_timeout_sec` | `8.0` | 验证平移最大时间，单位 s |
| `localization_exploration_min_front_clearance` | `0.45` | 前方有效激光必须大于的安全距离，单位 m |
| `localization_exploration_front_sector_half_angle` | `0.35` | 前方安全检查扇区半角，单位 rad，范围 `(0, pi]` |
| `localization_settle_duration_sec` | `1.0` | 停止旋转后的 AMCL 最终确认时间，单位 s |
| `costmap_clear_timeout_sec` | `5.0` | 等待并完成两张 costmap 清理的总超时，单位 s |
| `costmap_clear_settle_duration_sec` | `2.0` | 清理完成后等待当前扫描刷新 costmap 的时间，单位 s |
| `global_costmap_clear_service` | `/global_costmap/clear_entirely_global_costmap` | 全局代价地图清理服务 |
| `local_costmap_clear_service` | `/local_costmap/clear_entirely_local_costmap` | 局部代价地图清理服务 |
| `use_manual_initial_pose_fallback` | `false` | 是否使用已知坐标发布 `/initialpose` 的兼容模式 |
| `max_retries` | `1` | 单个 waypoint 失败后的最大重试次数 |
| `stable_amcl_samples` | `5` | 需要连续稳定的 AMCL 样本数 |
| `stable_position_tolerance` | `0.05` | 位置容差，单位 m |
| `stable_yaw_tolerance` | `0.10` | yaw 容差，单位 rad |
| `max_message_age_sec` | `2.0` | 消息允许的最大年龄，单位 s |
| `future_message_tolerance_sec` | `0.5` | 消息允许超前当前时间的范围，单位 s |
| `initial_pose_x/y/yaw` | `0/0/0` | 初始位姿 |
| `initial_pose_publish_count` | `5` | 初始位姿发布次数 |
| `initial_pose_publish_period_sec` | `1.0` | 发布周期，单位 s |
| `initial_pose_wait_sec` | `3.0` | 发布完成后的等待时间，单位 s |
| `waypoint_1/2/3` | `[0, 0, 0]` | `[x, y, yaw]`，yaw 单位 rad |

参数会在节点启动时校验。空 Topic、负重试次数、非法容差、非有限浮点数、
错误 waypoint 数组长度以及同时启用或关闭两种定位模式，都会导致节点启动失败。

默认使用自动全局定位时，`initial_pose_x/y/yaw` 不参与定位。只有将
`use_manual_initial_pose_fallback` 设置为 `true` 时，程序才会重复发布这些坐标。

## Nav2 平滑调参

`src/config/nav2_waffle.yaml` 默认保留当前 TurtleBot3 `waffle` 的 DWB 参数，
仅显式补充 `velocity_smoother`，并让它的限制与 DWB 保持一致。这样不会擅自改变
已经完成三点巡检的速度规划，同时为后续调节提供唯一入口。

| 调节目标 | 需要同步修改的字段 |
| --- | --- |
| 降低直线速度 | `FollowPath.max_vel_x`、`max_speed_xy`、`velocity_smoother.max_velocity[0]` |
| 降低转向突兀感 | `FollowPath.max_vel_theta`、`velocity_smoother.max_velocity[2]` |
| 降低起步和刹停冲击 | `acc_lim_*`、`decel_lim_*`、`velocity_smoother.max_accel/max_decel` |
| 提升速度输出连续性 | `velocity_smoother.smoothing_frequency` |

不要只提高 `velocity_smoother` 的速度或加速度上限。平滑器的上限应当不超过 DWB，
否则会使两层约束含义不一致。建议每次只改变一组参数，并在
`TurtleBot3 waffle + turtlebot3_world` 中重新验证三条完整导航路径、目标点转向和
障碍物绕行过程。

## 构建

```bash
cd /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select auto_patrol --symlink-install
source install/setup.bash
```

`--symlink-install` 便于开发阶段迭代；每次修改 `src/config`、`src/launch` 或
工作区根目录 `map/` 后，仍应重新执行一次构建。普通安装会复制地图资源，重新构建
才能让 `install/auto_patrol/share/auto_patrol/map/` 与源码保持一致。

## 测试

测试不依赖真实机器人、摄像头、LaserScan 发布器或 Nav2 Action Server：

```bash
cd /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select auto_patrol
colcon test-result --verbose
```

测试覆盖参数校验、角度转换、时间戳检查、AMCL 稳定样本、主动旋转下的连续
低协方差样本、过期定位清零、前方净空拒绝策略以及 odom 平面距离计算。
`CostmapCleaner` 使用本地 Fake 服务验证“双服务均成功才放行”和“服务不可用则
超时失败”两条契约；Launch 结构测试还会确认根目录地图 YAML 存在且其 `image`
文件可用。真实 Nav2 与 Gazebo 的端到端效果仍需在下面的集成步骤中验证。

## 运行

### 仅启动自动巡检

当 Nav2 已经通过其他终端启动时，使用：

```bash
source /opt/ros/humble/setup.bash
source /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws/install/setup.bash
ros2 launch auto_patrol auto_patrol.launch.py use_sim_time:=true
```

### 一条命令启动完整仿真与巡检

完成构建并执行 `source install/setup.bash` 后，只需在一个终端运行：

```bash
source /opt/ros/humble/setup.bash
source /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws/install/setup.bash
ros2 launch auto_patrol patrol_with_nav2.launch.py
```

组合 Launch 会依次纳入官方 `turtlebot3_world`、官方 Nav2、RViz 和
`auto_patrol_node`。它会设置 `TURTLEBOT3_MODEL=waffle`，以默认位置
`(-2.0, -0.5)` 生成机器人，使用本包 `nav2_waffle.yaml` 启动 Nav2，并默认加载
本项目 `map/map.yaml`。
巡检节点自身等待 `/navigate_to_pose`、`/scan` 与 AMCL 数据，因此不依赖容易失效的
固定启动延迟。

Gazebo 与官方 Nav2 Launch 会并行启动，因此 RViz 可能先于 Gazebo 窗口出现；
这不影响巡检，因为节点会等待 `/navigate_to_pose`、`/scan` 与 AMCL 数据就绪。
巡检节点在三点任务完成、超时或发生不可恢复错误时只结束自身进程，Gazebo、Nav2
和 RViz 会继续保留，便于检查最终机器人位置、路径和代价地图。需要结束整套仿真时，
在该终端按 `Ctrl-C` 即可；不要额外启动第二个 Gazebo 或 RViz。

若需要临时使用其他地图，可覆盖 `map` 参数。下面的命令使用当前工作区根目录的地图，
与默认行为等价，也适合排查安装后的路径问题：

```bash
cd /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws
ros2 launch auto_patrol patrol_with_nav2.launch.py \
  map:="$PWD/map/map.yaml"
```

启动后可在另一个终端确认地图已经由 map_server 发布：

```bash
source /opt/ros/humble/setup.bash
source /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws/install/setup.bash
ros2 topic echo /map --once
ros2 param get /map_server yaml_filename
```

也可以在调试出生位置时覆盖世界坐标：

```bash
ros2 launch auto_patrol patrol_with_nav2.launch.py \
  x_pose:=-2.0 \
  y_pose:=-0.5
```

### 直接运行节点

不使用 Launch 时，下面的原始命令仍然可用，但需要自行保证 Nav2 与节点都启用了
同一个 `use_sim_time` 值：

```bash
source /opt/ros/humble/setup.bash
source /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws/install/setup.bash
ros2 run auto_patrol auto_patrol_node
```

上面的默认流程会自动调用：

```text
/reinitialize_global_localization
类型：std_srvs/srv/Empty
```

可以通过 ROS2 参数覆盖默认航点，例如：

```bash
ros2 run auto_patrol auto_patrol_node --ros-args \
  -p use_sim_time:=true \
  -p waypoint_1:='[0.0, 0.0, 0.0]' \
  -p waypoint_2:='[1.0, 0.0, 0.0]' \
  -p waypoint_3:='[1.0, 1.0, 1.57]'
```

只有在明确知道机器人初始坐标、或需要调试 AMCL 时，才使用手动兼容模式：

```bash
ros2 run auto_patrol auto_patrol_node --ros-args \
  -p automatic_global_localization:=false \
  -p use_manual_initial_pose_fallback:=true \
  -p initial_pose_x:=0.0 \
  -p initial_pose_y:=0.0 \
  -p initial_pose_yaw:=0.0
```

## 生命周期和线程安全契约

- `PatrolController` 和 `InitialPosePublisher` 的 `stop()` 可重复调用；
- 组件析构时先取消 Timer 和 Action，再释放相关资源；
- `LocalizationMonitor` 用 mutex 保护传感器状态、AMCL 样本和稳定计数；
- mutex 不覆盖 Publisher、Action、Timer 等 ROS2 慢速或外部调用；
- 全局定位服务只请求一次，服务 Future 在组件停止后不会再推进状态机；
- 每次全局重定位前清除旧 AMCL 样本，收敛必须使用新的传感器数据；
- 手动初始位姿模式要求连续位姿稳定；自动全局定位允许探索期间的正常旋转，
  但要求 `/scan`、`/amcl_pose` 新鲜且连续低协方差；
- 自动探索有最大持续时间；在此时间内没有得到首次低协方差候选时直接失败，避免
  未验证定位进入导航。成功、失败、总超时、停止和析构路径都会取消 Timer 并发布
  零速度，避免定位状态机退出后机器人继续旋转或继续前进；
- 首次候选定位必须经过前方激光安全检查、odom 实测平移和二次稳定样本确认；
  平移前样本会被明确清除，不能跨越该验证边界复用；
- `CostmapCleaner` 同时等待两张代价地图的清理响应。任一失败会阻止 PatrolController
  发送第一个 Action 目标，服务回调、等待 Timer 和停止路径均可重复处理；
- Action 回调使用 waypoint index 和 generation 校验，旧目标不能影响新目标；
- 正常完成返回退出码 `0`，导航失败返回退出码 `1`；
- Ctrl-C 等外部停止由 ROS2 负责结束 spin，不伪装成导航成功。

## 验收录制建议

1. 从一个干净终端执行“**一条命令启动完整仿真与巡检**”中的命令，并从该时刻开始录制。
2. 画面中保留 Gazebo 与 RViz；不要点击 `2D Pose Estimate` 或 `Navigation2 Goal`。
3. 记录自动 AMCL 定位、三个目标点的连续到达、至少一段绕障路径，以及终端的
   `Patrol completed successfully: all 3 goals reached.`。
4. 任务完成后，Gazebo、Nav2 和 RViz 会保留以便检查路径、TF 和代价地图；按启动
   Launch 的终端执行 `Ctrl-C` 才会关闭整套仿真。

提交 GitHub 前请将地图资源一起纳入版本控制：

```bash
cd /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3
git add turbot_ws/map/map.yaml turbot_ws/map/map.png turbot_ws/map/map.pgm
git status --short
```
