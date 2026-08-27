# TurtleBot3 `waffle` 自动巡检

> ROS 2 Humble | `turtlebot3_world` | Nav2 | 三目标点自主巡检

`auto_patrol` 是一个面向 TurtleBot3 的 ROS2 自动巡航节点。默认定位流程先将当前
`/scan` 的障碍端点与 `/map` 的静态障碍物做全图几何匹配，只有最优候选误差足够小、
且明显优于空间分离的其他候选时，才把这个不依赖 AMCL 的位姿发布到 `/initialpose`。
随后节点调用 AMCL 的 `/request_nomotion_update`，在连续新鲜、低协方差的
`/amcl_pose` 样本和 `map -> odom` TF 均就绪后，静止确认一次定位结果，再依次执行
三个 waypoint。

本包不会在启动定位时向 `/cmd_vel` 或 `/cmd_vel_nav` 发布旋转、平移或零速度指令。
因此不会通过原地自转“制造” AMCL 更新；实际的几何一致性由扫描匹配和 AMCL 二次复核
共同保证。关闭扫描匹配后，才回退到 AMCL 的 `/reinitialize_global_localization` 服务。
确认后程序会清理全局和局部代价地图，等待其根据当前扫描重新生成障碍与膨胀层。
默认流程不要求在 RViz 中手动点击 `2D Pose Estimate`，也不要求预先填写机器人
出生位置。

## 作业目标与已实现能力

作业统一环境为 ROS 2 Humble、TurtleBot3 `waffle` 与 `turtlebot3_world`。项目以
`NavigateToPose` Action 向 Nav2 提交三个 `map` 坐标系目标点，而不是绕过 Nav2
直接控制机器人速度；规划、避障、恢复和速度平滑仍由 Nav2 负责。

| 作业要求或加分方向 | 本项目实现 |
| --- | --- |
| 三个具有空间跨度的目标点 | `patrol.yaml` 中的 `waypoint_1/2/3`；三点均使用 `[x, y, yaw]`（m、m、rad）配置。 |
| 避开环境障碍物 | 目标通过 Nav2 全局规划、局部 DWB 控制器、代价地图和恢复行为执行，不由巡检节点硬编码路径。 |
| 无需 RViz 手动发送目标 | `PatrolController` 仅在上一个 `NavigateToPose` 返回 `SUCCEEDED` 后发送下一点。 |
| 无需手动设置初始位姿 | 默认全图匹配 `/scan` 与 `/map` 后发布计算出的 `/initialpose`，再由 AMCL 无运动更新确认；不向速度 Topic 发送定位动作。 |
| 少量命令启动 | `patrol_with_nav2.launch.py` 一次启动 Gazebo 世界、Nav2、RViz 和巡检节点。 |
| 导航失败 Recovery | Nav2 负责路径无效、局部障碍等恢复；巡检节点对 `ABORTED` 或 `CANCELED` 的目标按 `max_retries` 重试。 |
| 运动稳定性调节入口 | `nav2_waffle.yaml` 集中保存 DWB 与 `velocity_smoother` 参数。 |

开发过程曾得到三点均返回 `Goal reached`、最终输出
`Patrol completed successfully: all 3 goals reached.` 的运行结果

## 组件职责

- `AutoPatrolNode`：组装组件、创建 ROS2 通信资源、驱动 Timer 和编排流程；
- `PatrolConfig`：声明、读取和校验参数；
- `pose_utils`：提供无副作用的角度、姿态和时间戳计算；
- `LocalizationMonitor`：检查 `/scan` 时间戳以及 AMCL 定位稳定性；
- `ScanMapMatcher`：保存静态地图和 LaserScan 快照，构造地图障碍距离场并执行粗到细的
  全图位姿搜索；它不依赖 AMCL，因此能发现“低协方差但点云未对齐”的错误局部模式；
- `LocalizationBootstrapper`：编排扫描匹配、`/initialpose` 发布、AMCL 无运动更新、
  匹配结果复核和 `map -> odom` TF 检查，所有等待都由 Timer 驱动；
- `CostmapCleaner`：异步调用 Nav2 的全局、局部代价地图清理服务，并等待刷新窗口；
- `InitialPosePublisher`：按固定次数发布扫描匹配或手动兼容模式得到的 `/initialpose`；
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
等待静态 /map、当前 /scan 和 base_footprint -> scan TF
        ↓
在全图粗搜索并细化 LaserScan 到静态障碍物的匹配位姿
        ↓
最优误差与次优空间分离候选的误差间隔是否达标？
        ↓
否：继续等待新扫描，超时失败
是：重复发布计算得到的 /initialpose
        ↓
清除发布前的 AMCL 样本，等待 /request_nomotion_update 服务
        ↓
定时请求无运动 AMCL 更新（不发布 Twist）
        ↓
连续新鲜、低协方差的 /amcl_pose，并复核其与扫描匹配结果一致
        ↓
确认 map -> odom TF，保持静止并完成最终确认
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

扫描匹配会把每个有效 LaserScan 端点变换到待评估的 `map -> base_footprint` 位姿，
并计算端点到静态地图最近障碍物的平均距离。它还会均匀抽取部分激光束，检查从
真实 LaserScan 原点到端点前约 1.5 个地图栅格的路径是否始终处于已知自由空间；
若候选要求激光穿过静态墙体或未知区域，就按每束固定惩罚加入总误差。这样可以排除
“端点看似都贴墙、但中间路径不可能存在”的相似位置。搜索先用较大平移、角度步长
覆盖全图，再细化误差最低的一组候选；这避免把所有细粒度候选一次性展开。最优误差超过
`localization_scan_match_max_mean_error_m`，或最优解相对另一空间分离候选的优势小于
`localization_scan_match_min_margin_m` 时，节点拒绝发布 `/initialpose`，不会凭 AMCL
低协方差直接启动巡检。

匹配成功后，`InitialPosePublisher` 将计算出的候选重复发布给 AMCL。程序随后周期性
调用 `/request_nomotion_update`，要求 AMCL 使用当前静态 LaserScan 更新粒子滤波器。
同一时刻最多保留一个未完成服务请求；AMCL 新样本不仅必须低协方差、时间新鲜和连续
稳定，还必须在位置、朝向和扫描误差上与扫描匹配候选相符。若 AMCL 又回到不一致的
局部模式，节点最多按 `localization_scan_match_max_initial_pose_retries` 重发候选，超过
次数即失败退出。

这不是对任何地图都能从单帧扫描唯一定位的数学保证：完全对称、障碍极少或可见范围
不足时，即使加入自由空间约束，候选间隔仍会不足，程序应当拒绝而不是错误出发。此时应
改变起始区域、增加环境特征或使用更可靠的初始位姿来源；不要把
`localization_scan_match_min_margin_m` 降到小于地图分辨率的数值（例如 `0.003 m`）来
强行放行。过期或未来时间戳也会清零稳定计数并阻止导航。

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
| `localization_map_topic` | `/map` | 扫描匹配使用的静态 OccupancyGrid Topic |
| `localization_base_frame` | `base_footprint` | 输出扫描匹配位姿的机体坐标系；必须存在到 LaserScan frame 的 TF |
| `automatic_global_localization` | `true` | 是否执行自动定位流程；扫描匹配关闭时才调用 AMCL 全局重定位服务 |
| `localization_timeout_sec` | `120.0` | 扫描匹配、AMCL 服务和最终确认共享的总超时时间，单位 s |
| `localization_position_variance_threshold` | `0.25` | x/y 方差上限，单位 m² |
| `localization_yaw_variance_threshold` | `0.10` | yaw 方差上限，单位 rad² |
| `localization_global_localization_service` | `/reinitialize_global_localization` | 请求 AMCL 在全图重新分布粒子的服务名 |
| `localization_nomotion_update_service` | `/request_nomotion_update` | 请求 AMCL 使用当前静态 LaserScan 更新的服务名 |
| `localization_nomotion_update_period_sec` | `0.5` | 两次无运动更新请求的最小间隔，单位 s，范围 `[0.05, +inf)` |
| `localization_odom_frame` | `odom` | 成功前必须存在的 TF 源坐标系；与 `goal_frame` 共同构成 `map -> odom` |
| `localization_settle_duration_sec` | `1.0` | AMCL 与 TF 就绪后保持静止的最终确认时间，单位 s |
| `localization_scan_match_enabled` | `true` | 是否启用独立的全图 LaserScan-静态地图匹配；推荐保持启用 |
| `localization_scan_match_coarse_step_m` | `0.20` | 全图粗搜索平移步长，单位 m；越小越精确但启动计算更慢 |
| `localization_scan_match_coarse_yaw_step_rad` | `0.261799` | 全图粗搜索朝向步长，单位 rad |
| `localization_scan_match_refine_step_m` | `0.05` | 候选细化平移步长，单位 m |
| `localization_scan_match_refine_yaw_step_rad` | `0.034907` | 候选细化朝向步长，单位 rad |
| `localization_scan_match_max_beams` | `90` | 参与匹配的有效激光端点上限；超过时均匀抽样 |
| `localization_scan_match_max_range_m` | `3.0` | 参与匹配的最大有效测距，单位 m |
| `localization_scan_match_free_space_max_beams` | `12` | 均匀抽取并检查束中段自由空间的最大束数，范围 `[3, +inf)`；增大可提高歧义排除能力，但会增加全图搜索时间 |
| `localization_scan_match_free_space_penalty_m` | `0.25` | 一束激光中段穿过静态占据或未知栅格时加入的平均误差，单位 m；必须为正数 |
| `localization_scan_match_max_mean_error_m` | `0.10` | 接受候选的最大平均障碍距离，单位 m；不应仅为减少拒绝而随意增大 |
| `localization_scan_match_min_margin_m` | `0.05` | 最优候选相对空间分离次优候选所需的最小误差优势，单位 m |
| `localization_scan_match_min_separation_m` | `0.50` | 判断两个位置候选不同的最小平移差，单位 m |
| `localization_scan_match_min_yaw_separation_rad` | `0.523599` | 判断两个朝向候选不同的最小角度差，单位 rad |
| `localization_scan_match_pose_tolerance_m` | `0.20` | AMCL 相对匹配候选允许的位置偏差，单位 m |
| `localization_scan_match_yaw_tolerance_rad` | `0.20` | AMCL 相对匹配候选允许的朝向偏差，单位 rad |
| `localization_scan_match_max_initial_pose_retries` | `1` | AMCL 与匹配结果不一致时可重发 `/initialpose` 的次数 |
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
cd TurtleBot3/turbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select auto_patrol --symlink-install
source install/setup.bash
```

`--symlink-install` 便于开发阶段迭代；每次修改 `src/config`、`src/launch` 或
工作区根目录 `map/` 后，仍应重新执行一次构建。普通安装会复制地图资源，重新构建
才能让 `install/auto_patrol/share/auto_patrol/map/` 与源码保持一致。

## 测试

测试不依赖真实机器人、LaserScan 发布器或 Nav2 Action Server：

```bash
cd TurtleBot3/turbot_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select auto_patrol
colcon test-result --verbose
```

测试覆盖参数校验、全图 LaserScan-静态地图匹配、无运动服务状态机、角度转换、
时间戳检查、AMCL 稳定样本、过期定位清零和低协方差门限。
`CostmapCleaner` 使用本地 Fake 服务验证“双服务均成功才放行”和“服务不可用则
超时失败”两条契约；Launch 结构测试还会确认根目录地图 YAML 存在且其 `image`
文件可用。真实 Nav2 与 Gazebo 的端到端效果仍需在下面的集成步骤中验证。

## 运行

### 仅启动自动巡检

当 Nav2 已经通过其他终端启动时，使用：

```bash
cd TurtleBot3/turbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch auto_patrol auto_patrol.launch.py use_sim_time:=true
```

### 一条命令启动完整仿真与巡检

完成构建并执行 `source install/setup.bash` 后，只需在一个终端运行：

```bash
cd TurtleBot3/turbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
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
cd TurtleBot3/turbot_ws
ros2 launch auto_patrol patrol_with_nav2.launch.py \
  map:="$PWD/map/map.yaml"
```

启动后可在另一个终端确认地图已经由 map_server 发布：

```bash
cd TurtleBot3/turbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
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
cd TurtleBot3/turbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run auto_patrol auto_patrol_node
```

默认流程会订阅 `/map`、`/scan` 并向 `/initialpose` 发布扫描匹配得到的候选，随后自动调用：

```text
/request_nomotion_update
类型：std_srvs/srv/Empty
```

`/request_nomotion_update` 是 AMCL 显式提供的无运动更新入口。本包不会把
`update_min_a`、`update_min_d` 改为零或负数来模拟该行为；保持 AMCL 的正常更新阈值，
由服务请求一次明确的静止更新更符合其接口语义。

若因调试需要关闭 `localization_scan_match_enabled`，自动模式才会先调用
`/reinitialize_global_localization`，再执行同样的无运动更新。这条兼容路径不具备
扫描匹配的错误位置排除能力，不是推荐配置。

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
- `ScanMapMatcher` 用独立 mutex 保护静态地图和扫描快照；距离场和全图搜索在锁外对
  本地副本执行，不阻塞 ROS2 订阅回调；
- mutex 不覆盖 Publisher、Action、Timer 等 ROS2 慢速或外部调用；
- 扫描匹配开启时不会请求 AMCL 全局定位服务；关闭时该服务只请求一次，服务 Future
  在组件停止后不会再推进状态机；
- 每次发布匹配初始位姿或请求全局重定位前都会清除旧 AMCL 样本，收敛必须使用后续
  新传感器数据；
- 自动全局定位定时异步请求 `/request_nomotion_update`，同一时刻最多一个请求在途；
  成功、失败、总超时、停止和析构路径都会取消 Timer，且定位组件从不创建速度发布器；
- 自动定位要求扫描匹配候选通过误差与唯一性门限、`/scan` 和 `/amcl_pose` 新鲜且连续
  低协方差、AMCL 与候选一致，并确认 `map -> odom` TF；端到端验证仍应在 RViz 中观察
  LaserScan 与地图是否重合；
- `CostmapCleaner` 同时等待两张代价地图的清理响应。任一失败会阻止 PatrolController
  发送第一个 Action 目标，服务回调、等待 Timer 和停止路径均可重复处理；
- Action 回调使用 waypoint index 和 generation 校验，旧目标不能影响新目标；
- 正常完成返回退出码 `0`，导航失败返回退出码 `1`；
- Ctrl-C 等外部停止由 ROS2 负责结束 spin，不伪装成导航成功。
