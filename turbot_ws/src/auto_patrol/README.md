# auto_patrol

`auto_patrol` 是一个面向 TurtleBot3 的 ROS2 自动巡航节点。节点默认请求
Nav2 AMCL 进行全局重定位，在确认传感器和定位状态有效后，依次执行三个
waypoint。定位阶段会自动通过 `/cmd_vel_nav` 低速原地旋转，主动获取不同方向
的激光观测；达到置信度或探索上限后，会先停止旋转并等待最后一次 AMCL 更新。
默认流程不要求在 RViz 中手动点击 `2D Pose Estimate`，也不要求预先填写机器人
出生位置。

## 组件职责

- `AutoPatrolNode`：组装组件、创建 ROS2 通信资源、驱动 Timer 和编排流程；
- `PatrolConfig`：声明、读取和校验参数；
- `pose_utils`：提供无副作用的角度、姿态和时间戳计算；
- `LocalizationMonitor`：检查 `/scan` 时间戳以及 AMCL 定位稳定性；
- `LocalizationBootstrapper`：调用 `/reinitialize_global_localization`，清除旧定位
  样本，管理探索旋转、静止确认、超时和收敛状态机；
- `InitialPosePublisher`：仅在手动兼容模式下按配置重复发布 `/initialpose`；
- `PatrolController`：管理 Nav2 Action、waypoint 推进、失败重试和终态。

入口 Node 不直接承担参数校验、姿态计算、传感器稳定判断和 Action 重试，
这样可以在不启动真实机器人和 Nav2 的情况下测试核心逻辑。

## 配置与启动文件

本包按作业目录要求，将配置和 Launch 源文件放在 `src/` 下；构建后会安装到
`share/auto_patrol/config` 和 `share/auto_patrol/launch`，因此仍可使用标准的
`ros2 launch auto_patrol ...` 命令。

```text
src/
├── config/
│   ├── patrol.yaml          # waypoint、自动定位、超时和重试配置
│   └── nav2_waffle.yaml     # 完整 Nav2 参数副本及速度平滑器配置
└── launch/
    ├── auto_patrol.launch.py       # Nav2 已启动时，仅启动巡检节点
    └── patrol_with_nav2.launch.py  # 启动 Nav2 和自动巡检节点
```

`nav2_waffle.yaml` 基于当前 TurtleBot3 Humble 的
`turtlebot3_navigation2/param/humble/waffle.yaml` 完整复制。不能以一个只含少量
速度参数的 YAML 替代它，因为官方 Nav2 Launch 只接收一个 `params_file`；不完整
文件会遗漏 AMCL、costmap、planner 等节点的必需参数。

`patrol.yaml` 使用本项目已经成功完成三点巡检时的 waypoint 和定位等待时间。
它是推荐的启动配置；表中“默认值”仍表示不加载 YAML、直接 `ros2 run` 时 C++
节点内部声明的默认值。

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
连续低协方差 AMCL 样本
        ↓
发布零速度并静止确认最后一帧 AMCL
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
通过 `/cmd_vel_nav` 低速原地旋转。探索过程中机器人自身的 yaw 会变化，因此
自动全局定位累计的是连续低协方差样本，而不是相邻 yaw 不变的样本。探索达到
置信度或上限后，程序先发布零速度，再在短暂确认窗口内检查新鲜且低协方差的
AMCL 位姿。过期或未来时间戳不会触发导航；收到过期 AMCL 后，稳定样本计数和
置信度会被清零。

`/reinitialize_global_localization` 不能保证瞬间得到唯一位置。如果地图有对称
区域，或者机器人没有获得足够的扫描变化，AMCL 可能在超时时间内无法收敛；此时
程序会失败退出，而不会带着未知或低置信度的位置发送导航目标。

## 主要参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `goal_frame` | `map` | waypoint 和初始位姿使用的坐标系 |
| `amcl_pose_topic` | `/amcl_pose` | AMCL 位姿 Topic |
| `scan_topic` | `/scan` | LaserScan Topic |
| `automatic_global_localization` | `true` | 是否调用 AMCL 全局重定位服务 |
| `localization_timeout_sec` | `120.0` | AMCL 服务和定位收敛总超时时间，单位 s |
| `localization_position_variance_threshold` | `0.25` | x/y 方差上限，单位 m² |
| `localization_yaw_variance_threshold` | `0.10` | yaw 方差上限，单位 rad² |
| `localization_exploration_enabled` | `true` | 是否在定位阶段自动旋转探索 |
| `localization_cmd_vel_topic` | `/cmd_vel_nav` | 定位探索速度输入 Topic |
| `localization_exploration_angular_speed` | `0.20` | 探索角速度，单位 rad/s，不能为 0 |
| `localization_exploration_max_duration_sec` | `40.0` | 自动探索最大持续时间，单位 s |
| `localization_settle_duration_sec` | `1.0` | 停止旋转后的 AMCL 最终确认时间，单位 s |
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

`--symlink-install` 便于开发阶段迭代；每次修改 `src/config` 或 `src/launch` 后，
仍应重新执行一次构建，以确保安装空间包含当前配置。

## 测试

测试不依赖真实机器人、摄像头、LaserScan 发布器或 Nav2 Action Server：

```bash
cd /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select auto_patrol
colcon test-result --verbose
```

测试覆盖参数校验、角度转换、时间戳检查、AMCL 稳定样本、主动旋转下的连续
低协方差样本和过期定位清零。

## 运行

### 仅启动自动巡检

当 Nav2 已经通过其他终端启动时，使用：

```bash
source /opt/ros/humble/setup.bash
source /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws/install/setup.bash
ros2 launch auto_patrol auto_patrol.launch.py use_sim_time:=true
```

### 启动 Nav2 和自动巡检

先在终端 A 启动统一作业环境：

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

再在终端 B 启动本包的整合 Launch：

```bash
source /opt/ros/humble/setup.bash
source /home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws/install/setup.bash
ros2 launch auto_patrol patrol_with_nav2.launch.py use_sim_time:=true
```

整合 Launch 会使用 `src/config/nav2_waffle.yaml` 启动官方 Nav2，并自动启动
`auto_patrol_node`。节点会先等待 `/navigate_to_pose`，因此不需要人为添加启动延迟。
该方式仍需要 Gazebo 先运行，且官方 Nav2 Launch 会启动 RViz；不要额外再启动第二个
RViz，以免干扰定位和代价地图的观察。

若使用自己保存的地图，可覆盖 `map` 参数：

```bash
ros2 launch auto_patrol patrol_with_nav2.launch.py \
  use_sim_time:=true \
  map:=/home/ljq/map.yaml
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
- 自动探索有最大持续时间，达到上限后先进入静止确认而不是立刻失败；成功、
  失败、总超时、停止和析构路径都会取消 Timer 并发布零速度，避免定位状态机
  退出后机器人继续旋转；
- Action 回调使用 waypoint index 和 generation 校验，旧目标不能影响新目标；
- 正常完成返回退出码 `0`，导航失败返回退出码 `1`；
- Ctrl-C 等外部停止由 ROS2 负责结束 spin，不伪装成导航成功。
