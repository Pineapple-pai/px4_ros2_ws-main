# PX4 光流定位与局部规划架构

## 目标

主运行链使用 PX4 EKF 提供的本地位置与航向。光流传感器由 PX4 驱动并融合，ROS 2 通过 PX4 ROS 2 接口读取 `/fmu/out/vehicle_local_position_v1` 对应的本地状态。主链不启动激光雷达、FAST-LIO、点云建图、OctoMap、Nav2 或全局规划器。

```text
光流传感器 ──> PX4 EKF ──> PX4 ROS 2 Interface ──> px4_autonomy_mode
                                                   ↑       │
局部目标 /autonomy/target_ned ─────────────────────┘       │ PX4 GotoSetpoint
                                                          ↓
                                                     PX4 飞控

外接近场测距节点 ──> /perception/{front,left,right}_obstacle_distance
                              └──> 临时绕行点 / 停止 / 中止
```

光流用于估计运动，不提供障碍物位置或距离。因此局部避障距离由外接 ToF、超声或视觉测距节点提供；当前接口只定义 ROS 2 话题，不绑定具体传感器。

## 主要行为

- 自主模式启动后等待单个运行时目标，不会自动生成方形航线或多航点任务。
- 目标到达后当前模式正常结束；下一个局部目标由上层再发布并重新切入自主模式。
- 目标通过 `/autonomy/target_ned` 发布，坐标为 PX4 本地 NED：x 北、y 东、z 向下。目标高度例如离地 1.5 米应使用 `z: -1.5`。
- 目标附近的前向测距触发局部绕行；左右测距用来选择较空的一侧。无法绕行时按停止距离保持，达到紧急距离或保持超时则中止模式。
- `local_autonomy.launch.py` 要求前、左、右三个距离话题都有新鲜有效数据才会移动；缺少或过期时保持当前位置。
- PX4 继续负责姿态/位置控制、状态估计和模式切换。自主模式不负责解锁；人工仍可切回 PX4 原生模式。

这是近场目标跟随和临时绕行，不维护环境地图，也不生成跨区域全局路线。

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_autonomy_bringup local_autonomy.launch.py
```

也可用 `scripts/start_local_autonomy.sh`，它会先等待 PX4 本地位置话题出现。

局部目标示例：

```bash
ros2 topic pub --once /autonomy/target_ned geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'px4_local_ned'}, point: {x: 1.0, y: 0.0, z: -1.5}}"
```

距离节点默认发布以下 `std_msgs/msg/Float32` 话题，单位米：

- `/perception/front_obstacle_distance`
- `/perception/left_obstacle_distance`
- `/perception/right_obstacle_distance`

距离话题必须表达相对机体前、左、右方向的最近有效距离，正值表示有效测量。数据缺失或超时会阻止局部目标执行。可通过 launch 参数修改话题名和距离阈值。

## PX4 配置前提

PX4 必须已启用光流传感器驱动，并由 EKF 融合光流和所需的测距/高度来源。切入 ROS 2 模式前，应在 QGC 确认 PX4 本地位置有效且稳定。ROS 2 直接使用 PX4 本地 NED 状态，不运行第二套 FAST-LIO 定位。

## 项目入口

仿真由 `scripts/start_px4_sim.sh` 启动；局部自主模式由 `scripts/start_local_autonomy.sh` 单独启动。外接距离传感器需要由对应 ROS 2 节点发布上述三个距离话题。
