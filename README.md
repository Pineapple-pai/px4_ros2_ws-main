# PX4 ROS 2 局部自主飞行

项目主链使用 PX4 EKF 融合光流，ROS 2 发送局部目标并做近场绕行。系统不建图、不使用全局规划，也不依赖 LiDAR、FAST-LIO、Nav2 或 Ego-Planner。

光流只提供运动估计，不检测障碍物。外接 ToF、超声或视觉测距节点负责发布前、左、右三个方向的有效距离。

## 项目结构

- `src/px4_msgs`：PX4 ROS 2 消息。
- `src/px4_ros2_interface_lib`：PX4 ROS 2 控制接口。
- `src/px4_autonomy_mode`：PX4 自定义自主模式和局部避障逻辑。
- `src/px4_autonomy_bringup`：唯一的 ROS 2 启动入口。
- `scripts`：PX4 仿真、局部自主启动和仿真清理。
- `docs/LOCAL_AUTONOMY_ARCHITECTURE.md`：坐标、话题和运行方式。
- `docs/REAL_FLIGHT_CHECKLIST.md`：上机前检查。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/p/下载/px4_ros2_ws-main
colcon build --packages-up-to px4_autonomy_bringup
source install/setup.bash
```

## 运行

先启动 PX4 和 Micro XRCE-DDS Agent。PX4 必须启用光流驱动及 EKF 融合，并提供有效本地位置。启动自主模式：

```bash
./scripts/start_local_autonomy.sh
```

模式等待单个局部目标，不会生成方形路线或多航点任务。先用 PX4 原生模式人工解锁、起飞并稳定悬停，再切入 `ROS2 Autonomy`。

发送 PX4 本地 NED 目标，单位米，z 轴向下为正：

```bash
ros2 topic pub --once /autonomy/target_ned geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'px4_local_ned'}, point: {x: 1.0, y: 0.0, z: -1.5}}"
```

到达目标后模式结束。下一个目标需重新发布并切入自主模式。

## 障碍距离输入

外接节点以 `std_msgs/msg/Float32` 发布米制距离：

- `/perception/front_obstacle_distance`
- `/perception/left_obstacle_distance`
- `/perception/right_obstacle_distance`

三个话题都需持续发布正值有效数据；缺失或过期时自主模式保持当前位置。前方距离触发减速/绕行/停止，左右距离用于选择绕行侧。

## 仿真与安全

`scripts/start_px4_sim.sh` 启动 PX4 SITL、Gazebo、Micro XRCE-DDS Agent 和 QGroundControl。使用圆圆机自定义模型时运行：

```bash
VEHICLE_PROFILE=yuanyuan_quad ./scripts/start_px4_sim.sh
```

仿真就绪后另开终端运行 `./scripts/start_local_autonomy.sh`。自定义模型文件位于 `sim/models/yuanyuan_quad`，机型 profile 和 PX4 参数位于 `sim/vehicles/yuanyuan_quad.*`。首次使用前需在 PX4 固件中注册对应 airframe；当前 profile 暂用 PX4 `quad_x`（autostart 4001）完成初步验证。

自主模式不解锁。保留 RC/QGC 对 PX4 原生 `Position`、`Land` 和 `RTL` 的接管能力；定位、测距或控制异常时切回原生模式。
