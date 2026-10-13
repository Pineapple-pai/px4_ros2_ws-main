# SolidWorks 装配体转换为 PX4 仿真机型

本文说明如何把 SolidWorks 无人机装配体整理成项目可用的完整 PX4 SITL / Gazebo Classic 机型。目标是让仿真使用自定义整机几何、动力学和传感器配置，并把后续换机型操作收敛到一个机型名称/配置。

## 当前项目约定

项目的 `scripts/start_px4_sim.sh` 使用 Gazebo Classic。脚本从以下目录安装自定义模型：

```text
sim/models/<模型名>/
```

模型 SDF 文件名要和目录名一致：

```text
sim/models/my_quad/my_quad.sdf
```

启动脚本能复制模型目录并按机型 profile 选择 SDF、PX4 autostart ID 和参数文件。PX4 airframe 定义仍需存在于 PX4 固件中；若新增 airframe ID，还要按所用固件版本注册并构建。

## 最终文件结构

```text
sim/
├── models/
│   └── my_quad/
│       ├── model.config
│       ├── my_quad.sdf
│       └── meshes/
│           ├── frame_visual.stl
│           ├── rotor_visual.stl
│           └── sensor_visual.stl
└── vehicles/
    └── my_quad.env        # 机型选择、PX4 参数文件等 profile 信息
```

`model.config` 和 `my_quad.sdf` 是 Gazebo 模型文件；`my_quad.env` 是建议添加的项目级机型配置，供启动脚本选择 SDF 和 PX4 参数。PX4 固件内的 airframe 配置仍需按所用 PX4 版本安装/构建。

## 第 1 步：整理 SolidWorks 装配体

1. 打开完整总装配体，检查单位、比例、装配约束和零件位置。
2. 确认整机坐标系。建议将原点放在整机重心附近，并记录前、左、上方向。必须与采用的 Gazebo 基准模型坐标约定一致；不要把 PX4 的 NED/FRD 轴向和 Gazebo 的世界/模型轴向混为一谈。
3. 给零件指定合理材料密度。检查 SolidWorks 的质量属性，记录整机质量、重心位置和惯性矩阵。没有真实材料信息时，CAD 质量属性只能作为估值，需用实际称重校准。
4. 将下列零件分组并记录坐标：
   - 机架和主要载荷；
   - 每个电机与桨叶；
   - IMU（通常位于飞控板上）；
   - 下视光流模块和下视测距传感器；
   - 前、左、右近场测距传感器。
5. 隐藏或抑制螺钉、线缆、内部电路细节等小零件。视觉模型可保留外观；碰撞体应简化，避免直接使用高面数装配网格做碰撞检测。

## 第 2 步：从 SolidWorks 导出网格

对每个需要单独放置或运动的部件分别导出。第一版至少导出机架外观；旋翼需要独立旋转时，单独导出旋翼。

1. 在 SolidWorks 中选中目标零件/子装配体，使用 **文件 → 另存为**。
2. 选择 STL（`.stl`）；如需保留材质/颜色，可另行尝试 OBJ/DAE，但必须确认目标 Gazebo Classic 安装支持对应资源格式。
3. 在导出选项中指定单位和网格精度。推荐使用米制并从中等精度开始；不要不检查就使用极高精度。
4. 不要把所有可动部件合并成一个 STL。导出后用网格查看器检查：尺寸、朝向、原点和相对位置是否正确。
5. 如果 SolidWorks 导出的单个零件原点不在装配位置，记录它在总装坐标系中的平移/旋转，之后在 SDF 对应 `<visual><pose>` 中补上。

SolidWorks 导出的 STL 只包含三角网格，不包含 PX4 电机模型、传感器插件、质量惯量或飞控配置。

## 第 3 步：整理仿真所需的真实参数

仅有 CAD 几何不足以准确模拟飞行动力学。整理以下数据，保存为机型参数表：

| 类别 | 必需数据 |
|---|---|
| 刚体 | 整机质量、重心位置、惯性矩阵 `Ixx/Iyy/Izz/Ixy/Ixz/Iyz` |
| 电机布局 | 电机数量、每个电机相对重心的 XYZ 坐标、编号、旋向 |
| 电机/桨 | 桨直径/螺距、电机最大转速或推力曲线、推力/扭矩系数、响应时间 |
| 光流 | 安装坐标、朝向、视场/更新率/质量值配置（依传感器/仿真插件） |
| 下视测距 | 安装坐标、朝向、有效量程、更新率、噪声 |
| 避障测距 | 前/左/右传感器安装坐标、朝向、量程、更新率、ROS 话题映射 |
| PX4 | PX4 版本、机架类型、执行器分配、光流/测距 EKF 参数 |

电机推力、最大转速和响应时间应尽量来自电机/桨测试或厂家数据。未知参数可以先使用基准模型的估值启动仿真，但要明确标记为待校准，不能据此认为仿真已接近真实机体。

## 第 4 步：建立 Gazebo Classic 模型

创建目录和文件：

```text
sim/models/my_quad/model.config
sim/models/my_quad/my_quad.sdf
sim/models/my_quad/meshes/
```

`model.config` 的基本形式：

```xml
<?xml version="1.0"?>
<model>
  <name>my_quad</name>
  <version>1.0</version>
  <sdf version="1.6">my_quad.sdf</sdf>
  <description>Custom PX4 quadrotor model</description>
</model>
```

SDF 中至少要定义：

- `<model>`、机身 `<link>`、`<inertial>` 和质量/惯量；
- 用 SolidWorks 网格配置的 `<visual>`；
- 简化的 `<collision>`；
- 电机、旋翼 link/joint 及 PX4 Gazebo Classic 电机插件；
- IMU、光流和下视距离传感器插件；
- 若在仿真中验证局部避障，还要配置前/左/右距离数据源并接入项目订阅接口。

### 重要做法：从带光流的 PX4 Classic 模型移植插件

以已安装 PX4 版本中的 `iris_opt_flow` 或等效机型为参考，先复制其 SDF 和 `model.config`，再逐步替换几何和参数。保留并逐项理解以下部分：

- 电机插件的 `jointName`、电机编号、旋向和控制话题；
- IMU、光流和测距插件；
- 传感器相对机体的 pose；
- 模型名、文件名和资源 URI。

不要从空白 SDF 猜插件参数。Gazebo Classic 的插件与 PX4 版本耦合，应该以该 PX4 安装目录中的模型和插件为准。PX4 文档列出了带光流的 Classic 机型 `gazebo-classic_iris_opt_flow`。

### 碰撞几何和质量惯量

- 视觉网格用于外观，碰撞体优先用 box/cylinder 等基本体或低面数网格。
- `<inertial>` 中填写相对重心坐标系的质量和惯性矩阵；矩阵单位为 kg·m²。
- 不要只改视觉网格而忽略碰撞体、电机坐标和惯量，否则画面看起来是新飞机，飞行行为仍可能像旧飞机。

## 第 5 步：配置 PX4 机架和传感器参数

完整替换模型后，PX4 端需要与 SDF 对齐：

1. 为机型选择合适的 PX4 airframe/机架配置，并设置唯一的 `PX4_SYS_AUTOSTART`。
2. 配置控制分配器/执行器几何，使电机编号、位置、旋向与 SDF 插件一致。
3. 设置光流启用和 EKF 融合参数；配置下视测距源、量程和传感器相对机体中心的偏移。
4. 配置电机最大输出、旋转方向和相关动力学参数。
5. 将 airframe 配置加入当前 PX4 固件版本要求的构建/启动索引中。不同 PX4 版本的目录和启动方法可能不同，以当前 PX4 checkout 为准。

PX4 光流 EKF 需要有效的测距数据。这里需要的是向下的高度/距离测量，可以使用 ToF 或超声；这不要求使用扫描环境的 LiDAR。前、左、右避障距离则是局部避障另外需要的数据源。

## 第 6 步：统一机型选择入口

为每种机型建立一个 profile，例如 `sim/vehicles/my_quad.env`：

```bash
MODEL_NAME="my_quad"
MODEL_SDF="sim/models/my_quad/my_quad.sdf"
PX4_SYS_AUTOSTART="<分配给该机型的 autostart ID>"
PX4_PARAM_FILE="sim/vehicles/my_quad.params"
```

`scripts/start_px4_sim.sh` 已支持用 `VEHICLE_PROFILE` 加载这个文件，并从同一 profile 设置：

- Gazebo 模型名和 SDF 路径；
- PX4 airframe/autostart ID；
- 机型 PX4 参数文件；
- 模型资源路径。

这样后续切换模型只需选择 profile 名称。启动命令示例：

```bash
VEHICLE_PROFILE=my_quad ./scripts/start_px4_sim.sh
```

新增 airframe ID 前，确认对应配置已经加入并构建进 PX4 固件；否则 SITL 虽能找到 Gazebo SDF，也不能按该 airframe 启动 PX4。

## 第 7 步：逐层验证

建议按以下顺序，不要一开始就让自主模式飞完整轨迹：

1. **资源验证**：模型能加载，网格尺寸/朝向/原点正确，没有缺失 mesh。
2. **静态验证**：机体落在地面上，碰撞体不穿地，重心和测距/光流朝向正确。
3. **传感器验证**：PX4 收到 IMU、光流和有效下视距离；逐项确认更新率、符号和传感器偏移。
4. **执行器验证**：按安全方式确认电机编号、旋向和输出映射与 PX4 配置一致。
5. **动力学校准**：低高度悬停，比较真实机与仿真机的起飞推力、姿态响应和位置保持；调整质量、惯量、推力系数、时延和噪声。
6. **局部规划验证**：再接入前/左/右测距话题，验证测距失效时保持、障碍触发时停止/绕行的行为。

仿真参数拟合后，仍需通过真实机低风险测试校验。CAD 几何正确不等同于动力学正确。

## URDF 是否还需要

完整 PX4/Gazebo Classic 替换以 SDF 为仿真模型来源。若 ROS 需要 `robot_state_publisher`、RViz 机器人显示或 TF 树，可额外生成 URDF；但不要分别手工维护两份几何和传感器安装位置。对本项目的自主模式和局部目标接口，URDF 不是启动飞行控制所必需的。

## 交付前检查清单

- [ ] SolidWorks 总装体、单位和坐标系已确认。
- [ ] 整机质量、重心和惯性矩阵已记录并校核。
- [ ] 电机编号、位置、旋向、推力/扭矩数据已确认。
- [ ] 光流和下视测距安装位姿及 PX4 参数已确认。
- [ ] SDF 的 visual、collision、inertial 和插件全部对应同一机体。
- [ ] 前/左/右避障传感器仿真数据链路已明确（如果要在仿真中测避障）。
- [ ] PX4 airframe/autostart 配置与电机分配一致。
- [ ] `start_px4_sim.sh` 的 profile 能同时选择模型与 PX4 参数。
- [ ] 传感器、执行器和悬停行为已逐层验证。

## 参考

- [PX4 Gazebo Classic 仿真](https://docs.px4.io/main/en/sim_gazebo_classic/index)
- [PX4 Gazebo Classic 机型列表](https://docs.px4.io/main/en/sim_gazebo_classic/vehicles)
- [PX4 添加机架配置](https://docs.px4.io/v1.14/en/dev_airframes/adding_new_frame)
- [PX4 光流指南](https://docs.px4.io/v1.18/en/sensor/optical_flow)
- [SolidWorks 网格导出选项](https://help.solidworks.com/2025/english/SolidWorks/sldworks/c_stl_files.htm?id=18.15.29)
