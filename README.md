# PlannerForge

规划参数、运行行为和当前实现限制见
[规划参数说明](docs/planner_parameters.md)。

每次实现迭代、问题根因和实测结果记录在
[开发日志](docs/DEVELOPMENT_LOG.md)中。

PlannerForge 是一个二维导航规划框架。当前版本包含地图读取、栅格搜索、轨迹生成、MINCO 解算和 RViz 显示，可以单独构建运行。

## 目录

```text
PlannerForge/
├── src/
│   ├── plan_interfaces/  # 轨迹与控制消息
│   ├── simple_map/       # 地图图片、参数和 /grid_map 发布节点
│   └── gcopter/          # 规划流程和 /minco_trajectory 发布节点
├── docs/
│   └── 流程引导.md       # 各阶段的修改目标和代码入口
└── README.md
```

静态地图放在 `src/simple_map/maps/`，对应参数放在 `src/simple_map/config/`。

## 构建

在 PlannerForge 的上一级目录中执行：

```bash
cd PlannerForge
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch gcopter planner_forge.launch.py
```

启动后会打开 RViz。默认 `is_plan_from_ego_pose: false`，第一次使用 `2D Goal Pose` 设置起点，第二次设置终点。规划结果会显示离散路径、连续轨迹和沿轨迹推进的当前位置。

规划采用分级保底发布：前端 A* 成功后会立即发布黄色离散路径；后端优化失败时优先发布经过硬安全校验的未优化 MINCO 轨迹；若无法生成可安全执行的连续轨迹，则仍保留黄色路径，并在 `/planner/status` 中标明当前完成层级和不可直接执行的原因。标准离散路径同时发布在 `/planner/path`。

密集动态障碍物暂时压住起点或终点时，状态会显示 `WAITING_DYNAMIC_CLEARANCE`。规划器会保留你点选的原坐标并自动重试，无需重复选择两次。

运动中的轨迹与动态障碍物发生预测冲突时，规划器优先从当前 P/V/A 局部 A* 到原路径前方的接入点，接入点之后复用原路径拓扑。拼接后的整条剩余路线会重新生成 corridor 并重新执行 MINCO，避免接入处速度或加速度不连续。局部搜索超过配置的绝对距离限制时才回退为当前位置到原终点的全局 A*。

演示默认启用 12 个密集动态障碍物，包含横向、纵向、对角和椭圆运动。临时关闭动态障碍物：

```bash
ros2 launch gcopter planner_forge.launch.py generate_dynamic_obstacles:=false
```

关闭 RViz：

```bash
ros2 launch gcopter planner_forge.launch.py rviz:=false
```

后续修改顺序和源码中的 `TODO` 位置见 [docs/流程引导.md](docs/流程引导.md)。
