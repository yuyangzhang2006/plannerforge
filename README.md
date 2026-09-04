# PlannerForge

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

启用示例动态障碍：

```bash
ros2 launch gcopter planner_forge.launch.py generate_dynamic_obstacles:=true
```

关闭 RViz：

```bash
ros2 launch gcopter planner_forge.launch.py rviz:=false
```

后续修改顺序和源码中的 `TODO` 位置见 [docs/流程引导.md](docs/流程引导.md)。
