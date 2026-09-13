# 规划器实现与参数说明

现有的 `/goal_pose`、`/grid_map`、`/Odometry_to_base_link` 和 `/minco_trajectory` 接口保持不变。演示地图节点另外发布 `/dynamic_obstacles`，消息类型为 `plan_interfaces/msg/DynamicObstacleArray`，规划器据此进行匀速预测，而不是从占据栅格快照中推测障碍物速度。

## 搜索与地图

- `RobotRadiusNormal`：普通导航使用的机器人 footprint 半径，单位为米。
- `RobotRadiusCompact`：狗洞区域内使用的紧凑 footprint 半径，单位为米。
- `StaticSafetyMargin`：硬膨胀前叠加到两种 footprint 半径上的安全余量。
- `TurnCostWeight`：每弧度方向变化对应的等价距离权重。
- `SafetyCostWeight`：软净空代价权重；设为零时关闭该代价。
- `SafetyCostDistance`：软净空代价衰减为零时的障碍物距离，单位为米。
- `SpecialRegionCostWeight`：狗洞等特殊区域内的额外等价距离权重。
- `CrossHoleRegions`：地图坐标系内的扁平数组，每四个数依次表示 `xmin,xmax,ymin,ymax`。没有狗洞区域时应省略此参数，因为 ROS 2 不接受无法推断类型的空 YAML 数组。

规划器会根据每次收到的 `OccupancyGrid` 生成正常/紧凑两套硬占据层和八邻域距离场。未知栅格与非零栅格均视为占据。狗洞区域采用紧凑 footprint 膨胀，同时保留语义标签，供前端代价计算和穿越区间提取使用。

## 安全 corridor

- `CorridorObstacleSearchRadius`：局部膨胀的最大范围，单位为米。
- `CorridorFiriIterations`：有界膨胀迭代次数，默认值为 `4`。
- `CorridorMinOverlap`：路径关节处要求的重叠种子区域直径。
- `CorridorMergeTolerance`：半空间包含判断的数值容差。

corridor 是按路径顺序排列的有向凸矩形，以 `A*x <= b` 表示。每个 corridor 从完整路径段开始生长，并通过硬占据地图进行保守采样。如果稀疏种子构造失败，规划器会恢复已有 A* 路径点后重试 corridor，而不会重新执行全局搜索。

## MINCO 轨迹

- `NominalVel`：计算初始分段时长使用的标称速度。
- `MaxVelMag`：为兼容旧配置保留的 `NominalVel` 别名，已不推荐使用。
- `MaxPlanVel`、`MaxPlanAcc`：规划速度和加速度包络上限。
- `MinSegmentDuration`：避免数值不稳定的最短分段时长。
- `WeightTime`、`WeightVelPenalty`、`WeightAccPenalty`：后端目标函数的时间、速度和加速度权重。
- `OptimizationMaxIterations`、`OptimizationTimeBudgetMs`：投影搜索的迭代次数与时间预算。

后端继续使用 `MINCO_S3NU`。中间点只能在相邻且顺序固定的 corridor 重叠区域内调整，分段时长保持为正，并通过时间缩放满足规划包络。如果采样轨迹离开对应 corridor、发生碰撞、包含非法数据或超过运动包络，该结果会被拒绝，不会发布。

## 动态障碍物

- `PredictionHorizonSec`：未来轨迹与障碍物预测窗口。
- `DynamicSafetyMargin`：叠加到障碍物和机器人半径上的额外安全距离。
- `DynamicObstacleTimeoutSec`：动态障碍物观测允许的最大时间延迟。
- `ReplanCooldownMs`：两次有效重规划之间的最短间隔。
- `PlanningRetryIntervalMs`：起点或终点暂时被动态障碍物覆盖时，自动重试原任务的时间间隔。
- `PlanningRetryLimit`：自动重试次数上限；达到上限后等待用户重新选择起终点。
- `LocalReplanMinLookaheadDistance`：局部重规划接入点沿原路径向前的最小距离。
- `LocalReplanMaxLookaheadDistance`：搜索原路径接入点的最远前视距离。
- `LocalReplanMaxPathDistance`：当前位置到接入点的局部路径最大绝对长度。
- `LocalReplanMaxExtraDistance`：局部路径相对当前位置到接入点直线距离允许增加的最大米数；不是百分比。

每个动态障碍物观测包含位置、速度、矩形尺寸或圆形半径、时间戳、形状和有效时长。只有障碍物的预测运动与当前轨迹未来部分冲突时才触发重规划。重规划失败后，仅当旧轨迹仍安全时才保留旧轨迹；否则规划器会清除活动轨迹状态，并向控制器或上层监督节点报告错误。

RViz 演示默认启用确定性的密集压力测试场景，共 12 个不同尺寸、速度和初始相位的动态矩形，运动模式包括横向往返、纵向往返、对角往返和椭圆运动。可用 `generate_dynamic_obstacles:=false` 启动参数关闭该场景。椭圆轨迹发布当前位置处的瞬时速度，规划器仍按 V1 的短时匀速模型进行保守预测。

如果 A* 失败是因为起点、终点暂时被动态障碍物的硬膨胀区覆盖，规划器会发布 `WAITING_DYNAMIC_CLEARANCE`，保留用户选择的原始坐标，并在后续地图更新时按配置间隔自动重试。不会把起终点静默移动到别处。

活动轨迹发生预测动态障碍物冲突时，规划器先在当前位置与原路径前方接入点之间执行局部 A*。接入点后保留原路径的空间拓扑，但不会直接拼接原来的多项式：系统使用当前实际 P/V/A 作为新起始边界，对“局部绕行 + 原路径后缀”重新生成全部剩余 corridor 与 MINCO 分段和时长。这样接入点不需要强行匹配旧轨迹的速度和加速度。局部接入无法满足绝对距离约束时，才回退为到原终点的全局 A*。

## 狗洞时间区间

- `CrossHoleShapeTime`：实际进入狗洞前的准备时间。
- `CrossHoleRecoveryTime`：实际离开狗洞后的恢复时间。
- `CrossHoleTimeMargin`：进入和离开两侧附加的时间余量。

狗洞区间根据连续轨迹实际落入语义区域的采样点提取。仅从附近经过不会生成区间；每次生成新轨迹时都会清除旧区间。

## 诊断信息与当前限制

每次成功规划都会记录 A*、corridor、MINCO 和总耗时，以及离散路径长度、连续轨迹时长、最小障碍物净空、最大速度和最大加速度。

规划结果采用以下分级状态，并通过可靠、瞬态本地的 `/planner/status`（`std_msgs/msg/String`）发布。前端或稀疏后的标准路径通过 `/planner/path`（`nav_msgs/msg/Path`）发布：

- `OPTIMIZED_MINCO`：完整优化轨迹，能够执行。
- `LOCAL_REJOIN_OPTIMIZED_MINCO`：局部 A* 接回原路径后，对全部剩余路线重新优化得到的轨迹。
- `SAFE_INITIAL_MINCO`：优化失败后使用的安全初始 MINCO，能够执行，但平滑性或时间代价不是最优。
- `LOCAL_REJOIN_SAFE_INITIAL_MINCO`：局部接入后的优化失败，改用通过硬检查的安全初始 MINCO。
- `CORRIDOR_PATH_ONLY`：前端路径和 corridor 成功，连续轨迹不可安全执行。
- `FRONTEND_PATH_ONLY`：仅前端硬膨胀地图路径成功，不可直接作为 MINCO 控制轨迹执行。
- `LOCAL_REJOIN_PATH_ONLY`、`LOCAL_REJOIN_CORRIDOR_PATH_ONLY`：局部接入路径已得到，但对应后端阶段未形成可执行连续轨迹。
- `FAILED_NO_PATH`：前端也未找到可行路径。
- `WAITING_DYNAMIC_CLEARANCE`：起终点或路线暂时被动态障碍物阻断，正在自动重试原任务。
- `FAILED_NO_PATH_RETRY_EXHAUSTED`：自动重试达到配置上限，等待重新选择起终点。
- `PATH_ONLY_STOP_REQUIRED`：动态重规划只能得到离散路径，且旧轨迹已不安全，控制器必须停车。

黄色前端路径会在 A* 完成后立即显示，不再等待后端全部结束。所有可执行降级轨迹仍必须通过数据、corridor、硬碰撞、速度和加速度检查；保底机制不会发布穿越硬障碍物的轨迹。

当前 corridor 是有界的二维 FIRI 风格有向矩形膨胀，还不是通用的最大体积凸多边形求解器。动态矩形障碍物的碰撞预测使用保守外接圆。现有外部消息中没有明确的停车指令，因此危险状态下重规划失败会通过活动轨迹状态和 ROS 错误日志暴露；接入实体机器人时，应将此状态映射到控制器的停车接口。
