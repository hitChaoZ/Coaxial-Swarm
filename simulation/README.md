# 9 机集群协同避障仿真

这是一个二维多无人机协同导航示例。9 架 UAV 从左侧 3×3 编队出发，在有限空域内绕过禁飞区、静态障碍和往返运动的动态障碍，最终到达右侧对应目标位置。绕障时各机可偏离初始编队。项目的控制链路是：

```text
场景参数 swarm_world
        |
        v
RRT 全局航迹 ----> APF 局部排斥与弯曲 ----> DMPC 加速度优化 ----> 双积分器积分
        ^                                                   |
        +------------------ 滚动状态反馈 -------------------+
```

## 目录和调用关系

| 文件 | 作用 | 在闭环中的位置 |
| --- | --- | --- |
| `swarm_world.m` | 集中定义空域、障碍、编队和控制参数 | 参数源 |
| `main_swarm_sim.m` | MATLAB/Octave 入口，打印统计并调用仿真 | 顶层入口 |
| `run_swarm_sim.m` | 规划、状态初始化、主循环、GIF 导出和统计 | 主循环 |
| `swarm_rrt.m` | 采样式全局规划、线段碰撞检测和捷径平滑 | 全局规划层 |
| `resample_path.m` | 将折线路径按弧长重新采样 | 参考轨迹预处理 |
| `path_nearest_s.m` | 将 UAV 投影到路径并返回弧长坐标 | 路径进度 |
| `path_point.m` | 按弧长插值取参考点 | 前视参考 |
| `path_lookahead.m` | 顶点最近点版本的前瞻函数 | 当前主循环未调用 |
| `swarm_apf.m` | 目标吸引、障碍/禁飞区/友机排斥 | 局部导航层 |
| `swarm_mpc.m` | 双积分器预测、位置/速度/控制代价和软防撞惩罚 | 动力学可行层 |
| `swarm_draw.m` | 绘制空域、路径、轨迹、安全圈、机体和速度矢量 | 动画帧 |
| `render_matlab_history.py` | 读取保存的 MATLAB 状态历史并生成 PNG/GIF/指标图 | 交付物生成器 |
| `generate_swarm_visuals.py` | 提供绘图函数；旧的近似仿真入口已禁用 | 绘图辅助 |
| `verify_matlab_history.py` | 核验 MATLAB 轨迹的步间最小净空 | 结果检查 |

## 像素级解读：一帧图上每个元素代表什么

- **浅蓝虚线**：每架 UAV 独立规划的 RRT 参考航迹。规划时逐线段检查静态障碍和禁飞区，绕障时航迹可以分开。
- **彩色细线**：每架 UAV 的历史位置拖尾。拖尾不是规划线，而是实际离散积分得到的状态轨迹。
- **彩色三角形**：当前位置和速度方向；三角形尖端沿 `atan2(vy,vx)` 旋转。
- **彩色点划圆**：以 `W.dsafe = 6 m` 为半径的机间安全可视化圈。它是诊断标记，不是硬几何约束。
- **红色半透明圆**：禁飞区 `W.nofly`。RRT 的线段检测和 APF 的排斥项都处理它，且禁飞区排斥增益更大、影响半径更长。
- **深灰圆**：静态障碍 `W.static_obs`。它们参与 RRT 碰撞检测、APF 排斥和 MPC 障碍惩罚。
- **橙色圆及外圈**：当前动态障碍位置和局部影响圈。动态障碍按匀速运动，碰到空域边界后反弹；MPC 只做常速短期外推。
- **动态障碍与历史轨迹的交叉**：轨迹线记录的是过去的位置，橙色圆显示的是当前障碍位置；两者在图上交叉不一定意味着同一时刻碰撞，需用同步状态的净空统计判断。
- **黑色方块 / 绿色星号**：初始编队 / 目标编队。
- **左上角统计框**：当前帧中距离目标小于 3 m 的 UAV 数、最小机间距离、静态/动态障碍数。

## 动力学与控制细节

每架 UAV 使用状态 `x = [px, py, vx, vy]` 和加速度控制 `u = [ax, ay]`。主循环的离散更新严格对应双积分器：

```text
p(k+1) = p(k) + v(k)·dt + 0.5·u(k)·dt²
v(k+1) = v(k) + u(k)·dt
```

其中 `dt = 0.1 s`、`vmax = 9 m/s`、`amax = 7 m/s²`。速度按向量模长限幅，MPC 加速度按各坐标分量限幅；位置还会被钳制在空域边界内。

### RRT 全局层

`swarm_rrt.m` 在 `[0,150]×[0,90] m` 内随机采样，`step = 6 m`，`goalBias = 0.1`。每架 UAV 分别规划；新节点加入前检查线段到所有禁飞区和静态障碍的最短距离，并保留 `2.5 m` 余量。找到目标邻域后回溯父节点，并用最多 5 轮贪心捷径平滑。若端点不可行或 4000 次采样未找到路径，程序显式报错，不返回未经验证的直线。

### APF 局部层

`swarm_apf.m` 的合力为吸引力和排斥力之和：

```text
F = Fatt + Frep_obs + Frep_nofly + Frep_dynamic + Frep_neighbor
Fatt = Katt · unit(waypoint - position) · min(distance, 25)
```

排斥力在影响半径内按 `(1/d - 1/d0)/d² · (position - center)` 增长。主循环使用排斥项 `v_rep` 对路径参考做有限横向弯曲；`swarm_apf` 输出的 `v_des` 本身没有直接送入积分器，这一点是阅读代码时容易漏掉的关键事实。

### DMPC 局部优化层

`swarm_mpc.m` 用矩阵递推得到 `Phi_all` 和 `Xoff`，构造未来 `N = 10` 步的跟踪代价。代价包括位置跟踪 `Q`、速度阻尼 `Rv`、控制能量 `R`、末端跟踪 `Qf`，以及机间/障碍进入软惩罚 `Wc`、`Wo`。障碍距离惩罚围绕当前控制序列线性化，并使用阻尼迭代；机间和障碍预测分别加入 `1.5 m` 和 `3 m` 余量。邻居和动态障碍采用常速外推。软惩罚不能保证任意初始条件下无碰撞，因此需要检查实际轨迹。

## 运行和交付物

### MATLAB/Octave 原始仿真

在 MATLAB 或 GNU Octave 中把当前目录加入路径后运行：

```matlab
main_swarm_sim
```

入口会调用 `run_swarm_sim(W, true)`，实时绘图并在当前目录写出 `swarm_animation.gif`。只做数值回归时：

```matlab
rng(7, 'twister');
W = swarm_world();
[stats, gifPath, history] = run_swarm_sim(W, false);
disp(stats)
save('visualizations/matlab_simulation.mat', 'W', 'stats', 'history', '-v7')
```

第三个输出包含初始时刻及每个仿真步的状态、动态障碍状态和路径。`stats` 的碰撞计数按离散时刻统计，同一碰撞跨多个时刻会计入多个样本。`arriveTime` 是所有 UAV 曾进入各自目标 3 m 范围的首次时刻，属于锁存判定；它不单独保证此时所有 UAV 同时在目标范围内。

### 轨迹核验与可视化

```powershell
python verify_matlab_history.py visualizations/matlab_simulation.mat --output visualizations/matlab_clearance_audit.json
python render_matlab_history.py visualizations/matlab_simulation.mat --output-dir visualizations
```

Python 脚本需要 NumPy、SciPy、Matplotlib 和 Pillow。步间核验将无人机每一步的运动按恒加速度重建，将动态障碍中心在相邻记录点之间线性插值，并求解相对距离的驻点。它检查的是本次记录及该步间模型，并不是对所有场景和控制器的安全证明。

默认输出到 `visualizations/`：

```text
visualizations/
├── swarm_animation.gif       # 可播放的动态演示
├── matlab_simulation.mat     # MATLAB 原控制器状态历史及统计
├── matlab_clearance_audit.json # 步间净空核验结果
├── stats.json                # MATLAB 离散统计摘要
└── figures/
    ├── swarm_t000.0s.png     # 起始帧
    ├── ...                   # 其他时刻的状态快照
    └── swarm_metrics.png     # 到目标距离与最小间距曲线
```

本次交付的动图和快照来自 `matlab_simulation.mat` 中的 MATLAB 控制器实际状态，而非先前穿越障碍的 Python 近似轨迹。

## 本次仿真结果

使用 `rng(7,'twister')`，仿真持续 `18.0 s`；9 架 UAV 均曾进入目标 3 m 范围，全部锁存时间为 `15.5 s`，最终各机最大目标误差为 `0.334 m`。离散记录中静态障碍、禁飞区、动态障碍碰撞样本及机间低于 6 m 的样本均为 0。

| 指标 | 步间最小值 | 含义 |
| --- | ---: | --- |
| 静态障碍表面净空 | 3.116 m | UAV 中心到圆形障碍边界的距离 |
| 禁飞区表面净空 | 6.560 m | UAV 中心到禁飞圆边界的距离 |
| 动态障碍表面净空 | 2.436 m | 动态障碍中心按相邻记录点线性插值 |
| 机间中心距离 | 7.614 m | 阈值为 6 m |

## 阅读时应保留的工程边界

1. 这不是带姿态、推力、通信时延和传感器噪声的飞行器动力学仿真，而是集群层二维质点包络。
2. 机间和障碍安全在 MPC 中是软惩罚，主循环没有独立的硬碰撞投影或安全过滤器；应同时检查机间距离以及静态、动态障碍和禁飞区净空，不能只看“抵达率”。
3. 每架 UAV 独立规划静态航迹，但 RRT 本身不考虑动态障碍、其他飞机或实际跟踪误差。随机种子、参数或场景改变后需要重新核验。
4. `path_nearest_s.m` 使用线段投影，`path_lookahead.m` 只比较顶点最近点，二者精度不同；当前主循环采用前者与 `path_point.m` 的组合。
5. 动态障碍的 APF 只使用当前中心，短期运动预测由 MPC 层补充；这不是完整的速度障碍或可达集避碰证明。

