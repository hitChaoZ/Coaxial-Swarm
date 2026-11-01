% main_swarm_sim —— 9 机集群协同避障仿真入口
%
% 运行方式（MATLAB 或 Octave）：
%   直接运行本脚本即可。将依次完成：
%     1) 加载场景（禁飞区 / 静态障碍 / 动态障碍 / 9 机编队）
%     2) 为每架 UAV 规划 RRT 全局航迹
%     3) 滚动执行 APF 局部势场 + DMPC 动力学可行跟踪
%     4) 实时动画演示，并导出 swarm_animation.gif
%     5) 输出抵达率、最小机间间距、耗时等统计指标
%
% 算法三层架构（详见配套说明）：
%   RRT（静态路径规划） + APF（局部势场） + DMPC（软碰撞惩罚）

clear; clc;

W = swarm_world();

fprintf('===== 9 机集群协同避障仿真 =====\n');
fprintf('规模: %d 架 UAV | 禁飞区: %d | 静态障碍: %d | 动态障碍: %d\n', ...
    size(W.start,1), size(W.nofly,1), size(W.static_obs,1), length(W.dyn_obs));
fprintf('控制参数: dt=%.2f s, vmax=%.1f m/s, amax=%.1f m/s^2, dsafe=%.1f m, MPC-N=%d\n\n', ...
    W.dt, W.vmax, W.amax, W.dsafe, W.N);

[stats, gifPath] = run_swarm_sim(W, true);

fprintf('\n===== 仿真结果统计 =====\n');
fprintf('抵达 UAV 数   : %d / %d  (抵达率 %.0f%%)\n', ...
    stats.arrived, stats.nUAV, stats.arriveRate*100);
if stats.allArrived
    fprintf('全员抵达耗时  : %.1f s\n', stats.arriveTime);
else
    fprintf('全员抵达耗时  : 未在 %g s 内完成\n', W.Tmax);
end
fprintf('最小机间间距  : %.2f m  (安全间距设定 %.1f m)\n', stats.minSeparation, W.dsafe);
fprintf('最小障碍表面净空: 静态 %.2f m，禁飞区 %.2f m，动态 %.2f m\n', ...
    stats.minStaticClearance, stats.minNoflyClearance, stats.minDynamicClearance);
fprintf('离散碰撞样本数: 静态 %d，禁飞区 %d，动态 %d；机间间距不足 %d\n', ...
    stats.staticCollisionSamples, stats.noflyCollisionSamples, ...
    stats.dynamicCollisionSamples, stats.separationViolationSamples);
fprintf('总仿真时间    : %.1f s\n', stats.time);
if ~isempty(gifPath)
    fprintf('动画文件      : %s\n', gifPath);
else
    fprintf('动画文件      : 未生成（无显示环境或导出失败）\n');
end
