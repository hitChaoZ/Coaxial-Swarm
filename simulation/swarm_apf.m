function [v_des, F, v_rep] = swarm_apf(W, pos, waypoint, neighbors, dyn_obs_state)
% swarm_apf —— 人工势场法（APF）局部导航层
%
% 输入：
%   W             —— 世界参数（含 nofly / static_obs / apf / vmax / dsafe）
%   pos           —— 当前位置 [x y]
%   waypoint      —— 当前 RRT 航点（APF 的吸引目标，一般取路径上前瞻点）[x y]
%   neighbors     —— 邻居 UAV 位置矩阵 M x 2
%   dyn_obs_state —— 动态障碍结构体数组（含 .pos .r）
% 输出：
%   v_des         —— 合成期望速度 [vx vy]（幅值限到 vmax），作为整体导航意图
%   F             —— 合成虚拟势场力（吸引 + 排斥），用于可视化
%   v_rep         —— 仅由排斥项构成的“局部避障速度”，用于弯曲 MPC 参考轨迹
%
% 势场构成（验收书 4.4.3）：
%   ① 目标吸引力：引导 UAV 飞向当前 RRT 航点；
%   ② 障碍排斥力：静态障碍 + 动态障碍；
%   ③ 禁飞区排斥力：更强、更大影响半径，确保不进入禁区；
%   ④ 友机防撞排斥力：维持安全间距。
% UAV 沿“势场负梯度”方向运动，即沿合力 F 方向以期望速度前进。

    Katt     = W.apf.Katt;
    KrepObs  = W.apf.KrepObs;
    KrepNof  = W.apf.KrepNofly;
    KrepNei  = W.apf.KrepNei;
    d0Obs    = W.apf.d0Obs;
    d0Nof    = W.apf.d0Nofly;
    d0Nei    = W.apf.d0Nei;
    dsafe    = W.dsafe;

    Fatt = [0 0];   % 吸引力
    Frep = [0 0];   % 排斥力

    % ① 目标吸引力（指向 RRT 航点，距离越远力越大，限幅避免过冲）
    dAtt = waypoint - pos;
    distAtt = norm(dAtt);
    if distAtt > 1e-3
        Fatt = Fatt + Katt * dAtt / distAtt * min(distAtt, 25);
    end

    % ② 静态障碍排斥力
    for i = 1:size(W.static_obs,1)
        c = W.static_obs(i,1:2); r = W.static_obs(i,3);
        d = pos - c; dist = norm(d);
        if dist < d0Obs + r && dist > 1e-3
            Frep = Frep + KrepObs * (1/dist - 1/(d0Obs+r)) / (dist^2) * d;
        end
    end

    % ③ 禁飞区排斥力（更强、影响半径更大）
    for i = 1:size(W.nofly,1)
        c = W.nofly(i,1:2); r = W.nofly(i,3);
        d = pos - c; dist = norm(d);
        if dist < d0Nof + r && dist > 1e-3
            Frep = Frep + KrepNof * (1/dist - 1/(d0Nof+r)) / (dist^2) * d;
        end
    end

    % ④ 动态障碍排斥力（使用当前状态，短期预测由 MPC 负责）
    for i = 1:length(dyn_obs_state)
        c = dyn_obs_state(i).pos; r = dyn_obs_state(i).r;
        d = pos - c; dist = norm(d);
        if dist < d0Obs + r && dist > 1e-3
            Frep = Frep + KrepObs * (1/dist - 1/(d0Obs+r)) / (dist^2) * d;
        end
    end

    % ⑤ 友机防撞排斥力
    for i = 1:size(neighbors,1)
        d = pos - neighbors(i,:); dist = norm(d);
        if dist < d0Nei && dist > 1e-3
            Frep = Frep + KrepNei * (1/dist - 1/d0Nei) / (dist^2) * d;
        end
    end

    F = Fatt + Frep;

    % 合成期望速度（幅值限到 vmax）
    speed = norm(F);
    if speed > 1e-6
        v_des = F / speed * min(speed, W.vmax);
    else
        v_des = [0 0];
    end

    % 仅排斥项构成的局部避障速度（用于弯曲 MPC 参考轨迹）
    rspeed = norm(Frep);
    if rspeed > 1e-6
        v_rep = Frep / rspeed * min(rspeed, W.vmax);
    else
        v_rep = [0 0];
    end
end
