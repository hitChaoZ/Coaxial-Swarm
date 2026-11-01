function [stats, gifPath, history] = run_swarm_sim(W, DO_ANIM)
% run_swarm_sim —— 9 机集群协同避障仿真主循环
%
% 输入：
%   W        —— 世界参数（swarm_world）
%   DO_ANIM  —— true 时绘制动画并导出 GIF；false 时仅跑数值（用于无显示环境测试）
% 输出：
%   stats    —— 统计指标结构体（抵达率、最小间距、耗时等）
%   gifPath  —— 生成的动画 GIF 路径（DO_ANIM 时有效）
%   history  —— 可选的逐步状态、动态障碍及各机路径（含 t=0）
%
% 算法流程（完善后的三层架构）：
%   ① RRT 全局规划：为每架 UAV 规划避开禁飞区/静态障碍的粗略航迹；
%   ② APF 局部势场：以障碍/禁飞/友机排斥项弯曲局部参考轨迹；
%   ③ DMPC 滚动优化：优化加速度序列，并加入机间及障碍软惩罚。
%      安全性仍须检查输出的实际轨迹净空，软惩罚不能保证无碰撞。

    if nargin < 2, DO_ANIM = true; end

    dt = W.dt;
    nUAV = size(W.start, 1);

    % 每条航迹分别检查完整线段与静态障碍/禁飞区，允许编队变形绕行。
    % 动态障碍和机间安全仍需依靠局部控制和实际轨迹诊断。
    paths = cell(nUAV, 1);
    for i = 1:nUAV
        paths{i} = resample_path(swarm_rrt(W, W.start(i,:), W.goal(i,:)), 1.0);
    end

    % ===== 2. 初始化状态与动态障碍 =====
    state = zeros(nUAV, 4);          % [px py vx vy]
    state(:,1:2) = W.start;
    dyn_obs = W.dyn_obs;             % 动态障碍（含 .pos .vel .r）
    trails = cell(nUAV, 1);
    for i = 1:nUAV, trails{i} = zeros(0,2); end

    maxSteps = ceil(W.Tmax / dt);
    arrived = false(nUAV, 1);
    if nargout >= 3
        history.time = zeros(maxSteps + 1, 1);
        history.state = zeros(nUAV, 4, maxSteps + 1);
        history.state(:,:,1) = state;
        history.dynObs = cell(maxSteps + 1, 1);
        history.dynObs{1} = dyn_obs;
        history.paths = paths;
    end

    % 障碍/禁飞信息（用于 MPC 惩罚）：半径向量
    nS = size(W.static_obs,1); nF = size(W.nofly,1); nD = length(dyn_obs);
    nObsTot = nS + nF + nD;
    obs_rad = [W.static_obs(:,3); W.nofly(:,3); [dyn_obs.r].'];

    % GIF 帧缓存（采用 print 抓帧，兼容 MATLAB GUI 与无头 Octave）
    frameFiles = {}; gifEvery = 3; stepCount = 0;
    frameDir = fullfile(pwd, 'swarm_gif_frames');
    if DO_ANIM && ~exist(frameDir, 'dir'), mkdir(frameDir); end

    % ===== 3. 主仿真循环 =====
    t = 0; allArrivedStep = -1;
    minSepEver = inf;
    minStaticClearance = inf; minNoflyClearance = inf; minDynamicClearance = inf;
    staticCollisionSamples = 0; noflyCollisionSamples = 0; dynamicCollisionSamples = 0;
    separationViolationSamples = 0;
    % Sample t=0 as well as each completed control step.
    [minSepEver, minStaticClearance, minNoflyClearance, minDynamicClearance, ...
        staticCollisionSamples, noflyCollisionSamples, dynamicCollisionSamples, ...
        separationViolationSamples] = updateSafetyStats(W, state, dyn_obs, ...
        minSepEver, minStaticClearance, minNoflyClearance, minDynamicClearance, ...
        staticCollisionSamples, noflyCollisionSamples, dynamicCollisionSamples, separationViolationSamples);
    settleSteps = 25;                 % 全员抵达后再空跑若干步，使其精确收敛到目标点

    for step = 1:maxSteps
        % (a) 更新动态障碍（匀速直线运动，碰边界反弹）
        for d = 1:nD
            dyn_obs(d).pos = dyn_obs(d).pos + dyn_obs(d).vel * dt;
            for dim = 1:2
                lo = W.bounds(dim,1) + dyn_obs(d).r;
                hi = W.bounds(dim,2) - dyn_obs(d).r;
                if dyn_obs(d).pos(dim) < lo
                    dyn_obs(d).pos(dim) = lo; dyn_obs(d).vel(dim) = -dyn_obs(d).vel(dim);
                elseif dyn_obs(d).pos(dim) > hi
                    dyn_obs(d).pos(dim) = hi; dyn_obs(d).vel(dim) = -dyn_obs(d).vel(dim);
                end
            end
        end

        % (b) 逐架 UAV 决策 + 动力学积分
        for i = 1:nUAV
            pi = state(i,1:2); vi = state(i,3:4);

            % —— 是否抵达目标：用于切换航点 ——
            if norm(pi - W.goal(i,:)) < 3
                arrived(i) = true;
            end

            % —— ① RRT 路径参考 + APF 局部弯曲 ——
            % 沿 RRT 路径按弧长前进生成参考轨迹；
            % APF 的局部排斥速度 v_rep 对参考做有限幅度的横向弯曲（局部避障）。
            if arrived(i)
                ref = repmat(W.goal(i,:), W.N, 1);   % 抵达后原地保持
                wp  = W.goal(i,:);
                v_rep = [0 0];
            else
                s0 = path_nearest_s(paths{i}, pi);
                ds = W.vmax * dt;                    % 沿路径前进步长
                bendTau = 1.5;                       % 弯曲时间常数(s)
                maxBend = 5.0;                       % 弯曲最大幅度(m)
                wp = path_point(paths{i}, s0 + 10);   % 前瞻航点（APF 吸引目标）
                others = state(setdiff(1:nUAV, i), 1:2);
                [~, ~, v_rep] = swarm_apf(W, pi, wp, others, dyn_obs);
                ref = zeros(W.N, 2);
                for k = 1:W.N
                    base = path_point(paths{i}, s0 + k*ds);
                    bend = v_rep * min(k*dt, bendTau);
                    bend = bend / (norm(bend) + 1e-6) * min(norm(bend), maxBend);
                    ref(k,:) = base + bend;
                end
            end

            % —— 构建邻居预测（常速外推，DMPC frozen-neighbor）——
            nNei = nUAV - 1;
            neighbors_pred = zeros(W.N, 2, nNei);
            m = 0;
            for j = setdiff(1:nUAV, i)
                m = m + 1;
                for kk = 1:W.N
                    neighbors_pred(kk,:,m) = state(j,1:2) + kk*dt*state(j,3:4);
                end
            end

            % —— 构建障碍/禁飞预测（静态/禁飞常值，动态常速外推）——
            obs_pred = zeros(W.N, 2, nObsTot);
            idx = 0;
            for s = 1:nS
                idx = idx + 1;
                for kk = 1:W.N, obs_pred(kk,:,idx) = W.static_obs(s,1:2); end
            end
            for f = 1:nF
                idx = idx + 1;
                for kk = 1:W.N, obs_pred(kk,:,idx) = W.nofly(f,1:2); end
            end
            for d = 1:nD
                idx = idx + 1;
                for kk = 1:W.N
                    obs_pred(kk,:,idx) = dyn_obs(d).pos + kk*dt*dyn_obs(d).vel;
                end
            end

            % —— ② DMPC 求解当前步加速度（跟踪参考 + 防撞/避障惩罚）——
            x0 = [pi vi];
            u0 = swarm_mpc(W, x0, ref, neighbors_pred, obs_pred, obs_rad);

            % —— 双积分器动力学积分（含速度限幅，体现姿控包络）——
            % 注意：位置更新使用“旧速度”，严格对应 p_{k+1}=p_k+v_k*dt+0.5*u_k*dt^2
            p_next = pi + vi * dt + 0.5 * u0 * dt^2;
            v_next = vi + u0 * dt;
            sp = norm(v_next);
            if sp > W.vmax, v_next = v_next / sp * W.vmax; end
            pi = p_next; vi = v_next;

            % 边界约束（不飞出空域）
            pi(1) = max(W.bounds(1,1)+1, min(W.bounds(1,2)-1, pi(1)));
            pi(2) = max(W.bounds(2,1)+1, min(W.bounds(2,2)-1, pi(2)));

            state(i,:) = [pi vi];
            trails{i} = [trails{i}; pi];
        end

        % (c) 统计实际位置的机间距离与障碍表面净空（离散采样）。
        [minSepEver, minStaticClearance, minNoflyClearance, minDynamicClearance, ...
            staticCollisionSamples, noflyCollisionSamples, dynamicCollisionSamples, ...
            separationViolationSamples] = updateSafetyStats(W, state, dyn_obs, ...
            minSepEver, minStaticClearance, minNoflyClearance, minDynamicClearance, ...
            staticCollisionSamples, noflyCollisionSamples, dynamicCollisionSamples, separationViolationSamples);

        t = t + dt; stepCount = stepCount + 1;
        if nargout >= 3
            history.time(step + 1) = t;
            history.state(:,:,step + 1) = state;
            history.dynObs{step + 1} = dyn_obs;
        end

        % (d) 动画：绘制当前帧并抓图（print 在无显示环境亦可工作）
        if DO_ANIM
            swarm_draw(W, t, state, paths, dyn_obs, trails);
            drawnow;
            if mod(stepCount, gifEvery) == 0
                try
                    fn = fullfile(frameDir, sprintf('frame_%05d.png', stepCount));
                    print(gcf, fn, '-dpng', '-r72');
                    frameFiles{end+1} = fn;
                catch
                    % 单帧捕获失败则跳过
                end
            end
        end

        % (e) 终止判定：全部抵达后，再空跑 settleSteps 步使其精确收敛到目标点
        if all(arrived)
            if allArrivedStep < 0
                allArrivedStep = step;
            elseif step - allArrivedStep >= settleSteps
                break;
            end
        end
    end

    % ===== 4. 组装 GIF（逐帧写入，兼容 MATLAB 与 Octave）=====
    % 直接以 RGB 帧写入 GIF（imwrite 会自动转索引）；首帧建立文件，
    % 后续帧以 WriteMode='append' 续写。LoopCount=0 表示无限循环。
    gifPath = '';
    if DO_ANIM && ~isempty(frameFiles)
        try
            delay = 0.10;                       % 每帧显示时长 (s)
            gifPath = fullfile(pwd, 'swarm_animation.gif');
            for k = 1:length(frameFiles)
                B = imread(frameFiles{k});
                if k == 1
                    imwrite(B, gifPath, 'gif', 'DelayTime', delay, 'LoopCount', 0);
                else
                    imwrite(B, gifPath, 'gif', 'WriteMode', 'append', ...
                        'DelayTime', delay);
                end
            end
            % 清理临时帧
            for k = 1:length(frameFiles), delete(frameFiles{k}); end
            rmdir(frameDir, 's');
        catch err
            disp('GIF 组装失败:'); disp(err.message);
            gifPath = '';
        end
    end

    % ===== 5. 统计指标 =====
    nArrived = sum(arrived);
    stats.nUAV = nUAV;
    stats.arrived = nArrived;
    stats.arriveRate = nArrived / nUAV;
    stats.minSeparation = minSepEver;
    stats.minStaticClearance = minStaticClearance;
    stats.minNoflyClearance = minNoflyClearance;
    stats.minDynamicClearance = minDynamicClearance;
    stats.staticCollisionSamples = staticCollisionSamples;
    stats.noflyCollisionSamples = noflyCollisionSamples;
    stats.dynamicCollisionSamples = dynamicCollisionSamples;
    stats.separationViolationSamples = separationViolationSamples;
    stats.paths = paths;
    stats.time = t;
    stats.allArrived = all(arrived);
    stats.arrivedVec = arrived;
    stats.finalPos = state(:,1:2);
    if nargout >= 3
        history.time = history.time(1:stepCount+1);
        history.state = history.state(:,:,1:stepCount+1);
        history.dynObs = history.dynObs(1:stepCount+1);
    end
    % 最近的一对及其间距（便于诊断）
    best = inf; bi = 0; bj = 0;
    for a = 1:nUAV-1
        for b = a+1:nUAV
            dab = norm(state(a,1:2) - state(b,1:2));
            if dab < best, best = dab; bi = a; bj = b; end
        end
    end
    stats.minPair = [bi bj best];
    if allArrivedStep > 0
        stats.arriveTime = allArrivedStep * dt;
    else
        stats.arriveTime = NaN;
    end
end

function [minSep, minStatic, minNofly, minDynamic, staticHits, noflyHits, dynamicHits, pairHits] = ...
        updateSafetyStats(W, state, dynObs, minSep, minStatic, minNofly, minDynamic, ...
        staticHits, noflyHits, dynamicHits, pairHits)
% Counts are UAV-obstacle or UAV-pair sample occurrences, not distinct events.
    n = size(state,1);
    for i = 1:n
        p = state(i,1:2);
        for j = i+1:n
            d = norm(p - state(j,1:2));
            minSep = min(minSep, d);
            pairHits = pairHits + (d < W.dsafe);
        end
        for j = 1:size(W.static_obs,1)
            clearance = norm(p - W.static_obs(j,1:2)) - W.static_obs(j,3);
            minStatic = min(minStatic, clearance);
            staticHits = staticHits + (clearance <= 0);
        end
        for j = 1:size(W.nofly,1)
            clearance = norm(p - W.nofly(j,1:2)) - W.nofly(j,3);
            minNofly = min(minNofly, clearance);
            noflyHits = noflyHits + (clearance <= 0);
        end
        for j = 1:numel(dynObs)
            clearance = norm(p - dynObs(j).pos) - dynObs(j).r;
            minDynamic = min(minDynamic, clearance);
            dynamicHits = dynamicHits + (clearance <= 0);
        end
    end
end

