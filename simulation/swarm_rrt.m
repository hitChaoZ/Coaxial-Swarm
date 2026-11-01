function path = swarm_rrt(W, start, goal, offsets)
% swarm_rrt —— 基于 RRT 的全局路径规划（避开禁飞区与静态障碍）
%
% 输入：
%   W      —— 世界参数（含 bounds / nofly / static_obs / rrt）
%   start  —— 起点 [x y]
%   goal   —— 终点 [x y]
%   offsets —— 相对于规划点的编队偏移，默认 [0 0]
% 输出：
%   path   —— K x 2 航点序列（含起终点），已做捷径平滑
%
% 作用：对静态障碍和禁飞区进行线段碰撞检查，输出带余量的路径。
%       可选 offsets 用于验证路径平移后的所有航迹；主循环逐机规划。
%       动态障碍、其他飞机及实际跟踪误差不在 RRT 检查范围内。
%       未找到路径或端点不可行时显式报错，不返回未经检查的直线。

    maxIter  = W.rrt.maxIter;
    step     = W.rrt.step;
    goalBias = W.rrt.goalBias;
    goalR    = W.rrt.goalR;
    margin   = W.rrt.margin;
    if nargin < 4, offsets = [0 0]; end
    if isempty(offsets) || size(offsets,2) ~= 2 || any(~isfinite(offsets(:)))
        error('swarm_rrt:InvalidOffsets', 'offsets must be a nonempty finite M-by-2 matrix.');
    end
    if segCollides(W, start, start, margin, offsets) || ...
            segCollides(W, goal, goal, margin, offsets)
        error('swarm_rrt:InvalidEndpoint', ...
            'A formation endpoint is outside the bounds or inside an expanded obstacle.');
    end

    nodes  = start;          % 已采样节点 N x 2
    parent = -1;             % 父节点索引（根节点为 -1）

    for it = 1:maxIter
        % (1) 采样：以 goalBias 概率直接采样目标，否则在空域内随机采样
        if rand < goalBias
            sample = goal;
        else
            sample = [ rand*(W.bounds(1,2)-W.bounds(1,1)) + W.bounds(1,1), ...
                       rand*(W.bounds(2,2)-W.bounds(2,1)) + W.bounds(2,1) ];
        end

        % (2) 找最近节点
        d = sum((nodes - sample).^2, 2);
        [~, nidx] = min(d);
        pnear = nodes(nidx, :);

        % (3) 朝采样点“转向”扩展一个步长
        dir = sample - pnear;
        dist = norm(dir);
        if dist < 1e-6, continue; end
        if dist > step
            dir = dir / dist * step;
        end
        pnew = pnear + dir;

        % (4) 碰撞检测：新节点与扩展线段不得进入禁飞区 / 静态障碍
        if ~segCollides(W, pnear, pnew, margin, offsets)
            nodes  = [nodes; pnew];
            parent = [parent; nidx];
            % (5) 若抵达目标邻域，校验末段可直连目标后再回溯、追加精确目标、平滑
            if norm(pnew - goal) < goalR
                % 若末段 pnew->goal 被障碍/禁飞阻挡，则不在此收尾，继续生长
                if segCollides(W, pnew, goal, margin, offsets)
                    continue;
                end
                path = reconstruct(nodes, parent, size(nodes,1));
                path = [path; goal];              % 追加精确目标点，保证路径真正抵达
                path = smoothPath(W, path, margin, offsets);
                return;
            end
        end
    end

    error('swarm_rrt:PlanningFailed', ...
        'No collision-free formation path found in %d iterations.', maxIter);
end

% ===================== 局部工具函数 =====================

function hit = segCollides(W, a, b, margin, offsets)
% 检查每架飞机平移后的整条线段，而非只检查编队质心。
    hit = false;
    for j = 1:size(offsets,1)
        aj = a + offsets(j,:); bj = b + offsets(j,:);
        if any(aj < W.bounds(:,1)') || any(aj > W.bounds(:,2)') || ...
                any(bj < W.bounds(:,1)') || any(bj > W.bounds(:,2)')
            hit = true; return;
        end
        for i = 1:size(W.nofly,1)
            c = W.nofly(i,1:2); r = W.nofly(i,3) + margin;
            if pointSegDist(c, aj, bj) <= r, hit = true; return; end
        end
        for i = 1:size(W.static_obs,1)
            c = W.static_obs(i,1:2); r = W.static_obs(i,3) + margin;
            if pointSegDist(c, aj, bj) <= r, hit = true; return; end
        end
    end
end

function d = pointSegDist(p, a, b)
% 点 p 到线段 a-b 的最短距离
    ab = b - a;
    t = dot(p - a, ab) / max(dot(ab,ab), 1e-9);
    t = max(0, min(1, t));
    proj = a + t * ab;
    d = norm(p - proj);
end

function path = reconstruct(nodes, parent, idx)
% 从节点 idx 回溯到根，重建路径（逆序后翻转）
    rev = zeros(0,2);
    while idx ~= -1
        rev = [rev; nodes(idx,:)];
        idx = parent(idx);
    end
    path = flipud(rev);
end

function path = smoothPath(W, path, margin, offsets)
% 贪心捷径平滑：尝试跨过中间节点直接相连（前提是不碰撞）
    if size(path,1) < 3, return; end
    changed = true;
    pass = 0;
    while changed && pass < 5
        changed = false;
        pass = pass + 1;
        i = 1;
        while i < size(path,1)
            jmax = size(path,1);
            advanced = false;
            for j = jmax:-1:(i+2)
                if ~segCollides(W, path(i,:), path(j,:), margin, offsets)
                    path = [path(1:i,:); path(j:end,:)];
                    changed = true;
                    advanced = true;
                    break;
                end
            end
            if ~advanced
                i = i + 1;
            end
        end
    end
end
