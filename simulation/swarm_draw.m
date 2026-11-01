function swarm_draw(W, t, state, paths, dyn_obs_state, trails)
% swarm_draw —— 集群仿真动画单帧绘制
%
% 输入：
%   W             —— 世界参数
%   t             —— 当前仿真时间 (s)
%   state         —— 9 x 4 各 UAV 状态 [px py vx vy]
%   paths         —— 元胞数组，每架 UAV 的 RRT 航迹（K x 2）
%   dyn_obs_state —— 动态障碍当前状态（含 .pos .r）
%   trails        —— 元胞数组，每架 UAV 的历史轨迹点

    clf;

    % ---- 空域边界 ----
    rectangle('Position', [W.bounds(1,1) W.bounds(2,1) ...
        (W.bounds(1,2)-W.bounds(1,1)) (W.bounds(2,2)-W.bounds(2,1))], ...
        'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 1.2);

    hold on; axis equal; grid on;
    xlim(W.bounds(1,:)); ylim(W.bounds(2,:));

    % ---- 禁飞区（红，半透明填充）----
    for i = 1:size(W.nofly,1)
        c = W.nofly(i,1:2); r = W.nofly(i,3);
        circle_outline(c, r, [0.85 0.2 0.2], 1.5);
        fill_circle(c, r, [0.85 0.2 0.2], 0.12);
    end

    % ---- 静态障碍（深灰）----
    for i = 1:size(W.static_obs,1)
        c = W.static_obs(i,1:2); r = W.static_obs(i,3);
        circle_outline(c, r, [0.4 0.4 0.4], 1.5);
        fill_circle(c, r, [0.55 0.55 0.55], 0.7);
    end

    % ---- 动态障碍（橙）----
    for i = 1:length(dyn_obs_state)
        c = dyn_obs_state(i).pos; r = dyn_obs_state(i).r;
        circle_outline(c, r, [0.95 0.55 0.1], 1.5);
        fill_circle(c, r, [0.95 0.55 0.1], 0.55);
    end

    % ---- RRT 全局航迹（淡蓝虚线）----
    for i = 1:length(paths)
        if ~isempty(paths{i})
            plot(paths{i}(:,1), paths{i}(:,2), '--', ...
                'Color', [0.4 0.6 0.9], 'LineWidth', 1.0);
        end
    end

    % ---- 各 UAV：轨迹拖尾 + 安全圈 + 机体三角 + 速度矢量 ----
    cmap = [ ...
        0.00 0.45 0.74;   % U1 蓝
        0.85 0.33 0.10;   % U2 橙
        0.47 0.67 0.19;   % U3 绿
        0.49 0.01 0.66;   % U4 紫
        0.90 0.70 0.00;   % U5 黄
        0.00 0.60 0.65;   % U6 青
        0.78 0.00 0.16;   % U7 红
        0.35 0.35 0.35;   % U8 灰
        0.00 0.30 0.55];  % U9 深蓝
    for i = 1:size(state,1)
        p = state(i,1:2); v = state(i,3:4);
        % 拖尾（trails{i} 为 N x 2 矩阵，逐行追加位置）
        if size(trails{i}, 1) >= 2
            tr = trails{i};
            plot(tr(:,1), tr(:,2), '-', 'Color', cmap(i,:)*0.7, 'LineWidth', 1.2);
        end
        % 安全圈
        circle_outline(p, W.dsafe, cmap(i,:)*0.6, 1.0, ':');
        % 机体（朝向速度方向的三角）
        ang = atan2(v(2), v(1));
        draw_triangle(p, ang, 2.2, cmap(i,:));
        % 速度矢量
        quiver(p(1), p(2), v(1), v(2), 0.6, 'Color', cmap(i,:), ...
            'LineWidth', 1.2, 'MaxHeadSize', 3);
        % 编号
        text(p(1), p(2)+3, sprintf('U%d', i), 'Color', cmap(i,:), ...
            'FontSize', 8, 'HorizontalAlignment', 'center');
    end

    % ---- 出发区 / 目标区标注 ----
    plot(W.start(:,1), W.start(:,2), 'ks', 'MarkerSize', 4, 'MarkerFaceColor', [0 0 0]);
    plot(W.goal(:,1),  W.goal(:,2),  'g*', 'MarkerSize', 8);

    title(sprintf('9 机集群协同避障仿真  t = %.1f s', t), 'FontSize', 12);
    xlabel('x (m)'); ylabel('y (m)');
    legend({'禁飞区','静态障碍','动态障碍','RRT航迹','UAV','出发','目标'}, ...
        'Location', 'northwest', 'FontSize', 8);
end

% ===================== 绘图小工具 =====================
function h = fill_circle(c, r, col, alpha)
    th = linspace(0, 2*pi, 60);
    x = c(1) + r*cos(th); y = c(2) + r*sin(th);
    h = patch(x, y, col, 'FaceAlpha', alpha, 'EdgeColor', 'none');
end

function circle_outline(c, r, col, lw, varargin)
% 不依赖工具箱的圆形轮廓绘制（兼容 MATLAB 与 Octave）
    th = linspace(0, 2*pi, 60);
    if nargin >= 5
        plot(c(1)+r*cos(th), c(2)+r*sin(th), varargin{1}, ...
            'Color', col, 'LineWidth', lw);
    else
        plot(c(1)+r*cos(th), c(2)+r*sin(th), 'Color', col, 'LineWidth', lw);
    end
end

function draw_triangle(p, ang, s, col)
    % 以 p 为顶点、朝向 ang 的等腰三角形（pts 为 2 x 3 的顶点坐标）
    pts = [0,        -s*0.6,  s*0.6;    % 三个顶点的 x
            s,        -s*0.4, -s*0.4];  % 三个顶点的 y
    R = [cos(ang) -sin(ang); sin(ang) cos(ang)];
    xy = (R * pts)' + repmat(p, 3, 1);
    patch(xy(:,1), xy(:,2), col, 'EdgeColor', 'k', 'LineWidth', 0.5);
end
