function u0 = swarm_mpc(W, x0, ref, neighbors_pred, obs_pred, obs_rad)
% swarm_mpc —— 分布式模型预测控制（DMPC）局部跟踪与可行性层
%
% 输入：
%   W              —— 世界参数（含 dt / N / vmax / amax / mpc / dsafe）
%   x0             —— 当前状态 [px py vx vy]
%   ref            —— 参考轨迹 N x 2（由 RRT 路径前进 + APF 局部弯曲得到）
%   neighbors_pred —— 邻居预测位置，N x 2 x M（M 为邻居数，按常速外推）
%   obs_pred       —— 障碍预测中心，N x 2 x K（静态/动态/禁飞，按常速外推）
%   obs_rad        —— 各障碍物理半径，K x 1；预测余量由 W.mpc.margin 添加
% 输出：
%   u0             —— 当前步近似优化的加速度指令 [ax ay]
%
% 模型：二维质点“双积分器”（代表经姿控环封装后的单机运动学包络）
%   p_{k+1} = p_k + v_k*dt + 0.5*u_k*dt^2
%   v_{k+1} = v_k + u_k*dt
% 控制量 u_k = [ax ay] 的各分量被限制在 [-amax, amax]；主循环在积分后
% 对速度模长限幅。预测模型本身未施加速度硬约束。
%
% 求解策略：跟踪和控制代价是二次型；距离相关的铰链惩罚是非线性的。
%   每轮对活动残差作 Gauss-Newton 线性化，解二次近似并做分量限幅和阻尼。
%   这是局部近似求解，软惩罚不保证无碰撞或全局最优。
%
% 代价（验收书 4.4.2 一致性代价函数的工程化实现）：
%   J = Σ_k 位置跟踪 + Σ_k 速度阻尼 + Σ_k 控制能量 + 末端跟踪
%       + Σ_k 机间防撞惩罚 + Σ_k 障碍/禁飞进入惩罚
%
% 说明：每架 UAV 独立求解自身 MPC，使用邻居的常速预测作为软防撞项；
%       这是 frozen-neighbor 近似，预测覆盖 N 个离散步。

    dt = W.dt;
    N  = W.N;

    % 状态/控制/输出矩阵（双积分器）
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
    B = [0.5*dt^2 0; 0 0.5*dt^2; dt 0; 0 dt];
    C = [1 0 0 0; 0 1 0 0];          % 仅取位置参与跟踪/碰撞

    Q  = W.mpc.Q;  R = W.mpc.R;  Qf = W.mpc.Qf;
    Rv = W.mpc.Rv;                       % 速度跟踪（阻尼）权重
    Wc = W.mpc.Wc; Wo = W.mpc.Wo;
    dsafe   = W.dsafe + W.mpc.pairMargin;
    margin  = W.mpc.margin;
    nMPC = N * 2;                        % 控制变量总数（堆叠向量维度）

    r = ref;                             % N x 2 参考轨迹
    % 参考速度（用于速度跟踪/阻尼）：沿参考的轨迹速度
    v_ref = zeros(N, 2);
    for k = 1:N-1
        v_ref(k,:) = (r(k+1,:) - r(k,:)) / dt;
    end
    v_ref(N,:) = v_ref(N-1,:);

    % ---- 敏感度 Phi(k+1) = dX_{k+1}/du  与  “由初始状态 x0 产生”的偏移 Xoff ----
    Phi_all = zeros(4, nMPC, N+1);
    Phi = zeros(4, nMPC);
    Xoff = zeros(N+1, 4); Xoff(1,:) = x0;
    for k = 1:N
        Phi = A * Phi;
        Phi(:, 2*(k-1)+1:2*k) = Phi(:, 2*(k-1)+1:2*k) + B;   % 叠加 B 到当前控制块
        Phi_all(:,:,k+1) = Phi;
        Xoff(k+1,:) = (A * Xoff(k,:)')';
    end

    nNei = size(neighbors_pred, 3);
    nObs = size(obs_pred, 3);

    % ---- 控制初值：朝参考首点所需加速度（限幅），作为活动集迭代初值 ----
    u0g = (r(1,:) - x0(1:2) - x0(3:4)*dt) / (0.5*dt^2);
    u0g = max(min(u0g, W.amax), -W.amax);
    u_flat = repmat(u0g', N, 1);         % 2N x 1

    % ---- 活动集迭代（Gauss-Newton 近似处理铰链平方惩罚）----
    for outer = 1:8
        % 前向积分得到当前位置（用于判定防撞/避障活动约束）
        u_vec = reshape(u_flat, 2, N)';
        P = zeros(N+1, 2); P(1,:) = x0(1:2);
        X = zeros(N+1, 4); X(1,:) = x0;
        for k = 1:N
            X(k+1,:) = (A * X(k,:)' + B * u_vec(k,:)')';
            P(k+1,:) = X(k+1, 1:2);
        end

        % 组装二次型 J(u) = 1/2 u'H u + c'u  （H 对称，直接求解 H·u = -c）
        H = zeros(nMPC);
        c = zeros(nMPC, 1);

        % (a) 位置跟踪：Σ 1/2 (S u - rhs)' Q (S u - rhs)
        for k = 1:N
            S   = C * Phi_all(:,:,k+1);              % 2 x 2N
            rhs = r(k,:)' - C * Xoff(k+1,:)';        % 2 x 1 （位置中不含 u 的部分）
            H = H + S' * (Q * S);
            c = c - S' * (Q * rhs);
        end
        % 速度阻尼：Σ 1/2 (Sv u - rhsv)' Rv (Sv u - rhsv)
        for k = 1:N
            Sv  = Phi_all(3:4,:,k+1);
            rhsv = v_ref(k,:)' - Xoff(k+1,3:4)';
            H = H + Sv' * (Rv * Sv);
            c = c - Sv' * (Rv * rhsv);
        end
        % 控制能量：Σ 1/2 u_k' R u_k  = 1/2 u' (kron(R,I_N)) u
        H = H + kron(R, eye(N));
        % 末端跟踪：1/2 (Sf u - rhsf)' Qf (Sf u - rhsf)
        Sf  = C * Phi;
        rhsf = r(N,:)' - C * Xoff(N+1,:)';
        H = H + Sf' * (Qf * Sf);
        c = c - Sf' * (Qf * rhsf);

        % (b) 机间防撞惩罚：围绕当前 u_flat 线性化距离残差。
        for k = 1:N
            S  = C * Phi_all(:,:,k+1);
            Pk = P(k+1,:)';
            for ni = 1:nNei
                pj = neighbors_pred(k,:,ni)';
                diff = Pk - pj; dist = norm(diff);
                if dist < dsafe && dist > 1e-3
                    g = -S' * (diff / dist);
                    residual = dsafe - dist;
                    H = H + Wc * (g * g');
                    c = c + Wc * g * (residual - g' * u_flat);
                end
            end
        end

        % (c) 障碍 / 禁飞进入惩罚
        for k = 1:N
            S  = C * Phi_all(:,:,k+1);
            Pk = P(k+1,:)';
            for oi = 1:nObs
                oc = obs_pred(k,:,oi)';
                rEff = obs_rad(oi) + margin;
                diff = Pk - oc; dist = norm(diff);
                if dist < rEff && dist > 1e-3
                    g = -S' * (diff / dist);
                    residual = rEff - dist;
                    H = H + Wo * (g * g');
                    c = c + Wo * g * (residual - g' * u_flat);
                end
            end
        end

        % 求解 H·u = -c（加极小正则项防奇异），并限幅到 amax（姿控执行器上限）
        H = H + 1e-6 * eye(nMPC);
        u_candidate = -(H \ c);
        u_candidate = max(min(u_candidate, W.amax), -W.amax);
        u_flat = 0.5 * u_flat + 0.5 * u_candidate;
    end

    % 数值保护：若优化异常，退化为朝参考首点的加速度限幅
    if any(~isfinite(u_flat(1:2)))
        u0 = max(min(u0g, W.amax), -W.amax);
    else
        u0 = u_flat(1:2)';
    end
end
