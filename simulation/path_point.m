function p = path_point(path, s)
% path_point —— 按弧长 s 在路径上插值取点（用于沿路径前进的参考生成）
% 越界则钳制到起/终点。
    if size(path,1) < 2
        p = path(end,:);
        return;
    end
    seg = diff(path, 1, 1);
    segLen = sqrt(sum(seg.^2, 2));
    cum = [0; cumsum(segLen)];
    L = cum(end);
    if s <= 0
        p = path(1,:);
    elseif s >= L
        p = path(end,:);
    else
        p = interp1(cum, path, s);
    end
end
