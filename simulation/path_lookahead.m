function wp = path_lookahead(path, pos, lookahead)
% path_lookahead —— 在路径上取前瞻航点
% 找到离 pos 最近的投影点，沿路径向前 lookahead 距离插值得到目标航点。
% 若已接近路径终点，则返回终点。
    if size(path,1) < 2
        wp = path(end,:);
        return;
    end
    seg = diff(path, 1, 1);
    segLen = sqrt(sum(seg.^2, 2));
    cum = [0; cumsum(segLen)];
    [~, nearest] = min(sum((path - pos).^2, 2));
    s0 = cum(nearest);
    sTarget = s0 + lookahead;
    if sTarget >= cum(end)
        wp = path(end,:);
        return;
    end
    wp = interp1(cum, path, sTarget);
end
