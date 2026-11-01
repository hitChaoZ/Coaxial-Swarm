function p2 = resample_path(path, ds)
% resample_path —— 将路径按固定弧长间隔 ds 重新加密采样，
% 使 path_point 给出的参考轨迹更平滑（避免 RRT 原始折线造成的尖角过冲）。
    if size(path,1) < 2
        p2 = path;
        return;
    end
    seg = diff(path, 1, 1);
    segLen = sqrt(sum(seg.^2, 2));
    cum = [0; cumsum(segLen)];
    L = cum(end);
    snew = 0:ds:L;
    if snew(end) < L            % 保证末点（目标点）一定被包含
        snew(end+1) = L;
    end
    p2 = interp1(cum, path, snew);
end
