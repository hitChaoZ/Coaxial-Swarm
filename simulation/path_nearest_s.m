function s0 = path_nearest_s(path, pos)
% path_nearest_s —— 返回路径上离 pos 最近点（线段投影）的弧长参数 s
% 注意：必须投影到各条线段上，而非仅比较顶点距离，否则弧长参数无法随
%        UAV 沿路径前进而推进。
    if size(path,1) < 2
        s0 = 0;
        return;
    end
    seg = diff(path, 1, 1);
    segLen = sqrt(sum(seg.^2, 2));
    cum = [0; cumsum(segLen)];
    best = inf; s0 = 0;
    for i = 1:size(seg,1)
        a = path(i,:);   b = path(i+1,:);  ab = b - a;
        t = dot(pos - a, ab) / max(dot(ab,ab), 1e-9);
        t = max(0, min(1, t));
        proj = a + t * ab;
        d = norm(pos - proj);
        s = cum(i) + t * segLen(i);
        if d < best
            best = d; s0 = s;
        end
    end
end
