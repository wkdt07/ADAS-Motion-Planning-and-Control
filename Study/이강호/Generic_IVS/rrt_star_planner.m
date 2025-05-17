function [x_path, y_path] = rrt_star_planner(map, startPose, goalPose)
    %% 설정
    visualize = false;  % true로 설정 시 그림 출력됨

    %% 입력 포맷 정규화 (행벡터로 통일)%굳이 필요없음 
    startPose = reshape(startPose, 1, []);
    goalPose  = reshape(goalPose, 1, []);

    %% 1. RRT* 파라미터
    maxIter        = 5000;
    stepSize       = 2;
    goalSampleRate = 0.1;
    successDistTh  = 5;
    collisionRes   = 0.5;
    searchRadius   = 10;
    safetyDist     = 5;

    %% 2. 트리 초기화
    nodes = struct('x', startPose(1), 'y', startPose(2), ...
                   'parent', 0, 'cost', 0);

    %% 3. RRT* 메인 루프
    for iter = 1:maxIter
        if rand < goalSampleRate
            xr = goalPose;
        else
            xr = [randi([1, size(map,2)]), randi([1, size(map,1)])];
        end

        [nearestIdx, nearXY] = findNearest(nodes, xr);
        theta = atan2(xr(2)-nearXY(2), xr(1)-nearXY(1));
        newXY = nearXY + stepSize * [cos(theta), sin(theta)];

        if isCollision(newXY, map) || ...
           isPathCollision(nearXY, newXY, map, collisionRes) || ...
           isNearObstacle(newXY, map, safetyDist)
            continue;
        end

        nearIdxs = findNearNodes(nodes, newXY, searchRadius);

        minCost = nodes(nearestIdx).cost + norm(newXY - nearXY);
        minParent = nearestIdx;
        for i = nearIdxs
            pxy = [nodes(i).x, nodes(i).y];
            if ~isPathCollision(pxy, newXY, map, collisionRes)
                c = nodes(i).cost + norm(newXY - pxy);
                if c < minCost
                    minCost = c;
                    minParent = i;
                end
            end
        end

        newNode = struct('x', newXY(1), 'y', newXY(2), ...
                         'parent', minParent, 'cost', minCost);
        nodes(end+1) = newNode;
        newIdx = numel(nodes);

        for i = nearIdxs
            pxy = [nodes(i).x, nodes(i).y];
            c = newNode.cost + norm([newNode.x, newNode.y] - pxy);
            if c < nodes(i).cost && ...
               ~isPathCollision([newNode.x,newNode.y], pxy, map, collisionRes)
                nodes(i).parent = newIdx;
                nodes(i).cost = c;
            end
        end

        if visualize
            plot([nodes(newNode.parent).x, newNode.x], ...
                 [nodes(newNode.parent).y, newNode.y], '-r');
            hold on;
        end

        if norm(newXY - goalPose) < successDistTh
            if visualize
                disp('[-] GOAL REACHED');
            end
            path = backtrace(nodes, newIdx);
            break;
        end
    end

    %% 4. 결과 경로 출력
    if exist('path','var') && ~isempty(path)
        if visualize
            for k = 2:numel(path)
                plot([path(k-1).x, path(k).x], ...
                     [path(k-1).y, path(k).y], 'b-', 'LineWidth', 2);
            end
        end
        x_path = [path.x]';
        y_path = [path.y]';
    else
        x_path = [];
        y_path = [];
    end
end

%% ====== 헬퍼 함수들 ======

function [idx, xy] = findNearest(nodes, sampleXY)
    d = arrayfun(@(n) hypot(n.x - sampleXY(1), n.y - sampleXY(2)), nodes);
    [~, idx] = min(d);
    xy = [nodes(idx).x, nodes(idx).y];
end

function nearIdxs = findNearNodes(nodes, pos, radius)
    nearIdxs = find(arrayfun(@(n) hypot(n.x - pos(1), n.y - pos(2)) <= radius, nodes));
end

function flag = isCollision(pt, map)
    x = round(pt(1)); y = round(pt(2));
    flag = x < 1 || x > size(map,2) || y < 1 || y > size(map,1) || map(y,x) == 1;
end

function flag = isPathCollision(p1, p2, map, step)
    dist = norm(p2 - p1);
    N = max(1, ceil(dist / step));
    flag = false;
    for i = 0:N
        pt = p1 + (p2 - p1) * (i / N);
        if isCollision(pt, map)
            flag = true; return;
        end
    end
end

function flag = isNearObstacle(pt, map, safeDist)
    x0 = round(pt(1)); y0 = round(pt(2));
    r = ceil(safeDist);
    [X, Y] = meshgrid((x0 - r):(x0 + r), (y0 - r):(y0 + r));
    D2 = (X - pt(1)).^2 + (Y - pt(2)).^2;
    mask = D2 <= safeDist^2;
    X = X(mask); Y = Y(mask);
    valid = X >= 1 & X <= size(map,2) & Y >= 1 & Y <= size(map,1);
    flag = any(map(sub2ind(size(map), Y(valid), X(valid))) == 1);
end

function path = backtrace(nodes, idx)
    path = [];
    while idx ~= 0
        path = [nodes(idx), path];
        idx = nodes(idx).parent;
    end
end
