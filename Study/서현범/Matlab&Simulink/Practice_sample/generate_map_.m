function rrt_star_planner()
    % RRT* 경로 계획기 (Dubins 모델)
    rho = 3;  % 최소 회전반경

    %% 1. 맵 생성
    map_boundary = [0 -100 100 -100 100 0 0 0];
    traffic_info = [40, -12, 1.5708;
                    40, -24, 1.5708;
                    40, -36, 1.5708;
                    40, -48, 1.5708;
                    39, -50, 0;
                    51, -50, 0;
                    63, -50, 0];
    traffic_size = repmat([11.5 2.48], size(traffic_info,1), 1);
    map = generate_map_1(map_boundary, traffic_info, traffic_size);

    %% 2. 시작과 목표 → map 좌표로 변경
    x_vals = map_boundary(1:2:end);
    y_vals = map_boundary(2:2:end);
    x_min = min(x_vals);   x_max = max(x_vals);
    y_min = min(y_vals);   y_max = max(y_vals);
    resolution = 1;

    startPose = [0, -20, pi/2];
    sx = round((startPose(1)-x_min)/resolution)+1;
    sy = round((y_max-startPose(2))/resolution)+1;
    startPose = [sx, sy, startPose(3)];

    goalPose = [80, -3, 0];
    gx = round((goalPose(1)-x_min)/resolution)+1;
    gy = round((y_max-goalPose(2))/resolution)+1;
    goalPose = [gx, gy, goalPose(3)];

    %% 3. 파라미터
    maxIter        = 5000;
    stepSize       = 2;
    goalSampleRate = 0.1;
    successDistTh  = 5;
    safetyDist     = 2;
    searchRadius   = 10;

    %% 4. 시각화
    figure; imagesc(1-map); axis equal tight; hold on;
    colormap(gray);
    plot(startPose(1), startPose(2), 'ro','MarkerFaceColor','r','MarkerSize',8);
    plot(goalPose(1),  goalPose(2),  'go','MarkerFaceColor','g','MarkerSize',8);
    title('RRT* Path Planning with Safety Distance');

    %% 5. 트리 초기화
    nodes = struct('x', startPose(1), 'y', startPose(2), ...
                   'yaw', startPose(3), 'parent', 0, 'cost', 0, 'traj',   {[]});

    %% 6. 메인 루프
    for iter = 1:maxIter
        % 6.1) 샘플링
        if rand < goalSampleRate
            xr = goalPose;
        else
            xr = [ randi([1,size(map,2)]), randi([1,size(map,1)]), 2*pi*rand ];
        end

        % 6.2) 최단 거리 노드 찾기
        [nearestIdx, nearXY] = findNearest(nodes, xr);

        % 6.3) Dubins 경로 계산
        nearState = [nearXY, nodes(nearestIdx).yaw];
        % → 객체 생성: 이름-값 쌍만
        conn = dubinsConnection('MinTurningRadius', rho);
        % → 실제 경로 연결: start, goal
        [pathSegObj, pathCost] = connect(conn, nearState, xr);
        % 일정 간격으로 궤적 점 샘플링
        t = 0:stepSize:pathSegObj{1}.Length;
        if t(end) < pathSegObj{1}.Length
            t(end+1) = pathSegObj{1}.Length;
        end
        poses     = interpolate(pathSegObj{1}, t);  % N×3
        dist_along = [0; cumsum(hypot(diff(poses(:,1)), diff(poses(:,2))))];
        idx = find(dist_along >= stepSize, 1);
        if isempty(idx), idx = size(poses,1); end
        newXY  = poses(idx,1:2);
        newYaw = poses(idx,3);
        rrt_path = poses';

        % 6.4) 충돌·안전거리 검사
        if isCollision(newXY, map) || ...
           isPathCollisionDubins(rrt_path, map) || ...
           isNearObstacle(newXY, map, safetyDist)
            continue;
        end

        % 6.5) 주변 노드 수집
        nearIdxs = findNearNodes(nodes, newXY, searchRadius);

        % 6.6) 최소 비용 부모 선택
        minCost   = nodes(nearestIdx).cost + pathCost;
        minParent = nearestIdx;
        for i = nearIdxs
            fromState = [nodes(i).x, nodes(i).y, nodes(i).yaw];
            toState   = [newXY, newYaw];
            conn2     = dubinsConnection('MinTurningRadius', rho);
            [ps2, cost2] = connect(conn2, fromState, toState);
            if ~isPathCollisionDubins(interpolate(ps2{1}, linspace(0,ps2{1}.Length,50))', map)
                c = nodes(i).cost + cost2;
                if c < minCost
                    minCost   = c;
                    minParent = i;
                end
            end
        end

        % 6.7) 노드 추가
        newNode = struct('x', newXY(1), 'y', newXY(2), ...
                         'yaw', newYaw, 'parent', minParent, 'cost', minCost,'traj',   rrt_path );
        nodes(end+1) = newNode;
        newIdx = numel(nodes);

        % 6.8) 리와이어
        for i = nearIdxs
            fromState = [nodes(i).x, nodes(i).y, nodes(i).yaw];
            toState   = [newXY, newYaw];
            conn2     = dubinsConnection('MinTurningRadius', rho);
            [ps2, cost2] = connect(conn2, fromState, toState);
            if cost2 + newNode.cost < nodes(i).cost && ...
               ~isPathCollisionDubins(interpolate(ps2{1}, linspace(0,ps2{1}.Length,50))', map)
                nodes(i).cost   = cost2 + newNode.cost;
                nodes(i).parent = newIdx;
            end
        end

        % 6.9) 시각화
        plot([nodes(newNode.parent).x, newNode.x], ...
             [nodes(newNode.parent).y, newNode.y], '-r');
        drawnow limitrate

        % 6.10) 목표 도달 확인
        if norm(newXY - goalPose(1:2)) < successDistTh
            disp('GOAL REACHED');
            path = backtrace(nodes, newIdx);
            break;
        end
    end

    %% 7. 최종 경로 시각화 (piecewise-linear→curve)
    if exist('path','var') && ~isempty(path)
        % 1) 백트레이스로 경로 노드 배열(pathNodes) 구하고
        pathNodes = backtrace(nodes, newIdx);

        % 2) 각 segment의 traj(3×Ni)를 옆으로 이어붙이기
        fullPath = [];
        for k = 2:numel(pathNodes)
            fullPath = [ fullPath, pathNodes(k).traj ];  %#ok<AGROW>
        end

        % 3) 궤적 전체를 한번에 그리기 (x,y만 사용)
        plot(fullPath(1,:), fullPath(2,:), 'b-', 'LineWidth',2);
    else
        disp('[-] No path found.');
    end
end

%% ======== 지도 생성 함수 ========
function mapMatrix = generate_map_1(map_boundary, traffic_info, traffic_size)
    resolution = 1;
    x_values = map_boundary(1:2:end);
    y_values = map_boundary(2:2:end);
    x_min = min(x_values);  x_max = max(x_values);
    y_min = min(y_values);  y_max = max(y_values);
    x_size = ceil((x_max - x_min)/resolution);
    y_size = ceil((y_max - y_min)/resolution);
    mapMatrix = zeros(y_size, x_size);

    if isempty(traffic_info) || isempty(traffic_size)
        return;
    end

    N = min(size(traffic_info,1), size(traffic_size,1));
    for i = 1:N
        x_c  = traffic_info(i,1);
        y_c  = traffic_info(i,2);
        yaw  = traffic_info(i,3);
        w    = traffic_size(i,1);
        h    = traffic_size(i,2);
        dx   = w/2;  
        dy   = h/2;
        local = [-dx,-dy; dx,-dy; dx,dy; -dx,dy];
        R     = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
        corners = (R*local')' + [x_c, y_c];
        x_idx   = (corners(:,1)-x_min)/resolution + 1;
        y_idx   = (y_max-corners(:,2))/resolution + 1;
        mask    = poly2mask(x_idx, y_idx, y_size, x_size);
        mapMatrix(mask) = 1;
    end
end

%% ======== 헬퍼 함수들 ========
function [idx, xy] = findNearest(nodes, sampleXY)
    d = arrayfun(@(n) hypot(n.x - sampleXY(1), n.y - sampleXY(2)), nodes);
    [~, idx] = min(d);
    xy = [nodes(idx).x, nodes(idx).y];
end

function nearIdxs = findNearNodes(nodes, pos, radius)
    flags = arrayfun(@(n) hypot(n.x-pos(1), n.y-pos(2)) <= radius, nodes);
    nearIdxs = find(flags);
end

function flag = isCollision(pt, map)
    x = round(pt(1));  y = round(pt(2));
    flag = x<1 || x>size(map,2) || y<1 || y>size(map,1) || map(y,x)==1;
end

function flag = isPathCollisionDubins(rrt_path, map)
    % rrt_path: 3×N array (x; y; yaw)
    N = size(rrt_path,2);
    for k = 1:N
        if isCollision(rrt_path(1:2,k)', map)
            flag = true;
            return;
        end
    end
    flag = false;
end

function flag = isNearObstacle(pt, map, safeDist)
    x0 = round(pt(1));  y0 = round(pt(2));
    r  = ceil(safeDist);
    [X,Y] = meshgrid((x0-r):(x0+r), (y0-r):(y0+r));
    D2 = (X-pt(1)).^2 + (Y-pt(2)).^2;
    mask = D2 <= safeDist^2;
    X = X(mask);  Y = Y(mask);
    valid = X>=1 & X<=size(map,2) & Y>=1 & Y<=size(map,1);
    idx = sub2ind(size(map), Y(valid), X(valid));
    flag = any(map(idx)==1);
end

function path = backtrace(nodes, idx)
    path = nodes(idx);
    while path(1).parent ~= 0
        idx = path(1).parent;
        path = [nodes(idx), path];
    end
end


% rho = 5;  % 최소 회전반경
% 
% 
% function rrt_star_planner()
%     %% 1. 맵 생성
%     map_boundary = [0 -100 100 -100 100 0 0 0];
%     traffic_info = [40, -12, 1.5708;
%                     40, -24, 1.5708;
%                     40, -36, 1.5708;
%                     40, -48, 1.5708;
%                     39, -50, 0;
%                     51, -50, 0;
%                     63, -50, 0];
%     traffic_size = repmat([11.5 2.48], size(traffic_info,1), 1);
%     map = generate_map_1(map_boundary, traffic_info, traffic_size);
% 
%     %% 2. 시작과 목표 -> global 좌표를 map 좌표로 변경
%     x_vals = map_boundary(1:2:end);
%     y_vals = map_boundary(2:2:end);
%     x_min = min(x_vals);   x_max = max(x_vals);
%     y_min = min(y_vals);   y_max = max(y_vals);
%     resolution = 1;
% 
%     startPose = [0, -20,pi/2];
%     sx = round((startPose(1) - x_min)/resolution) + 1;
%     sy = round((y_max - startPose(2))/resolution) + 1;
%     startPose = [sx, sy,startPose(3)];
% 
%     goalPose = [80, -3,0];
%     gx = round((goalPose(1) - x_min)/resolution) + 1;
%     gy = round((y_max - goalPose(2))/resolution) + 1;
%     goalPose  = [gx, gy,goalPose(3)];
% 
%     %% 3. RRT* 파라미터
%     maxIter        = 5000;
%     stepSize       = 2;
%     goalSampleRate = 0.1;
%     successDistTh  = 5;
%     collisionRes   = 0.5;
%     searchRadius   = 10;
%     safetyDist     = 2;   % 장애물과 최소 이격 거리(m)
% 
%     %% 4. 시각화
%     figure;
%     imagesc(1 - map); axis equal tight; hold on;
%     colormap(gray);
%     plot(startPose(1), startPose(2), 'ro','MarkerFaceColor','r','MarkerSize',8);
%     plot(goalPose(1),  goalPose(2),  'go','MarkerFaceColor','g','MarkerSize',8);
%     title('RRT* Path Planning with Safety Distance');
% 
%     %% 5. 트리 초기화
%  nodes = struct('x', startPose(1), 'y', startPose(2), ...
%                 'yaw', startPose(3), ...   % 초기 yaw
%                  'parent', 0, 'cost', 0);
%     %% 6. RRT* 메인 루프
%     for iter = 1:maxIter
% 
%         % 6.1) 샘플링
%         if rand < goalSampleRate
%             xr = goalPose;
%         else
%             randYaw = 2*pi*rand;
%             xr = [ randi([1,size(map,2)]), randi([1,size(map,1)]),randYaw ];
%         end
% 
%         % 6.2) 최단 거리 노드 찾기
%         [nearestIdx, nearXY] = findNearest(nodes, xr);
% 
%         % 6.3) 새 노드 생성
%         % theta = atan2(xr(2)-nearXY(2), xr(1)-nearXY(1));
%         % newXY = nearXY + stepSize * [cos(theta), sin(theta)];
% 
%         nearState = [nearXY(1), nearXY(2), nodes(nearestIdx).yaw];
%         % make a Dubins connection object
%         conn = dubinsConnection(nearState, ...
%             [xr(1),xr(2),xr(3)],...
%             MinTurningRadius = rho);
% 
%         [pathSegObj, pathCost] = connect(conn, nearState, [xr(1), xr(2), xr(3)]);
%         poses   = interpolate(pathSegObj{1}, 0:pathSegObj{1}.Length/100:pathSegObj{1}.Length);
%         lengths = pathSegObj{1}.Length; 
%         % poses: N×3 array of [x, y, yaw]; lengths total path length
% 
%         dist_along = [0; cumsum( sqrt(sum(diff(poses(:,1:2)).^2,2)) )];
%         idx = find(dist_along >= stepSize, 1, 'first');
%        if isempty(idx)  % 만약 전체 경로 길이가 작다면
%            idx = size(poses,1);
%        end
%        newXY  = poses(idx, 1:2);
%        newYaw = poses(idx, 3);
%        rrt_path = poses';  % 3×N
%         % 6.4) 충돌·경로충돌·안전거리 검사
%         if isCollision(newXY, map) || ...
%            isPathCollisionDubins(rrt_path, map) || ...
%            isNearObstacle(newXY, map, safetyDist)
%             continue;
%         end
% 
%         % 6.5) 주변 노드 수집
%         nearIdxs = findNearNodes(nodes, newXY, searchRadius);
% 
%         % 6.6) 최소 비용 부모 선택
%         minCost = nodes(nearestIdx).cost + lengths;
%         minParent = nearestIdx;
%         for i = nearIdxs
%             pxy = [nodes(i).x, nodes(i).y];
%             fromState = [ pxy,        nodes(i).yaw ];
%             toState   = [ newXY,      newYaw ];
%             conn2     = dubinsConnection(fromState,toState,MinTurningRadius=rho);
%             [poses2,~,dp2] = plan(conn2);
%             rrt_path2 = poses2';
% 
%             if ~isPathCollisionDubins(rrt_path2, map)
%                 c = nodes(i).cost + norm(newXY - pxy);
%                 if c < minCost
%                     minCost   = c;
%                     minParent = i;
%                 end
%             end
%         end
% 
%         % 6.7) 노드 추가 및 리와이어링
%         newNode = struct('x', newXY(1), 'y', newXY(2), ...
%                  'yaw', newYaw, 'parent', minParent, 'cost', minCost);
%         nodes(end+1) = newNode;
%         newIdx = numel(nodes);
% 
%         for i = nearIdxs
%             pxy = [nodes(i).x, nodes(i).y];
%             c = newNode.cost + norm([newNode.x, newNode.y] - pxy);
%             fromState = [nodes(i).x, nodes(i).y, nodes(i).yaw];
%             toState   = [newXY, newYaw];
%             conn2     = dubinsConnection(fromState,toState,MinTurningRadius=rho);
%             [poses2,~,dp2] = plan(conn2);
%             rrt_path2 = poses2';
%             if c < nodes(i).cost && ~isPathCollisionDubins(rrt_path2,map)
%                 minCost   = c;
%                 minParent = i;
%             end
%         end
% 
%         % 6.8) 시각화
%         plot([nodes(newNode.parent).x, newNode.x], ...
%              [nodes(newNode.parent).y, newNode.y], '-r');
%         drawnow limitrate
% 
%         % 6.9) 목표 도달 확인
%         if norm(newXY - goalPose) < successDistTh
%             disp('[-] GOAL REACHED');
%             path = backtrace(nodes, newIdx);
%             break;
%         end
%     end
% 
%     %% 7. 경로 시각화
%     if exist('path','var') && ~isempty(path)
%         for k = 2:numel(path)
%             plot([path(k-1).x, path(k).x], ...
%                  [path(k-1).y, path(k).y], 'b-', 'LineWidth',2);
%         end
%     else
%         disp('[-] No path found.');
%     end
% end
% 
% %% ======== generate_map_1 ========
% function mapMatrix = generate_map_1(map_boundary, traffic_info, traffic_size)
%     resolution = 1;
%     x_values = map_boundary(1:2:end);
%     y_values = map_boundary(2:2:end);
%     x_min = min(x_values);  x_max = max(x_values);
%     y_min = min(y_values);  y_max = max(y_values);
%     x_size = ceil((x_max-x_min)/resolution);
%     y_size = ceil((y_max-y_min)/resolution);
%     mapMatrix = zeros(y_size, x_size);
%     if isempty(traffic_info) || isempty(traffic_size)
%         return;
%     end
%     N = min(size(traffic_info,1), size(traffic_size,1));
%     for i = 1:N
%         x_c = traffic_info(i,1);
%         y_c = traffic_info(i,2);
%         yaw = traffic_info(i,3);
%         w = traffic_size(i,1);
%         h = traffic_size(i,2);
%         dx = w/2;  dy = h/2;
%         local = [-dx,-dy; dx,-dy; dx,dy; -dx,dy];
%         R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
%         rotated = (R*local')';
%         forward_vec = [cos(yaw), sin(yaw)];
%         side_vec    = [-sin(yaw), cos(yaw)];
%         center = [x_c, y_c] + (w/2)*forward_vec - (h/2)*side_vec;
%         corners = rotated + center;
%         x_idx = (corners(:,1)-x_min)/resolution + 1;
%         y_idx = (y_max-corners(:,2))/resolution + 1;
%         mask = poly2mask(x_idx, y_idx, y_size, x_size);
%         mapMatrix(mask) = 1;
%     end
% end
% 
% %% ======== 헬퍼 함수들 ========
% function [idx, xy] = findNearest(nodes, sampleXY)
%     N = numel(nodes);
%     d = inf(N,1);
%     for i=1:N
%         d(i) = hypot(nodes(i).x-sampleXY(1), nodes(i).y-sampleXY(2));
%     end
%     [~, idx] = min(d);
%     xy = [nodes(idx).x, nodes(idx).y];
% end
% 
% function nearIdxs = findNearNodes(nodes, pos, radius)
%     nearIdxs = [];
%     for i = 1:numel(nodes)
%         if hypot(nodes(i).x-pos(1), nodes(i).y-pos(2)) <= radius
%             nearIdxs(end+1) = i;
%         end
%     end
% end
% 
% function flag = isCollision(pt, map)
%     x = round(pt(1));  y = round(pt(2));
%     if x<1 || x>size(map,2) || y<1 || y>size(map,1)
%         flag = true;
%     else
%         flag = (map(y,x)==1);
%     end
% end
% 
% function flag = isPathCollision(p1, p2, map, step)
%     dist = hypot(p2(1)-p1(1), p2(2)-p1(2));
%     N    = max(1, ceil(dist/step));
%     flag = false;
%     for i=0:N
%         pt = p1 + (p2-p1)*(i/N);
%         if isCollision(pt, map)
%             flag = true; return;
%         end
%     end
% end
% 
% function flag = isNearObstacle(pt, map, safeDist)
%     x0 = round(pt(1));  y0 = round(pt(2));
%     r  = ceil(safeDist);
%     [X,Y] = meshgrid((x0-r):(x0+r), (y0-r):(y0+r));
%     D2 = (X-pt(1)).^2 + (Y-pt(2)).^2;
%     mask = (D2 <= safeDist^2);
%     X = X(mask);  Y = Y(mask);
%     valid = X>=1 & X<=size(map,2) & Y>=1 & Y<=size(map,1);
%     X = X(valid);  Y = Y(valid);
%     flag = any(map(sub2ind(size(map), Y, X)) == 1);
% end
% 
% function path = backtrace(nodes, idx)
%     path = [];
%     while idx ~= 0
%         path = [nodes(idx), path];
%         idx = nodes(idx).parent;
%     end
% end
% 
% function flag = isPathCollisionDubins(rrt_path, map)
%     % rrt_path: 3×N array (x; y; yaw)
%     flag = false;
%     for k = 1:size(rrt_path,2)
%         pt = rrt_path(1:2,k);
%         if isCollision(pt, map)
%             flag = true;
%             return;
%         end
%     end
% end



%%
%%
%%
% function [x_idx, y_idx] = world2map(x, y, map_boundary, resolution)
%     % 1) 경계값 가져오기
%     x_values = map_boundary(1:2:end);
%     y_values = map_boundary(2:2:end);
%     x_min = min(x_values);
%     y_max = max(y_values);
%     % 2) 변환
%     x_idx = round((x - x_min) / resolution) + 1;
%     y_idx = round((y_max - y)        / resolution) + 1;
% end
% 
% function mapMatrix = generate_map_1(map_boundary, traffic_info, traffic_size)
% %#codegen
%     resolution = 1;
% 
%     % 1. 경계 처리 (1x8 → x,y 나누기)
%     x_values = map_boundary(1:2:end);
%     y_values = map_boundary(2:2:end);
% 
%     x_min = min(x_values);
%     x_max = max(x_values);
%     y_min = min(y_values);
%     y_max = max(y_values);
% 
%     x_size = ceil((x_max - x_min) / resolution);
%     y_size = ceil((y_max - y_min) / resolution);
% 
%     mapMatrix = zeros(y_size, x_size);
% 
%     if isempty(traffic_info) || isempty(traffic_size)
%         return;
%     end
% 
%     N = min(size(traffic_info,1), size(traffic_size,1));
% 
%     for i = 1:N
%         % 위치
%         if size(traffic_info,2) >= 2
%             x_c = traffic_info(i,1);
%             y_c = traffic_info(i,2);
%         else
%             x_c = 0;
%             y_c = 0;
%         end
% 
%         % 방향 (yaw)
%         if size(traffic_info,2) >= 3
%             yaw = traffic_info(i,3);
%         else
%             yaw = 0;
%         end
% 
%         % 크기
%         if size(traffic_size,2) >= 2
%             w = traffic_size(i,1);
%             h = traffic_size(i,2);
%         else
%             w = 11.5;
%             h = 2.48;
%         end
% 
%         % 2. 로컬 사각형 (차량 중심 기준)
%         dx = w / 2;
%         dy = h / 2;
%         local = [ -dx, -dy;
%                    dx, -dy;
%                    dx,  dy;
%                   -dx,  dy ];
% 
%         % 회전 행렬
%         %yaw = -yaw;
%         R = [cos(yaw), -sin(yaw);
%              sin(yaw),  cos(yaw)];
% 
%         rotated = (R * local')';  % 4x2
% 
%         % 중심점 보정 (헤딩 기준 위치 → 차량 중심 위치)
%         forward_vec = [cos(yaw), sin(yaw)];
%         side_vec = [-sin(yaw), cos(yaw)];
%         center = [x_c, y_c] + (w / 2) * forward_vec - (h / 2) * side_vec;
% 
%         % 글로벌 좌표 변환
%         corners = rotated + center;
% 
%         % 3. 맵 인덱스로 변환
%         [x_idx, y_idx] = world2map(corners(:,1), corners(:,2), map_boundary, resolution);
%        % x_idx = round((corners(:,1) - x_min) / resolution + 1);
%        %  y_idx = round((y_max - corners(:,2)) / resolution + 1);
%         % y_idx = (corners(:,2) - y_min) / resolution + 1;
%         % 4. poly2mask 사용
%         mask = poly2mask(x_idx, y_idx, y_size, x_size);
%         mapMatrix(mask) = 1;
%     end
% end
% 
% 
% %% 입력값
% % 맵 범위 조정
% map_boundary = [0 -100 100 -100 100 0 0 0];
% 
% traffic_info = [40, -12, 1.5708;
%                 40, -24, 1.5708;
%                 40, -36, 1.5708;
%                 40,-48,1.5708;
%                 39, -50, 0;
%                 51, -50, 0;
%        63,-50,0;
%          ];
% traffic_size = repmat([11.5 2.48], 7, 1);
% 
% map = generate_map_1(map_boundary, traffic_info, traffic_size);
% 
% % 반전 시각화
% imagesc(1 - map);
% axis equal;
% colormap(gray);
% hold on
% plot(0, 20, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
% plot(80, 3, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
% title('Occupancy Grid Map (Fixed Range)'); 
% 
% %% run_rrt_occupancy.m
% function run_rrt_vehicle_modeling()
%     %% Occupancy Map 생성
%     map_boundary = [0 -100 100 -100 100 0 0 0];
%     traffic_info = [40, -12, 1.5708;
%                     40, -24, 1.5708;
%                     40, -36, 1.5708;
%                     40, -48, 1.5708;
%                     39, -50, 0;
%                     51, -50, 0;
%                     63, -50, 0];
%     traffic_size = repmat([11.5 2.48], 7, 1);
%     map = generate_map_1(map_boundary, traffic_info, traffic_size);
% 
%         % 세계 좌표계 범위 계산
%     x_vals = map_boundary(1:2:end);   % [0,100,100,0]
%     y_vals = map_boundary(2:2:end);   % [-100,-100,0,0]
%     x_min = min(x_vals);   x_max = max(x_vals);
%     y_min = min(y_vals);   y_max = max(y_vals);
% 
% 
%     %% 시작, 목표 정의 및 시각화
%     startPose = [0,-20, pi/2];   % (x,y,yaw) %shb
%     goalPose  = [80,-3];     % (x,y,yaw)
%     [sx, sy] = world2map(0, -20, map_boundary, 1);
%     [gx, gy] = world2map(80, -3, map_boundary, 1);
% 
%     vehicle_size = [11.48, 2.0];  % 차량 길이, 폭 shb
%     resolution = 1;
%     if length(goalPose) < 3
%         goalPose(3) = 0;
%     end
%     %% connectivity 테스트용(임시)
%     free = ~map;
%     labels = bwlabel(free, 4);
%     sInd = sub2ind(size(map), sy, sx);
%     gInd = sub2ind(size(map), gy, gx);
%     if labels(sInd) == labels(gInd)
%         disp('Start와 Goal이 같은 빈 공간에 있습니다. 경로가 있어야 합니다.')
%     else
%         disp('Start와 Goal이 다른 빈 공간에 있습니다. 경로를 만들 수 없습니다.')
%     end
% 
%     [ys, xs] = find(~map);
%     for k = 1:20
%         idx = randi(numel(xs));
%         wx = x_min + (xs(idx)-1)*resolution;
%         wy = y_max - (ys(idx)-1)*resolution;
%         c = isVehicleCollision([wx,wy], 0, vehicle_size, map, map_boundary, resolution);
%         fprintf('(%5.1f, %5.1f) → collision? %d\n', wx, wy, c);
%     end
% %%
% 
% 
% 
% 
%     figure;
%     imagesc(1-map); colormap(gray); axis equal; hold on;
%     title('RRT with Vehicle Body and Steering');
%     plot(sx, sy, 'ro');    % (1,21)에 찍힘
%     plot(gx, gy, 'go');    % (81,4)에 찍힘
% 
%     %% RRT 파라미터
%     maxIter = 5000;
%     stepSize = 1.0; % shb -> 얘는 아님
%     goalSampleRate = 0.1;
%     successDistTh = 7;
%     collisionRes = 1.0;
% 
%     %% 트리 초기화
%     nodes = struct('x', startPose(1), 'y', startPose(2), 'yaw', startPose(3), 'parent', 0);
% 
%     %% 최종 경로 변수 초기화
%     final_path = [];
% 
%     %% RRT 탐색 루프
%     for iter = 1:maxIter
%         if rand < goalSampleRate
%             xr = goalPose;
%         else
%              xr = [ 
%                 x_min + rand*(x_max - x_min),   % x
%                 y_min + rand*(y_max - y_min),   % y
%                 rand*2*pi                       % yaw
%             ];
%         end
% 
%         [nearestIdx, nearXY, nearYaw] = findNearestNode(nodes, xr);
% 
%         valid_found = false;
%         for attempt = 1:10
%             % 1. yaw에 랜덤 노이즈 추가
%             newYaw = atan2(xr(2)-nearXY(2), xr(1)-nearXY(1)) + (rand()-0.5)*pi/4;
% 
%             % 2. 새 노드 위치
%             newXY = nearXY + stepSize * [cos(newYaw), sin(newYaw)];
% 
%             % 3. 경로 보간
%             rrt_path = [linspace(nearXY(1), newXY(1), 10);
%                         linspace(nearXY(2), newXY(2), 10);
%                         linspace(nearYaw, newYaw, 10)];
% 
%             % 4. 충돌 체크
%             if ~isPathCollisionWithBody(rrt_path, map, map_boundary, resolution, vehicle_size)
%                 valid_found = true;
%                 break;
%             end
%         end
% 
%         if ~valid_found
%             continue;  % 10번 시도했는데도 충돌이면 skip
%         end
% 
%         nodes(end+1) = struct('x', newXY(1), 'y', newXY(2), 'yaw', newYaw, 'parent', nearestIdx);
%         plot([nearXY(1), newXY(1)], [nearXY(2), newXY(2)], '-r');
%         drawnow limitrate
% 
%         if norm(newXY - goalPose(1:2)) < successDistTh
%             disp('[-] GOAL REACHED');
%             final_path = backtrace(nodes, numel(nodes));
%             break;
%         end
%     end
% 
%     %% 경로 시각화
%     if ~isempty(final_path)
%         for k = 2:numel(final_path)
%             plot(final_path(k-1).x, final_path(k-1).y, 'bo');
%             plot([final_path(k-1).x, final_path(k).x], [final_path(k-1).y, final_path(k).y], 'b-', 'LineWidth', 2);
%         end
%     else
%         disp('Error, no path found!');
%     end
% end
% 
% function [idx, xy, yaw] = findNearestNode(nodes, sample)
%     min_dist = inf;
%     idx = 1;
%     for i = 1:length(nodes)
%         d = norm([nodes(i).x, nodes(i).y] - sample(1:2));
%         if d < min_dist
%             min_dist = d;
%             idx = i;
%         end
%     end
%     xy = [nodes(idx).x, nodes(idx).y];
%     yaw = nodes(idx).yaw;
% end
% 
% function flag = isPathCollisionWithBody(rrt_path, map, map_boundary, res, vehicle_size)
%     flag = false;
%     for i = 1:size(rrt_path,2)
%         x = rrt_path(1,i);
%         y = rrt_path(2,i);
%         yaw = rrt_path(3,i);
%         if isVehicleCollision([x,y], yaw, vehicle_size, map, map_boundary, res)
%             flag = true;
%             return;
%         end
%     end
% end
%%
% function flag = isVehicleCollision(pt, yaw, vehicle_size, map, map_boundary, res)
%     % 맵 크기
%     [h, w] = size(map);
% 
%     % 1) 차량 사각형 로컬 코너 (센터 기준)
%     len = vehicle_size(1);
%     wid = vehicle_size(2);
%     dx = len/2;
%     dy = wid/2;
%     local = [-dx, -dy;
%               dx, -dy;
%               dx,  dy;
%              -dx,  dy];
% 
%     % 2) 회전 & 전역 이동 → world 좌표계 코너 4개
%     R = [cos(yaw), -sin(yaw);
%          sin(yaw),  cos(yaw)];
%     corners = (R * local')' + pt;   % 4×2
% 
%     % 3) (테스트용) buffer = 0 으로 설정
%     % 실제로 쓸 땐 아래 줄 주석 해제 후 사용
%     % buffer = sqrt(len^2 + wid^2) / 2;
%     buffer = 0;
% 
%     % 4) world coord 기준 buffer 영역 밖 검사
%     x_vals = map_boundary(1:2:end);
%     y_vals = map_boundary(2:2:end);
%     x_min = min(x_vals) + buffer;
%     x_max = max(x_vals) - buffer;
%     y_min = min(y_vals) + buffer;
%     y_max = max(y_vals) - buffer;
%     if any(corners(:,1) < x_min | corners(:,1) > x_max | ...
%            corners(:,2) < y_min | corners(:,2) > y_max)
%         flag = true;
%         return;
%     end
% 
%     % 5) world → map index 변환
%     [x_idx, y_idx] = world2map(corners(:,1), corners(:,2), map_boundary, res);
% 
%     % 6) 맵 밖 클램핑 제거:
%     %    범위를 벗어난다면 곧바로 충돌로 간주
%     if any(x_idx < 1 | x_idx > w | y_idx < 1 | y_idx > h)
%         flag = true;
%         return;
%     end
% 
%     % 7) poly2mask 호출 (정상 인덱스만 들어감)
%     mask = poly2mask(x_idx, y_idx, h, w);
% 
%     % 8) 실제 장애물(U)와 충돌 여부
%     flag = any(map(mask) == 1);
% end

% function flag = isVehicleCollision(pt, yaw, vehicle_size, map, map_boundary, resolution)
%     % map 크기
%     [h, w] = size(map);
% 
%     % 1) 차량 footprint 로컬 코너 (센터 기준)
%     len = vehicle_size(1);
%     wid = vehicle_size(2);
%     dx  = len/2;
%     dy  = wid/2;
%     local = [ -dx, -dy;
%                dx, -dy;
%                dx,  dy;
%               -dx,  dy ];
% 
%     % 2) 회전 + 전역 이동 → world 좌표계 corner 4개
%     R       = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
%     corners = (R * local')' + pt;  % 4×2
% 
%     % 3) world → map index
%     [x_idx, y_idx] = world2map(corners(:,1), corners(:,2), map_boundary, resolution);
% 
%     % 4) 인덱스 클램핑 (맵 밖은 그냥 경계 픽셀로)
%     x_idx = min(max(x_idx, 1), w);
%     y_idx = min(max(y_idx, 1), h);
% 
%     % 5) poly2mask으로 장애물 영역 마스크
%     mask = poly2mask(x_idx, y_idx, h, w);
% 
%     % 6) 실제 장애물과 겹치면 충돌
%     flag = any(map(mask) == 1);
% end

%%
% function flag = isVehicleCollision(pt, yaw, vehicle_size, map, map_boundary, res)
%     % x_min = map_boundary(1);
%     % y_max = map_boundary(6); % y 좌표 최대값은 경계의 6번째 값입니다
%     % [h, w] = size(map);
%     % 
%     % len = vehicle_size(1);
%     % wid = vehicle_size(2);
%     % dx = len/2; dy = wid/2;
%     % local = [-dx,-dy; dx,-dy; dx,dy; -dx,dy];
%     % R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
%     % rotated = (R * local')';
%     % corners = rotated + pt;
%     % 
%     % x_idx = round((corners(:,1) - x_min)/res + 1);
%     % y_idx = round((y_max - corners(:,2))/res + 1);
% 
%     [x_idx, y_idx] = world2map(corners(:,1), corners(:,2), map_boundary, res);
%     if any(x_idx<1 | x_idx>w | y_idx<1 | y_idx>h)
%         disp(['[OUT OF BOUND] x = ' num2str(x_idx') ', y = ' num2str(y_idx')]);
%         flag = true; return;
%     end
% 
%     mask = poly2mask(x_idx, y_idx, h, w);
%     if any(map(mask)==1)
%         flag = true;
%     else
%         flag = false;
%     end
% end

% function final_path = backtrace(nodes, idx)
%     final_path = [];
%     while idx ~= 0
%         final_path = [nodes(idx), final_path];
%         idx = nodes(idx).parent;
%     end
% end
% function drawVehicleDebugBox(x, y, yaw, vehicle_size, color)
%     % 차량 박스를 현재 위치(x, y), 방향(yaw) 기준으로 그려줌
%     len = vehicle_size(1);
%     wid = vehicle_size(2);
%     dx = len/2;
%     dy = wid/2;
% 
%     local_box = [-dx, -dy;
%                   dx, -dy;
%                   dx,  dy;
%                  -dx,  dy];
% 
%     R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
%     global_box = (R * local_box')' + [x, y];
% 
%     patch(global_box(:,1), global_box(:,2), color, 'FaceAlpha', 0.3, 'EdgeColor','k','LineWidth', 1.5);
% end
% run_rrt_vehicle_modeling();


% 
% function mapMatrix = generate_map_(map_boundary, traffic_info, traffic_size)
% %#codegen
%     resolution = 1;
% 
%     % 1. 경계 처리 (1x8 → x,y 나누기)
%     x_values = map_boundary(1:2:end);
%     y_values = map_boundary(2:2:end);
% 
%     x_min = min(x_values);
%     x_max = max(x_values);
%     y_min = min(y_values);
%     y_max = max(y_values);
% 
%     x_size = ceil((x_max - x_min) / resolution);
%     y_size = ceil((y_max - y_min) / resolution);
% 
%     mapMatrix = zeros(y_size, x_size);
% 
%     if isempty(traffic_info) || isempty(traffic_size)
%         return;
%     end
% 
%     N = min(size(traffic_info,1), size(traffic_size,1));
% 
%     for i = 1:N
%         % 위치
%         if size(traffic_info,2) >= 2
%             x_c = traffic_info(i,1);
%             y_c = traffic_info(i,2);
%         else
%             x_c = 0;
%             y_c = 0;
%         end
% 
%         % 방향 (yaw)
%         if size(traffic_info,2) >= 3
%             yaw = traffic_info(i,3);
%         else
%             yaw = 0;
%         end
% 
%         % 크기
%         if size(traffic_size,2) >= 2
%             w = traffic_size(i,1);
%             h = traffic_size(i,2);
%         else
%             w = 11.5;
%             h = 2.48;
%         end
% 
%         % 2. 로컬 사각형 (차량 중심 기준)
%         dx = w / 2;
%         dy = h / 2;
%         local = [ -dx, -dy;
%                    dx, -dy;
%                    dx,  dy;
%                   -dx,  dy ];
% 
%         % 회전 행렬
%         %yaw = -yaw;
%         R = [cos(yaw), -sin(yaw);
%              sin(yaw),  cos(yaw)];
% 
%         rotated = (R * local')';  % 4x2
% 
%         % 중심점 보정 (헤딩 기준 위치 → 차량 중심 위치)
%         forward_vec = [cos(yaw), sin(yaw)];
%         side_vec = [-sin(yaw), cos(yaw)];
%         center = [x_c, y_c] + (w / 2) * forward_vec - (h / 2) * side_vec;
% 
%         % 글로벌 좌표 변환
%         corners = rotated + center;
% 
%         % 3. 맵 인덱스로 변환
%         x_idx = (corners(:,1) - x_min) / resolution + 1;
%         y_idx = (y_max - corners(:,2)) / resolution + 1;  % y 상하반전
%         % y_idx = (corners(:,2) - y_min) / resolution + 1;
%         % 4. poly2mask 사용
%         mask = poly2mask(x_idx, y_idx, y_size, x_size);
%         mapMatrix(mask) = 1;
%     end
% end
% 
