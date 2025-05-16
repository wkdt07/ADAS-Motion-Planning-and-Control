resolution = 0.5;
function mapMatrix = generate_map_1(map_boundary, traffic_info, traffic_size)
%#codegen
    resolution = 0.5;

    % 1. 경계 처리 (1x8 → x,y 나누기)
    x_values = map_boundary(1:2:end);
    y_values = map_boundary(2:2:end);

    x_min = min(x_values);
    x_max = max(x_values);
    y_min = min(y_values);
    y_max = max(y_values);

    x_size = ceil((x_max - x_min) / resolution);
    y_size = ceil((y_max - y_min) / resolution);

    mapMatrix = zeros(y_size, x_size);

    if isempty(traffic_info) || isempty(traffic_size)
        return;
    end

    N = min(size(traffic_info,1), size(traffic_size,1));

    for i = 1:N
        % 위치
        if size(traffic_info,2) >= 2
            x_c = traffic_info(i,1);
            y_c = traffic_info(i,2);
        else
            x_c = 0;
            y_c = 0;
        end

        % 방향 (yaw)
        if size(traffic_info,2) >= 3
            yaw = traffic_info(i,3);
        else
            yaw = 0;
        end

        % 크기
        if size(traffic_size,2) >= 2
            w = traffic_size(i,1);
            h = traffic_size(i,2);
        else
            w = 11.5;
            h = 2.48;
        end

        % 2. 로컬 사각형 (차량 중심 기준)
        dx = w / 2;
        dy = h / 2;
        local = [ -dx, -dy;
                   dx, -dy;
                   dx,  dy;
                  -dx,  dy ];

        % 회전 행렬
        %yaw = -yaw;
        R = [cos(yaw), -sin(yaw);
             sin(yaw),  cos(yaw)];

        rotated = (R * local')';  % 4x2

        % 중심점 보정 (헤딩 기준 위치 → 차량 중심 위치)
        forward_vec = [cos(yaw), sin(yaw)];
        side_vec = [-sin(yaw), cos(yaw)];
        center = [x_c, y_c] + (w / 2) * forward_vec - (h / 2) * side_vec;

        % 글로벌 좌표 변환
        corners = rotated + center;

        % 3. 맵 인덱스로 변환
        x_idx = (corners(:,1) - x_min) / resolution + 1;
        y_idx = (y_max - corners(:,2)) / resolution + 1;  % y 상하반전
        % y_idx = (corners(:,2) - y_min) / resolution + 1;
        % 4. poly2mask 사용
        mask = poly2mask(x_idx, y_idx, y_size, x_size);
        mapMatrix(mask) = 1;
    end
end


%% 입력값
% 맵 범위 조정
map_boundary = [0 -100 100 -100 100 0 0 0];

traffic_info = [40, -12, 1.5708;
                40, -24, 1.5708;
                40, -36, 1.5708;
                40,-48,1.5708;
       39, -50, 0;
                51, -50, 0;
       63,-50,0;
         ];
traffic_size = repmat([11.5 2.48], 7, 1);

map = generate_map_1(map_boundary, traffic_info, traffic_size);

% 반전 시각화
imagesc(1 - map);
axis equal;
colormap(gray);
hold on
plot(0, 20, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(80, 3, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
title('Occupancy Grid Map (Fixed Range)'); 

%% run_rrt_occupancy.m


% --- 1) 맵 불러오기 또는 생성 --------------------------------------------
% 예시: 이미 workspace 에 mapMatrix(100×100)가 있을 경우 생략 가능
% 만약 파일로 저장했다면:
%   load('mapMatrix.mat','mapMatrix');
% 여기서는 예제로 간단한 L자 모양 장애물을 직접 만듭니다:
% mapMatrix = zeros(100,100);
% mapMatrix(1:50,40) = 1;    % 세로 줄
% mapMatrix(50,40:70) = 1;   % 가로 줄

% --- 2) 시작·목표 정의 --------------------------------------------------
startPose = [1,20];   % (x,y) — 빨간 점
goalPose  = [80,3];  % (x,y) — 초록 점
startPose = round(startPose / resolution) + 1;  % 좌표를 해상도에 맞춰 인덱스로 변환
goalPose = round(goalPose / resolution) + 1;
% --- 3) RRT 파라미터 ----------------------------------------------------
maxIter         = 5000;
stepSize        = 2;     % 각 확장 스텝 길이
goalSampleRate  = 0.1;   % 목표 샘플링 확률
successDistTh   = 5;     % 목표 도달 기준 거리
collisionRes    = 0.5;   % 경로 충돌 검사 분할 간격

% --- 4) 화면에 맵 표시 --------------------------------------------------
figure; 
imagesc(1-map); 
colormap(gray); 
axis equal tight; hold on;
title('RRT on Occupancy Grid');
plot(startPose(1), startPose(2), 'ro','MarkerFaceColor','r','MarkerSize',8);
plot(goalPose(1),  goalPose(2),  'go','MarkerFaceColor','g','MarkerSize',8);

% --- 5) 트리 초기화 ----------------------------------------------------
nodes = struct('x',startPose(1),'y',startPose(2),'parent',0);

% --- 6) RRT 메인 루프 --------------------------------------------------
for iter = 1:maxIter
    % 6.1) 샘플링 (목표 샘플링 포함)
    if rand < goalSampleRate
        xr = goalPose;
    else
        xr = [ randi([1,size(map,2)]), randi([1,size(map,1)]) ];
    end

    % 6.2) 가장 가까운 노드 찾기
    [nearestIdx, nearXY] = findNearest(nodes, xr);

    % 6.3) 새로운 노드 생성
    theta = atan2(xr(2)-nearXY(2), xr(1)-nearXY(1));
    newXY = nearXY + stepSize*[cos(theta), sin(theta)];

    % 6.4) 충돌 검사 (노드+경로)
    if isCollision(newXY, map) || isPathCollision(nearXY, newXY, map, collisionRes)
        continue;
    end

    % 6.5) 트리에 추가 및 시각화
    nodes(end+1) = struct('x', newXY(1), 'y', newXY(2), 'parent', nearestIdx);
    plot([nearXY(1), newXY(1)], [nearXY(2), newXY(2)], '-r');    
    drawnow limitrate

    % 6.6) 목표 도달 확인
    if norm(newXY - goalPose) < successDistTh
        disp('[-] GOAL REACHED');
        path = backtrace(nodes, numel(nodes));
        break;
    end
end

% --- 7) 경로 복원 및 시각화 ---------------------------------------------
if exist('path','var') && ~isempty(path)
    for k = 2:numel(path)
        plot([path(k-1).x, path(k).x], [path(k-1).y, path(k).y], 'b-', 'LineWidth',2);
    end
else
    disp('Error, no path found!');
end


%% ======= 헬퍼 함수들 ========

function [idx, xy] = findNearest(nodes, sampleXY)
    % 노드 리스트에서 sampleXY 와 가장 가까운 노드를 찾습니다.
    N = numel(nodes);
    d = inf(N,1);
    for i=1:N
        d(i) = hypot(nodes(i).x - sampleXY(1), nodes(i).y - sampleXY(2));
    end
    [~, idx] = min(d);
    xy = [nodes(idx).x, nodes(idx).y];
end

function flag = isCollision(pt, map)
    % 단일 점 pt 가 맵 밖이거나 장애물(1) 위인지 검사
    x = round(pt(1)); y = round(pt(2));
    if x<1 || x>size(map,2) || y<1 || y>size(map,1)
        flag = true; 
    else
        flag = (map(y,x)==1);
    end
end

function flag = isPathCollision(p1, p2, map, step)
    % 두 점 사이를 step 간격으로 샘플링해서 장애물 충돌 검사
    dist = hypot(p2(1)-p1(1), p2(2)-p1(2));
    N = max(1, ceil(dist/step));
    flag = false;
    for i=0:N
        pt = p1 + (p2-p1)*(i/N);
        if isCollision(pt, map)
            flag = true;
            return;
        end
    end
end

function path = backtrace(nodes, idx)
    % idx 인덱스 노드부터 root(1)까지 parent를 따라가며 경로 복원
    path = [];
    while idx ~= 0
        path = [nodes(idx), path];
        idx = nodes(idx).parent;
    end
end
