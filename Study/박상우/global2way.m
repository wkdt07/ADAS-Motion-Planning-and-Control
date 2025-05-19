function [local_point, dist, idx_closest] = global_to_local_waypoints( ...
         waypoints_x, waypoints_y, X_ego, Y_ego, head)
%#codegen
% 가장 가까운 글로벌 웨이포인트를 찾아 로컬 좌표 변환
% 추가 출력: idx_closest (현재 선택된 인덱스), N (전체 웨이포인트 수)

    %% 1) 전체 포인트 개수
    N = numel(waypoints_x);

    %% 2) Ego 차량 위치
    %X_ego = vehicle_position(1);
    %Y_ego = vehicle_position(2);

    %% 3) 모든 웨이포인트와 거리 제곱 계산
    dx_all = waypoints_x - X_ego;     % 1×N
    dy_all = waypoints_y - Y_ego;     % 1×N
    d2_all = dx_all.^2 + dy_all.^2;   % 1×N

    %% 4) 최소 거리인 인덱스 찾기
    [~, idx_closest] = min(d2_all);

    %% 5) 해당 점 로컬 좌표 변환
    gx = waypoints_x(idx_closest);
    gy = waypoints_y(idx_closest);
    c  = cos(-head);
    s  = sin(-head);
    R  = [c, -s; s, c];
    d  = R * ([gx; gy] - [X_ego; Y_ego]);
    local_point = d';                  % 1×2

    %% 6) 실제 거리 계산
    dist = sqrt(d2_all(idx_closest));  % 스칼라
end
