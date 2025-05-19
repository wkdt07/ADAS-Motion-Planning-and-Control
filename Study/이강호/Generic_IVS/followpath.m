function steering_angle = follow_path(Local_Waypoints, idx_closest)
% Local_Waypoints : Nx2 로컬 좌표계 웨이포인트
% idx_closest     : 현재 위치에서 가장 가까운 웨이포인트 인덱스

    % 파라미터
    L = 4.0;            % 휠베이스
    epsilon = 1e-3;     % 분모 안정화
    reach_thresh = 1.0; % 도달 거리 기준

    % 유효성 검사
    if isempty(Local_Waypoints) || size(Local_Waypoints, 1) < idx_closest
        steering_angle = 0;
        return;
    end

    % 1. 가까운 지점이 너무 가까우면 다음 점 사용
    N = size(Local_Waypoints, 1);
    current_idx = idx_closest;

    % 최소 거리 이상인 첫 번째 점 찾기
    for i = idx_closest:N
        dist = norm(Local_Waypoints(i, :));
        if dist > reach_thresh
            current_idx = i;
            break;
        end
    end

    % 추종할 로컬 타겟 포인트
    x_t = Local_Waypoints(current_idx, 1);
    y_t = Local_Waypoints(current_idx, 2);

    % 조향각 계산 (Pure Pursuit)
    denominator = x_t^2 + y_t^2 + epsilon;
    steering_angle = atan2(2 * L * y_t, denominator);
end
