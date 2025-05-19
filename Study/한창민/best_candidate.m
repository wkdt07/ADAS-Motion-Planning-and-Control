% 차량 시각화 파라미터
vehicle_length = 4;
vehicle_width = 2;
LANE_WIDTH = 3;

x_path = route2_waypoint_raw(:,1);
y_path = route2_waypoint_raw(:,2);

s_ref = zeros(size(x_path));
for i = 2:length(x_path)
    dx = x_path(i) - x_path(i-1);
    dy = y_path(i) - y_path(i-1);
    s_ref(i) = s_ref(i-1) + sqrt(dx^2 + dy^2);
end

% 비용 함수 가중치
K_J = 0.1; K_T = 0.1; K_D = 1.0; K_V = 1.0; K_LAT = 1.0; K_LON = 1.0;

figure;
grid on; axis equal;
xlabel('X 좌표'); ylabel('Y 좌표');
title('Frenet 기반 궤적 후보 + 최적 경로 선택');
set(gca, 'FontSize', 12);
hold on;

plot(route1_waypoint_raw(:,1), route1_waypoint_raw(:,2), 'b-', 'LineWidth', 1.5);
plot(route2_waypoint_raw(:,1), route2_waypoint_raw(:,2), 'r-', 'LineWidth', 2);
plot(route3_waypoint_raw(:,1), route3_waypoint_raw(:,2), 'y-', 'LineWidth', 1.5);

car_plot = []; dot_plot = []; traj_plots = []; best_traj_plot = [];

for i = 1:3:length(x_path)
    cx = x_path(i);
    cy = y_path(i);

    % 이전 궤적/차량 삭제
    if exist('traj_plots', 'var')
        for p = traj_plots
            if ishandle(p)
                delete(p);
            end
        end
        traj_plots = [];
    end
    if exist('best_traj_plot', 'var') & ishandle(best_traj_plot)
        delete(best_traj_plot);
    end
    if exist('car_plot', 'var') & ishandle(car_plot)
        delete(car_plot);
    end
    if exist('dot_plot', 'var') & ishandle(dot_plot)
        delete(dot_plot);
    end

    [s0, d0] = get_frenet(cx, cy, x_path, y_path);
    [~, ~, yaw] = get_cartesian(s0, d0, x_path, y_path, s_ref);

    lateral_offsets = linspace(-2*LANE_WIDTH, 2*LANE_WIDTH, 11);
    T = 3;
    t_sample = linspace(0, T, 60);

    % s(t): 종방향 quartic
    s0_d = 6.0; s0_dd = 0.0;
    sT_d = 6.0; sT_dd = 0.0;
    A_s = [3*T^2, 4*T^3; 6*T, 12*T^2];
    b_s = [sT_d - (s0_d + s0_dd*T); sT_dd - s0_dd];
    x_s = A_s \ b_s;
    a0_s = s0; a1_s = s0_d; a2_s = 0.5*s0_dd;
    a3_s = x_s(1); a4_s = x_s(2);
    s_target = a0_s + a1_s*t_sample + a2_s*t_sample.^2 + a3_s*t_sample.^3 + a4_s*t_sample.^4;
    s_jerk = 6*a3_s + 24*a4_s*t_sample;

    best_cost = inf;
    best_traj = [];

    for d = lateral_offsets
        % d(t): 횡방향 quintic
        di = d0; di_d = 0; di_dd = 0;
        df = d;  df_d = 0; df_dd = 0;

        A_d = [T^3, T^4, T^5; 3*T^2, 4*T^3, 5*T^4; 6*T, 12*T^2, 20*T^3];
        b_d = [df - (di + di_d*T + 0.5*di_dd*T^2);
               df_d - (di_d + di_dd*T);
               df_dd - di_dd];
        x_d = A_d \ b_d;
        a0_d = di; a1_d = di_d; a2_d = 0.5*di_dd;
        a3_d = x_d(1); a4_d = x_d(2); a5_d = x_d(3);
        d_target = a0_d + a1_d*t_sample + a2_d*t_sample.^2 + ...
                   a3_d*t_sample.^3 + a4_d*t_sample.^4 + a5_d*t_sample.^5;
        d_jerk = 6*a3_d + 24*a4_d*t_sample + 60*a5_d*t_sample.^2;

        % 비용 계산
        J_lat = sum(d_jerk.^2);
        J_lon = sum(s_jerk.^2);
        d_diff = (d_target(end) - df)^2;
        v_diff = (sT_d - s0_d)^2;
        cost = K_LAT*(K_J*J_lat + K_T*T + K_D*d_diff) + ...
               K_LON*(K_J*J_lon + K_T*T + K_V*v_diff);

        % Frenet → Global 변환
        x_traj = zeros(size(s_target)); y_traj = zeros(size(s_target));
        for k = 1:length(s_target)
            [x_traj(k), y_traj(k), ~] = get_cartesian(s_target(k), d_target(k), x_path, y_path, s_ref);
        end

        traj_plots(end+1) = plot(x_traj, y_traj, '--', 'Color', [0.6 0.6 0.6]);

        if cost < best_cost
            best_cost = cost;
            best_traj.x = x_traj;
            best_traj.y = y_traj;
        end
    end

    best_traj_plot = plot(best_traj.x, best_traj.y, 'b-', 'LineWidth', 2.5);

    % 차량 시각화
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    rect = [-vehicle_length/2, vehicle_length/2, vehicle_length/2, -vehicle_length/2;
            -vehicle_width/2, -vehicle_width/2, vehicle_width/2, vehicle_width/2];
    rect_rotated = R * rect;
    rect_rotated(1,:) = rect_rotated(1,:) + cx;
    rect_rotated(2,:) = rect_rotated(2,:) + cy;

    car_plot = fill(rect_rotated(1,:), rect_rotated(2,:), 'cyan');
    dot_plot = plot(cx, cy, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');

    pause(0.05);
end

%% === 보조 함수들 ===

function [s, d] = get_frenet(x, y, x_ref, y_ref)
    idx = get_closest_waypoint(x, y, x_ref, y_ref);
    if idx == 1
        prev_idx = 1; next_idx = 2;
    elseif idx == length(x_ref)
        prev_idx = length(x_ref) - 1; next_idx = length(x_ref);
    else
        prev_idx = idx - 1; next_idx = idx;
    end
    vec_path = [x_ref(next_idx) - x_ref(prev_idx); y_ref(next_idx) - y_ref(prev_idx)];
    vec_ego  = [x - x_ref(prev_idx); y - y_ref(prev_idx)];
    proj = dot(vec_path, vec_ego) / norm(vec_path)^2 * vec_path;
    perp = vec_ego - proj;
    d = norm(perp);
    if det([vec_path, vec_ego]) < 0
        d = -d;
    end
    s = 0;
    for i = 1:(prev_idx - 1)
        s = s + norm([x_ref(i+1) - x_ref(i), y_ref(i+1) - y_ref(i)]);
    end
    s = s + norm(proj);
end

function [x, y, heading] = get_cartesian(s, d, x_ref, y_ref, s_ref)
    s = mod(s, s_ref(end));
    idx = find(s_ref <= s, 1, 'last');
    if idx >= length(s_ref), idx = length(s_ref) - 1; end
    dx = x_ref(idx+1) - x_ref(idx);
    dy = y_ref(idx+1) - y_ref(idx);
    heading = atan2(dy, dx);
    ds = s - s_ref(idx);
    x_base = x_ref(idx) + ds * cos(heading);
    y_base = y_ref(idx) + ds * sin(heading);
    x = x_base + d * cos(heading + pi/2);
    y = y_base + d * sin(heading + pi/2);
end

function idx = get_closest_waypoint(x, y, x_ref, y_ref)
    dists = sqrt((x_ref - x).^2 + (y_ref - y).^2);
    [~, idx] = min(dists);
end
