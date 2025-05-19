% 환경 초기화
    [x_path, y_path, s_ref, obstacles] = initialize_environment(route2_waypoint_raw);
    plot_environment(route1_waypoint_raw, route2_waypoint_raw, route3_waypoint_raw, obstacles);

    % 초기화
    car_plot = []; dot_plot = []; traj_plots = []; best_traj_plot = [];

    % 시뮬레이션 반복
    for i = 1:3:length(x_path)
        cx = x_path(i); cy = y_path(i);
        [car_plot, dot_plot, traj_plots, best_traj_plot] = clear_previous(car_plot, dot_plot, traj_plots, best_traj_plot);
        [s0, d0] = get_frenet(cx, cy, x_path, y_path);
        [~, ~, yaw] = get_cartesian(s0, d0, x_path, y_path, s_ref);
        [best_traj, traj_plots] = plan_trajectories(s0, d0, s_ref, x_path, y_path, obstacles);

        if ~isempty(best_traj)
            best_traj_plot = plot(best_traj.x, best_traj.y, 'b-', 'LineWidth', 2.5);
        end
        [car_plot, dot_plot] = draw_vehicle(cx, cy, yaw);
        pause(0.05);
    end

function [x_path, y_path, s_ref, obstacles] = initialize_environment(route2)
    x_path = route2(:,1);
    y_path = route2(:,2);
    s_ref = zeros(size(x_path));
    for i = 2:length(x_path)
        dx = x_path(i) - x_path(i-1);
        dy = y_path(i) - y_path(i-1);
        s_ref(i) = s_ref(i-1) + hypot(dx, dy);
    end
    % 초기화 시
    obstacles_sd = [30, 2; 50, -1.5; 70, 0];  % 기존처럼 s, d
    obstacles = zeros(size(obstacles_sd));
    for i = 1:size(obstacles_sd,1)
        [obstacles(i,1), obstacles(i,2), ~] = get_cartesian(obstacles_sd(i,1), obstacles_sd(i,2), x_path, y_path, s_ref);
    end
end

function plot_environment(r1, r2, r3, obstacles)
    figure; grid on; axis equal; hold on;
    set(gca, 'FontSize', 12); xlabel('X 좌표'); ylabel('Y 좌표'); title('Frenet 기반 궤적 시각화');
    plot(r1(:,1), r1(:,2), 'b-', 'LineWidth', 1.5);
    plot(r2(:,1), r2(:,2), 'r-', 'LineWidth', 2);
    plot(r3(:,1), r3(:,2), 'y-', 'LineWidth', 1.5);
    for i = 1:size(obstacles,1)
        plot(obstacles(i,1), obstacles(i,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    end
end

function [car_plot, dot_plot, traj_plots, best_traj_plot] = clear_previous(car_plot, dot_plot, traj_plots, best_traj_plot)
    if ~isempty(traj_plots)
        for i = 1:length(traj_plots)
            if ishandle(traj_plots(i)), delete(traj_plots(i)); end
        end
    end
    if ishandle(best_traj_plot), delete(best_traj_plot); end
    if ishandle(car_plot), delete(car_plot); end
    if ishandle(dot_plot), delete(dot_plot); end
    traj_plots = []; best_traj_plot = []; car_plot = []; dot_plot = [];
end

function [best_traj, traj_plots] = plan_trajectories(s0, d0, s_ref, x_ref, y_ref, obstacles)
    LANE_WIDTH = 3.0; K_J = 0.1; K_T = 0.1; K_D = 1.0;
    K_V = 1.0; K_LAT = 1.0; K_LON = 1.0; COLLISION_DIST = 2.0;

    lateral_offsets = linspace(-2*LANE_WIDTH, 2*LANE_WIDTH, 11);
    T = 3; t = linspace(0, T, 60);
    s0_d = 6.0; s0_dd = 0.0; sT_d = 6.0; sT_dd = 0.0;

    A_s = [3*T^2, 4*T^3; 6*T, 12*T^2]; b_s = [sT_d - (s0_d + s0_dd*T); sT_dd - s0_dd];
    x_s = A_s \ b_s;
    a0_s = s0; a1_s = s0_d; a2_s = 0.5*s0_dd; a3_s = x_s(1); a4_s = x_s(2);
    s_target = a0_s + a1_s*t + a2_s*t.^2 + a3_s*t.^3 + a4_s*t.^4;
    s_jerk = 6*a3_s + 24*a4_s*t;

    best_cost = inf; best_traj = []; traj_plots = [];

    for d = lateral_offsets
        [d_target, d_jerk] = generate_lateral_trajectory(d0, d, T, t);
        [x_traj, y_traj] = frenet_to_global(s_target, d_target, x_ref, y_ref, s_ref);
        if check_collision(x_traj, y_traj, obstacles, COLLISION_DIST), continue; end

        J_lat = sum(d_jerk.^2); J_lon = sum(s_jerk.^2);
        d_diff = (d_target(end) - d)^2; v_diff = (sT_d - s0_d)^2;
        cost = K_LAT*(K_J*J_lat + K_T*T + K_D*d_diff) + ...
               K_LON*(K_J*J_lon + K_T*T + K_V*v_diff);

        traj_plots(end+1) = plot(x_traj, y_traj, '--', 'Color', [0.6 0.6 0.6]);
        if cost < best_cost
            best_cost = cost; best_traj.x = x_traj; best_traj.y = y_traj;
        end
    end
end

function [d_target, d_jerk] = generate_lateral_trajectory(d0, d, T, t)
    di = d0; di_d = 0; di_dd = 0; df = d; df_d = 0; df_dd = 0;
    A_d = [T^3, T^4, T^5; 3*T^2, 4*T^3, 5*T^4; 6*T, 12*T^2, 20*T^3];
    b_d = [df - (di + di_d*T + 0.5*di_dd*T^2); df_d - (di_d + di_dd*T); df_dd - di_dd];
    x_d = A_d \ b_d;
    a0 = di; a1 = di_d; a2 = 0.5*di_dd; a3 = x_d(1); a4 = x_d(2); a5 = x_d(3);
    d_target = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    d_jerk = 6*a3 + 24*a4*t + 60*a5*t.^2;
end

function [x, y] = frenet_to_global(s, d, x_ref, y_ref, s_ref)
    x = zeros(size(s)); y = zeros(size(s));
    for i = 1:length(s)
        [x(i), y(i), ~] = get_cartesian(s(i), d(i), x_ref, y_ref, s_ref);
    end
end

function [car_plot, dot_plot] = draw_vehicle(cx, cy, yaw)
    L = 4; W = 2;
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    rect = [-L/2, L/2, L/2, -L/2; -W/2, -W/2, W/2, W/2];
    rotated = R * rect;
    rotated(1,:) = rotated(1,:) + cx;
    rotated(2,:) = rotated(2,:) + cy;
    car_plot = fill(rotated(1,:), rotated(2,:), 'cyan');
    dot_plot = plot(cx, cy, 'ko', 'MarkerFaceColor', 'k');
end

function collision = check_collision(x, y, obs, th)
    collision = false;
    for i = 1:size(obs,1)
        dx = x - obs(i,1); dy = y - obs(i,2);
        if any(dx.^2 + dy.^2 < th^2), collision = true; return; end
    end
end

function [s, d] = get_frenet(x, y, x_ref, y_ref)
    idx = get_closest_waypoint(x, y, x_ref, y_ref);
    if idx == 1, prev = 1; next = 2;
    elseif idx == length(x_ref), prev = idx-1; next = idx;
    else, prev = idx-1; next = idx;
    end
    vec_path = [x_ref(next)-x_ref(prev); y_ref(next)-y_ref(prev)];
    vec_ego = [x - x_ref(prev); y - y_ref(prev)];
    proj = dot(vec_path, vec_ego) / norm(vec_path)^2 * vec_path;
    perp = vec_ego - proj;
    d = norm(perp); if det([vec_path, vec_ego]) < 0, d = -d; end
    s = sum(sqrt(diff(x_ref(1:prev)).^2 + diff(y_ref(1:prev)).^2)) + norm(proj);
end

function [x, y, heading] = get_cartesian(s, d, x_ref, y_ref, s_ref)
    s = mod(s, s_ref(end));
    idx = find(s_ref <= s, 1, 'last');
    if idx >= length(s_ref), idx = length(s_ref)-1; end
    dx = x_ref(idx+1) - x_ref(idx); dy = y_ref(idx+1) - y_ref(idx);
    heading = atan2(dy, dx); ds = s - s_ref(idx);
    x_base = x_ref(idx) + ds*cos(heading); y_base = y_ref(idx) + ds*sin(heading);
    x = x_base + d*cos(heading + pi/2); y = y_base + d*sin(heading + pi/2);
end

function idx = get_closest_waypoint(x, y, x_ref, y_ref)
    dists = sqrt((x_ref - x).^2 + (y_ref - y).^2);
    [~, idx] = min(dists);
end
