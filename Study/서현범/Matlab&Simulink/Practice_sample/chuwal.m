close all

% === 전역 변수 초기화 ===
global K_J K_T K_D K_V W_LAT W_LON
% K_J Jerk 웨이트 s,d 둘다 , K_T t_f에 대한 항, K_D Diff d에대한 diff, K_V s에 대한 diff(속도 s_d에 대한 diff) 

K_J = 0.1; K_T = 1; K_D = 1; K_V = 1;

W_LAT = 1.0; W_LON = 1.0; %종방향 횡방향 웨이트

global fp_templet 
fp_templet= struct( ...
    't', [], 'd', 0, 'd_d', 0, 'd_dd', 0, 'd_ddd', 0, ...
    's', 0, 's_d', 0, 's_dd', 0, 's_ddd', 0, ...
    'c_lat', 0, 'c_lon', 0, 'c_tot', 0, ...
    'x', [], 'y', [], 'yaw', [], 'ds', [], 'kappa', []);

%%

% === 초기 설정 === %
mapx = route2_waypoint(:,1);
mapy = route2_waypoint(:,2);

% 기준 경로 누적 거리 %종방향 map 그리기 
s_ref = zeros(size(mapx));
for i = 2:length(mapx)
    dx = mapx(i) - mapx(i-1);
    dy = mapy(i) - mapy(i-1);
    s_ref(i) = s_ref(i-1) + sqrt(dx^2 + dy^2);
end

% === 장애물 Frenet 좌표 정의 === s,d 좌표계
% obs_sd = [
%           0, 6;
%           112, 0;
%           31.5, -6
%           113.5 -6
%           ];

obs_sd = [
            30,3;
            30,-3
]

% Global 좌표 변환
obs_global = zeros(size(obs_sd));
for i = 1:size(obs_sd, 1)
    [x_obs, y_obs, ~] = get_cartesian(obs_sd(i,1), obs_sd(i,2), mapx, mapy, s_ref);
    obs_global(i,:) = [x_obs, y_obs];
end

% === 초기 자차 위치 및 상태 ===
x0 = mapx(10);  % 차선 바깥에서 시작
y0 = mapy(10); % 
yaw0 = 0;%
v0 = 0; a0 = 0; %% 초기값 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[s0, d0] = get_frenet(x0, y0, mapx, mapy); 
[~, ~, yaw_path] = get_cartesian(s0, d0, mapx, mapy, s_ref);
yawi = yaw0 - yaw_path;

% Frenet 상태 초기화
si = s0;
si_d = v0 * cos(yawi);
si_dd = a0 * cos(yawi);
sf_d = 1.0; sf_dd = 0;

di = d0;
di_d = v0 * sin(yawi);
di_dd = a0 * sin(yawi);
df_d = 0; df_dd = 0;

opt_d = di; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure;
grid on; axis equal;
xlabel('X 좌표'); ylabel('Y 좌표');
title('Frenet 기반 궤적 후보 + 최적 경로 선택');
set(gca, 'FontSize', 12);
hold on;

plot(route1_waypoint(:,1), route1_waypoint(:,2), 'b-', 'LineWidth', 1.5);
plot(route2_waypoint(:,1), route2_waypoint(:,2), 'r-', 'LineWidth', 2);
plot(route3_waypoint(:,1), route3_waypoint(:,2), 'y-', 'LineWidth', 1.5);



% 반복 시각화용 변수 초기화
dot_plot = []; best_traj_plot =[]; traj_plots=[]; obs_plots = [];
% 종료 조건 설정 (기준 경로 마지막 s값 기준)
while si < s_ref(end) - 5


    for i = 1:size(obs_sd, 1)
        if i == 1 || i == 2
            obs_sd(i,1) = obs_sd(i,1) + 1.0;  % 빠르게 이동하는 장애물
        elseif i == 3 || i == 4
            obs_sd(i,1) = obs_sd(i,1) + 1.0;  % 느리게 이동하는 장애물
        end
    end


    % s,d → x,y 변환 후 전역 장애물 위치 갱신
    for i = 1:size(obs_sd, 1)
        [x_obs, y_obs, ~] = get_cartesian(obs_sd(i,1), obs_sd(i,2), mapx, mapy, s_ref);
        obs_global(i,:) = [x_obs, y_obs];
    end
    % 후보 경로 및 최적 경로 계산
    [path, opt_ind] = frenet_optimal_planning(si, si_d, si_dd,...
                                              sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, ...
                                              obs_sd, mapx, mapy, s_ref, opt_d);

    %==== 기존 시각화 삭제 =====%
    if exist('traj_plots', 'var') && ~isempty(traj_plots)
        for p = traj_plots
            if ishandle(p)
                delete(p);
            end
        end
        traj_plots = [];
    end

    if exist('best_traj_plot', 'var') && ~isempty(best_traj_plot) && all(ishandle(best_traj_plot))

        delete(best_traj_plot);
    end

    if exist('dot_plot', 'var') && ~isempty(dot_plot) && ishandle(dot_plot)
        delete(dot_plot);
    end

    if ~isempty(obs_plots)
        for p = obs_plots
            if ishandle(p)
                delete(p);
            end
        end
        obs_plots = [];
    end

    %==== 현재 상태 갱신 =====%
    si    = path(opt_ind).s(7);
    si_d  = path(opt_ind).s_d(7);
    si_dd = path(opt_ind).s_dd(7);

    di    = path(opt_ind).d(7);
    di_d  = path(opt_ind).d_d(7);
    di_dd = path(opt_ind).d_dd(7);

    opt_d = path(opt_ind).d(end);  % 일관성 유지용
        length(path(opt_ind).x)

    %==== 시각화 =====%
    for k = 1:length(path)
        traj_plots(end+1) = plot(path(k).x, path(k).y, '--', 'Color', [0.6 0.6 0.6]);
    end

    best_traj_plot = plot(path(opt_ind).x, path(opt_ind).y, 'b-', 'LineWidth', 2.5);

    car_x = path(opt_ind).x(1);
    car_y = path(opt_ind).y(1);
    dot_plot = plot(car_x, car_y, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');

    drawnow;     % 그래프 갱신
      % 애니메이션처럼 보이게

    % === 장애물 시각화 ===
    for i = 1:size(obs_global,1)
        obs_plots(i) = plot(obs_global(i,1), obs_global(i,2), 's', ...
            'MarkerSize', 10, ...
            'MarkerEdgeColor', 'k', ...
            'MarkerFaceColor', 'r');
    end

    pause(0.05);

end

function [path, opt_ind] = frenet_optimal_planning(si, si_d, si_dd, ...
                                              sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, ...
                                              obs, mapx, mapy, s_ref, opt_d)

    % 1. Frenet 궤적 후보 생성
    fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d);

    % 2. Global 궤적 변환
    fplist = calc_global_paths(fplist, mapx, mapy, s_ref);

    % 3. 제약조건 및 충돌 검사
    fplist = check_path(fplist, obs, mapx, mapy, s_ref);

    % 4. 최적 경로 선택
    min_cost = inf;
    opt_ind = 1;  % 초기값
    for i = 1:length(fplist)
        if fplist(i).c_tot < min_cost
            min_cost = fplist(i).c_tot;
            opt_ind = i;
        end
    end

    % 5. 출력
    path = fplist; 
end

%%
%s,d 조합에 따른 frenet 경로를 생성 -> 비용함수 값도 함께 전달 
function fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)
    global K_J K_T K_D K_V W_LAT W_LON TARGET_SPEED MIN_T MAX_T DT_T DT DF_SET fp_templet
    TARGET_SPEED=20;
    DF_SET = linspace(-6, 6, 3); 
    DT = 0.2; 
    DT_T = 0.5; 
    MIN_T = 11.0; 
    MAX_T = 14.0; 
    
    fplist = repmat(fp_templet, 1, 0);  % 빈 구조체 배열로 초기화
    fp=fp_templet;
    for df = DF_SET
        for T = MIN_T:DT_T:MAX_T
            fp=fp_templet;
            % Lateral trajectory (Quintic)
            [a0_d, a1_d, a2_d, a3_d, a4_d, a5_d] = calc_quintic(di, di_d, di_dd, df, df_d, df_dd, T);
            
            t = 0:DT:T;
     
            d = a0_d + a1_d*t + a2_d*t.^2 + a3_d*t.^3 + a4_d*t.^4 + a5_d*t.^5;
            d_d = a1_d + 2*a2_d*t + 3*a3_d*t.^2 + 4*a4_d*t.^3 + 5*a5_d*t.^4;
            d_dd = 2*a2_d + 6*a3_d*t + 12*a4_d*t.^2 + 20*a5_d*t.^3;
            d_ddd = 6*a3_d + 24*a4_d*t + 60*a5_d*t.^2;
            
            
            % Longitudinal trajectory (Quartic)
            %sf_d에 Target Speed를 기준으로 여러개 생성해야된다
            %sf 인자는 어디로 갔나
            [a0_s, a1_s, a2_s, a3_s, a4_s] = calc_quartic(si, si_d, si_dd, sf_d, sf_dd, T);
            s = a0_s + a1_s*t + a2_s*t.^2 + a3_s*t.^3 + a4_s*t.^4;
            s_d = a1_s + 2*a2_s*t + 3*a3_s*t.^2 + 4*a4_s*t.^3;
            s_dd = 2*a2_s + 6*a3_s*t + 12*a4_s*t.^2;
            s_ddd = 6*a3_s + 24*a4_s*t;
            
           
            %% 비용 계산
            J_lat = sum(d_ddd.^2); %적분항 인데 왜 T로 안나눠 임마 
            J_lon = sum(s_ddd.^2); %적분항 
            d_diff = (d(end) - opt_d)^2;  %차선변경 최소화
            s_diff = (TARGET_SPEED - s_d(end))^2; %ACC랑 최대한 유지하기 위함
            
            %% 비용함수
            c_lat = K_J * J_lat  + K_D * d_diff + K_T * T; %% cost_lateral
            c_lon = K_J * J_lon  + K_V * s_diff + K_T * T; %% cost_longitual
            c_tot = W_LAT * c_lat + W_LON * c_lon; %% total Cost가 낮은것을 선택 

            fp = struct('t', t, 'd', d, 'd_d', d_d, 'd_dd', d_dd, 'd_ddd', d_ddd, ...
            's', s, 's_d', s_d, 's_dd', s_dd, 's_ddd', s_ddd, ...
            'c_lat', c_lat, 'c_lon', c_lon, 'c_tot', c_tot, ...
            'x', [], 'y', [], 'yaw', [], 'ds', [], 'kappa', []);

            fplist(end+1) = fp;
        end 
    end
end

function fplist = calc_global_paths(fplist, mapx, mapy, s_ref)
    for i = 1:length(fplist)
        fp = fplist(i);
        fp.x = zeros(size(fp.s));
        fp.y = zeros(size(fp.s));
        fp.yaw = zeros(size(fp.s));
        fp.ds = zeros(size(fp.s)); % 곡률계산을 변환을 위해 
        fp.kappa = zeros(size(fp.s)); 

        for j = 1:length(fp.s)
            [fp.x(j), fp.y(j), ~] = get_cartesian(fp.s(j), fp.d(j), mapx, mapy, s_ref);
        end

        for j = 1:length(fp.x)-1
            dx = fp.x(j+1) - fp.x(j);
            dy = fp.y(j+1) - fp.y(j);
            fp.yaw(j) = atan2(dy, dx);
            fp.ds(j) = hypot(dx, dy);
        end
        fp.yaw(end) = fp.yaw(end-1);
        fp.ds(end) = fp.ds(end-1);

        for j = 1:length(fp.yaw)-1
            dyaw = atan2(sin(fp.yaw(j+1) - fp.yaw(j)), cos(fp.yaw(j+1) - fp.yaw(j)));
            fp.kappa(j) = dyaw / fp.ds(j);
        end
        fp.kappa(end) = fp.kappa(end-1);

        fplist(i) = fp;
    end
end


function fplist = check_path(fplist, obs, mapx, mapy, s_ref)
    global V_MAX ACC_MAX K_MAX COL_CHECK fp_templet
    V_MAX = 10;
    ACC_MAX=10;
    K_MAX=4; %
    COL_CHECK=0.25;
    valid_paths = repmat(fp_templet, 1, 0);  % 빈 구조체 배열로 초기화
   

    for i = 1:length(fplist)
        fp = fplist(i);
        acc_squared = fp.s_dd.^2 + fp.d_dd.^2;

        if all(fp.s_d) > V_MAX
            continue;
        elseif any(acc_squared > ACC_MAX^2)
            continue;
        elseif any(abs(fp.kappa) > K_MAX)
            continue;
        elseif collision_check(fp, obs, mapx, mapy, s_ref)
            continue;
        else
            valid_paths(end+1) = fp; 
        end
    end

    fplist = valid_paths;
end

function collision = collision_check(fp, obs, mapx, mapy, s_ref)
    
    CAR_RADIUS = 2;
    OBSTACLE_RADIUS = 0.5;
    SAFE_DIST = CAR_RADIUS + OBSTACLE_RADIUS;

    collision = false;
    for i = 1:size(obs,1)
        [x_obs, y_obs, ~] = get_cartesian(obs(i,1), obs(i,2), mapx, mapy, s_ref);
        dists = (fp.x - x_obs).^2 + (fp.y - y_obs).^2;
        if any(dists <= SAFE_DIST^2)
            collision = true;
            return;
        end
    end
end





%====좌표축변환===%

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

function [a0, a1, a2, a3, a4, a5] = calc_quintic(xi, vi, ai, xf, vf, af, T)
    a0 = xi;
    a1 = vi;
    a2 = 0.5 * ai;
    A = [T^3, T^4, T^5;
         3*T^2, 4*T^3, 5*T^4;
         6*T, 12*T^2, 20*T^3];
    b = [xf - a0 - a1*T - a2*T^2;
         vf - a1 - 2*a2*T;
         af - 2*a2];
    x = A\b;
    a3 = x(1); a4 = x(2); a5 = x(3);
end

function [a0, a1, a2, a3, a4] = calc_quartic(xi, vi, ai, vf, af, T)
    a0 = xi;
    a1 = vi;
    a2 = 0.5 * ai;
    A = [3*T^2, 4*T^3;
         6*T, 12*T^2];
    b = [vf - a1 - 2*a2*T;
         af - 2*a2];
    x = A\b;
    a3 = x(1); a4 = x(2);
end