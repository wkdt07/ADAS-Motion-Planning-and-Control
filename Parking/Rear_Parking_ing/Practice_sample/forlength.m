% timeseries 또는 timetable에서 데이터 추출
x_path_vals = x_path.Data;   % timetable일 경우 .x 또는 .Variables로 접근
y_path_vals = y_path.Data;
ego_x_vals = Ego_X.Data;  % timeseries 형식이면 .Data 사용
ego_y_vals = Ego_Y.Data;

% 그래프 그리기
figure;
plot(x_path_vals, y_path_vals, 'b-', 'LineWidth', 2); hold on;
plot(ego_x_vals, ego_y_vals, 'r--o', 'LineWidth', 1.5);
legend('Target Path', 'Ego Trajectory');
xlabel('X Position');
ylabel('Y Position');
title('경로 vs 자차 주행 궤적');
grid on; axis equal;
