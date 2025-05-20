% figure;
% hold on;
% 
% % route1_waypoint (기준 경로)
% plot(route1_waypoint(:,1), route1_waypoint(:,2), 'k-', 'LineWidth', 2);  % 검정 실선
% 
% % route2_waypoint (예: 왼쪽 차선)
% plot(route2_waypoint(:,1), route2_waypoint(:,2), 'r--', 'LineWidth', 1.5);  % 빨강 점선
% 
% % route3_waypoint (예: 오른쪽 차선)
% plot(route3_waypoint(:,1), route3_waypoint(:,2), 'b-.', 'LineWidth', 1.5);  % 파랑 점-선
% 
% % 그래프 설정
% legend('route1\_waypoint', 'route2\_waypoint', 'route3\_waypoint');
% title('경로 비교 (route1, route2, route3)');
% xlabel('X [m]');
% ylabel('Y [m]');
% axis equal;
% grid on;
% hold off;
% 

%% 

% plot(route1_waypoint(:,1), route1_waypoint(:,2), 'o-'); axis equal;
% hold on; 
% quiver(route1_waypoint(:,1), route1_waypoint(:,2), ...
%        diff([route1_waypoint(:,1);route1_waypoint(1,1)]), ...
%        diff([route1_waypoint(:,2);route1_waypoint(1,2)]))
% 


%% 

% % 1차선부터 5차선까지 모두 그리기
% figure; hold on; grid on;
% for i = 1:5
%     x = eval(sprintf('route%d_waypoint_x.Data', i));
%     y = eval(sprintf('route%d_waypoint_y.Data', i));
%     plot(x, y, '.-', 'DisplayName', sprintf('Route %d', i));
% end
% xlabel('X [m]');
% ylabel('Y [m]');
% legend show;
% title('All Route Waypoints');
% axis equal;
% hold off;



% load('final_map_waypoint_raw_data.mat');  % 파일명에 맞게

% figure; hold on; grid on;
% labels = {'first', 'second', 'third', 'fourth', 'fifth'};
% for i = 1:5
%     x = eval(sprintf('left_%s_line_point_x.Data', labels{i}));
%     y = eval(sprintf('left_%s_line_point_y.Data', labels{i}));
%     plot(x, y, '.-', 'DisplayName', sprintf('Left %s', labels{i}));
% end
% xlabel('X [m]'); ylabel('Y [m]');
% legend show; axis equal;
% title('Left Side Lines');
% hold off;

% 
% figure; hold on; grid on;
% for i = 1:5
%     x = eval(sprintf('right_%s_line_point_x.Data', labels{i}));
%     y = eval(sprintf('right_%s_line_point_y.Data', labels{i}));
%     plot(x, y, '.-', 'DisplayName', sprintf('Right %s', labels{i}));
% end
% xlabel('X [m]'); ylabel('Y [m]');
% legend show; axis equal;
% title('Right Side Lines');
% hold off;


%% 
% load('final_map_waypoint_raw_data.mat'); % 파일명에 맞게!
% route2_waypoint_x_vec = route2_waypoint_x.Data;
% route2_waypoint_y_vec = route2_waypoint_y.Data;


first_pt_x = cellfun(@(x) x(1,1), local_waypoint{:,1});
first_pt_y = cellfun(@(x) x(1,2), local_waypoint{:,1});
plot(local_waypoint.Time, first_pt_x)
xlabel('Time'), ylabel('첫번째 waypoint X값')

