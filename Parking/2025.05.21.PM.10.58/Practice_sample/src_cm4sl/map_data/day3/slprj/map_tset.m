% 맵 크기 설정
map_size = 1000;

% 장애물 개수
num_obstacles = 30;

% Figure 초기화
figure;
hold on;
axis equal;
xlim([0, map_size]);
ylim([0, map_size]);
xlabel('X');
ylabel('Y');
title('1000x1000 Map with Random Obstacles');

% 배경 그리드 (optional)
% grid on;

% 장애물 추가
for i = 1:num_obstacles
    % 장애물 위치 및 크기
    x = randi([1, map_size - 50]);
    y = randi([1, map_size - 50]);
    w = randi([10, 50]);
    h = randi([10, 50]);

    % 사각형 그리기 (장애물)
    rectangle('Position', [x, y, w, h], ...
              'FaceColor', 'k', 'EdgeColor', 'k');
end

% 시작/목표 지점 예시 (옵션)
plot(50, 50, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');     % 시작점
plot(950, 950, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');   % 목표점

hold off;
