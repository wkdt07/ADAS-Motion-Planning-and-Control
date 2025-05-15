function mapMatrix = ggenerate_map(map_boundary, traffic_info)
    % 예시로 직접 설정한 traffic_size 값
    rows = 100;  % 행 크기
    cols = 100;  % 열 크기
    mapMatrix = zeros(rows, cols);  % 0으로 초기화된 맵

    % 1) map_boundary -> 다각형 내부를 1로 채우기
    pts = reshape(map_boundary, 2, [])';  % [x y] 쌍
    min_x = min(pts(:,1)); 
    max_x = max(pts(:,1));
    min_y = min(pts(:,2)); 
    max_y = max(pts(:,2));

    % zero-division 방지: 범위가 0이면 1로 대체
    dx = max_x - min_x;  if dx == 0, dx = 1; end
    dy = max_y - min_y;  if dy == 0, dy = 1; end

    % 디버깅 출력
    disp(['dx: ', num2str(dx), ', dy: ', num2str(dy)]);
    
    % 좌표를 그리드 인덱스로 변환
    x_idx = round((pts(:,1) - min_x) / dx * (cols - 1)) + 1;
    y_idx = round((pts(:,2) - min_y) / dy * (rows - 1)) + 1;

    % 디버깅 출력
    disp(['x_idx: ', mat2str(x_idx)]);
    disp(['y_idx: ', mat2str(y_idx)]);
    
    % 범위 클리핑
    x_idx = max(1, min(cols, x_idx));
    y_idx = max(1, min(rows, y_idx));

    % 다각형 내부 채우기
    mask = poly2mask(x_idx, y_idx, rows, cols);
    mapMatrix(mask) = 1;

    % 2) traffic_info -> 장애물 영역을 2로 채우기
    for i = 1:size(traffic_info, 1)
        t0x = traffic_info(i,1);
        t0y = traffic_info(i,2);
        r   = traffic_info(i,3);

        x_min = t0x - r;  x_max = t0x + r;
        y_min = t0y - r;  y_max = t0y + r;

        xx = round(([x_min, x_max] - min_x) / dx * (cols - 1)) + 1;
        yy = round(([y_min, y_max] - min_y) / dy * (rows - 1)) + 1;
        
        % 경계값 클리핑
        xx = max(1, min(cols, xx));
        yy = max(1, min(rows, yy));

        mapMatrix(yy(1):yy(2), xx(1):xx(2)) = 2;
    end
end
% 예시 map_boundary와 traffic_info 설정
map_boundary = [40, -12, 40, -24, 40, -36, 40, -47];  % 4개 점으로 다각형 정의
traffic_info = [40, -12, 1.5708; 40, -24, 1.5708; 40, -36, 1.5708; 39, -50, 1.5708; 50, -50, 1.5708];  % 장애물 정보

% 맵 생성 함수 호출
mapMatrix = ggenerate_map(map_boundary, traffic_info);

% 맵을 이미지로 시각화
imshow(mapMatrix);
