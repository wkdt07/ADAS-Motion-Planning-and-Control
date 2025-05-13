import numpy as np
import matplotlib.pyplot as plt
from lane_2 import lane
from ex01_PathPlanning_BothLane import Global2Local, Polyfit, VehicleModel_Lat, PurePursuit

class LaneWidthEstimator(object):
    def __init__(self, Lw_init=3.0):
        self.Lw = Lw_init
        self.valid = False  ## 이번 프레임에서 차선폭 추정 성공 여부

    def update(self, coeff_L, coeff_R, isLaneValid_L, isLaneValid_R):
        self.valid = False

        # 1) 유효한 포인트 수 검사
        min_valid_points = 5
        if np.sum(isLaneValid_L) < min_valid_points or np.sum(isLaneValid_R) < min_valid_points:
            return

        # 2) 전방 0~5m 구간에서 차선 y 좌표 계산
        mid_x = np.linspace(0.0, 5.0, num=10)
        y_L = np.polyval(coeff_L, mid_x)
        y_R = np.polyval(coeff_R, mid_x)

        # 3) NaN/Inf 제거
        valid_mask = (~np.isnan(y_L) & ~np.isnan(y_R) &
                      ~np.isinf(y_L) & ~np.isinf(y_R))
        y_L = y_L[valid_mask]
        y_R = y_R[valid_mask]
        if len(y_L) < 3:
            return

        # 4) 차선 폭 평균 계산
        lane_widths = np.abs(y_R - y_L)
        mean_width = np.mean(lane_widths)

        # 5) 허용 범위 내일 때만 평활화하여 업데이트
        if 2.0 <= mean_width <= 5.0:
            self.Lw = 0.9 * self.Lw + 0.1 * mean_width
            self.valid = True

def EitherLane2Path(coeff_L, coeff_R, isLaneValid_L, isLaneValid_R, Lw):
    """
    좌·우 차선 중 유효한 정보를 골라 중앙 경로 계수를 반환.
    valid_L, valid_R: 각 차선에 대해 유효 포인트 수 >= 5 인지 여부
    """
    MIN_LANE_WIDTH = 2.5

    valid_L = np.sum(isLaneValid_L) >= 5
    valid_R = np.sum(isLaneValid_R) >= 5

    # 양쪽 다 유효하면 평균 계수 사용
    if valid_L and valid_R:
        return (np.array(coeff_L) + np.array(coeff_R)) / 2.0

    # 좌측만 유효할 때, y절편을 +Lw/2 만큼 이동
    elif valid_L:
        shifted = np.array(coeff_L, dtype=float)
        shifted[-1] -= max(Lw, MIN_LANE_WIDTH) / 2.0
        return shifted

    # 우측만 유효할 때, y절편을 -Lw/2 만큼 이동
    elif valid_R:
        shifted = np.array(coeff_R, dtype=float)
        shifted[-1] += max(Lw, MIN_LANE_WIDTH) / 2.0
        return shifted

    # 둘 다 유효하지 않으면 None 반환
    else:
        return None

if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0

    # Reference lane
    X_lane = np.arange(0.0, 100.0, 0.1)
    Y_lane_L, Y_lane_R, Valid_L, Valid_R = lane(X_lane)

    # Estimator, vehicle model, controller 초기화
    LaneWidth = LaneWidthEstimator()
    ego_vehicle = VehicleModel_Lat(step_time, Vx)
    controller = PurePursuit()

    time = []
    X_ego = []
    Y_ego = []

    for i in range(int(simulation_time / step_time)):
        time.append(step_time * i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)

        # 1) 전방 5m 구간 샘플링
        X_ref = np.arange(ego_vehicle.X, ego_vehicle.X + 5.0, 1.0)
        Y_ref_L, Y_ref_R, isLaneValid_L, isLaneValid_R = lane(X_ref)

        # 2) Global → Local 변환
        global_L = np.column_stack([X_ref, Y_ref_L]).tolist()
        global_R = np.column_stack([X_ref, Y_ref_R]).tolist()
        local_L = Global2Local(global_L, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        local_R = Global2Local(global_R, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)

        # 3) 3차 폴리피팅
        coeff_L = Polyfit(local_L, num_order=3)
        coeff_R = Polyfit(local_R, num_order=3)

        # 4) 차선 폭 갱신 & 중앙 경로 추정
        LaneWidth.update(coeff_L, coeff_R, isLaneValid_L, isLaneValid_R)
        coeff_path = EitherLane2Path(coeff_L, coeff_R, isLaneValid_L, isLaneValid_R, LaneWidth.Lw)

        # 5) Pure Pursuit 제어
        controller.ControllerInput(coeff_path, Vx)
        ego_vehicle.update(controller.u, Vx)

    # 결과 시각화
    plt.figure(figsize=(13, 2))
    plt.plot(X_lane, Y_lane_L, 'k--')
    plt.plot(X_lane, Y_lane_R, 'k--', label="Reference")
    plt.plot(X_ego, Y_ego, 'b.', label="Vehicle Position")
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend(loc="best")
    plt.grid(True)
    plt.show()
