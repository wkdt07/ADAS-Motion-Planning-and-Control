import numpy as np
import matplotlib.pyplot as plt
from lane_1 import lane
from ex01_PathPlanning_BothLane import (
    Global2Local,
    Polyfit,
    Polyval,
    BothLane2Path,
    VehicleModel_Lat,
    PurePursuit,
)

def compute_ego_path(
    coeff_path_lead,
    lead_X, lead_Y, lead_yaw,
    ego_X, ego_Y, ego_yaw,
    lookahead=10.0,
    num_pts=20
):
    """
    1) leader의 로컬경로(coeff_path_lead)를 lookahead 길이만큼 샘플링
    2) leader 로컬→전역 좌표 변환
    3) 전역→ego 로컬 좌표 변환
    4) ego의 Polyfit 경로 생성
    """
    # 1) leader 로컬 경로 샘플링
    Xl = np.linspace(0, lookahead, num_pts)
    Yl = Polyval(coeff_path_lead, Xl)

    # 2) leader 로컬→전역 변환
    cos_y = np.cos(lead_yaw)
    sin_y = np.sin(lead_yaw)
    global_pts = []
    for x_l, y_l in zip(Xl, Yl):
        Xg = cos_y * x_l - sin_y * y_l + lead_X
        Yg = sin_y * x_l + cos_y * y_l + lead_Y
        global_pts.append([Xg, Yg])

    # 3) 전역→ego 로컬 변환
    local_pts_ego = Global2Local(global_pts, ego_yaw, ego_X, ego_Y)

    # 4) ego 경로 다항식 생성
    coeff_ego = Polyfit(local_pts_ego, num_order=3)
    return coeff_ego

if __name__ == "__main__":
    # 시뮬레이션 파라미터
    step_time      = 0.1
    simulation_time = 30.0
    Vx             = 3.0

    # 차량 및 컨트롤러 초기화
    leading_vehicle = VehicleModel_Lat(step_time, Vx)
    ego_vehicle     = VehicleModel_Lat(step_time, Vx, Pos=[-10.0, 0.0, 0.0])
    controller_lead = PurePursuit()
    controller_ego  = PurePursuit()

    plt.figure(figsize=(13,2))
    for _ in range(int(simulation_time/step_time)):
        # --- 1) 선행 차량 경로 계산 (lane 기반) ---
        X_ref = np.arange(leading_vehicle.X, leading_vehicle.X + 5.0, 1.0)
        Y_ref_L, Y_ref_R = lane(X_ref)

        global_L = np.transpose([X_ref, Y_ref_L]).tolist()
        global_R = np.transpose([X_ref, Y_ref_R]).tolist()
        local_L  = Global2Local(global_L, leading_vehicle.Yaw, leading_vehicle.X, leading_vehicle.Y)
        local_R  = Global2Local(global_R, leading_vehicle.Yaw, leading_vehicle.X, leading_vehicle.Y)

        coeff_L = Polyfit(local_L, num_order=3)
        coeff_R = Polyfit(local_R, num_order=3)
        coeff_path_lead = BothLane2Path(coeff_L, coeff_R)

        # --- 2) 추종 차량 경로 생성 (leader 경로 → ego 로컬) ---
        coeff_path_ego = compute_ego_path(
            coeff_path_lead,
            leading_vehicle.X, leading_vehicle.Y, leading_vehicle.Yaw,
            ego_vehicle.X,       ego_vehicle.Y,       ego_vehicle.Yaw,
            lookahead=10.0,
            num_pts=20
        )

        # --- 3) 컨트롤러 입력 & 상태 업데이트 ---
        controller_lead.ControllerInput(coeff_path_lead, Vx)
        controller_ego.ControllerInput(coeff_path_ego, Vx)
        leading_vehicle.update(controller_lead.u, Vx)
        ego_vehicle.update(controller_ego.u, Vx)

        # --- 4) 시각화 ---
        plt.plot(leading_vehicle.X, leading_vehicle.Y, 'ro', markersize=4)
        plt.plot(ego_vehicle.X,       ego_vehicle.Y,       'bo', markersize=4)
        plt.axis("equal")
        plt.pause(0.01)

    plt.show()
