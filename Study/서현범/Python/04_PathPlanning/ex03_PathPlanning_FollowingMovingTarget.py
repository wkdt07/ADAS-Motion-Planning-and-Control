import numpy as np
import matplotlib.pyplot as plt
from lane_1 import lane
from ex01_PathPlanning_BothLane import Global2Local, Polyfit, Polyval, BothLane2Path,VehicleModel_Lat, PurePursuit
# 차선 정보 없이 앞 차를 따라가면서 차선 정보를 추정하는 코드


class LeadingVehiclePos(object):
    '''추종할 차량의 위치를 PosArray 버퍼에 저장'''
    def __init__(self, num_data_store=5):
        self.max_num_array = num_data_store
        self.PosArray = []

    def update(self, pos_lead, Vx, yawrate, step_time):
        # pos_lead: [[x, y]] 형태로 들어옴
        if len(self.PosArray) >= self.max_num_array:
            self.PosArray.pop(0) # 오래된 데이터는 제거하고
        self.PosArray.append(pos_lead[0])  # [[x, y]] → [x, y] 형태로 저장




def TargetFollowingPath(PosArray):
    '''PosArray를 기준으로 3차 다항식 피팅하여 경로를 생성'''
    if len(PosArray) < 2:
        return [0.0, 0.0, 0.0, 0.0]  # 데이터 부족 시 평직선


    # x = [p[0] for p in PosArray] # 이렇게 하면 x좌표가 불균등하게 나온다 -> 0 근처의 작은 값만 누적될 수 있음 -> 다항식 만들면 일직선 형태로 나와버림
    x = np.linspace(0, 5, len(PosArray))  # 균등한 간격으로 x 생성
    y = [p[1] for p in PosArray]  # lateral displacement만 사용
    coeff = np.polyfit(x, y, 3)
    return coeff

if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    X_lane = np.arange(0.0, 100.0, 0.1)
    Y_lane_L, Y_lane_R = lane(X_lane)
    
    leading_vehicle = VehicleModel_Lat(step_time, Vx)
    ego_vehicle = VehicleModel_Lat(step_time, Vx, Pos=[-10.0, 0.0, 0.0])
    controller_lead = PurePursuit()
    controller_ego = PurePursuit()
    leading_vehicle_pos = LeadingVehiclePos()
    
    time = []
    X_lead = []
    Y_lead = []
    X_ego = []
    Y_ego = []
    plt.figure(figsize=(13,2))
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_lead.append(leading_vehicle.X)
        Y_lead.append(leading_vehicle.Y)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        # Lane for leading vehicle
        X_ref = np.arange(leading_vehicle.X, leading_vehicle.X+5.0, 1.0)
        Y_ref_L, Y_ref_R = lane(X_ref)
        global_points_L = np.transpose(np.array([X_ref, Y_ref_L])).tolist()
        global_points_R = np.transpose(np.array([X_ref, Y_ref_R])).tolist()
        local_points_L = Global2Local(global_points_L, leading_vehicle.Yaw, leading_vehicle.X, leading_vehicle.Y)
        local_points_R = Global2Local(global_points_R, leading_vehicle.Yaw, leading_vehicle.X, leading_vehicle.Y)
        coeff_L = Polyfit(local_points_L, num_order=3)
        coeff_R = Polyfit(local_points_R, num_order=3)
        coeff_path_lead = BothLane2Path(coeff_L, coeff_R)
        # Path for ego vehicle
        pos_lead = Global2Local([[leading_vehicle.X, leading_vehicle.Y]], ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        # Code. Reference Code : leading_vehicle_pos.update(pos_lead, Vx, ego_vehicle.yawrate, step_time)
        leading_vehicle_pos.update(pos_lead, Vx, ego_vehicle.yawrate, step_time)
        coeff_path_ego = TargetFollowingPath(leading_vehicle_pos.PosArray)
        # Controller input
        controller_lead.ControllerInput(coeff_path_lead, Vx)
        controller_ego.ControllerInput(coeff_path_ego, Vx)
        leading_vehicle.update(controller_lead.u, Vx)
        ego_vehicle.update(controller_ego.u, Vx)
        print(coeff_path_ego)
        print("")
        plt.plot(ego_vehicle.X, ego_vehicle.Y, 'bo')
        plt.plot(leading_vehicle.X, leading_vehicle.Y, 'ro')
        plt.axis("equal")
        plt.pause(0.01)
    plt.show()
                
    #plt.figure(1, figsize=(13,2))
    #plt.plot(X_lead, Y_lead,'r--',label = "Leading vehicle")
    #plt.plot(X_ego, Y_ego,'b.',label = "Vehicle Position")
    #plt.xlabel('X')
    #plt.ylabel('Y')
    #plt.legend(loc="best")
    #plt.grid(True)    
    #plt.show()