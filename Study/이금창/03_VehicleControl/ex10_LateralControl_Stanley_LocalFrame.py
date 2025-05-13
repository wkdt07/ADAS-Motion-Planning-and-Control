import numpy as np
import matplotlib.pyplot as plt

from VehicleModel_Lat import VehicleModel_Lat
from ex06_GlobalFrame2LocalFrame import Global2Local
from ex06_GlobalFrame2LocalFrame import PolynomialFitting
from ex06_GlobalFrame2LocalFrame import PolynomialValue

    
if __name__ == "__main__":
    step_time = 0.1
    simulation_time = 30.0
    Vx = 3.0
    X_ref = np.arange(0.0, 100.0, 0.1)
    Y_ref = 2.0-2*np.cos(X_ref/10)
    num_degree = 3
    num_point = 5
    x_local = np.arange(0.0, 10.0, 0.5)

    class StanleyMethod(object):
        def __init__(self, step_time, k=3.5, wheelbase=2.5):
            self.k = k  # Pure Pursuit의 gain (차량의 위치와 경로의 각도)
            self.L = wheelbase  # 차량의 축간 거리
            self.u = 0.0  # 제어 입력 (조향각)

        def ControllerInput(self, local_points, yaw_angle, vx):
            # 가장 가까운 look-ahead point 찾기
            target = None
            for pt in local_points:
                if pt[0] > 0 and np.hypot(pt[0], pt[1]) >= 2.0:  # look-ahead 거리 제한
                    target = pt
                    break
            
            if target is None:
                self.u = 0.0
                return

            lx, ly = target[0], target[1]
            
            # 경로의 기울기 및 차량 헤딩을 고려한 오차 계산
            path_yaw = np.arctan2(ly, lx + 1e-6)
            
            # Stanley Control Law (k 값 반영)
            delta = path_yaw + np.arctan2(self.k * ly, vx + 1e-5)
            
            # 조향각 계산 후, 제어 입력에 반영
            self.u = delta



    
    time = []
    X_ego = []
    Y_ego = []
    ego_vehicle = VehicleModel_Lat(step_time, Vx)

    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree,num_point)
    polynomialvalue = PolynomialValue(num_degree,np.size(x_local))
    #controller = StanleyMethod(step_time, polynomialfit.coeff, Vx)

    controller = StanleyMethod(step_time)

    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_ego.append(ego_vehicle.X)
        Y_ego.append(ego_vehicle.Y)
        X_ref_convert = np.arange(ego_vehicle.X, ego_vehicle.X+5.0, 1.0)
        Y_ref_convert = 2.0-2*np.cos(X_ref_convert/10)
        Points_ref = np.transpose(np.array([X_ref_convert, Y_ref_convert]))
        frameconverter.convert(Points_ref, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        polynomialfit.fit(frameconverter.LocalPoints)
        polynomialvalue.calculate(polynomialfit.coeff, x_local)
        #controller.ControllerInput(polynomialfit.coeff, Vx)
        controller.ControllerInput(frameconverter.LocalPoints, ego_vehicle.Yaw, Vx)

        ego_vehicle.update(controller.u, Vx)

        
    plt.figure(1)
    plt.plot(X_ref, Y_ref,'k-',label = "Reference")
    plt.plot(X_ego, Y_ego,'b-',label = "Position")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
#    plt.axis("best")
    plt.grid(True)    
    plt.show()


