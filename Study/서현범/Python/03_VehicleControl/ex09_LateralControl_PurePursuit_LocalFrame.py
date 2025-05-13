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

    class PurePursuit(object):
        def __init__(self, step_time, coeff, Vx,):
            # Code
            wheelbase=2.5
            lookahead=2.0
            self.L = wheelbase       # 차량의 휠베이스
            self.Ld = lookahead      # Lookahead 거리
            self.u = 0.0             # 조향각 출력

        def ControllerInput(self, coeff, Vx):
            # 1. lookahead 위치의 로컬 좌표 계산
            x_look = self.Ld
            y_look = coeff[0]*x_look**3 + coeff[1]*x_look**2 + coeff[2]*x_look + coeff[3]

            # 2. lookahead 방향 각도 계산
            alpha = np.arctan2(y_look, x_look)  # 전방 목표점과 차량 x축 사이의 각도

            # 3. 조향각 계산 (Pure Pursuit 공식)
            self.u = np.arctan2(2 * self.L * np.sin(alpha), self.Ld)
    time = []
    X_ego = []
    Y_ego = []
    ego_vehicle = VehicleModel_Lat(step_time, Vx)

    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree,num_point)
    polynomialvalue = PolynomialValue(num_degree,np.size(x_local))
    controller = PurePursuit(step_time, polynomialfit.coeff, Vx) # coeff는 3차 다항식의 계수들
    
    for i in range(int(simulation_time/step_time)):
        time.append(step_time*i)
        X_ego.append(float(ego_vehicle.X))
        Y_ego.append(float(ego_vehicle.Y))
        X_ref_convert = np.arange(float(ego_vehicle.X), float(ego_vehicle.X)+5.0, 1.0)
        Y_ref_convert = 2.0-2*np.cos(X_ref_convert/10)
        Points_ref = np.transpose(np.array([X_ref_convert, Y_ref_convert]))
        frameconverter.convert(Points_ref, ego_vehicle.Yaw, ego_vehicle.X, ego_vehicle.Y)
        polynomialfit.fit(frameconverter.LocalPoints)
        polynomialvalue.calculate(polynomialfit.coeff, x_local)
        controller.ControllerInput(polynomialfit.coeff, Vx)
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


