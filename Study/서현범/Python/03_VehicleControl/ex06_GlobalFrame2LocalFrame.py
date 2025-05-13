import numpy as np
import matplotlib.pyplot as plt

# global 좌표의 waypoint를 local 좌표로 변환하는 클래스
# local 좌표계는 차량의 위치를 원점으로 하고, 차량의 진행 방향을 x축으로 하는 좌표계
# 차량의 위치는 (X_ego, Y_ego)이고, 차량의 진행 방향은 Yaw_ego로 주어진다.
# 차량의 진행 방향은 시계 방향으로 증가하는 각도로 정의된다.
# 차량의 진행 방향을 기준으로 좌표계를 회전시키는 변환 행렬을 사용하여 global 좌표를 local 좌표로 변환한다.
class Global2Local(object):
    def __init__(self, num_points):
        self.n = num_points
        self.GlobalPoints = np.zeros((num_points,2))
        self.LocalPoints = np.zeros((num_points,2))
    
    def convert(self, points, Yaw_ego, X_ego, Y_ego):
        # Code
        self.GlobalPoints = points
        for i in range(self.n):
            dx = points[i][0] - X_ego
            dy = points[i][1] - Y_ego
            self.LocalPoints[i][0] =  dx * np.cos(Yaw_ego) + dy * np.sin(Yaw_ego)
            self.LocalPoints[i][1] = -dx * np.sin(Yaw_ego) + dy * np.cos(Yaw_ego)


# 3차 다항식으로 waypoint들을 local 경로로 근사
class PolynomialFitting(object):
    def __init__(self, num_degree, num_points):
        self.nd = num_degree
        self.np = num_points
        self.A = np.zeros((self.np, self.nd+1))
        self.b = np.zeros((self.np,1))
        self.coeff = np.zeros((num_degree+1,1))
        
    def fit(self, points):
        for i in range(self.np):
            x = points[i][0]
            y = points[i][1]
            for j in range(self.nd + 1):
                self.A[i][j] = x ** (self.nd - j)
            self.b[i][0] = y
        self.coeff = np.linalg.pinv(self.A.T @ self.A) @ self.A.T @ self.b



# 로컬 x 좌표들을 넣어서 y = f(x) 값을 계산하는 클래스 -> y 값을 계산해서 적절한 헤딩 앵글 추종값 계산
class PolynomialValue(object):
    def __init__(self, num_degree, num_points):
        self.nd = num_degree
        self.np = num_points
        self.x = np.zeros((1, self.nd+1))
        self.y = np.zeros((num_points, 1))
        self.points = np.zeros((self.np, 2))
        
    def calculate(self, coeff, x):
        for i in range(self.np):
            self.x = np.array([[x[i]**(self.nd - j) for j in range(self.nd + 1)]])
            self.y[i] = self.x @ coeff
            self.points[i][0] = x[i]
            self.points[i][1] = self.y[i][0]
        
        
if __name__ == "__main__":
    num_degree = 3
    num_point = 4
    points = np.array([[1,2],[3,3],[4,4],[5,5]])
    X_ego = 2.0
    Y_ego = 0.0
    Yaw_ego = np.pi/4
    x_local = np.arange(0.0, 10.0, 0.5)
    
    frameconverter = Global2Local(num_point)
    polynomialfit = PolynomialFitting(num_degree,num_point)
    polynomialvalue = PolynomialValue(num_degree,np.size(x_local))
    frameconverter.convert(points, Yaw_ego, X_ego, Y_ego)
    polynomialfit.fit(frameconverter.LocalPoints)
    polynomialvalue.calculate(polynomialfit.coeff, x_local)
    
    plt.figure(1)
    for i in range(num_point):
        plt.plot(points[i][0],points[i][1],'b.')
    plt.plot(X_ego,Y_ego,'ro',label = "Vehicle")
    plt.plot([X_ego, X_ego+0.2*np.cos(Yaw_ego)],[Y_ego, Y_ego+0.2*np.sin(Yaw_ego)],'r-')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.title("Global Frame")
    plt.grid(True)    
    
    plt.figure(2)
    for i in range(num_point):
        plt.plot(frameconverter.LocalPoints[i][0],frameconverter.LocalPoints[i][1],'b.')
    plt.plot(polynomialvalue.points.T[0],polynomialvalue.points.T[1],'b:')
    plt.plot(0.0, 0.0,'ro',label = "Vehicle")
    plt.plot([0.0, 0.5],[0.0, 0.0],'r-')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend(loc="best")
    plt.axis((-10,10,-10,10))
    plt.title("Local Frame")
    plt.grid(True)   
    
    plt.show()