import numpy as np
import matplotlib.pyplot as plt

class Global2Local(object):
    def __init__(self, num_points):
        self.n = num_points
        self.GlobalPoints = np.zeros((num_points,2))
        self.LocalPoints = np.zeros((num_points,2))
    
    def convert(self, points, Yaw_ego, X_ego, Y_ego):
        self.GlobalPoints = points
        for i in range(self.n):
            dx = points[i][0] - X_ego
            dy = points[i][1] - Y_ego
            # 회전 변환 (Global → Local)
            self.LocalPoints[i][0] = dx * np.cos(-Yaw_ego) - dy * np.sin(-Yaw_ego)
            self.LocalPoints[i][1] = dx * np.sin(-Yaw_ego) + dy * np.cos(-Yaw_ego)

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
            for j in range(self.nd+1):
                self.A[i][j] = x ** (self.nd - j)
            self.b[i][0] = y
        # 최소자승법 계수 계산
        self.coeff = np.linalg.inv(self.A.T @ self.A) @ self.A.T @ self.b

class PolynomialValue(object):
    def __init__(self, num_degree, num_points):
        self.nd = num_degree
        self.np = num_points
        self.x = np.zeros((1, self.nd+1))
        self.y = np.zeros((num_points, 1))
        self.points = np.zeros((self.np, 2))
        
    def calculate(self, coeff, x_vals):
        for i in range(self.np):
            x = x_vals[i]
            self.points[i][0] = x
            y = 0.0
            for j in range(self.nd+1):
                y += coeff[j][0] * x ** (self.nd - j)
            self.points[i][1] = y

# ------------------------------
# Main 실행부
# ------------------------------
if __name__ == "__main__":
    num_degree = 3
    num_point = 5
    global_points = np.array([[1, 2], [2, 3], [3, 5], [4, 7], [5, 11]])  # Reference points in global frame
    X_ego = 2.5
    Y_ego = 1.0
    Yaw_ego = np.pi / 6  # 30 degrees
    x_local = np.arange(0.0, 6.0, 0.1)

    # 변환 및 피팅
    converter = Global2Local(num_point)
    converter.convert(global_points, Yaw_ego, X_ego, Y_ego)

    fitter = PolynomialFitting(num_degree, num_point)
    fitter.fit(converter.LocalPoints)

    evaluator = PolynomialValue(num_degree, len(x_local))
    evaluator.calculate(fitter.coeff, x_local)

    # ------------------------------
    # 시각화
    # ------------------------------

    # Global Frame
    plt.figure(1)
    for p in global_points:
        plt.plot(p[0], p[1], 'bo')
    plt.plot(X_ego, Y_ego, 'ro', label='Ego Vehicle')
    plt.plot([X_ego, X_ego + 0.5 * np.cos(Yaw_ego)],
             [Y_ego, Y_ego + 0.5 * np.sin(Yaw_ego)], 'r-')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Global Frame')
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    # Local Frame
    plt.figure(2)
    for p in converter.LocalPoints:
        plt.plot(p[0], p[1], 'bo')
    plt.plot(evaluator.points[:, 0], evaluator.points[:, 1], 'g--', label="Fitted Polynomial")
    plt.plot(0.0, 0.0, 'ro', label='Ego Vehicle (Local Origin)')
    plt.plot([0, 1], [0, 0], 'r-')
    plt.xlabel('x (local)')
    plt.ylabel('y (local)')
    plt.title('Local Frame & Fitted Curve')
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    plt.show()
