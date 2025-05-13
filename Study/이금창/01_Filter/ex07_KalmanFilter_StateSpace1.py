import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, y_Measure_init, step_time=0.01, m=1.0, Q_x=0.02, Q_v=0.05, R=5.0, errorCovariance_init=10.0):
        self.A = np.array([[1.0, step_time], [0.0, 1.0]])
        self.B = np.array([[0.0], [step_time/m]])
        self.C = np.array([[1.0, 0.0]])
        self.D = 0.0
        self.Q = np.array([[Q_x, 0.0], [0.0, Q_v]])
        self.R = R
        self.x_estimate = np.array([[y_Measure_init], [0.0]])
        self.P_estimate = np.array([[errorCovariance_init, 0.0], [0.0, errorCovariance_init]])

    def estimate(self, y_measure, input_u):
        # Prediction
        self.x_estimate = np.dot(self.A, self.x_estimate) + np.dot(self.B, input_u)
        self.P_estimate = np.dot(self.A, np.dot(self.P_estimate, self.A.T)) + self.Q

        # Update
        S = np.dot(self.C, np.dot(self.P_estimate, self.C.T)) + self.R
        K = np.dot(np.dot(self.P_estimate, self.C.T), np.linalg.inv(S))
        y_error = y_measure - np.dot(self.C, self.x_estimate)
        self.x_estimate = self.x_estimate + np.dot(K, y_error)
        I = np.eye(self.P_estimate.shape[0])
        self.P_estimate = np.dot((I - np.dot(K, self.C)), self.P_estimate)

if __name__ == "__main__":
    signal = pd.read_csv("01_filter/Data/example07.csv")
    signal["y_estimate"] = 0.0

    y_estimate = KalmanFilter(signal.y_measure[0])

    for i, row in signal.iterrows():
        y_estimate.estimate(signal.y_measure[i], signal.u[i])
        signal.y_estimate[i] = y_estimate.x_estimate[0][0]

    plt.figure()
    plt.plot(signal.time, signal.y_measure, 'k.', label="Measure")
    plt.plot(signal.time, signal.y_estimate, 'r-', label="Estimate")
    plt.xlabel('time (s)')
    plt.ylabel('signal')
    plt.legend(loc="best")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
