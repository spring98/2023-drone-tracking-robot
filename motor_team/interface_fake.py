from dynamixel_sdk import *
import numpy as np
from scipy.integrate import odeint

# noinspection,DuplicatedCode
class InterfaceFake:
    def __init__(self, utils):
        # 모터 파라미터 초기화
        self.J = 0.0039  # 관성 모멘트 (kg.m^2)
        self.b = 0.01  # 마찰 계수 (Nms)
        self.K = 1.84  # 모터 상수 (Nm/A)
        self.motor1_state = np.array([0.0, 0.0])  # 초기 상태 [theta, omega]
        self.motor2_state = np.array([0.0, 0.0])  # 초기 상태 [theta, omega]
        self.t = 0.0

    def sendCURFake(self, torque1, torque2, dt=0.005):
        t_span = [self.t, self.t + dt]
        # print(t_span)
        self.motor1_state = odeint(self.motor_dynamics, self.motor1_state, t_span, args=(self.J, self.b, self.K, torque1))[-1]
        self.motor2_state = odeint(self.motor_dynamics, self.motor2_state, t_span, args=(self.J, self.b, self.K, torque2))[-1]

        self.t += dt

        # 모터1과 모터2의 현재 위치와 속도 반환
        return self.motor1_state[0], self.motor1_state[1], self.motor2_state[0], self.motor2_state[1]

    # 운동 방정식 정의
    def motor_dynamics(self, y, t, J, b, K, torque):
        theta, omega = y  # 각도와 각속도
        dydt = [omega, (1 / J) * (torque - b * omega)]
        return dydt
