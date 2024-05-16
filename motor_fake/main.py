# import numpy as np
# from scipy.integrate import odeint
# # 각도와 각속도 결과 출력
# import matplotlib.pyplot as plt
#
# # 모터 파라미터
# J = 0.0039  # 관성 모멘트 (kg.m^2) - 모터 데이터 시트에서 얻은 값
# b = 0.01    # 마찰 계수 (Nms)
# K = 1.84    # 모터 상수 (Nm/A)
#
# # 입력 토크 (시간에 따라 변할 수 있음)
# def torque(t):
#     # 여기서는 단순히 일정한 토크를 적용
#     return 2.0  # 일정한 토크 2 Nm
#
# # 운동 방정식
# def motor_dynamics(y, t, J, b, K):
#     theta, omega = y  # 각도와 각속도
#     dydt = [omega, (1/J)*(torque(t) - b*omega)]
#     print(dydt)
#     return dydt
#
# # 초기 조건
# y0 = [0.0, 0.0]
#
# # 시간 변수 (0에서 10초까지 1000개의 포인트)
# t = np.linspace(0, 0.1, 1)
#
# # ODE 해석기를 사용하여 운동 방정식 풀기
# solution = odeint(motor_dynamics, y0, t, args=(J, b, K))
#
# # 결과
# theta = solution[:, 0]
# omega = solution[:, 1]
#
# plt.plot(t, theta, label='Theta (rad)')
# plt.plot(t, omega, label='Omega (rad/s)')
# plt.legend()
# plt.xlabel('Time (s)')
# plt.ylabel('Motor Response')
# plt.title('Motor Dynamics Simulation')
# plt.show()


import numpy as np
from scipy.integrate import odeint

class MotorInterface:
    def __init__(self):
        # 모터 파라미터 초기화
        self.J = 0.0039  # 관성 모멘트 (kg.m^2)
        self.b = 0.01    # 마찰 계수 (Nms)
        self.K = 1.84    # 모터 상수 (Nm/A)
        self.state = np.array([0.0, 0.0])  # 초기 상태 [theta, omega]

    # 운동 방정식 정의
    def motor_dynamics(self, y, t, J, b, K, torque):
        theta, omega = y  # 각도와 각속도
        dydt = [omega, (1/J)*(torque - b*omega)]
        return dydt

    # Motor 상태 업데이트
    def update(self, t, torque_input, dt=0.005):
        # ODE 해석기를 사용하여 운동 방정식 풀기
        # dt는 시간 스텝으로, 이 시간동안 시뮬레이션을 진행
        t_span = [t, t + dt]
        new_state = odeint(self.motor_dynamics, self.state, t_span, args=(self.J, self.b, self.K, torque_input))[-1]
        self.state = new_state
        return self.state

# 다른 클래스에서 MotorInterface를 사용하는 예시
motor = MotorInterface()

# 시뮬레이션 매개변수
t = 0  # 시작 시간
dt = 0.005  # 시간 스텝
torques = [2.0 if i < 20 else 0.0 for i in range(100)]  # 예시 토크 시퀀스

# 시간 스텝에 따라 MotorInterface 업데이트
for torque in torques:
    state = motor.update(t, torque, dt)
    t += dt
    # 이곳에서 필요한 추가 처리를 수행할 수 있습니다.
    print(f"Time: {t:.3f} sec - Theta: {state[0]:.4f} rad, Omega: {state[1]:.4f} rad/s")
