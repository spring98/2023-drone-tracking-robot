import time
import numpy as np
from motor_team.interface import Interface
from motor_team.utils import Utils
import matplotlib.pyplot as plt
import cmath

class MPCController:
    def __init__(self, motor, utils, zeta1, zeta2, w0_1, w0_2):
        # 현재값, 에러값
        self.th_1 = 0
        self.dth_1 = 0
        self.integral_rad_error = 0

        # 토크 상수 N*m / A
        self.Kt = 1.84

        self.motor = motor
        self.utils = utils

        self.x1_lst = []
        self.x2_lst = []
        self.des_x1_lst = []
        self.des_x2_lst = []
        self.error_x1_lst = []
        self.error_x2_lst = []
        self.torque = []
        
        self.zeta1 = zeta1
        self.zeta2 = zeta2
        
        self.w0_1 = w0_1
        self.w0_2 = w0_2

        self.utils = utils

        self.x = np.array([0.0, 0.0])
        self.dx = np.array([0.0, 0.0])

        self.h_1, self.rho_1 = self.solve_for_h_rho(self.zeta1, self.w0_1)
        self.h_2, self.rho_2 = self.solve_for_h_rho(self.zeta2, self.w0_2)

    # Feedback Linearization
    def feedback_linearization(self, x, dx, v):
        # Transform the nonlinear system to a linear system

        # camera: 75.5
        # motor : 82
        # laser : 100
        # top: 58.5
        # braket: 7.5

        m1 = 0.075
        m2 = 0.316
        l1 = 0.1
        l2 = 0.055
        l3 = 0.02
        g = 9.81

        theta1 = x[0]
        theta2 = x[1]

        dtheta1 = dx[0]
        dtheta2 = dx[1]

        A = l2*np.sin(theta2) - l3*np.cos(theta2)
        B = l2*np.cos(theta2) + l3*np.sin(theta2)

        G = np.array([0, -m2*A*g])
        C = np.array([2*m2*A*B*dtheta1*dtheta2, -m2*A*B*(dtheta1**2)])
        M = np.array([[m2*(A**2), 0],
            [0,  m2*((l2**2) + (l3**2))]])

        u = M@v + C + G

        return u

    def solve_for_h_rho(self, zeta, w):
        # Coefficients for the quadratic equation
        a = w**2
        b = -4 * zeta * w
        c = 2

        # Calculating the discriminant
        discriminant = cmath.sqrt(b**2 - 4*a*c)

        # Calculating the two solutions
        h1 = (-b + discriminant) / (2*a)
        h2 = (-b - discriminant) / (2*a)

        rho1 = (2-(w*h1)**2)/(4*(w**2))
        rho2 = (2-(w*h2)**2)/(4*(w**2))

        return h2, rho2

    def mpc(self, x, dx, ref):
        # horizon time of prediction
        h_1 = self.h_1
        h_2 = self.h_2

        # print("h_1: ", h_1)
        # print("h_2: ", h_2)

        # weight factor to velocity
        rho_1 = self.rho_1
        rho_2 = self.rho_2

        # print("rho_1: ", rho_1)
        # print("rho_2: ", rho_2)

        ref = ref 

        k1_1 = 2/((h_1**2)+4*rho_1)
        k2_1 = (2*(h_1**2)+4*rho_1)/((h_1**3)+4*rho_1*h_1)
        k3_1 = k1_1

        k1_2 = 2/((h_2**2)+4*rho_2)
        k2_2 = (2*(h_2**2)+4*rho_2)/((h_2**3)+4*rho_2*h_2)
        k3_2 = k1_2

        v = np.array([k3_1*ref[0] - k1_1*x[0] - k2_1*dx[0], (k3_2*ref[1] - k1_2*x[1] - k2_2*dx[1])])
        
        return v

    def mpc_controller(self, x, dx, ref):
        v = self.mpc(x, dx, ref)
        u = self.feedback_linearization(x, dx, v)

        return u

    def execute_dynamics(self, th_d1, dth_d1, ddth_d1, th_d2, dth_d2, ddth_d2, dt):
        # Reference --> sin, cos
        u = self.mpc_controller(self.x, self.dx, np.array([th_d1, th_d2]))
        current1 = u[0] / self.Kt
        current2 = u[1] / self.Kt

        # 데이터 출력
        # print(f'전류량: {current1 * 1000:7.3f}mA\t\t'
        #     f'전류 디지털 (max: 1193): {self.utils.amp2digit(current1)}')
        
        # print(f'전류량: {current2 * 1000:7.3f}mA\t\t'
        #     f'전류 디지털 (max: 1193): {self.utils.amp2digit(current2)}')

        # 모터로 토크 신호 전송, 현재 상태 업데이트
        self.x[0], self.dx[0], self.x[1], self.dx[1] = self.motor.sendCUR(current1, current2)

        return self.x[0], self.dx[0], self.x[1], self.dx[1], u[0], u[1]
