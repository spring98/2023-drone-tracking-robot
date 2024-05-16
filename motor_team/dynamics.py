import sympy as sp
import time

class Dynamics:
    def __init__(self):
        # 심볼들을 정의합니다.
        self.theta1, self.theta2, self.dtheta1, self.dtheta2, self.ddtheta1, self.ddtheta2 \
            = sp.symbols('theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2')
        self.m2, self.g, self.l2, self.l3 = sp.symbols('m2 g l2 l3')
        self.tau1, self.tau2 = sp.symbols('tau1 tau2')
        self.mass_tau1, self.mass_tau2 = sp.symbols('mass_tau1 mass_tau2')
        self.coriolis_tau1, self.coriolis_tau2 = sp.symbols('coriolis_tau1 coriolis_tau2')

        self.A = self.l2 * sp.sin(self.theta2) - self.l3 * sp.cos(self.theta2)
        self.B = self.l3 * sp.sin(self.theta2) + self.l2 * sp.cos(self.theta2)

        # 질량 매트릭스
        self.M = sp.Matrix([
            [self.m2 * self.A ** 2, 0],
            [0, self.m2 * (self.l2 ** 2 + self.l3 ** 2)]
        ])

        # 코리올리스와 원심력 매트릭스
        self.V = sp.Matrix([
            [2 * self.m2 * self.A * self.B * self.dtheta1 * self.dtheta2],
            [-self.m2 * self.A * self.B * self.dtheta1 ** 2]
        ])

        # 중력 매트릭스
        self.G = sp.Matrix([
            [0],
            [-self.m2 * self.A * self.g]
        ])

        # 토크 벡터
        self.tau = sp.Matrix([self.tau1, self.tau2])

        # 실제 값 정의
        self.m2_value = 0.316

        self.l2_value = 0.055
        self.l3_value = 0.02
        self.g_value = 9.81

        # 심볼릭 표현을 수치 함수로 변환
        self.mass_torque_func = sp.lambdify(
            (self.theta1, self.theta2, self.ddtheta1, self.ddtheta2, self.m2, self.l2, self.l3),
            self.M * sp.Matrix([[self.ddtheta1], [self.ddtheta2]]),
            modules='numpy'
        )

        self.coriolis_gravity_func = sp.lambdify(
            (self.theta1, self.theta2, self.dtheta1, self.dtheta2, self.m2, self.l2, self.l3, self.g),
            self.V + self.G,
            modules='numpy'
        )

        self.m2A_square_func = sp.lambdify((self.m2, self.theta2, self.l2, self.l3), self.m2 * self.A ** 2)
        self.m2AB_dth1_dth2_func = sp.lambdify((self.m2, self.theta2, self.l2, self.l3, self.dtheta1, self.dtheta2), self.m2 * self.A * self.B * self.dtheta1 * self.dtheta2)

        self.calcM2AB_dth1_square_func = sp.lambdify((self.m2, self.theta2, self.l2, self.l3, self.dtheta1), self.m2 * self.A * self.B * self.dtheta1 ** 2)
        self.calcM2Ag_func = sp.lambdify((self.m2, self.theta2, self.l2, self.l3, self.g), self.m2 * self.A * self.g)

        # Coriolis와 중력 토크를 수치적으로 계산하는 함수
    def calcCoriolisGravityTorque_numerical(self, th1, th2, dth1, dth2):
        # 함수에 값을 대입하여 수치 계산
        coriolis_gravity_torque = self.coriolis_gravity_func(th1, th2, dth1, dth2, self.m2_value, self.l2_value, self.l3_value, self.g_value)
        return coriolis_gravity_torque[0][0], coriolis_gravity_torque[1][0]

    # Mass 토크를 수치적으로 계산하는 함수
    def calcMassTorque_numerical(self, th1, th2, ddth1, ddth2):
        # 함수에 값을 대입하여 수치 계산
        mass_torque = self.mass_torque_func(th1, th2, ddth1, ddth2, self.m2_value, self.l2_value, self.l3_value)
        return mass_torque[0][0], mass_torque[1][0],

    # SMC Tau1(1/2)
    def calcM2A_square_numerical(self, th1, th2):
        # 함수에 값을 대입하여 수치 계산
        return self.m2A_square_func(self.m2_value, th2, self.l2_value, self.l3_value)

    # SMC Tau1(2/2)
    def calcM2AB_dth1_dth2_numerical(self, th1, th2, dth1, dth2):
        # 함수에 값을 대입하여 수치 계산
        return self.m2AB_dth1_dth2_func(self.m2_value, th2, self.l2_value, self.l3_value, dth1, dth2)

    # SMC Tau2(1/3)
    def calcM2C(self):
        # 함수에 값을 대입하여 수치 계산
        c = self.l2_value**2 + self.l3_value**2
        return self.m2_value * c
    
    # SMC Tau2(2/3)
    def calcM2AB_dth1_square(self, th1, th2, dth1, dth2):
        # 함수에 값을 대입하여 수치 계산
        return self.calcM2AB_dth1_square_func(self.m2_value, th2, self.l2_value, self.l3_value, dth1)
    
    # SMC Tau2(3/3)
    def calcM2Ag(self, th1, th2):
        # 함수에 값을 대입하여 수치 계산
        return self.calcM2Ag_func(self.m2_value, th2, self.l2_value, self.l3_value, self.g_value)
