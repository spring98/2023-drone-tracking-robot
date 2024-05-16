import time

class SMCController:
    def __init__(self, motor, dynamics, utils):
        # smc 상수
        # self.C1 = 1.5
        # self.K1 = 0.10
        # self.C2 = 5
        # self.K2 = 0.20

        # self.C1 = 30
        # self.K1 = 0.06
        # self.W1 = 0.85

        # self.C2 = 30
        # self.K2 = 0.15
        # self.W2 = 2.0

        self.C1 = 10
        self.K1 = 0.039
        self.W1 = 0.5

        self.C2 = 30
        self.K2 = 0.355
        self.W2 = 6

        # self.C1 = 20
        # self.K1 = 0.045
        # self.W1 = 0.65

        # self.C2 = 30
        # self.K2 = 0.15
        # self.W2 = 2.0

        # 현재값
        self.th_1 = 0
        self.dth_1 = 0
        self.th_2 = 0
        self.dth_2 = 0

        # 토크 상수 N*m / A
        self.Kt = 1.84

        self.motor = motor
        self.dynamics = dynamics
        self.utils = utils

    def execute_dynamics(self, th_d1, dth_d1, ddth_d1, th_d2, dth_d2, ddth_d2, dt):
        # 위치 및 속도 오류 계산
        e1 = th_d1 - self.th_1
        e2 = dth_d1 - self.dth_1
        e3 = th_d2 - self.th_2
        e4 = dth_d2 - self.dth_2

        # 슬라이딩 라인
        s1 = e1 * self.C1 + e2
        s2 = e3 * self.C2 + e4

        # 중간 계산
        m2A_square = self.dynamics.calcM2A_square_numerical(self.th_1, self.th_2)
        m2AB = self.dynamics.calcM2AB_dth1_dth2_numerical(self.th_1, self.th_2, self.dth_1, self.dth_2)
        m2C = self.dynamics.calcM2C()
        m2AB_dth1_square = self.dynamics.calcM2AB_dth1_square(self.th_1, self.th_2, self.dth_1, self.dth_2)
        m2AG = self.dynamics.calcM2Ag(self.th_1, self.th_2)
        
        # 최종 토크 신호 계산
        tau1 = m2A_square * (self.C1 * e2 + ddth_d1) + 2 * m2AB + self.K1 * self.utils.sat(s1, self.W1) # or self.utils.sgn(s1)
        # tau2 = m2A_square * (self.C2 * e4 + ddth_d2) + m2AB + self.K2 * self.utils.sat(s2, 0.5)
        tau2 = m2C * (self.C2 * e4 + ddth_d2) - m2AB_dth1_square - m2AG + self.K2 * self.utils.sat(s2, self.W2)

        # print(f'ddth_d1: {ddth_d1}, m2A_square: {m2A_square},m2AB/: {m2AB}, s1: {s1}, tau1: {tau1}')

        # 최종 토크를 전류량으로 변환 >> 단위는 암페어
        current1 = tau1 / self.Kt
        current2 = tau2 / self.Kt

        # 데이터 출력
        print(f'전류량: {current1 * 1000:7.3f}mA\t\t'
              f'전류 디지털 (max: 1193): {self.utils.amp2digit(current1)}')

        # 모터로 토크 신호 전송, 현재 상태 업데이트
        self.th_1, self.dth_1, self.th_2, self.dth_2 = self.motor.sendCUR(current1, current2)
        # self.th_1, self.dth_1, self.th_2, self.dth_2 = self.motor.sendCURFake(tau1, tau2)

        # 모터1의 각도, 각속도 피드백
        return self.th_1, self.dth_1, self.th_2, self.dth_2, tau1, tau2
