import time

class PIDController:
    # def __init__(self, motor, dynamics, utils, kp1, ki1, kd1, kp2, ki2, kd2):
    def __init__(self, motor, dynamics, utils):
        self.Ku = 1133
        self.Tu = 0.1425

        # 게인  
        self.Kp1 = 8000
        self.Ki1 = 500
        self.Kd1 = 0
        # self.Kp1 = kp1
        # self.Ki1 = ki1
        # self.Kd1 = kd1

        self.Kp2 = 5050
        self.Ki2 = 0
        self.Kd2 = 0
        
        # self.Kp2 = 10
        # self.Ki2 = 0
        # self.Kd2 = 1
        # self.Kp2 = kp2
        # self.Ki2 = ki2
        # self.Kd2 = kd2

        # 현재값, 에러값
        self.th_1 = 0
        self.dth_1 = 0
        self.integral_rad_error1 = 0

        self.th_2 = 0
        self.dth_2 = 0
        self.integral_rad_error2 = 0

        # 토크 상수 N*m / A
        self.Kt = 1.84

        self.motor = motor
        self.dynamics = dynamics
        self.utils = utils

    def execute_dynamics(self, th_d1, dth_d1, ddth_d1, th_d2, dth_d2, ddth_d2, dt):
        # 위치 및 속도 오류 계산
        rad_error1 = th_d1 - self.th_1
        radps_error1 = dth_d1 - self.dth_1

        # 위치 및 속도 오류 계산
        rad_error2 = th_d2 - self.th_2
        radps_error2 = dth_d2 - self.dth_2

        # 위치 적분 오류 업데이트
        self.integral_rad_error1 += rad_error1 * dt
        self.integral_rad_error2 += rad_error2 * dt

        # 각도에러, 각속도에러 제어값 계산
        signal1 = self.Kp1 * rad_error1 + self.Ki1 * self.integral_rad_error1 + self.Kd1 * radps_error1
        signal2 = self.Kp2 * rad_error2 + self.Ki2 * self.integral_rad_error2 + self.Kd2 * radps_error2
        # signal1 = self.Kp1 * radps_error1 + self.Ki1 * rad_error1 + self.Kd1 * radps_error1 / dt
        # signal2 = self.Kp2 * radps_error2 + self.Ki2 * rad_error2 + self.Kd2 * radps_error2 / dt

        # 질량 매트릭스에 들어갈 tau_apostrophe 계산
        tau_apostrophe1 = ddth_d1 + signal1
        tau_apostrophe2 = ddth_d2 + signal2

        # 동역학 모델을 사용하여 토크 계산
        mass_tau1, mass_tau2 = self.dynamics.calcMassTorque_numerical(th1=self.th_1, th2=self.th_2, ddth1=tau_apostrophe1, ddth2=tau_apostrophe2)
        coriolis_tau1, coriolis_tau2 = self.dynamics.calcCoriolisGravityTorque_numerical(th1=self.th_1, th2=self.th_2, dth1=self.dth_1, dth2=self.dth_2)

        # 최종 토크 신호 계산
        tau1 = mass_tau1 + coriolis_tau1
        tau2 = mass_tau2 + coriolis_tau2

        # 최종 토크를 전류량으로 변환 >> 단위는 암페어
        if self.th_2 > 0:
            current1 = tau1 * self.Kt * 2
        else:
            current1 = tau1 / self.Kt
        current2 = tau2 / self.Kt
        
        # 모터로 토크 신호 전송, 현재 상태 업데이트
        self.th_1, self.dth_1, self.th_2, self.dth_2 = self.motor.sendCUR(current1, current2)
        # self.th_1, self.dth_1, self.th_2, self.dth_2 = self.motor.sendCURFake(tau1, tau2)

        print(f'전류량1: {current1 * 1000:7.3f}mA, 전류 디지털1 (max: 1193): {self.utils.amp2digit(current1)}')
        # print(f'전류량2: {current2 * 1000:7.3f}mA, 전류 디지털2 (max: 1193): {self.utils.amp2digit(current2)}')

        # 모터1의 각도, 각속도 피드백
        return self.th_1, self.dth_1, self.th_2, self.dth_2, tau1, tau2
