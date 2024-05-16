from motor_team.interface import Interface
from motor_team.smc_controller import SMCController
from motor_team.trajectory import Trajectory
from motor_team.dynamics import Dynamics
from motor_team.utils import Utils
from motor_team.pid_controller import PIDController
from motor_team.mpc_controller import MPCController
import matplotlib.pyplot as plt
import time
import math
import signal

class Unit:
    def __init__(self, utils, dynamics, motor, controller):
        # 필요한 객체 생성
        self.utils = utils
        self.dynamics = dynamics
        self.motor = motor
        self.controller = controller

        # current data
        self.th1 = 0.0
        self.dth1 = 0.0
        self.th2 = 0.0
        self.dth2 = 0.0
        self.tau1 = 0.0
        self.tau2 = 0.0
        self.end_pos1_old = 0
        self.end_pos2_old = 0

        # boundary data
        self.maximum_pos1 = self.utils.rad2pos(rad=3.141592)
        self.maximum_pos2 = self.utils.rad2pos(rad=1.22173)

        # plot data
        self.desired_th1_list = []
        self.desired_th2_list = []
        self.tracking_th1_list = []
        self.tracking_th2_list = []

        self.desired_dth1_list = []
        self.desired_dth2_list = []
        self.tracking_dth1_list = []
        self.tracking_dth2_list = []

        try:
            self.motor.setHome()
            time.sleep(1)
        except:
            print('예외가 발생했습니다2.')

    def execute(self, drone_rad1, drone_rad2):
        
        # 모터 중지를 위한 인터럽트 시그널 함수
        # signal.signal(signal.SIGINT, self.motor.signal_handler)
        
        start = time.time()

        start_vel1 = self.utils.rad2pos(rad=self.dth1)
        start_vel2 = self.utils.rad2pos(rad=self.dth2)
        # end_pos1 = max(-self.maximum_pos1, min(self.maximum_pos1, self.utils.rad2pos(rad=drone_rad1)))
        # end_pos2 = max(-self.maximum_pos2, min(self.maximum_pos2, self.utils.rad2pos(rad=drone_rad2)))
        end_pos1 = max(-self.maximum_pos1, min(self.maximum_pos1, drone_rad1))
        end_pos2 = max(-self.maximum_pos2, min(self.maximum_pos2, drone_rad2))

        abs1 = abs(self.end_pos1_old - end_pos1)
        abs2 = abs(self.end_pos2_old - end_pos2)

        abs_total = max(abs1, abs2)

        # abs_total 이 100 보다 작다면 time slice 는 50 고정
        # abs_total 이 100 보다 크다면 1당 0.5 time slice 할당
        # time_slice = 1 : 0.005초
        if abs_total <= 100:
            time_slice = 30
        else:
            time_slice = math.ceil(abs_total * 0.3)

        # max_time 고정
        max_time = time_slice / 200

        # dt 계산
        dt = max_time / time_slice

        # print(f'start pos: {self.end_pos1_old}, end pos: {end_pos1}, start vel: {start_vel1}')

        trajectory1 = Trajectory(
            start_pos=self.end_pos1_old, end_pos=end_pos1, start_velocity=start_vel1,
            max_time=max_time, time_slice=time_slice, utils=self.utils
        )
        trajectory2 = Trajectory(
            start_pos=self.end_pos2_old, end_pos=end_pos2, start_velocity=start_vel2,
            max_time=max_time, time_slice=time_slice, utils=self.utils
        )

        # 필요한 경로 생성
        # print('MAKE TRAJECTORY')
        desired_rad1_list, desired_radps1_list, desired_radpss1_list = trajectory1.execute_rad()
        desired_rad2_list, desired_radps2_list, desired_radpss2_list = trajectory2.execute_rad()

        for i in range(time_slice):
            self.th1, self.dth1, self.th2, self.dth2, self.tau1, self.tau2 = self.controller.execute_dynamics(
                th_d1=desired_rad1_list[i], dth_d1=desired_radps1_list[i], ddth_d1=desired_radpss1_list[i],
                th_d2=desired_rad2_list[i], dth_d2=desired_radps2_list[i], ddth_d2=desired_radpss2_list[i], dt=dt
            )

            self.tracking_th1_list.append(self.th1)
            self.tracking_th2_list.append(self.th2)
            self.desired_th1_list.append(desired_rad1_list[i])
            self.desired_th2_list.append(desired_rad2_list[i])

            self.tracking_dth1_list.append(self.dth1)
            self.tracking_dth2_list.append(self.dth2)
            self.desired_dth1_list.append(desired_radps1_list[i])
            self.desired_dth2_list.append(desired_radps2_list[i])

        self.end_pos1_old = end_pos1
        self.end_pos2_old = end_pos2

        end = time.time()
        print(f'time slice: {time_slice}, abs1: {abs1:.2f}, abs2: {abs2:.2f}, elapse: {end - start:.2f}ms')
      

    def getRMSE(self):
        import numpy as np

        # 두 리스트의 요소들의 차이를 계산
        # differences = np.array(self.desired_th1_list+self.desired_th2_list) - np.array(self.tracking_th1_list+self.tracking_th2_list)
        differences = np.array(self.desired_th1_list) - np.array(self.tracking_th1_list)
        
        # 차이의 제곱
        squared_differences = differences ** 2
        
        # 제곱된 차이의 평균
        mean_squared_differences = np.mean(squared_differences)
        
        # 평균의 제곱근
        rmse = np.sqrt(mean_squared_differences)

        return rmse


    def plot(self):
        # 각도 그래프 플롯
        plt.subplot(2, 1, 1)

        plt.plot([x * 57 for x in self.desired_th1_list], 'r', label='desired th1 path')
        plt.plot([x * 57 for x in self.tracking_th1_list], 'b', label='tracking th1 path')
        # plt.ylim([-10, 30]) 

        plt.xlabel('time')
        plt.legend()
        
        # 각속도 그래프 플롯
        plt.subplot(2, 1, 2)
        # plt.plot([x * 57 for x in self.desired_th2_list], 'r', label='desired th2 path')
        # plt.plot([x * 57 for x in self.tracking_th2_list], 'b', label='tracking th2 path')
        plt.plot([x * 57 for x in self.desired_dth1_list], 'r', label='desired th2 path')
        plt.plot([x * 57 for x in self.tracking_dth1_list], 'b', label='tracking th2 path')
        # plt.ylim([-10, 30]) 
        
        plt.xlabel('time')
        plt.legend()

        plt.show()

if __name__ == "__main__":
    utils = Utils()
    dynamics = Dynamics()
    motor = Interface(utils=utils)
    controller = PIDController(motor=motor, dynamics=dynamics, utils=utils)
    # controller = SMCController(motor=motor, dynamics=dynamics, utils=utils)

    test = Unit(utils=utils, dynamics=dynamics, motor=motor, controller=controller)
    test.execute(500, 0)
    # test.execute(200, 0)
    # test.execute(0, 0)
    # test.execute(-100, 0)

    motor.disableTorque()
    # test.plot()
