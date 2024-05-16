from motor_team.pid_controller import PIDController
from motor_team.smc_controller import SMCController
from motor_team.mpc_controller import MPCController
from motor_team.interface import Interface
from motor_team.coordinate import Coordinate
from motor_team.interface import Interface
from motor_team.dynamics import Dynamics
from motor_team.utils import Utils
from motor_team.trajectory import Trajectory
import math, time, cmath
import signal
import time
import matplotlib.pyplot as plt
import math

class ControlFake:
    def __init__(self, motor, utils, coordinate, controller):
        # dependency injection
        self.motor = motor
        self.coordinate = coordinate
        self.utils = utils
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

        # plot data
        self.desired_th1_list = []
        self.desired_th2_list = []
        self.tracking_th1_list = []
        self.tracking_th2_list = []
        self.watt1_list = []
        self.watt2_list = []

        # fake data
        self.drone_rad1s = []
        self.drone_rad2s = []
        self.fake_times = []
        self.fake_time = time.time()

        # boundary data
        self.maximum_pos1 = self.utils.rad2pos(rad=3.141592)
        self.maximum_pos2 = self.utils.rad2pos(rad=1.22173)

    def getTrajectory(self, rad1, rad2):
        end_pos1 = max(-self.maximum_pos1, min(self.maximum_pos1, self.utils.rad2pos(rad=rad1)))
        end_pos2 = max(-self.maximum_pos2, min(self.maximum_pos2, self.utils.rad2pos(rad=rad2)))

        abs1 = abs(self.end_pos1_old - end_pos1)
        abs2 = abs(self.end_pos2_old - end_pos2)

        abs_total = max(abs1, abs2)

        # time_slice = 1 : 0.005초
        if abs_total <= 100:
            time_slice = 40
        else:
            time_slice = math.ceil(abs_total * 0.5)
        # time_slice = 40

        # max_time 고정
        max_time = time_slice / 200

        # dt 계산
        dt = max_time / time_slice

        trajectory1 = Trajectory(
            start_pos=self.end_pos1_old, end_pos=end_pos1, start_velocity=0,
            max_time=max_time, time_slice=time_slice, utils=self.utils
        )
        trajectory2 = Trajectory(
            start_pos=self.end_pos2_old, end_pos=end_pos2, start_velocity=0,
            max_time=max_time, time_slice=time_slice, utils=self.utils
        )

        # 필요한 경로 생성
        # print('MAKE TRAJECTORY')
        desired_rad1_list, desired_radps1_list, desired_radpss1_list = trajectory1.execute_rad()
        desired_rad2_list, desired_radps2_list, desired_radpss2_list = trajectory2.execute_rad()

        self.end_pos1_old = end_pos1
        self.end_pos2_old = end_pos2

        return desired_rad1_list, desired_radps1_list, desired_radpss1_list, desired_rad2_list, desired_radps2_list, desired_radpss2_list, time_slice


    def execute(self):
        import csv
        from collections import deque

        desired_rad1_list=[]
        desired_radps1_list=[]
        desired_radpss1_list=[]

        desired_rad2_list=[]
        desired_radps2_list=[]
        desired_radpss2_list=[]
        time_slice = 0

        # with open('fake/camera_20240413_110610.csv', 'r', encoding='utf-8') as f:
        # with open('fake/camera_20240413_103425.csv', 'r', encoding='utf-8') as f:
        with open('fake/camera_20240413_103425_2.csv', 'r', encoding='utf-8') as f:
            lines = deque(list(csv.reader(f)))

        while lines:
            drone_rad1, drone_rad2, current_time = list(map(float, lines.popleft()))
            a,b,c,d,e,f, slice = self.getTrajectory(drone_rad1, drone_rad2)
            desired_rad1_list += a.copy()
            desired_radps1_list += b.copy()
            desired_radpss1_list += c.copy()
            desired_rad2_list += d.copy()
            desired_radps2_list += e.copy()
            desired_radpss2_list += f.copy()
            time_slice += slice

        # 모터 중지를 위한 인터럽트 시그널 함수
        signal.signal(signal.SIGINT, self.motor.signal_handler)
        self.motor.setHome()
        time.sleep(1)
        times = []

        for i in range(time_slice):
            self.th1, self.dth1, self.th2, self.dth2, self.tau1, self.tau2 = self.controller.execute_dynamics(
                th_d1=desired_rad1_list[i], dth_d1=desired_radps1_list[i], ddth_d1=desired_radpss1_list[i],
                th_d2=desired_rad2_list[i], dth_d2=desired_radps2_list[i], ddth_d2=desired_radpss2_list[i], dt=0.01
            )

            self.tracking_th1_list.append(self.th1)
            self.tracking_th2_list.append(self.th2)
            self.desired_th1_list.append(desired_rad1_list[i])
            self.desired_th2_list.append(desired_rad2_list[i])
            self.watt1_list.append(self.dth1 * self.tau1)
            self.watt2_list.append(self.dth2 * self.tau2)
            times.append(time.time())

        self.motor.disableTorque()
        self.plot(self.desired_th1_list, self.desired_th2_list, self.tracking_th1_list, self.tracking_th2_list, times)
        # self.plotWH(self.watt1_list, self.watt2_list, times)
        self.plotError([ref-real for ref, real in zip(self.desired_th1_list, self.tracking_th1_list)]
                       ,[ref-real for ref, real in zip(self.desired_th2_list, self.tracking_th2_list)], times)

        import csv
        from datetime import datetime

        with open(f'fake/0413_last_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', encoding='utf-8') as f:
            wr = csv.writer(f)
            for fake in zip(self.desired_th1_list, self.desired_th2_list, self.tracking_th1_list, self.tracking_th2_list, self.watt1_list, self.watt2_list, times):
                wr.writerow(fake)


    def plot(self, desired_th1_list, desired_th2_list, tracking_th1_list, tracking_th2_list, time):
        plt.subplot(2, 1, 1)

        print("len(time): ", len(time))
        print("len(desired_th1_list): ", len(desired_th1_list))
        print("len(tracking_th1_list): ", len(tracking_th1_list))

        plt.plot([x * 57 for x in desired_th1_list], 'r', label='desired th1 path')
        plt.plot([x * 57 for x in tracking_th1_list], 'b', label='tracking th1 path')
        # plt.ylim([-10, 30]) 

        plt.xlabel('time')
        plt.legend()
        
        # 각속도 그래프 플롯
        plt.subplot(2, 1, 2)
        plt.plot([x * 57 for x in desired_th2_list], 'r', label='desired th2 path')
        plt.plot([x * 57 for x in tracking_th2_list], 'b', label='tracking th2 path')
        # plt.ylim([-10, 30]) 
        
        plt.xlabel('time')
        plt.legend()

        plt.show()

    def plotWH(self, watt1_list, watt2_list, time):
        # 각도 그래프 플롯
        plt.subplot(2, 1, 1)

        plt.plot(watt1_list, 'r', label='motor1 watt')
        plt.xlabel('time')
        plt.legend()
        
        # 각속도 그래프 플롯
        plt.subplot(2, 1, 2)
        plt.plot(watt2_list, 'r', label='motor2 watt')
        plt.xlabel('time')
        plt.legend()

        plt.show()

    def plotError(self, error1_list, error2_list, time):
        # 각도 그래프 플롯
        plt.subplot(2, 1, 1)
        plt.plot([x * 57 for x in error1_list], 'r', label='error1')
        plt.xlabel('time')
        plt.legend()
        
        # 각속도 그래프 플롯
        plt.subplot(2, 1, 2)
        plt.plot([x * 57 for x in error2_list], 'r', label='error2')
        plt.xlabel('time')
        plt.legend()

        plt.show()

if __name__ == "__main__":
    # utils
    coordinate = Coordinate()
    utils = Utils()
    dynamics = Dynamics()

    # PID 제어기 생성
    motor = Interface(utils=utils)
    # motor = InterfaceFake(utils=utils)

    controller = PIDController(motor=motor, utils=utils, dynamics=dynamics)
    # controller = SMCController(motor=motor, utils=utils, dynamics=dynamics)
    # controller = MPCController(motor=motor, utils=utils, zeta1=cmath.sqrt(2)/2, zeta2=cmath.sqrt(2)/2, w0_1=55, w0_2=30)
    
    control = ControlFake(motor=motor, utils=utils, coordinate=coordinate, controller=controller)
    control.execute()
