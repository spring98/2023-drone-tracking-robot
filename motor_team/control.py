from _thread import *
import json
from numpy import clip, radians, angle
from motor_team.trajectory import Trajectory
import signal
import time
import matplotlib.pyplot as plt
import numpy as np
import pickle
import math

class Control:
    def __init__(self, conn, motor, utils, coordinate, controller, isFake):
        # dependency injection
        self.conn = conn
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
        self.isFake = isFake

        # boundary data
        self.maximum_pos1 = self.utils.rad2pos(rad=3.141592)
        self.maximum_pos2 = self.utils.rad2pos(rad=1.22173)

    def execute(self):
        # 모터 중지를 위한 인터럽트 시그널 함수
        signal.signal(signal.SIGINT, self.motor.signal_handler)

        print('GO HOME')
        self.motor.setHome()

        time.sleep(1)

        times = []

        json_data = ''
        while True:
            chunk = self.conn.recv(2048).decode('utf-8')
            if not chunk:
                break
            json_data += chunk

            try:
                start = time.time()
                camera_data = json.loads(chunk)
                # print(f'camera: {camera_data}')

                if self.isFake:
                    drone_rad1, drone_rad2 = camera_data['drone_rad1'], camera_data['drone_rad2']
                    
                else:
                    droneXYZ = self.coordinate.camera2drone(
                        camera_data['x'], camera_data['y'], camera_data['depth'] * 0.01, self.th1, self.th2
                    )

                    drone_rad1, drone_rad2 = self.coordinate.drone2radian(droneXYZ[0], droneXYZ[1], droneXYZ[2])

                # print(f'Drone coord: {droneXYZ} \nDesired th1: {drone_rad1}, th2: {drone_rad2}')

                if drone_rad1 != -1 and drone_rad2 != -1:
                    start_vel1 = self.utils.rad2pos(rad=self.dth1)
                    start_vel2 = self.utils.rad2pos(rad=self.dth2)
                    end_pos1 = max(-self.maximum_pos1, min(self.maximum_pos1, self.utils.rad2pos(rad=drone_rad1)))
                    end_pos2 = max(-self.maximum_pos2, min(self.maximum_pos2, self.utils.rad2pos(rad=drone_rad2)))

                    abs1 = abs(self.end_pos1_old - end_pos1)
                    abs2 = abs(self.end_pos2_old - end_pos2)

                    abs_total = max(abs1, abs2)

                    # abs_total 이 100 보다 작다면 time slice 는 50 고정
                    # abs_total 이 100 보다 크다면 1당 0.5 time slice 할당

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

                    # print(f'start pos: {self.end_pos1_old}, end pos: {end_pos1}')

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
                        self.watt1_list.append(self.dth1 * self.tau1)
                        self.watt2_list.append(self.dth2 * self.tau2)
                        times.append(time.time())

                    self.end_pos1_old = end_pos1
                    self.end_pos2_old = end_pos2

                    self.drone_rad1s.append(drone_rad1)
                    self.drone_rad2s.append(drone_rad2)
                    self.fake_times.append(time.time() - self.fake_time)

                    end = time.time()
                    print(f'time slice: {time_slice}, abs1: {abs1:.2f}, abs2: {abs2:.2f}, elapse: {end - start:.2f}ms')

            except json.JSONDecodeError as e:
                pass
        
        times = [t - times[0] for t in times]
        # print(times)
        self.motor.disableTorque()
        self.plot(self.desired_th1_list, self.desired_th2_list, self.tracking_th1_list, self.tracking_th2_list, times)
        # self.plotWH(self.watt1_list, self.watt2_list, times)
        self.plotError([ref-real for ref, real in zip(self.desired_th1_list, self.tracking_th1_list)]
                       ,[ref-real for ref, real in zip(self.desired_th2_list, self.tracking_th2_list)], times)
        import csv
        from datetime import datetime

        with open(f'fake/graph_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', encoding='utf-8') as f:
            wr = csv.writer(f)
            for fake in zip(self.desired_th1_list, self.desired_th2_list, self.tracking_th1_list, self.tracking_th2_list, self.watt1_list, self.watt2_list, times):
                wr.writerow(fake)

        if not self.isFake:
            with open(f'fake/camera_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', encoding='utf-8') as f:
                wr = csv.writer(f)
                for fake in zip(self.drone_rad1s, self.drone_rad2s, self.fake_times):
                    wr.writerow(fake)


        self.conn.close()

    def plot(self, desired_th1_list, desired_th2_list, tracking_th1_list, tracking_th2_list, time):

        # 각도 그래프 플롯
        data = {
            "ref_pos": [self.utils.pos2rad(_) for _ in desired_th1_list],
            "current_pos": tracking_th1_list,
            "time": time
        }
        
        # with open('PID_study_room.pickle', 'wb') as f:
        #     pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

        # 각도 그래프 플롯
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
