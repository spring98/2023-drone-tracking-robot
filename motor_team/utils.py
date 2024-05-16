import numpy as np
import cmath
import matplotlib.pyplot as plt

class Utils:
    def pos2rad(self, value):
        pos = self.protectOverflow(value)
        rad = (2 * np.pi / 4096) * pos

        return rad

    def rad2pos(self, rad):
        pos = rad * 4096 / (2 * np.pi)

        return pos

    def protectOverflow(self, value):
        # value가 int 타입인지 확인
        if isinstance(value, int):
            if value & 0x80000000:
                value -= 0x100000000
        return value

    # 0.229: read 로 받아들인 값 1눈금 당 0.229 rpm 이라는 뜻
    def rpm2pps(self, value):
        rpm = self.protectOverflow(value)
        rps = rpm * 0.229 / 60  # Convert RPM to RPS
        pos_per_sec = rps * 4096  # Convert RPS to pos/sec

        return pos_per_sec

    def rpm2radps(self, value):
        rpm = self.protectOverflow(value)
        rps = rpm * 0.229 / 60  # Convert RPM to RPS
        rad_per_sec = rps * 2 * np.pi  # Convert RPS to rad/sec

        return rad_per_sec

    def pps2rpm(self, pps):
        rps = pps / 4096
        rpm = rps * 60 / 0.229

        return rpm

    # 1digit 당 2.69mA 이고, 들어 오는 값이 A 임.
    def amp2digit(self, ampere):
        digit = ampere / 0.00269

        return int(digit)

    def sgn(self, s):
        if s > 0:
            return 1
        elif s < 0:
            return -1
        else:
            return 0

    def sat(self, s, threshold=1):
        if s > threshold:
            return 1
        elif s < -threshold:
            return -1
        else:
            return s / threshold

    def plot(self, desired_th1_list, desired_th2_list, tracking_th1_list, tracking_th2_list, time):
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


if __name__ == "__main__":
    util = Utils()

    import csv
    from collections import deque
    desired_th1s = []
    desired_th2s = []
    tracking_th1s = []
    tracking_th2s = []
    watt1s = []
    watt2s = []
    times = []  

    with open('../fake/graph_20240410_144120.csv', 'r', encoding='utf-8') as f:
        lines = deque(list(csv.reader(f)))

    while lines:
        desired_th1, desired_th2, tracking_th1, tracking_th2, watt1, watt2, time = lines.popleft()

        desired_th1s.append(float(desired_th1))
        desired_th2s.append(float(desired_th2))
        tracking_th1s.append(float(tracking_th1))
        tracking_th2s.append(float(tracking_th2))
        watt1s.append(float(watt1))
        watt2s.append(float(watt2))
        times.append(float(time))

    util.plot(desired_th1s, desired_th2s, tracking_th1s, tracking_th2s, times)
    util.plotWH(watt1s, watt2s, times)
    util.plotError([ref-real for ref, real in zip(desired_th1s, tracking_th1s)]
                    ,[ref-real for ref, real in zip(desired_th2s, tracking_th2s)], times)
        

