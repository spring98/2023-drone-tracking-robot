from math import sin, cos, pi, atan2, asin, acos, sqrt
import numpy as np
from numpy import degrees


# noinspection PyBroadException
class Coordinate:
    def __init__(self):
        self.DEFAULT = 0
        self.L1 = 0.1
        self.L2 = 0.055
    
    def camera2drone(self, param_Xp, param_Yp, param_depth, param_th1=0, param_th2=0):
        # 설정해줘야하는 초기값
        depth = param_depth
        Xp, Yp = param_Xp, param_Yp
        th1, th2 = param_th1, param_th2

        # 한번만 설정하면 되는 시스템 값
        Fx, Fy = 731.31020846, 738.98151297
        Cx, Cy = 323.71648136, 292.21260431

        d1 = 0.07
        a2, a3 = 0.03, 0.085

        # Define camera extrinsic matrix using given angles and translations
        r11, r12, r13 = np.sin(th1), np.cos(th1) * np.sin(th2), np.cos(th1) * np.cos(th2)
        r21, r22, r23 = -np.cos(th1), np.sin(th1) * np.sin(th2), np.sin(th1) * np.cos(th2)
        r31, r32, r33 = 0, -np.cos(th2), np.sin(th2)
        tx, ty, tz = -a3 * np.cos(th1) * np.sin(th2), -a3 * np.sin(th1) * np.sin(th2), a2 + a3 * np.cos(th2) + d1

        # Camera intrinsic matrix
        CameraIntrinsic = np.array([
            [Fx, 0, Cx],
            [0, Fy, Cy],
            [0, 0, 1]
        ])

        # Camera extrinsic matrix
        CameraExtrinsic = np.array([
            [r11, r12, r13, tx],
            [r21, r22, r23, ty],
            [r31, r32, r33, tz],
            [0, 0, 0, 1]
        ])

        # Define the 3x4 identity matrix (eye34)
        eye34 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ])

        # 주의(카메라 외부 파라미터는 DH Parameter 의 inverse 이다.)
        # Calculate the XYZ position of the drone
        DroneXYZ = CameraExtrinsic @ np.linalg.pinv(eye34) @ np.linalg.inv(CameraIntrinsic) @ (depth * np.array([Xp, Yp, 1]))

        return DroneXYZ

    def drone2radian(self, param_Xd, param_Yd, param_Zd):
        A = (param_Zd + self.DEFAULT) - self.L1
        B = param_Xd**2 + param_Yd**2
        if self.L1 < param_Zd < self.L1 + self.L2:
            try:
                theta1 = -acos((-self.L2*A + sqrt((self.L2*A)**2 + (A**2+B)*(B-self.L2**2)))/(A**2+B))
            except Exception as e:
                print(e)
                theta1 = -1

        elif param_Zd <= self.L1:
            try:
                theta1 = -acos((self.L2*A + sqrt((self.L2*A)**2 + (A**2+B)*(B-self.L2**2)))/(A**2+B))
            except Exception as e:
                print(e)
                theta1 = -1
        
        else:
            try:
                theta1 = acos((self.L2*A + sqrt((self.L2*A)**2 + (A**2+B)*(B-self.L2**2)))/(A**2+B))
            except Exception as e:
                print(e)
                theta1 = -1

        try:
            theta2 = atan2(param_Yd, param_Xd)
        except Exception as e:
            theta2 = -1

        # return 30, 40
        if theta1 == -1 or theta2 == -1:
            return -1, -1
        else:
            # return degrees(theta1), degrees(theta2)
            # return theta1, theta2
            return theta2, theta1

    def TH2STEP(self, TH, offset):
        return (4096 / 360) * (180 - TH - offset)
