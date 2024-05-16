from dynamixel_sdk import *
import numpy as np
from scipy.integrate import odeint

# noinspection,DuplicatedCode
class Interface:
    def __init__(self, utils):
        # 환경 변수
        # self.DEVICE_NAME = '/dev/tty.usbserial-FT66WBIV' # Linux: "/dev/ttyUSB*"
        self.DEVICE_NAME = '/dev/ttyUSB0'
        # self.DEVICE_NAME = 'COM4'
        self.PROTOCOL_VERSION = 2.0

        # 모터 아이디
        self.DXL_ID1 = 1
        self.DXL_ID2 = 2

        # 모터 프로퍼티 주소
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_CURRENT = 102
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_GOAL_VELOCITY = 104
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_VELOCITY = 128
        self.ADDR_OPERATING_MODE = 11

        # Profile
        self.ADDR_PROFILE_ACCELERATION = 108
        self.ADDR_PROFILE_VELOCITY = 112

        # 모터 프로퍼티 데이터
        # self.BAUD_RATE = 57600
        self.BAUD_RATE = 3000000
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # 모터 컨트롤 상수
        self.DXL_MOVING_STATUS_THRESHOLD = 5  # Dynamixel moving status threshold

        # 모터 컨트롤 객체
        self.PORT_HANDLER = None
        self.PACKET_HANDLER = None

        # 모터 컨트롤 객체 초기화
        self.initializeHandler()
        self.enableTorque()

        # 모터 테스트 변수
        self.running = True

        # 유틸
        self.utils = utils

        # 모터 파라미터 초기화
        self.J = 0.0039  # 관성 모멘트 (kg.m^2)
        self.b = 0.01  # 마찰 계수 (Nms)
        self.K = 1.84  # 모터 상수 (Nm/A)
        self.motor1_state = np.array([0.0, 0.0])  # 초기 상태 [theta, omega]
        self.motor2_state = np.array([0.0, 0.0])  # 초기 상태 [theta, omega]
        self.t = 0.0

    def __del__(self):
        self.disableTorque()
        self.PORT_HANDLER.closePort()

    def initializeHandler(self):
        self.PORT_HANDLER = PortHandler(self.DEVICE_NAME)
        self.PACKET_HANDLER = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.PORT_HANDLER.openPort():
            pass
            # print("Succeeded to open the port")
        else:
            # print("Failed to open the port")
            # print("Program terminate...")
            quit()

        # Set port baudrate
        if self.PORT_HANDLER.setBaudRate(self.BAUD_RATE):
            pass
            # print("Succeeded to change the baud rate")
        else:
            # print("Failed to change the baud rate")
            # print("Program terminate...")
            quit()

    def enableTorque(self):
        # Enable Dynamixel Torque
        result1, error1 = self.PACKET_HANDLER.write1ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        result2, error2 = self.PACKET_HANDLER.write1ByteTxRx(self.PORT_HANDLER, self.DXL_ID2, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        
        self.printLog(result1, error1)
        self.printLog(result2, error2)

    def disableTorque(self):
        # Disable Dynamixel Torque
        result1, error1 = self.PACKET_HANDLER.write1ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        result2, error2 = self.PACKET_HANDLER.write1ByteTxRx(self.PORT_HANDLER, self.DXL_ID2, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

        self.printLog(result1, error1)
        self.printLog(result2, error2)

    def reboot(self, dxl_id):
        # Dynamixel 모터 재부팅
        result, error = self.PACKET_HANDLER.reboot(self.PORT_HANDLER, dxl_id)
        if result != COMM_SUCCESS:
            print("Reboot failed")
            print("%s" % self.PACKET_HANDLER.getTxRxResult(result))
        elif error != 0:
            print("Error occurred during reboot!")
            print("%s" % self.PACKET_HANDLER.getRxPacketError(error))
        else:
            print("Dynamixel has been successfully rebooted")

        # # 포트를 닫고 다시 열기 (재부팅 후 필요할 수 있음)
        # self.PORT_HANDLER.closePort()
        # if not self.PORT_HANDLER.openPort():
        #     print("Failed to reopen the port")
        #     return False

        # # 재부팅 후 보드레이트 재설정 (필요한 경우)
        # if not self.PORT_HANDLER.setBaudRate(self.BAUD_RATE):
        #     print("Failed to set the baud rate")
        #     return False
        
        return True

    def signal_handler(self, sig, frame):
        print(f'You pressed Ctrl+C! {sig} {frame}')
        self.running = False

    def printLog(self, result, error):
        if result != COMM_SUCCESS:
            pass
            # print("%s" % self.PACKET_HANDLER.getTxRxResult(result))
        elif error != 0:
            pass
            # print("%s" % self.PACKET_HANDLER.getRxPacketError(error))

    def readData(self):
        # Read CURRENT POSITION
        ID1_CURRENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1,
                                                                       self.ADDR_PRESENT_POSITION)

        ID2_CURRENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID2,
                                                                       self.ADDR_PRESENT_POSITION)

        # Read CURRENT VELOCITY
        ID1_CURRENT_VELOCITY, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1,
                                                                       self.ADDR_PRESENT_VELOCITY)

        ID2_CURRENT_VELOCITY, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID2,
                                                                       self.ADDR_PRESENT_VELOCITY)

        ID1_CURRENT_POSITION = self.utils.pos2rad(ID1_CURRENT_POSITION)
        ID1_CURRENT_VELOCITY = self.utils.rpm2radps(ID1_CURRENT_VELOCITY)

        return ID1_CURRENT_POSITION, ID1_CURRENT_VELOCITY

    def readPresent(self, poses, veles):
        # Read CURRENT POSITION
        ID1_CURRENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_POSITION)
        ID1_CURRENT_POSITION = self.utils.protectOverflow(ID1_CURRENT_POSITION)

        ID2_CURRENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID2, self.ADDR_PRESENT_POSITION)
        ID2_CURRENT_POSITION = self.utils.protectOverflow(ID2_CURRENT_POSITION)

        # Read CURRENT VELOCITY
        ID1_CURRENT_VELOCITY, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_VELOCITY)
        ID1_CURRENT_VELOCITY = self.utils.rpm2pps(ID1_CURRENT_VELOCITY)

        ID2_CURRENT_VELOCITY, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID2, self.ADDR_PRESENT_VELOCITY)
        ID2_CURRENT_VELOCITY = self.utils.rpm2pps(ID2_CURRENT_VELOCITY)

        poses.append(ID1_CURRENT_POSITION)
        veles.append(ID1_CURRENT_VELOCITY)

        return ID1_CURRENT_POSITION, ID1_CURRENT_VELOCITY

    def setHome(self):
        # 위치 제어 모드로 변경
        self.disableTorque()
        self.PACKET_HANDLER.write2ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_OPERATING_MODE, 1)
        self.PACKET_HANDLER.write2ByteTxRx(self.PORT_HANDLER, self.DXL_ID2, self.ADDR_OPERATING_MODE, 1)
        self.enableTorque()

        initial_po1, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_POSITION)
        initial_po2, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID2, self.ADDR_PRESENT_POSITION)

        initial_po1 = self.utils.protectOverflow(initial_po1)
        initial_po2 = self.utils.protectOverflow(initial_po2)

        max_pos1 = max(-initial_po1, initial_po1)
        max_pos2 = max(-initial_po2, initial_po2)

        while True:
            start_time = time.time()
            # 현재 위치 읽기
            ID1_PRESENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_POSITION)
            ID2_PRESENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID2, self.ADDR_PRESENT_POSITION)
            
            # 오버플로우 방지
            ID1_PRESENT_POSITION = self.utils.protectOverflow(ID1_PRESENT_POSITION)
            ID2_PRESENT_POSITION = self.utils.protectOverflow(ID2_PRESENT_POSITION)
            
            # 오차 계산
            error1 = 0 - ID1_PRESENT_POSITION
            error2 = 0 - ID2_PRESENT_POSITION
            
            # 에러값을 (-10 ~ 10) 범위로 조정하여 목표값 계산
            vel_digit_max = 40
            
            # goal1 = int((vel_digit_max / max_pos1) * error1) if int((vel_digit_max / max_pos1) * error1) != 0 else (-5 if error1 < 0 else 5)
            # goal2 = int((vel_digit_max / max_pos2) * error2) if int((vel_digit_max / max_pos2) * error2) != 0 else (-5 if error2 < 0 else 5)
            # goal1 = int((vel_digit_max / max_pos1) * error1) if abs(int((vel_digit_max / max_pos1) * error1) > 5) else (-5 if error1 < 0 else 5)
            # goal2 = int((vel_digit_max / max_pos2) * error2) if abs(int((vel_digit_max / max_pos2) * error2) > 5) else (-5 if error2 < 0 else 5)
            # goal1 = vel_digit_max if abs(int((vel_digit_max / max_pos1) * error1) > 1) else (-5 if error1 < 0 else 5)
            # goal2 = vel_digit_max if abs(int((vel_digit_max / max_pos2) * error2) > 1) else (-5 if error2 < 0 else 5)

            if abs(error1) > 50:
                if error1 > 0:
                    goal1 = vel_digit_max
                else:
                    goal1 = -vel_digit_max
            
            else:
                if error1 > 0:
                    goal1 = 5
                else:
                    goal1 = -5

            if abs(error2) > 50:
                if error2 > 0:
                    goal2 = vel_digit_max
                else:
                    goal2 = -vel_digit_max
            
            else:
                if error2 > 0:
                    goal2 = 1
                else:
                    goal2 = -1                
            
            # 모터 제어
            if abs(error1) > self.DXL_MOVING_STATUS_THRESHOLD - 2:
                self.PACKET_HANDLER.write4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_GOAL_VELOCITY, goal1)

            if abs(error2) > self.DXL_MOVING_STATUS_THRESHOLD - 2:
                self.PACKET_HANDLER.write4ByteTxRx(self.PORT_HANDLER, self.DXL_ID2, self.ADDR_GOAL_VELOCITY, goal2)

            # 출력
            # print("[ID:%03d] PRESENT_POS:%03d" % (self.DXL_ID1, ID1_PRESENT_POSITION), end='\t')
            # print("[ID:%03d] PRESENT_POS:%03d" % (self.DXL_ID2, ID2_PRESENT_POSITION))

            # 종료 조건
            if (abs(error1) < self.DXL_MOVING_STATUS_THRESHOLD) and (abs(error2) < self.DXL_MOVING_STATUS_THRESHOLD):
                # print('break...')
                # 토크 해제
                self.disableTorque()

                # 위치 제어 모드로 변경
                self.PACKET_HANDLER.write2ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_OPERATING_MODE, 0)
                self.PACKET_HANDLER.write2ByteTxRx(self.PORT_HANDLER, self.DXL_ID2, self.ADDR_OPERATING_MODE, 0)

                # 토크 인가
                self.enableTorque()
                break

            # 사용자 종료
            if not self.running:
                quit()
            
            end_time = time.time()

            # print("set_home_time: ", end_time-start_time)    

    def sendCUR(self, goal_current, goal_current2):
        if not self.running:
            quit()

        # GroupSyncWrite instance for goal current
        groupSyncWriteCurrent = GroupSyncWrite(self.PORT_HANDLER, self.PACKET_HANDLER, self.ADDR_GOAL_CURRENT, 2)

        # 모터1에 대한 전류 값 설정
        param_goal_current1 = [DXL_LOBYTE(self.utils.amp2digit(goal_current)),
                            DXL_HIBYTE(self.utils.amp2digit(goal_current))]
        # 모터2에 대한 전류 값 설정
        param_goal_current2 = [DXL_LOBYTE(self.utils.amp2digit(goal_current2)),
                            DXL_HIBYTE(self.utils.amp2digit(goal_current2))]

        # Syncwrite storage에 모터1과 모터2의 전류 값 추가
        groupSyncWriteCurrent.addParam(self.DXL_ID1, param_goal_current1)
        groupSyncWriteCurrent.addParam(self.DXL_ID2, param_goal_current2)

        # Syncwrite goal current
        groupSyncWriteCurrent.txPacket()

        # Syncwrite parameter storage를 정리
        groupSyncWriteCurrent.clearParam()

        # GroupSyncRead instance for present position and velocity (모터1과 모터2 모두에 대해)
        groupSyncReadPosition = GroupSyncRead(self.PORT_HANDLER, self.PACKET_HANDLER, self.ADDR_PRESENT_POSITION, 4)
        groupSyncReadVelocity = GroupSyncRead(self.PORT_HANDLER, self.PACKET_HANDLER, self.ADDR_PRESENT_VELOCITY, 4)

        # 모터1과 모터2에 대한 파라미터 저장소 추가
        groupSyncReadPosition.addParam(self.DXL_ID1)
        groupSyncReadPosition.addParam(self.DXL_ID2)
        groupSyncReadVelocity.addParam(self.DXL_ID1)
        groupSyncReadVelocity.addParam(self.DXL_ID2)

        # Syncread present position and velocity
        groupSyncReadPosition.txRxPacket()
        groupSyncReadVelocity.txRxPacket()

        # 모터1의 현재 위치와 속도 읽기
        dxl1_present_position = groupSyncReadPosition.getData(self.DXL_ID1, self.ADDR_PRESENT_POSITION, 4)
        dxl1_present_velocity = groupSyncReadVelocity.getData(self.DXL_ID1, self.ADDR_PRESENT_VELOCITY, 4)

        # 모터2의 현재 위치와 속도 읽기
        dxl2_present_position = groupSyncReadPosition.getData(self.DXL_ID2, self.ADDR_PRESENT_POSITION, 4)
        dxl2_present_velocity = groupSyncReadVelocity.getData(self.DXL_ID2, self.ADDR_PRESENT_VELOCITY, 4)

        # Syncread parameter storage를 정리
        groupSyncReadPosition.clearParam()
        groupSyncReadVelocity.clearParam()

        # print("dxl1_present_position: ", dxl1_present_position)
        # print("type(dxl1_present_position): ", type(dxl1_present_position))
        # print("self.utils.pos2rad(dxl1_present_position): ", self.utils.pos2rad(dxl1_present_position))
        # print("type(self.utils.pos2rad(dxl1_present_position)): ", type(self.utils.pos2rad(dxl1_present_position)))

        # 모터1과 모터2의 현재 위치와 속도 반환
        return (self.utils.pos2rad(dxl1_present_position), self.utils.rpm2radps(dxl1_present_velocity),
                self.utils.pos2rad(dxl2_present_position), self.utils.rpm2radps(dxl2_present_velocity))

    def sendCURFake(self, torque1, torque2, dt=0.005):
        if not self.running:
            quit()

        t_span = [self.t, self.t + dt]

        self.motor1_state = odeint(self.motor_dynamics, self.motor1_state, t_span, args=(self.J, self.b, self.K, torque1))[-1]
        self.motor2_state = odeint(self.motor_dynamics, self.motor2_state, t_span, args=(self.J, self.b, self.K, torque2))[-1]

        self.t += dt

        # 모터1과 모터2의 현재 위치와 속도 반환
        return self.motor1_state[0], self.motor1_state[1], self.motor2_state[0], self.motor2_state[1]

    # 운동 방정식 정의
    def motor_dynamics(self, y, t, J, b, K, torque):
        theta, omega = y  # 각도와 각속도
        dydt = [omega, (1 / J) * (torque - b * omega)]
        return dydt
