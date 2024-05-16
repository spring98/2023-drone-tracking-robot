from dynamixel_sdk import *

# noinspection
class InterfaceProfileImpl:
    def __init__(self, util):
        # 환경 변수
        self.DEVICE_NAME = '/dev/tty.usbserial-FT66WBIV' # Linux: "/dev/ttyUSB*"
        self.PROTOCOL_VERSION = 2.0

        # 모터 아이디
        self.DXL_ID1 = 4

        # 모터 프로퍼티 주소
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_PWM = 100
        self.ADDR_GOAL_CURRENT = 102
        self.ADDR_GOAL_POSITION = 116

        self.ADDR_PRESENT_PWM = 124
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_VELOCITY = 128

        # Trajectory
        self.ADDR_VELOCITY_TRAJECTORY = 136
        self.ADDR_POSITION_TRAJECTORY = 140

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
        self.util = util

    def __del__(self):
        self.disableTorque()
        self.PORT_HANDLER.closePort()

    def initializeHandler(self):
        self.PORT_HANDLER = PortHandler(self.DEVICE_NAME)
        self.PACKET_HANDLER = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.PORT_HANDLER.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Program terminate...")
            quit()

        # Set port baudrate
        if self.PORT_HANDLER.setBaudRate(self.BAUD_RATE):
            print("Succeeded to change the baud rate")
        else:
            print("Failed to change the baud rate")
            print("Program terminate...")
            quit()

    def enableTorque(self):
        # Enable Dynamixel Torque
        result, error = self.PACKET_HANDLER.write1ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        self.printLog(result, error)

    def disableTorque(self):
        # Disable Dynamixel Torque
        result, error = self.PACKET_HANDLER.write1ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.printLog(result, error)

    def printLog(self, result, error):
        if result != COMM_SUCCESS:
            print("%s" % self.PACKET_HANDLER.getTxRxResult(result))
        elif error != 0:
            print("%s" % self.PACKET_HANDLER.getRxPacketError(error))

    def setHome(self):
        # PID 게인 설정
        Kp = 0.5  # 비례 게인
        Ki = 0.1  # 적분 게인
        Kd = 0.01  # 미분 게인

        # PID 오차 초기화
        error_sum1 = 0
        last_error1 = 0

        while True:
            # 현재 위치 읽기
            ID1_CURRENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_POSITION)

            # 라디안으로 변환
            ID1_CURRENT_POSITION = self.util.protectOverflow(ID1_CURRENT_POSITION)

            # 오차 계산
            error1 = 0 - ID1_CURRENT_POSITION

            # 오차 누적
            error_sum1 += error1

            # 미분 계산
            delta_error1 = error1 - last_error1

            # PID 제어
            pwm1 = Kp * error1 + Ki * error_sum1 + Kd * delta_error1

            # PWM 제한
            pwm1 = min(max(pwm1, -100), 100)

            # 모터가 목표 위치에 도달했는지 확인
            if abs(error1) < self.DXL_MOVING_STATUS_THRESHOLD:
                pwm1 = 0  # 첫 번째 모터의 PWM을 0으로 설정하여 정지

            # 모터 제어
            self.PACKET_HANDLER.write4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_GOAL_PWM, int(pwm1))

            # 이전 오차 업데이트
            last_error1 = error1

            # 출력
            print("[ID:%03d] CURR_POS:%03d  PWM:%03d" % (self.DXL_ID1, ID1_CURRENT_POSITION, pwm1))

            # 종료 조건
            if abs(error1) < self.DXL_MOVING_STATUS_THRESHOLD:
                print('break...')
                break

            # 사용자 종료
            if not self.running:
                quit()

    def sendPOS(self, goal_pos_1):
        # Write GOAL PWM
        result, error = self.PACKET_HANDLER.write4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_GOAL_POSITION, goal_pos_1)
        self.printLog(result, error)

        while True:
            # Read CURRENT POSITION
            ID1_CURRENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_POSITION)

            # Read CURRENT VELOCITY
            ID1_CURRENT_VELOCITY, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_VELOCITY)

            ID1_CURRENT_POSITION = self.util.protectOverflow(ID1_CURRENT_POSITION)
            ID1_CURRENT_VELOCITY = self.util.rpm2pps(ID1_CURRENT_VELOCITY)
            print("[ID:%03d] GOAL_POS:%03d  CURR_POS:%03d GOAL_VEL:%03d  CURR_VEL:%03d" % (self.DXL_ID1, goal_pos_1, ID1_CURRENT_POSITION, 100, ID1_CURRENT_VELOCITY))

            if not abs(goal_pos_1 - ID1_CURRENT_POSITION) > self.DXL_MOVING_STATUS_THRESHOLD:
                break
                # quit()

            if not self.running:
                quit()

    def sendPWM(self, goal_pwm_1):
        # Write GOAL PWM
        result, error = self.PACKET_HANDLER.write4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_GOAL_PWM, goal_pwm_1)
        self.printLog(result, error)

        while True:
            # Read CURRENT POSITION
            ID1_CURRENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_POSITION)

            # Read CURRENT VELOCITY
            ID1_CURRENT_VELOCITY, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_VELOCITY)

            ID1_CURRENT_POSITION = self.util.protectOverflow(ID1_CURRENT_POSITION)
            ID1_CURRENT_VELOCITY = self.util.rpm2pps(ID1_CURRENT_VELOCITY)
            print("[ID:%03d] GOAL_POS:%03d  CURR_POS:%03d GOAL_VEL:%03d  CURR_VEL:%03d" % (self.DXL_ID1, goal_pwm_1, ID1_CURRENT_POSITION, 100, ID1_CURRENT_VELOCITY))

            if not abs(goal_pwm_1 - ID1_CURRENT_POSITION) > self.DXL_MOVING_STATUS_THRESHOLD:
                break
                # quit()

            if not self.running:
                quit()

    # Current Based Position 0 ~ 1,193
    def sendCBP(self, profile_vel, profile_acc, position):
        # Plot Data
        poses = []
        veles = []

        result, error = self.PACKET_HANDLER.write4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PROFILE_ACCELERATION, profile_acc)
        self.printLog(result, error)

        result, error = self.PACKET_HANDLER.write4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PROFILE_VELOCITY, profile_vel)
        self.printLog(result, error)

        # Write GOAL PWM
        result, error = self.PACKET_HANDLER.write4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_GOAL_POSITION, position)
        self.printLog(result, error)

        self.readPresent(poses, veles)

        while True:
            ID1_CURRENT_POSITION, ID1_CURRENT_VELOCITY = self.readPresent(poses, veles)
            print("[ID:%03d] GOAL_POS:%03d CURR_POS:%03d" % (self.DXL_ID1, position, ID1_CURRENT_POSITION))

            if not abs(position - ID1_CURRENT_POSITION) > self.DXL_MOVING_STATUS_THRESHOLD:
                self.readPresent(poses, veles)
                return poses, veles
                # quit()

            if not self.running:
                quit()

    def readPresent(self, poses, veles):
        # Read CURRENT POSITION
        ID1_CURRENT_POSITION, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_POSITION)

        # Read CURRENT VELOCITY
        ID1_CURRENT_VELOCITY, _, _ = self.PACKET_HANDLER.read4ByteTxRx(self.PORT_HANDLER, self.DXL_ID1, self.ADDR_PRESENT_VELOCITY)

        ID1_CURRENT_POSITION = self.util.protectOverflow(ID1_CURRENT_POSITION)
        ID1_CURRENT_VELOCITY = self.util.rpm2pps(ID1_CURRENT_VELOCITY)

        poses.append(ID1_CURRENT_POSITION)
        veles.append(ID1_CURRENT_VELOCITY)

        return ID1_CURRENT_POSITION, ID1_CURRENT_VELOCITY

    def signal_handler(self, sig, frame):
        print('You pressed Ctrl+C!')
        self.running = False
