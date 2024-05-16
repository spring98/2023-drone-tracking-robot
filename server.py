import socket
from _thread import *
from motor_team.pid_controller import PIDController
from motor_team.smc_controller import SMCController
from motor_team.mpc_controller import MPCController
from motor_team.interface import Interface
from motor_team.coordinate import Coordinate
from motor_team.control import Control
from motor_team.interface import Interface
from motor_team.interface_fake import InterfaceFake
from motor_team.dynamics import Dynamics
from motor_team.utils import Utils
import cmath

if __name__ == "__main__":
    # 소켓 초기화
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('localhost', 5004))
    server.listen(5)
    print("Waiting for a connection")

    while True:
        conn, addr = server.accept()
        print('Connected to:', addr)

        # utils
        coordinate = Coordinate()
        utils = Utils()
        dynamics = Dynamics()

        # PID 제어기 생성
        motor = Interface(utils=utils)
        # motor = InterfaceFake(utils=utils)

        # controller = PIDController(motor=motor, utils=utils, dynamics=dynamics)
        controller = SMCController(motor=motor, utils=utils, dynamics=dynamics)
        # controller = MPCController(motor=motor, utils=utils, zeta1=cmath.sqrt(2)/2, zeta2=cmath.sqrt(2)/2, w0_1=55, w0_2=30)
        
        control = Control(conn=conn, motor=motor, utils=utils, coordinate=coordinate, controller=controller, isFake=True)

        # 새로운 클라이언트에 대한 스레드 시작
        start_new_thread(control.execute(), ())
