import socket
from vision_team.camera_v5 import CameraV5
from vision_team.camera_v8 import CameraV8
from vision_team.camera_fake import CameraFake

if __name__ == "__main__":
    # 소켓 초기화
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('localhost', 5004))

    # vision_team = CameraV5(client=client)
    # vision_team = CameraV8(client=client)
    vision_team = CameraFake(client=client)
    vision_team.execute()
