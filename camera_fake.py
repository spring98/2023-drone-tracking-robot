import json
import time

class CameraFake:
    def __init__(self, client):
        self.client = client

    # def execute(self):
    #     import csv
    #     from collections import deque

    #     with open('fake/camera_20240410_182511.csv', 'r', encoding='utf-8') as f:
    #         lines = deque(list(csv.reader(f)))

    #     time.sleep(5)
    #     while lines:
    #         # x, y, cm, current_time, type = list(map(float, lines.popleft()))
            
    #         x, y, cm, current_time, type = lines.popleft()
    #         x = float(x)
    #         y = float(y)
    #         cm = float(cm)
    #         current_time = float(current_time)

    #         if lines:
    #             next_time = float(lines[0][3])
    #             delay = next_time - current_time

    #             time.sleep(delay)
    #             print(f'delay: {delay}')
    #             self.send(x, y, cm)

    # def send(self, x, y, depth):
    #     # 데이터를 JSON 형식으로 인코딩
    #     data_to_send = {
    #         'x': x,
    #         'y': y,
    #         'depth': depth
    #     }

    #     json_data = json.dumps(data_to_send)

    #     if round(depth, 2) != 0:
    #         # JSON 문자열을 서버로 전송
    #         self.client.sendall((json_data + '\n').encode('utf-8'))


    def execute(self):
        import csv
        from collections import deque

        with open('fake/camera_20240413_110610.csv', 'r', encoding='utf-8') as f:
            lines = deque(list(csv.reader(f)))

        time.sleep(5)
        while lines:
            drone_rad1, drone_rad2, current_time = list(map(float, lines.popleft()))
            
            if lines:
                next_time = float(lines[0][2])
                delay = next_time - current_time

                # time.sleep(delay)
                
                time.sleep(0.2)
                print(f'delay: {delay}')
                self.send(drone_rad1, drone_rad2)

    def send(self, rad1, rad2):
        # 데이터를 JSON 형식으로 인코딩
        data_to_send = {
            'drone_rad1': rad1,
            'drone_rad2': rad2,
        }

        json_data = json.dumps(data_to_send)

        # JSON 문자열을 서버로 전송
        self.client.sendall((json_data + '\n').encode('utf-8'))
