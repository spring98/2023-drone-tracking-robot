import pyrealsense2 as rs
import numpy as np
import cv2
import numpy as np
import torch
import json
import time

class CameraV5:
    def __init__(self, client):
        print('CameraV5 start')
        self.client = client

        self.x = 340
        self.y = 240

        self.WITDH = 640
        self.HEIGHT = 480

        self.num_detected = 0
        self.num_drone = 0
        self.idx_drone = []
        self.drone_confidences = []
        self.drone_depth = 0
        self.fakes = []
        
        self.start_time = time.time()
    
    def send(self, x, y, depth, type):
        cm = round(depth, 2)

        # 데이터를 JSON 형식으로 인코딩
        data_to_send = {
            'x': x,
            'y': y,
            'depth': cm,
            'type': type
        }
        json_data = json.dumps(data_to_send)

        if cm != 0:
            # JSON 문자열을 서버로 전송
            self.client.sendall((json_data + '\n').encode('utf-8'))
            # self.fakes.append([x, y, cm, time.time() - self.start_time, type])

    def execute(self):
        model = torch.hub.load("ultralytics/yolov5", "custom", "vision_team/weight/v5.pt")

        # Create a config and configure the pipeline to stream for Realsense-L515
        pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)
        config.enable_stream(rs.stream.depth, self.WITDH, self.HEIGHT, rs.format.z16, 30)
        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, self.WITDH, self.HEIGHT, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, self.WITDH, self.HEIGHT, rs.format.bgr8, 30)

        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)

        # Start streaming
        pipeline.start(config)
        align_to = rs.stream.color
        align = rs.align(align_to)

        # Streaming loop
        try:
            while True:
                # Get frameset of color and depth
                frames = pipeline.wait_for_frames()
                                
                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                depth_image = cv2.resize(depth_image, (self.WITDH, self.HEIGHT))
                color_image = cv2.resize(color_image, (self.WITDH, self.HEIGHT))
                color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

                depth_image_scaled = cv2.convertScaleAbs(depth_image, alpha=0.025)
                results = model(color_image)
                rendered_frame = results.render()[0]

                cv2.imshow("Bounding Box", color_image)
                #========================================================
                # results.xyxy[0][0][:4] --> bounding box's location
                # results.xyxy[0][0][4] --> confidence
                # results.xyxy[0][0][5] --> class
                # len(results.xyxy[0]) --> number of detected objects
                # Class --> {car:0, drone:1, kickboard:2} 
                #=========================================================

                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                depth_info = depth_frame.as_depth_frame()

                if len(results.xyxy[0]):
                    raw_data = results.xyxy[0]
                    self.num_detected = len(raw_data)
                    self.drone_confidences = []
                    self.num_drone = 0
                    self.idx_drone = []
                
                    for i in range(self.num_detected):
                        if int(raw_data[i][5]) == 1:
                            self.num_drone += 1
                            self.idx_drone.append(i)
                            self.drone_confidences.append(raw_data[i][4])
                    
                    if len(self.idx_drone) == 0:
                        print(f"잡긴했는데＿드론이＿아님: type3: {self.drone_depth}")
                        # Detecting car or kickboard, not drone
                        # missing --> just tracking prior drone location 
                        self.send(320, 240, self.drone_depth, "type3")

                    elif len(self.idx_drone) == 1:
                        print(f"섞인_상태에서_드론이_1개_있음: type1: {self.drone_depth}")
                        # just tracking one drone
                        xmin = float(raw_data[self.idx_drone[0]][0])
                        ymin = float(raw_data[self.idx_drone[0]][1])
                        xmax = float(raw_data[self.idx_drone[0]][2])
                        ymax = float(raw_data[self.idx_drone[0]][3])

                        self.x, self.y = int((xmin + xmax)/2), int((ymin + ymax)/2)
                        self.drone_depth = round((depth_info.get_distance(self.x, self.y) * 100), 2)

                        self.send(self.x, self.y, self.drone_depth, "type1")

                    else:
                        if max(self.drone_confidences) > 0.5:
                            print(f"섞인_상태에서_드론이_2개_이상_있음: type2: {self.drone_depth}")
                            # tracking much higher confidence drone
                            real_drone_idx = self.idx_drone[self.drone_confidences.index(max(self.drone_confidences))]
                            
                            xmin = float(raw_data[real_drone_idx][0])
                            ymin = float(raw_data[real_drone_idx][1])
                            xmax = float(raw_data[real_drone_idx][2])
                            ymax = float(raw_data[real_drone_idx][3])
                            
                            self.x, self.y = int((xmin + xmax)/2), int((ymin + ymax)/2)
                            self.drone_depth = round((depth_info.get_distance(self.x, self.y) * 100), 2)

                            self.send(self.x, self.y, self.drone_depth, "type2")

                else:
                    print(f"아무_것도_못_잡은상태: type4: {self.drone_depth}")
                    # There is no detected object!
                    # missing --> just tracking prior drone location
                    self.send(320, 240, self.drone_depth, "type4")

                color_image = np.asanyarray(color_frame.get_data())
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                color_image = cv2.circle(color_image, (self.x, self.y), 2, (0, 0, 255), -1)
                cv2.imshow('RealSense', color_image)
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            print("fake start")

            # import csv
            # from datetime import datetime

            # with open(f'fake/camera_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', encoding='utf-8') as f:
            #     wr = csv.writer(f)
            #     for fake in self.fakes:
            #         wr.writerow(fake)

            print('fake end')
            pipeline.stop()
