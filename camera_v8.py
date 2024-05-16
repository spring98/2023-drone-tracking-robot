import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

import json
from time import time

class CameraV8:
    def __init__(self, client):
        self.x = 340
        self.y = 240

        self.WITDH = 640
        self.HEIGHT = 480

        self.num_detected = 0
        self.num_drone = 0
        self.idx_drone = []
        self.drone_confidences = []

        self.drone_depth = 0

        self.client = client
    
    def send(self, x, y, depth):
        # 데이터를 JSON 형식으로 인코딩
        data_to_send = {
            'x': x,
            'y': y,
            'depth': round(depth, 2)
        }
        json_data = json.dumps(data_to_send)


        if round(depth, 2 != 0):
            # JSON 문자열을 서버로 전송
            self.client.sendall((json_data + '\n').encode('utf-8'))

    def execute(self):
        # parameters
        model = YOLO("vision_team/weight/11_30_v8.pt")
        CLASSES = {0: "car", 1: "drone", 2: "kickboard"}
        colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))



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


        # Start streaming
        profile = pipeline.start(config)

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
                    cv2.imshow("Bgr frame", np.zeros((self.HEIGHT, self.WITDH, 3), dtype=np.uint8))
                    # cv2.imshow("Depth frame", np.zeros((self.HEIGHT, self.WITDH), dtype=np.uint8))
                    
                    cv2.waitKey(1)

                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())


                depth_image = cv2.resize(depth_image, (self.WITDH, self.HEIGHT))
                color_image = cv2.resize(color_image, (self.WITDH, self.HEIGHT))
                color_image2 = np.asanyarray(color_frame.get_data())
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

                depth_image_scaled = cv2.convertScaleAbs(depth_image, alpha=0.025)

                results= model.track(color_image, tracker="bytetrack.yaml", stream=True)
                # results= model(color_image, stream=True)

                cv2.imshow("Bgr frame", depth_image_scaled)

                class_ids = []
                confidences = []
                bboxes = []
                obj_centers = []
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        confidence = box.conf
                        if confidence > 0.5:
                            xyxy = box.xyxy.tolist()[0]
                            bboxes.append(xyxy)
                            confidences.append(float(confidence))
                            class_ids.append(box.cls.tolist())
                            cx = int((xyxy[2]+xyxy[0])//2)
                            cy = int((xyxy[3]+xyxy[1])//2)
                            obj_centers.append([cx,cy])

                result_boxes = cv2.dnn.NMSBoxes(bboxes, confidences, 0.25, 0.45, 0.5)


                if len(class_ids):
                    # print(result_boxes)
                    font = cv2.FONT_HERSHEY_PLAIN
                    depth_list = list()
                    drone_idx = list()
                    drone_confidences = []

                    for i in range(len(class_ids)):

                        label = str(CLASSES[int(class_ids[i][0])])
                        # print(label)
                        if label == 'drone':
                            drone_idx.append(i)
                            drone_confidences.append(confidences[i])
                    
                    if len(drone_idx) > 0:
                        if len(drone_idx) > 1:
                            # tracking much higher confidence drone
                            real_drone_idx = drone_idx[drone_confidences.index(max(drone_confidences))]
                            print("real_drone_idx: ", real_drone_idx, "it's confidence: ", confidences[real_drone_idx])
                            
                            self.x = int(obj_centers[real_drone_idx][0])
                            self.y = int(obj_centers[real_drone_idx][1])

                            frames = pipeline.wait_for_frames()
                            aligned_frames = align.process(frames)
                            depth_frame = aligned_frames.get_depth_frame()
                            color_frame = aligned_frames.get_color_frame()
                            depth_info = depth_frame.as_depth_frame()

                            self.drone_depth = round((depth_info.get_distance(self.x, self.y) * 100), 2)

                            print("drone_depth1: ", self.drone_depth)
                            self.send(self.x, self.y, self.drone_depth)


                            color = colors[real_drone_idx]
                            color = (int(color[0]), int(color[1]), int(color[2]))

                            bbox = list(map(int, bboxes[real_drone_idx])) 
                            x, y, x2, y2 = bbox

                            cv2.rectangle(color_image, (x, y), (x2, y2), color, 2)
                            cv2.circle(depth_image_scaled,(self.x,self.y),20,color,20)
                            cv2.rectangle(depth_image_scaled, (x, y), (x2, y2), color, 2)
                            cv2.putText(color_image, "{} cm".format(self.drone_depth), (x + 5, y + 60), 0, 1.0, color, 2)
                            cv2.putText(depth_image_scaled, "{} cm".format(self.drone_depth), (x + 5, y + 60), 0, 1.0, color, 2)
                            cv2.putText(color_image, label, (x, y + 30), font, 3, color, 3)

                            cv2.imshow("Bgr frame", color_image)
                            # cv2.imshow("Depth frame", depth_image_scaled)

                            color_image2 = cv2.circle(color_image2, (self.x, self.y), 2, (0, 0, 255), -1)
                            cv2.imshow('RealSense', color_image2)
                        
                        else:
                            # just tracking one drone                            
                            self.x = int(obj_centers[drone_idx[0]][0])
                            self.y = int(obj_centers[drone_idx[0]][1])
                            
                            frames = pipeline.wait_for_frames()
                            aligned_frames = align.process(frames)
                            depth_frame = aligned_frames.get_depth_frame()
                            color_frame = aligned_frames.get_color_frame()
                            depth_info = depth_frame.as_depth_frame()

                            self.drone_depth = round((depth_info.get_distance(self.x, self.y) * 100), 2)
                            self.send(self.x, self.y, self.drone_depth)

                            print("drone_depth2: ", self.drone_depth)

                            color = colors[0]
                            color = (int(color[0]), int(color[1]), int(color[2]))

                            bbox = list(map(int, bboxes[drone_idx[0]])) 
                            x, y, x2, y2 = bbox

                            cv2.rectangle(color_image, (x, y), (x2, y2), color, 2)
                            cv2.circle(depth_image_scaled,(self.x,self.y),20,color,20)
                            cv2.rectangle(depth_image_scaled, (x, y), (x2, y2), color, 2)
                            cv2.putText(color_image, "{} cm".format(self.drone_depth), (x + 5, y + 60), 0, 1.0, color, 2)
                            cv2.putText(depth_image_scaled, "{} cm".format(self.drone_depth), (x + 5, y + 60), 0, 1.0, color, 2)
                            cv2.putText(color_image, label, (x, y + 30), font, 3, color, 3)

                            cv2.imshow("Bgr frame", color_image)
                            # cv2.imshow("Depth frame", depth_image_scaled)

                            color_image2 = cv2.circle(color_image2, (self.x, self.y), 2, (0, 0, 255), -1)
                            cv2.imshow('RealSense', color_image2)

                    else:
                        # Detecting car or kickboard, not drone
                        # missing --> just tracking prior drone location
                        frames = pipeline.wait_for_frames()
                        aligned_frames = align.process(frames)
                        depth_frame = aligned_frames.get_depth_frame()
                        color_frame = aligned_frames.get_color_frame()
                        depth_info = depth_frame.as_depth_frame()

                        self.drone_depth = round((depth_info.get_distance(self.x, self.y) * 100), 2)
                        self.send(self.x, self.y, self.drone_depth)

                        print("drone_depth3: ", self.drone_depth)

                        cv2.imshow("Bgr frame", color_image)
                        # cv2.imshow("Depth frame", depth_image_scaled)

                        
                        color_image2 = cv2.circle(color_image2, (self.x, self.y), 2, (0, 0, 255), -1)
                        cv2.imshow('RealSense', color_image2)

                color_image2 = cv2.circle(color_image2, (self.x, self.y), 2, (0, 0, 255), -1)
                cv2.imshow('RealSense', color_image2)
                cv2.imshow("Bgr frame", color_image)
                # cv2.imshow("Depth frame", depth_image_scaled)
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            cv2.destroyAllWindows()
            pipeline.stop()

if __name__ == "__main__":
    Depth_camera = CameraV8()
    try:
        Depth_camera.execute()
    except KeyboardInterrupt:
        print("Shutting down")


