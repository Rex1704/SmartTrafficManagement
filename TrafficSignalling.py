import math
import Simulation
import cv2
import numpy as np
from ultralytics import YOLO
from sort import *


with open("yolo-model/coco.names", "r") as f:
    classes = [line.strip() for line in f]
    classes = classes[:13]

total_Vehicles = {0:0,1:0,2:0,3:0}

model = YOLO("yolo-model/yolov8n.pt")

try:
    vid1 = cv2.VideoCapture("videos/vid1.mp4")
    vid2 = cv2.VideoCapture("videos/vid2.mp4")
    vid3 = cv2.VideoCapture("videos/vid3.mp4")
    vid4 = cv2.VideoCapture("videos/vid4.mp4")
except:
    print("Error: Video not found add videos in videos folder")
    exit()

while True:
    # success,frame = vid.read()
    isTrue1, frame1 = vid1.read()
    isTrue2, frame2 = vid2.read()
    isTrue3, frame3 = vid3.read()
    isTrue4, frame4 = vid4.read()

    if not (isTrue1 and isTrue2 and isTrue3 and isTrue4):
        break

    frame2 = cv2.resize(frame2, (frame1.shape[1] // 2, frame1.shape[0] // 2))
    frame3 = cv2.resize(frame3, (frame1.shape[1] // 2, frame1.shape[0] // 2))
    frame4 = cv2.resize(frame4, (frame1.shape[1] // 2, frame1.shape[0] // 2))
    frame1 = cv2.resize(frame1, (frame1.shape[1] // 2, frame1.shape[0] // 2))
    font = cv2.FONT_HERSHEY_PLAIN

    for frame, vid_num in zip([frame1, frame2, frame3, frame4], [1, 2, 3, 4]):
        object_counts = {}
        results = model(frame, stream=True)

        detection = np.empty((0, 5))

        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                conf = math.ceil((box.conf[0] * 100)) / 100

                cls = int(box.cls[0])
                currentClass = classes[cls]

                if ((int(box.cls[0]) > 1) & (int(box.cls[0]) <= 7)) :
                    if conf > 0.3:
                        label = "Vehicle"
                        currentArray = np.array([x1, y1, x2, y2, conf])
                        detection = np.vstack((detection, currentArray))
                else:
                    label = str(classes[int(box.cls[0])])

                if label not in object_counts:
                    object_counts[label] = 1
                else:
                    object_counts[label] += 1

        vehicles_new = object_counts["Vehicle"]


        if total_Vehicles[vid_num-1] < vehicles_new:
            total_Vehicles[vid_num-1] += (vehicles_new-total_Vehicles[vid_num-1])



        for label, count in object_counts.items():
            cv2.putText(frame, f"{label}: {count}", (10, 30 * (list(object_counts.keys()).index(label) + 1)),
                        font, 3, (0, 0, 0), 3)

        print(f"Video{vid_num}")

    frame_up = np.concatenate((frame1, frame2), axis=1)
    frame_down = np.concatenate((frame3, frame4), axis=1)
    frame_final = np.vstack((frame_up, frame_down))

    cv2.namedWindow("vid", cv2.WINDOW_NORMAL)
    cv2.imshow("vid", frame_final)

    if cv2.waitKey(1) == ord('q'):
        vid1.release()
        vid2.release()
        vid3.release()
        vid4.release()
        break


total_Vehicles1 = total_Vehicles[0]
total_Vehicles2 = total_Vehicles[1]
total_Vehicles3 = total_Vehicles[2]
total_Vehicles4 = total_Vehicles[3]

print(f"lane1:{total_Vehicles1}, lane2:{total_Vehicles2}, lane3:{total_Vehicles3}, lane4:{total_Vehicles4}")

Simulation.maxCounts = total_Vehicles
Simulation.Main()
