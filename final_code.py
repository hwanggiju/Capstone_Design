
import face_recognition
import cv2
import time
from scipy.spatial import distance as dist
import sys
import numpy as np

EYES_CLOSED_SECONDS = 5

model = 'res10_300x300_ssd_iter_140000.caffemodel'
config = 'deploy.prototxt.txt'

net = cv2.dnn.readNet(model, config)

if net.empty() :
    print('Net open failed!')
    sys.exit()

def main():
    closed_count = 0
    cap = cv2.VideoCapture(0)
    _, frame = cap.read(0)
    print(cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    if not cap.isOpened() :
        print('Camera open failed!')
        sys.exit()
        
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb_small_frame = small_frame[:, :, ::-1]
    face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)
    process = True
    
    while True:
        _, frame = cap.read(0)
        if frame is None :
            break

        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = small_frame[:, :, ::-1]
        
        blob = cv2.dnn.blobFromImage(frame, 1, (300, 300), (104, 177, 123))
        net.setInput(blob)
        detect = net.forward()
        
        (h, w) = frame.shape[:2]
        detect = detect[0, 0, :, :]
        
        for i in range(detect.shape[0]) : 
            confidence = detect[i, 2]
            if confidence < 0.5 :
                break
            
            x1 = int(detect[i, 3] * w)
            y1 = int(detect[i, 4] * h)
            x2 = int(detect[i, 5] * w)
            y2 = int(detect[i, 6] * h)
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0))
            
            area = (x2-x1) * (y2-y1)
            center_x = x1 + (x2-x1)/2 
            center_y = y1 + (y2-y1)/2
            
            cv2.imshow('Facerec_Video', frame)

            if process:
                face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)
                
                for face_landmark in face_landmarks_list:
                    left_eye = face_landmark['left_eye']
                    right_eye = face_landmark['right_eye']
                    color = (255,0,0)
                    thickness = 2

                    cv2.rectangle(small_frame, left_eye[0], right_eye[-1], color, thickness)

                    cv2.imshow('Sleep_Video', small_frame)

                    ear_left = get_ear(left_eye)
                    ear_right = get_ear(right_eye)
                    
                    print('area : %d    center_x : %d   center_y : %d   left_eye : %lf   left_eye : %lf' 
                          %(area, center_x, center_y, ear_left, ear_right))

                    closed = ear_left < 0.2 and ear_right < 0.2
                    closed = 0
                    if (closed):
                        closed_count += 1

                    else:
                        closed_count = 0

                    if (closed_count >= EYES_CLOSED_SECONDS):
                        asleep = True
                        while (asleep):
                            print("EYES CLOSED")

                            if cv2.waitKey(1) == 32: 
                                asleep = False
                                print("EYES OPENED")
                        closed_count = 0

        process = not process
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break


def get_ear(eye):
	A = dist.euclidean(eye[1], eye[5])
	B = dist.euclidean(eye[2], eye[4])
	C = dist.euclidean(eye[0], eye[3])
	ear = (A + B) / (2.0 * C)
	return ear


if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()

