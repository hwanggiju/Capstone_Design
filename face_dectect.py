from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2
import os

protoPath = 'deploy.prototxt'
modelPath = 'res10_300x300_ssd_iter_140000.caffemodel'
detector = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

vs = VideoStream(src=0).start()
time.sleep(2.0)

fps = FPS().start()
while True :
    frame = vs.read()
    frame = imutils.resize(frame, width=600)
    (height, width) = frame.shape[:2]
    
    imageBlob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0), swapRB = False, crop = False)
    
    detector.setInput(imageBlob)
    detections = detector.forward()