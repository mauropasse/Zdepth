import cv2
import numpy as np


cap1 = cv2.VideoCapture('depth_video.avi')
cap2 = cv2.VideoCapture('rgb_video.avi')

while cap1.isOpened() or cap2.isOpened():

    okay1  , frame1 = cap1.read()
    okay2 , frame2 = cap2.read()

    if okay1:
        cv2.imshow('DEPTH' , frame1)

    if okay2:
        cv2.imshow('RGB' , frame2)

    if not okay1 or not okay2:
        print('Cant read the video , Exit!')
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.waitKey(10)

cap1.release()
cap2.release()
cv2.destroyAllWindows()