import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
import pickle

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

count=265

while (cap.isOpened()):
    ret, frame = cap.read()

    cv2.imshow("output", frame)
    #cv2.imshow("output2", undist)

    cv2.imwrite("frame_day3_ %d.jpg" % count, frame)  # save frame as JPEG file
    success, image = cap.read()
    print('Read a new frame: ', success)
    count += 1

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.waitKey(1)
cv2.destroyAllWindows()
