from GUI import GUI
from HAL import HAL
import cv2
import numpy as np 

kp = 0.005 #Proportional Gain

while True:
    frame = HAL.getImage()
    #convert image to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range of red color in HSV
    red_low = np.array([0, 100, 100])
    red_up = np.array([10, 255, 255])

    mask = cv2.inRange(hsv, red_low, red_up)

    # Find the contour of the line
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    

    if len(contours) > 0:
        line_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(line_contour)
        
        # centroid calculation 
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # Find the center of the image
        img_center = frame.shape[1] // 2
        error = cX - img_center
        steering = kp * error

        cv2.drawContours(frame, [line_contour], -1, (0, 255, 0), 2)
        cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)

        HAL.setV(1)  

        ang_velocity = - steering #Keep negative for left turn and positive for right turn.

        HAL.setW(ang_velocity)

         
    GUI.showImage(frame)   
        