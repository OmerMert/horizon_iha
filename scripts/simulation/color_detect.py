import cv2
import numpy as np

def detect_center(frame):
    lower_red = np.array([0,100,30])
    upper_red = np.array([20,255,255])

    # Finding the center of the screen
    height, width, _ = frame.shape
    center_x = int(width / 2)
    center_y = int(height / 2)
    offset = 0.25 # Offset value for the center point +-%25
    x_min = center_x - center_x * offset
    x_max = center_x + center_x * offset
    y_min = center_y - center_y * offset
    y_max = center_y + center_y * offset

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Converting BGR image to HSV format

    red_mask = cv2.inRange(hsv_frame, lower_red, upper_red) # Masking the image to find our color
    
    area_detected = False

    M = cv2.moments(red_mask)
    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        cv2.circle(frame, (int(cx), int(cy)), 10, (255,255,255), -1)

        if ((x_min < cx) and (cx < x_max) and (y_min < cy) and (cy < y_max)):
            area_detected = True

    except:
        pass
    
    cv2.imshow("window", frame) # Displaying webcam image

    cv2.waitKey(1) 

    return area_detected