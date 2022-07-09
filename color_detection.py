import cv2
import numpy as np

def gstreamer_pipeline(
    capture_width=1920,
    capture_height=1080,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
    
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def show_camera():
    # Specifying upper and lower ranges of red color to detect in hsv format
    lower_red = np.array([160,100,10])
    upper_red = np.array([179,255,255])

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=4), cv2.CAP_GSTREAMER)

    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)

        # Window
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:

            success, frame = cap.read() # Reading webcam footage

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
            red = cv2.bitwise_and(frame, frame, mask = red_mask) # Masking with color
            
            mask_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Finding contours in mask image

            # Finding position of all contours
            if len(mask_contours) != 0:
                
                # Find the biggest countour (c) by the area
                max_contour = max(mask_contours, key = cv2.contourArea)

                if cv2.contourArea(max_contour) > 500:
                    x, y, w, h = cv2.boundingRect(max_contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3) #drawing rectangle
                    
                    # Finding center of countour
                    M = cv2.moments(max_contour)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    # Checking if the center of the contour is around the center of the screen
                    if ((x_min < cx) and (cx < x_max) and (y_min < cy) and (cy < y_max)):
                        cv2.circle(frame, (cx, cy), 7, (255,255, 255), -1)
                        cv2.putText(frame, "center", (cx - 20, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        #cv2.imwrite("frame.jpg", frame) # Capturing frame
                # INTERRUPT TO CAPTURE GPS DATA
            
            cv2.imshow("CSI Camera", frame) # Displaying webcam image

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


if __name__ == "__main__":
    show_camera()