# this program detect green and red object tested by paresh date :01/10/2024
# raspberry pi model 4b and Pi camera V2
# added min area threshold to filter unwanted small object detetion
# Bottom left coordinate added in V3
# Ditance from camera added in V4
# Increase preview size from 640x480 to 1280x720 (HD) V5
# also downscale frame size to 680x480 for faster process V5
# Serial communication between pi and esp32 normal board V6 - Not tested
# Serial communication between pi and esp32 normal board V7 - 
# Raspberry Pi to ESP32 Data Serial V8 - simpler sending format V8

import cv2
import numpy as np
from picamera2 import Picamera2
import serial
import time

# Initialize the camera
picam2 = Picamera2()

# Increase preview size from 640x480 to 1280x720 (HD)
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (1280, 720)}))
#picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640,480)}))
picam2.start()

# Minimum area threshold to filter out small objects
MIN_AREA_THRESHOLD = 1200  # You can adjust this value

# Constants for distance calculation
KNOWN_HEIGHT = 100  # Height of the object in mm (you need to adjust it according to your object)
FOCAL_LENGTH = 615  # Focal length of the camera in pixels (you can adjust this based on calibration)

# Setup Serial communication
serial_port = serial.Serial('/dev/serial0', 9600, timeout=1)

def detect_color_objects(frame):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the color green
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])

    # Define the lower and upper bounds for the color red
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])

    # Create a mask to isolate green objects
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Create masks to isolate red objects (red has two ranges in HSV)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Function to calculate the distance to the object
    def calculate_distance(pixel_height):
        if pixel_height > 0:
            return int((FOCAL_LENGTH * KNOWN_HEIGHT) / pixel_height)  # Convert to integer
        else:
            return 0

    nearest_object = {"color": None, "distance": float('inf'), "coordinates": (0, 0)}

    # Find contours of the green objects
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours_green:
        area = cv2.contourArea(contour)
        if area > MIN_AREA_THRESHOLD:
            x, y, w, h = cv2.boundingRect(contour)
            distance = calculate_distance(h)
            if distance < nearest_object['distance']:
                nearest_object = {"color": 1, "distance": distance, "coordinates": (x, y + h)}
            # Draw green bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Find contours of the red objects
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if area > MIN_AREA_THRESHOLD:
            x, y, w, h = cv2.boundingRect(contour)
            distance = calculate_distance(h)
            if distance < nearest_object['distance']:
                nearest_object = {"color": 2, "distance": distance, "coordinates": (x, y + h)}
            # Draw red bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

    return frame, nearest_object

def format_output_string(obj):
    color_code = str(obj['color'])  # 1 for green, 2 for red
    x_coord = str(obj['coordinates'][0]).zfill(3)  # 3-digit X-coordinate
    y_coord = str(obj['coordinates'][1]).zfill(3)  # 3-digit Y-coordinate
    distance = str(obj['distance']).zfill(4)  # 4-digit distance
    return color_code + x_coord + y_coord + distance

try:
    while True:
        # Capture the image frame from Picamera2
        frame = picam2.capture_array()
       
        # Optionally, downscale the frame for faster processing
        frame = cv2.resize(frame, (640, 480))

        # Detect green and red objects in the frame
        output_frame, nearest_object = detect_color_objects(frame)

        # Display the original frame with the detected objects, coordinates, and distance
        cv2.imshow("Green and Red Object Detection", output_frame)
        
        # Send nearest object details to the serial port
        if nearest_object['color']:
            message = format_output_string(nearest_object)
            serial_port.write((message + "\n").encode('utf-8'))
            serial_port.flush()
            print(f"Sent to Serial: {message}")  # Also print to the console for monitoring
            
        time.sleep(0.05)
        # Exit when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Clean up
    cv2.destroyAllWindows()
    picam2.stop()
    serial_port.close()
