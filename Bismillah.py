# -*- coding: utf-8 -*-

"""
© Copyright 2020, TSUPORT (Tsunami Post Disaster Robot) Research Team.
By: Hardefa Rizky Putu Rogonondo (Mechatronics Engineering 2016, 3110161019).
"""

# Library
from dronekit import connect, LocationGlobal, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil # Needed for command message definitions.
import cv2
import math
import numpy as np
import threading
import time

# Initialization
# Connection string for Ubuntu = /dev/ttyUSB0 | Windows = COM18 | MAVProxy = 127.0.0.1:14550.
connection_string = "/dev/ttyUSB0"
aTargetAltitude = 1
cap = cv2.VideoCapture("input.mp4")

# Opening Messages
print("\n\n---Welcome to TSUPORT Rescue Program---\n\n")

# Connect to The Vehicle
print("Connecting to vehicle on: %s" %connection_string)
vehicle = connect(connection_string, wait_ready = True, baud = 57600, heartbeat_timeout = 100)

def Control():
    # Arms vehicle and fly to aTargetAltitude without GPS data.
    def arm_and_takeoff_nogps(aTargetAltitude):
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready.
        #while not vehicle.is_armable:
        #    print("Waiting for vehicle to initialise...")
        #    time.sleep(1)
        print("Arming motors")
        # Vehicle should arm in GUIDED_NOGPS mode.
        vehicle.mode = VehicleMode("GUIDED_NOGPS")
        vehicle.armed = True
        # Confirm vehicle armed before attempting to take off.
        while not vehicle.armed:
            vehicle.mode = VehicleMode("STABILIZE")
            vehicle.mode = VehicleMode("GUIDED_NOGPS")
            print("Waiting for arming...")
            vehicle.armed = True
            time.sleep(1)
        # Take off to target aTargetAltitude.
        print("Target altitude = %s" %aTargetAltitude) 
        print("Taking off! Please be careful!!")
        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            print("Altitude: %f    Desired: %f" %(current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude * 0.95: # Trigger just below target altitude.
                print("Reached target altitude")
                break
            elif current_altitude >= aTargetAltitude * 0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            set_attitude(thrust = thrust)
            time.sleep(0.2)

    # Notes:
    # use_yaw_rate --> The yaw can be controlled using yaw_angle OR yaw_rate. When one is used, the other is ignored by Ardupilot.
    # thrust --> 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
    #        --> Note that as of AC 3.5, thrust = 0.5 triggers a special case in the code for maintaining current altitude.
    #        --> thrust >  0.5 = Ascend.
    #        --> thrust == 0.5 = Hold the altitude.
    #        --> thrust <  0.5 = Descend.
    def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5):
        # This value may be unused by the vehicle, depending on use_yaw_rate.
        if yaw_angle is None:
            yaw_angle = vehicle.attitude.yaw
        msg = vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms.
            1, # Target system.
            1, # Target component.
            0b00000000 if use_yaw_rate else 0b00000100,
            to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion.
            0, # Body roll rate in radian.
            0, # Body pitch rate in radian.
            math.radians(yaw_rate), # Body yaw rate in radian/second.
            thrust  # Thrust
        )
        vehicle.send_mavlink(msg)

    # Notes:
    # In AC 3.3 and later the message should be re-sent more often than every second, as an ATTITUDE_TARGET order has a timeout of 1s.
    # In AC 3.2.1 and earlier the specified attitude persists until it is canceled.
    # The code below should work on either version.
    # Sending the message multiple times is the recommended way.
    def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5, duration = 0):
        send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
        start = time.time()
        while time.time() - start < duration:
            send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout.
        send_attitude_target(0, 0, 0, 0, True, thrust)

    # Convert degrees to quaternions.
    def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))
        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
        return [w, x, y, z]

    # Landing the vehicle after mission accomplished.
    def landing():
        print("Mission Accomplished")
        print("Initiate Landing")
        vehicle.mode = VehicleMode("LAND")
        while not vehicle.mode.name == 'LAND':
            vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        vehicle.close()

    # -----------------------------------------------------------------------------------------
    def positioning(velocity_x, velocity_y, velocity_z):
        print("Entering positioning mode")
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
	    0,                                  # time_boot_ms (not used)
	    0, 0,                               # target system, target component
	    mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
	    0b0000111111000111,                 # type_mask (only speeds enabled)
	    0,                                  # lat_int - X Position in WGS84 frame in 1e7 * meters
	    0,                                  # lon_int - Y Position in WGS84 frame in 1e7 * meters
	    0,                                  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
	    # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
	    velocity_x,                         # X velocity in NED frame in m/s x+ forward x- backward kanan kiri
	    velocity_y,                         # Y velocity in NED frame in m/s y+ right y- left maju mundur
	    velocity_z,                         # Z velocity in NED frame in m/s z+ down z- up
	    0, 0, 0,                            # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
	    0, 0)                               # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        vehicle.send_mavlink(msg)
        vehicle.flush()

    # Main Mission.
    start = input("Please input flight code: ")
    if start == 777:
        arm_and_takeoff_nogps(aTargetAltitude)
        print("Hold position for 3 seconds")
        set_attitude(duration = 3)
        # Uncomment the lines below for testing roll angle and yaw rate.
        # Make sure that there is enough space for testing this.
        #set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
        #set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)
        # Move the drone forward and backward.
        # Note that it will be in front of original position due to inertia.
        #print("Move forward")
        #set_attitude(pitch_angle = -5, thrust = 0.5, duration = 3.21)
        #print("Move backward")
        #set_attitude(pitch_angle = 5, thrust = 0.5, duration = 3)
        landing()
        print(threading.currentThread().getName(), 'Shutting Down')
    else:
        print("\nWrong flight code!\n")
        print("Program will be closed...")

def Image():
    # Canny edge detection filter.
    def do_canny(frame):
        # Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive.
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # Applies a 5x5 gaussian blur with deviation of 0 to frame - not mandatory since Canny will do this for us.
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # Applies Canny edge detector with minimum value of 50 and maximum value of 150.
        canny = cv2.Canny(blur, 50, 150)
        return canny

    # Segmenting the frame.
    def do_segment(frame):
        # Since an image is a multi-directional array containing the relative intensities of each pixel in the image, we can use frame.shape to return a tuple: [number of rows, number of columns, number of channels] of the dimensions of the frame.
        # frame.shape[0] give us the number of rows of pixels the frame has.
        # Since height begins from 0 at the top, the y-coordinate of the bottom of the frame is its height.
        height = frame.shape[0]
        # Creates a triangular polygon for the mask defined by three (x, y) coordinates.
        polygons = np.array([[(0, height), (800, height), (380, 290)]])
        # Creates an image filled with zero intensities with the same dimensions as the frame.
        mask = np.zeros_like(frame)
        # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0.
        cv2.fillPoly(mask, polygons, 255)
        # A bitwise and operation between the mask and frame keeps only the triangular area of the frame.
        segment = cv2.bitwise_and(frame, mask)
        return segment

    # Calculate lines of detection.
    def calculate_lines(frame, lines):
        # Empty arrays to store the coordinates of the left and right lines.
        left = []
        right = []
        # Loops through every detected line.
        for line in lines:
            # Reshapes line from 2D array to 1D array.
            x1, y1, x2, y2 = line.reshape(4)
            # Fits a linear polynomial to the x and y coordinates and returns a vector of coefficients which describe the slope and y-intercept.
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            y_intercept = parameters[1]
            # If slope is negative, the line is to the left of the lane, and otherwise, the line is to the right of the lane.
            if slope < 0:
                left.append((slope, y_intercept))
            else:
                right.append((slope, y_intercept))
        # Averages out all the values for left and right into a single slope and y-intercept value for each line.
        left_avg = np.average(left, axis = 0)
        right_avg = np.average(right, axis = 0)
        # Calculates the x1, y1, x2, y2 coordinates for the left and right lines.
        left_line = calculate_coordinates(frame, left_avg)
        right_line = calculate_coordinates(frame, right_avg)
        return np.array([left_line, right_line])

    # Calculate detection coordinate.
    def calculate_coordinates(frame, parameters):
        slope, intercept = parameters
        # Sets initial y-coordinate as height from top down (bottom of the frame).
        y1 = frame.shape[0]
        # Sets final y-coordinate as 150 above the bottom of the frame.
        y2 = int(y1 - 150)
        # Sets initial x-coordinate as (y1 - b) / m since y1 = mx1 + b.
        x1 = int((y1 - intercept) / slope)
        # Sets final x-coordinate as (y2 - b) / m since y2 = mx2 + b.
        x2 = int((y2 - intercept) / slope)
        return np.array([x1, y1, x2, y2])

    # Draw lines of detection.
    def visualize_lines(frame, lines):
        # Creates an image filled with zero intensities with the same dimensions as the frame.
        lines_visualize = np.zeros_like(frame)
        # Checks if any lines are detected.
        if lines is not None:
            for x1, y1, x2, y2 in lines:
                # Draws lines between two coordinates with green color and 5 thickness.
                cv2.line(lines_visualize, (x1, y1), (x2, y2), (0, 255, 0), 5)
        return lines_visualize

    while (cap.isOpened()):
        # Notes:
        # ret --> a boolean return value from getting the frame.
        # frame --> the current frame being projected in the video.
        ret, frame = cap.read()
        canny = do_canny(frame)
        cv2.imshow("canny", canny)
        segment = do_segment(canny)
        hough = cv2.HoughLinesP(segment, 2, np.pi / 180, 100, np.array([]), minLineLength = 100, maxLineGap = 50)
        # Averages multiple detected lines from hough into one line for left border of lane and one line for right border of lane.
        lines = calculate_lines(frame, hough)
        # Visualizes the lines.
        lines_visualize = visualize_lines(frame, lines)
        cv2.imshow("hough", lines_visualize)
        # Overlays lines on frame by taking their weighted sums and adding an arbitrary scalar value of 1 as the gamma argument.
        output = cv2.addWeighted(frame, 0.9, lines_visualize, 1, 1)
        # Opens a new window and displays the output frame.
        cv2.imshow("output", output)
        # Frames are read by intervals of 10 milliseconds. The programs breaks out of the while loop when the user presses the 'q' key.
        if cv2.waitKey(10) == 27:
            cap.release()
            cv2.destroyAllWindows()
            break
    # The following frees up resources and closes all windows
    cap.release()
    cv2.destroyAllWindows()

Control = threading.Thread(name = 'Control', target = Control, args = ())
Image = threading.Thread(name = 'Image', target = Image,args = ())

Control.start()
Image.start()