# -*- coding: utf-8 -*-

"""
© Copyright 2020, TSUPORT (Tsunami Post Disaster Robot) Research Team, Politeknik Elektronika Negeri Surabaya.
By: Hardefa Rizky Putu Rogonondo (Mechatronics Engineering 2016, 3110161019).
"""

# Library
from dronekit import Command, connect, LocationGlobal, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil # Used for command message definitions.
import argparse
import cv2
import json
import math
import numpy as np
import pandas as pd
import threading
import time

# Initialization
# Connection string for Ubuntu = /dev/ttyUSB0 | Windows = COM18 | MAVProxy = 127.0.0.1:14551.
connection_string = "127.0.0.1:14551"
aTargetAltitude = 2 # Target altitude for the vehicle while taking off.
aerial_speed = 0.5 # Aerial speed for the vehicle when going to the waypoint.
waypoint = [] # Create an empty list of waypoints from the calibrated file.
cap = cv2.VideoCapture(2) # Opening camera streaming from video receiver.
histogram = {} # Create an empty histogram dictionary.
orientation = [] # Create an empty element for the histogram dictionary.
distance = [] # Create an empty element for the histogram dictionary.

# Opening Messages
print("\n\n---Welcome to TSUPORT Rescue Program---\n\n")

# Connect to The Vehicle
print("Connecting to vehicle on: %s" %connection_string)
vehicle = connect(connection_string, wait_ready = False, heartbeat_timeout = 100, baud = 57600) # wait_ready = False is used to bypass the paramters loading and connecting the vehicle first.
vehicle.wait_ready(True, timeout = 200) # wait_ready = True is to load parameters now.
#while vehicle.gps_0.fix_type < 6 and vehicle.gps_0.eph < 200: #and vehicle.gps_0.epv < 200:
while vehicle.gps_0.satellites_visible < 6 and vehicle.gps_0.eph < 200:
# Could also use vehicle.gps_0.fix_type for better GPS quality.
# EPH = HDOP * 100 | Ideal HDOP <= 1 | Excellent HDOP <= 2 | Good HDOP <= 5 | Moderate HDOP <= 10.
    print "Waiting for GPS...:", vehicle.gps_0.satellites_visible
    time.sleep(1)
print "Satellites Count = ", vehicle.gps_0.satellites_visible

def Control():
    # Arms vehicle and fly to aTargetAltitude
    def arm_and_takeoff(aTargetAltitude):
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready.
        #while not vehicle.is_armable:
            #print(" Waiting for vehicle to initialise...")
            #time.sleep(1)
        print("Arming motors")
        # Copter should arm in GUIDED mode.
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        # Confirm vehicle armed before attempting to take off.
        while not vehicle.armed:
            vehicle.mode = VehicleMode("STABILIZE")
            vehicle.mode = VehicleMode("GUIDED")
            vehicle.armed = True
            print("Waiting for arming...")
            time.sleep(1)
        # Take off to target aTargetAltitude.
        print("Target altitude = %s" %aTargetAltitude)
        print("Taking off!! Please be careful")
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude.
        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command after Vehicle.simple_takeoff will execute immediately).
        while True:
            print("Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    # Arms vehicle and fly to aTargetAltitude without GPS data
    def arm_and_takeoff_nogps(aTargetAltitude):
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready.
        #while not vehicle.is_armable:
            #print("Waiting for vehicle to initialise...")
            #time.sleep(1)
        print("Arming motors")
        # Vehicle should arm in GUIDED_NOGPS mode.
        vehicle.mode = VehicleMode("GUIDED_NOGPS")
        vehicle.armed = True
        # Confirm vehicle armed before attempting to take off.
        while not vehicle.armed:
            vehicle.mode = VehicleMode("STABILIZE")
            vehicle.mode = VehicleMode("GUIDED_NOGPS")
            vehicle.armed = True
            print("Waiting for arming...")
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

    # Fly the vehicle to the calibrated waypoints in certain duration
    def go_to(duration, waypoint):
        print ("Going towards %s ..." %waypoint)
        start = time.time()
        while time.time() - start <= duration:
            vehicle.simple_goto(waypoint)
            time.sleep(2)

    # Positioning the vehicle using compass oriented motion
    def positioning(velocity_x, velocity_y, velocity_z):
        print("Entering positioning mode")
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, # time_boot_ms (not used).
            0, 0, # Target system, target component.
            mavutil.mavlink.MAV_FRAME_BODY_NED, # NED frame.
            0b0000111111000111, # type_mask (only speeds enabled).
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters.
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters.
            0, # alt - Altitude in meters in AMSL altitude (not WGS84 if absolute or relative).
               # Altitude above terrain if GLOBAL_TERRAIN_ALT_INT.
            velocity_x, # X velocity in NED frame in m/s. +X = Right roll | -X = Left roll
            velocity_y, # Y velocity in NED frame in m/s. +Y = Front pitch | -Y = Backward pitch
            velocity_z, # Z velocity in NED frame in m/s. +Z = Going up | -Z = Going down
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

    # Convert degrees to quaternions
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

    # RTL the vehicle after mission accomplished
    def go_home():
        print("Mission Accomplished")
        print("Initiate Return to Launch")
        vehicle.mode = VehicleMode("RTL")
        while not vehicle.mode.name == 'RTL':
            vehicle.mode = VehicleMode("RTL")
        vehicle.close()

    # Landing the vehicle after mission accomplished
    def landing():
        print("Mission Accomplished")
        print("Initiate Landing")
        vehicle.mode = VehicleMode("LAND")
        while not vehicle.mode.name == 'LAND':
            vehicle.mode = VehicleMode("LAND")
        vehicle.close()

    # MAVLink message listener using the python decorator
    @vehicle.on_message('DISTANCE_SENSOR')
    def listener(self, name, message):
        global prox_id, prox_dist
        global orientation, distance
        prox_id = message.id
        prox_dist = message.current_distance
        orientation.append(prox_id)
        distance.append(prox_dist)
        # Use the command below to print the detected obstacle in terminal.
        #print "ID: %s, Distance: %s" % (prox_id, prox_dist)

    #Vector Field Histogram Algorithm calculation
    def VectorFieldHistogram():
        global robot_radius
        global desired_safety_distance
        global desired_min_turning_radius
        robot_radius = 0.74
        desired_safety_distance = 1.5
        desired_min_turning_radius = 0.1
        for i in range(len(tempdata)):
                histogram[tempdata[i]] = distance[np.where(orientation == tempdata[i])]
        if prox_dist <= 150:
            roll_counted = robot_radius + desired_safety_distance + desired_min_turning_radius
            set_attitude(roll_counted, 0, 0)

    # Main Mission
    start = input("Please input flight code: ")
    if start == 777:
        # Opening calibrated coordinates file
        with open("data_coordinate.txt") as dc:
            gps_save = json.load(dc)
        loop = 0
        # Load coordinates as waypoints
        while (loop < len(gps_save["latitude_longitude"])):
            latitude = float(gps_save["latitude_longitude"][loop]["latitude {}".format(loop+1)])
            longitude = float(gps_save["latitude_longitude"][loop]["longitude {}".format(loop+1)])
            waypoint.append(LocationGlobalRelative(latitude, longitude, aTargetAltitude))
            loop += 1
        # Choose mode to take off in (GUIDED Mode or GUIDED NO GPS Mode)
        arm_and_takeoff(aTargetAltitude)
        #arm_and_takeoff_nogps(aTargetAltitude)
        vehicle.airspeed = aerial_speed
        loop_waypoint = 0
        # Customize mission in between of each waypoints here
        while (loop_waypoint < len(gps_save["latitude_longitude"])):
            if (loop_waypoint == 0):
                go_to(30, waypoint[0]) # Duration to each waypoint needed to be counted first using the vehicle airspeed in consideration.
            #print("Masuk save csv")
            orientation = np.array(orientation)
            distance = np.array(distance)
            tempdata = np.unique(orientation)
            for i in range(len(tempdata)):
                histogram[tempdata[i]] = distance[np.where(orientation == tempdata[i])]
            df = pd.DataFrame.from_dict(histogram, orient='index')
            df = df.transpose()
            df.to_csv("histogram.csv", index = False)
            loop_waypoint += 1
            # Example of custom mission:
            # Positioning using compass oriented heading:
            #positioning(3, 0, 0)
            #print("Roll position for 3 meters without changing height")
            #set_attitude(duration = 10)
            #print("Hold position for 10 seconds")
            # Positioning using vehicle oriented heading:
            # Move the vehicle to right
            #set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
            #print("Roll position for 1 meter without changing height for 3 seconds")
            # Move the vehicle heading to right
            #set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)
            #print("Yaw position for 30 degrees without changing height for 3 seconds")
            # Move the vehicle backward
            #set_attitude(pitch_angle = -5, thrust = 0.5, duration = 3.21)
            #print("Move backward for 5 meters without changing height for 3.21 seconds")
            # Move the vehicle forward
            #set_attitude(pitch_angle = 5, thrust = 0.5, duration = 3)
            #print("Move forward for 5 meters without changing height for 3 seconds")
            # Note that it will be in front of original position due to inertia.
        landing()
        print threading.currentThread().getName(), 'Shutting Down'
    else:
        print("\nWrong flight code!\n")
        print("Program will be closed...")

def Image():
    # Check if the live streaming successfull
    if cap.isOpened() == False:
        print("Error Live Streaming")
        print threading .currentThread().getName(), 'Live Streaming Failed'
    # Default resolutions of the frame are obtained.The default resolutions are system dependent.
    #frame_width = int(cap.get(3))
    #frame_height = int(cap.get(4))
    # Define the codec and create VideoWriter object.The output is stored in 'Live_Stream.avi' file.
    #vid = cv2.VideoWriter('Live_Stream.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width, frame_height))
    # Live streaming until closed
    while (cap.isOpened()):
        # Capture frame by frame
        # Notes:
        # grabbed --> a boolean return value from getting the frame.
        # image --> the current frame being projected in the video.
        (grabbed, image)= cap.read()
        # Write the frame into the file Live_Stream.avi
        #vid.write(image)
        cv2.putText(image, "TSUPORT", (180,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)
        cv2.putText(image, "- Live Streaming -", (160,80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Live Streaming', image)
        # Frames are read by intervals of 10 milliseconds. The programs breaks out of the while loop when the user presses the 'esc' key.
        if cv2.waitKey(50) == 27:
            break
    # The following frees up resources and closes all windows
    cap.release()
    #vid.release()
    cv2.destroyAllWindows()
    print threading.currentThread().getName(), 'All Done'

Control = threading.Thread(name = 'Control', target = Control, args = ())
Image = threading.Thread(name = 'Image', target = Image, args = ())

Control.start()
Image.start()