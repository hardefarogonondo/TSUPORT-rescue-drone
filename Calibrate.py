# -*- coding: utf-8 -*-

"""
© Copyright 2020, TSUPORT (Tsunami Post Disaster Robot) Research Team, Politeknik Elektronika Negeri Surabaya.
By: Hardefa Rizky Putu Rogonondo (Mechatronics Engineering 2016, 3110161019).
"""

# Library
from dronekit import connect
import json
import time

# Initialization
# Connection string for Ubuntu = /dev/ttyUSB0 | Windows = COMXX | MAVProxy = 127.0.0.1:14551.
connection_string =  "127.0.0.1:14551"
gps_save = {} # Create an empty GPS coordinate dictionary.
gps_save["latitude_longitude"] = [] # Insert the dictionary with latitude_longitude variable.

# Opening Messages
print("\n\n---Welcome to TSUPORT Waypoint Calibration Program---\n\n")

# Connect to The Vehicle
print("Connecting to vehicle on: %s" %connection_string)
vehicle = connect(connection_string, wait_ready = False, heartbeat_timeout = 100, baud = 57600) # wait_ready = False is used to bypass the paramters loading and connecting the vehicle first.
vehicle.wait_ready(True, timeout = 150) # wait_ready = True is to load parameters now.
#while vehicle.gps_0.fix_type < 6 and vehicle.gps_0.eph <= 200: #and vehicle.gps_0.epv < 200:
while vehicle.gps_0.satellites_visible < 6 and vehicle.gps_0.eph <= 200:
# EPH = HDOP * 100 | Ideal HDOP <= 1 | Excellent HDOP <= 2 | Good HDOP <= 5 | Moderate HDOP <= 10.
    print "Waiting for GPS...:", vehicle.gps_0.fix_type
    time.sleep(1)
print "Satellites Count = ", vehicle.gps_0.satellites_visible

# Main Menu
waypoint_data = int(raw_input("Please input the number of waypoints: "))
loop = 0
while(loop < waypoint_data):
    click = raw_input("Click enter to take GPS coordinate {}!".format(loop + 1))
    if click == "":
        gps_save["latitude_longitude"].append({
            "latitude {}".format(loop + 1):"{}".format(vehicle.location.global_relative_frame.lat),
            "longitude {}".format(loop + 1):"{}".format(vehicle.location.global_relative_frame.lon),
        })
		# Write JSON file
        with open ("data_coordinate.txt", "w") as dc:
            json.dump(gps_save, dc)
        loop = loop + 1
    else:
        pass

# Clossing Messages
print("Calibration done successfully!")