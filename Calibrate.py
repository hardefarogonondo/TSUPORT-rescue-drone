# -*- coding: utf-8 -*-

"""
© Copyright 2020, TSUPORT (Tsunami Post Disaster Robot) Research Team.
By: Hardefa Rizky Putu Rogonondo (Mechatronics Engineering 2016, 3110161019).
"""

from dronekit import connect
import json
import time

# Connection Initialization
connection_string =  "/dev/ttyUSB0" #"127.0.0.1:14550"
# Database Initialization
gps_save = {} # beda {} dan [] itu apa?????----------------------------------------------
gps_save["latitude_longitude"] = []
# Loop Initialization
loop_var = 0

# Opening Messages
print("\n\n---Welcome to TSUPORT Waypoint Calibration Program---\n\n")

# Connect to The Vehicle
print("Connecting to vehicle on: %s" %connection_string)
vehicle = connect(connection_string, wait_ready = True, baud = 57600, heartbeat_timeout = 100)
while vehicle.gps_0.fix_type < 6: #and vehicle.gps_0.eph < 200 and vehicle.gps_0.epv < 200:
# EPH = HDOP * 100 | Good HDOP value is between 1 - 10
    print "Waiting for GPS...:", vehicle.gps_0.fix_type
    time.sleep(1)
print "Satellites Count = ", vehicle.gps_0.satellites_visible

# Main Menu
waypoint_data = int(raw_input("Please input number of waypoints: "))
while(loop_var < waypoint_data):
    click = raw_input("Click enter to take GPS coordinate {}!".format(loop_var + 1))
    if click == "":
        gps_save["latitude_longitude"].append({
            "latitute {}".format(loop_var + 1):"{}".format(vehicle.location.global_relative_frame.lat),
            "longitude {}".format(loop_var + 1):"{}".format(vehicle.location.global_relative_frame.lon),
        })
		# Write JSON file
        with open ("data_coordinate.txt", "w") as dc:
            json.dump(gps_save, dc)
        loop_var = loop_var + 1
    else:
        pass

# Clossing Messages
print("Calibration done successfully!")