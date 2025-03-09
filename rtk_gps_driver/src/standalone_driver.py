#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from rtk_gps_driver.msg import RTK_Customgps
import sys
import utm
import time 
from datetime import datetime, timezone
import os


class GPS_DRIVER():
    
    def __init__(self, file_path = None):
        
        current_path = os.path.join(os.getcwd(), "../data/raw_rtk_data/")
        if file_path is None:
            file_path = os.path.join(current_path, "open_stationary.ubx")
        self.file_path = file_path
        rospy.loginfo(f"GPS_DRIVER initialized with file path: {self.file_path}")
        self.sequence_number = 0
        self.node = rospy.init_node('gps_driver')
        self.publisher = rospy.Publisher('/gps', RTK_Customgps, queue_size=10)
        self.rate = rospy.Rate(1)

    def isGNGGAinString(self, inputString):
        '''Check whether the input string is GNGGA or not
        Args: Input String -> Data must be a string 
        Returns: False if the string is GNGGA, True otherwise'''
        if len(inputString) == 0 or not inputString[0].startswith("$GNGGA"):
            rospy.loginfo(f"NO GNGGA String found in {inputString}")
            return True
        else:
            rospy.loginfo(f"Verified GNGGA String found in {inputString}")
            return False
    
    def update_raw_data(self, raw_gps_data_split):
        '''Update raw GPS data from the split string.'''
        if len(raw_gps_data_split) < 15:
            rospy.logdebug("Not enough data points to update.")
            return [0.0] * 8  # Return a list with 0.0 values

        try:
            self.utc = raw_gps_data_split[1] if raw_gps_data_split[1] else 0.0
            self.latitude = raw_gps_data_split[2] if raw_gps_data_split[2] else 0.0
            self.latitudeDir = raw_gps_data_split[3] if raw_gps_data_split[3] else None
            self.longitude = raw_gps_data_split[4] if raw_gps_data_split[4] else 0.0
            self.longitudeDir = raw_gps_data_split[5] if raw_gps_data_split[5] else None
            self.altitude = float(raw_gps_data_split[9]) if raw_gps_data_split[9] else 0.0
            self.hdop = float(raw_gps_data_split[8]) if raw_gps_data_split[8] else 0.0
            self.fix = int(raw_gps_data_split[6]) if raw_gps_data_split[6] else 0.0
        except ValueError as e:
            rospy.logerr(f"ValueError: {e}")
            return [0.0] * 8  # Return a list with default values

        return [self.utc, self.latitude, self.latitudeDir, self.longitude, self.longitudeDir, self.altitude, self.hdop, self.fix]
       
    def degMinstoDegDec(self, LatOrLong, is_latitude=True):
        '''Converts latitude or longitude from DDmm.mm to DD.dddd format.'''
        if is_latitude:
            deg = int(LatOrLong[:2])  
            mins = float(LatOrLong[2:])
        else:
            deg = int(LatOrLong[:3])  
            mins = float(LatOrLong[3:])  
        return deg + (mins / 60.0)
        
    def LatLongSignConvetion(self, LatOrLong, LatOrLongDir):
        '''Converts latitude or longitude to signed value based on direction.'''
        if LatOrLongDir in ['S', 'W']:  
            return -1 * LatOrLong  
        else:
            return LatOrLong

    def convertToUTM(self, LatitudeSigned, LongitudeSigned):
        '''Converts latitude and longitude to UTM coordinates.'''
        UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
        UTMEasting = UTMVals[0]
        UTMNorthing = UTMVals[1]
        UTMZone = UTMVals[2]
        UTMLetter = UTMVals[3]
        return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

    def UTCtoUTCEpoch(self, UTC):
        '''Converts UTC time to epoch time.'''
        try:
            hours = int(UTC[:2])
            minutes = int(UTC[2:4])
            seconds = float(UTC[4:])
            
            now = datetime.utcnow()
            utc_time = now.replace(hour=hours, minute=minutes, second=int(seconds), microsecond=int((seconds % 1) * 1e6))
            
            CurrentTimeSec = int(utc_time.timestamp())
            CurrentTimeNsec = int((utc_time.timestamp() - CurrentTimeSec) * 1e9)

            return [CurrentTimeSec, CurrentTimeNsec]
        except Exception as e:
            rospy.logerr(f"Error converting UTC to epoch: {e}")
            return [0, 0]

    def convert_to_original_string(self, data):
        # Join the list elements with a comma separator to form the original string
        return ','.join(data)

    def update_ros_msg_and_publish(self, raw_gps_data_split):
        '''Update and publish the ROS message.'''
        self.msg = RTK_Customgps()
        self.msg.header = Header()
        self.msg.header.seq = self.sequence_number
        self.sequence_number += 1 
        self.msg.header.frame_id = 'GPS1_Frame'
        self.msg.header.stamp.secs = self.CurrentTimeSec
        self.msg.header.stamp.nsecs = self.CurrentTimeNsec
        self.msg.latitude = self.latitude_signed if self.latitude_signed is not None else 0.0
        self.msg.longitude = self.longitude_signed if self.longitude_signed is not None else 0.0
        self.msg.altitude = self.altitude
        self.msg.utm_easting = self.UTM_eastings
        self.msg.utm_northing = self.UTM_northings
        self.msg.zone = self.UTM_zone
        self.msg.letter = self.UTM_letter
        self.msg.hdop = self.hdop
        self.msg.fix =  self.fix
        self.msg.gngga_read = self.convert_to_original_string(raw_gps_data_split)
 
        rospy.loginfo(self.msg)
        self.publisher.publish(self.msg)
        self.rate.sleep()
    
    def get_gps_data(self):
        '''Get data from the given file and return it as a list.'''
        try:
            data = []
            with open(self.file_path, 'r', encoding='utf-8', errors='replace') as file:
                line_count = sum(1 for _ in file)

            rospy.loginfo(f"Number of lines in the file: {line_count}. This might take some time based on the number of lines.")

            with open(self.file_path, 'r', encoding='utf-8', errors='replace') as file:
                # Iterate through each line in the file and append to the list
                rospy.loginfo(f"Started reading the file.")
                for line in file:
                    read_value = line.strip()
                    split_data = read_value.split(",")
                    data.append(split_data)
            # rospy.loginfo(f"Raw GPS data: {data}")
            return data
        except Exception as error:
            rospy.logerr(f"Data parsing error: {error}")
            return[]
    
    def start_driver(self):
        '''Main loop for reading GPS data and publishing it.'''
        self.raw_data_list = self.get_gps_data()
        
        for raw_gps_data_split in self.raw_data_list:
            
            if rospy.is_shutdown():
                break  # Exit the loop if ROS is shutting down
            
            if self.isGNGGAinString(raw_gps_data_split):
                continue
            if len(raw_gps_data_split) < 15:
                rospy.logdebug("Missing a few data points, skipping this iteration")
                continue
            self.updated_values = self.update_raw_data(raw_gps_data_split)
            if None in self.updated_values or 0.0 in self.updated_values:
                rospy.logdebug("Missing a few data points, skipping this iteration")
                continue
            
            self.latitude_degrees = self.degMinstoDegDec(self.latitude, is_latitude=True)
            self.longitude_degrees = self.degMinstoDegDec(self.longitude, is_latitude=False)
            self.latitude_signed = self.LatLongSignConvetion(self.latitude_degrees, self.latitudeDir)
            self.longitude_signed = self.LatLongSignConvetion(self.longitude_degrees, self.longitudeDir)

            rospy.loginfo(f"Converted Latitude: {self.latitude_signed}, Converted Longitude: {self.longitude_signed}")
            self.UTM_eastings, self.UTM_northings, self.UTM_zone, self.UTM_letter = self.convertToUTM(self.latitude_signed, self.longitude_signed)

            rospy.loginfo(f"UTM conversion: UTM_easting: {self.UTM_eastings}, UTM_northing: {self.UTM_northings}, UTM_zone: {self.UTM_zone}, UTM_letter: {self.UTM_letter}")
            self.CurrentTimeSec, self.CurrentTimeNsec = self.UTCtoUTCEpoch(self.utc)
            self.update_ros_msg_and_publish(raw_gps_data_split)
        # Cleanly shut down the ROS node and exit the program
        rospy.loginfo("All data processed. Shutting down the GPS driver.")
        rospy.signal_shutdown("Processing completed.")
            

if __name__ == "__main__":
    try:
        rospy.init_node('gps_driver')
        # File paths to your UBX files
        current_path = os.path.join(os.getcwd(), "../data/raw_rtk_data/")
        if len(sys.argv) > 1:
            ubx_file_path = os.path.join(current_path, sys.argv[1])
            rospy.loginfo(f" Using file path: {ubx_file_path}")
        else:
            ubx_file_path = rospy.get_param('~file_path', None)  # Allow None to trigger default in the class
            rospy.loginfo(f"No file is specified, using the default file path: {ubx_file_path}")
        rospy.loginfo(f"Starting the GPS Driver with given file path: {ubx_file_path}")
        gps = GPS_DRIVER(file_path = ubx_file_path)
        gps.start_driver()
    except Exception as error:
        rospy.loginfo(f"Error: {error}")
