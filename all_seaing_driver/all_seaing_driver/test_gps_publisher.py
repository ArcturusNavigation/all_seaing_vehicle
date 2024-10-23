#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import GPSRAW  # Import the GPSRAW message type
import csv  # Assuming the file is in CSV format
import os

class GPSFilePublisher(Node):
    def __init__(self):
        super().__init__('gps_file_publisher')
        self.publisher_ = self.create_publisher(GPSRAW, 'test_topic', 10)

        # Timer to periodically publish messages
        self.timer = self.create_timer(1.0, self.publish_from_file)  # Publish every second
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.file_path = os.path.join(script_dir, 'test_gps.csv')
        self.data = self.read_file(self.file_path)
        self.counter = 0  # To keep track of which row to publish

    def read_file(self, file_path):
        # Read the CSV file containing GPSRAW data
        with open(file_path, 'r') as file:
            reader = csv.DictReader(file)
            data = [row for row in reader]
        return data

    def publish_from_file(self):
        if self.counter >= len(self.data):
            self.get_logger().info('Reached end of file')
            return

        row = self.data[self.counter]
        gpsraw_msg = GPSRAW()

        # Convert lat/lon from microdegrees to degrees, alt from mm to meters
        gpsraw_msg.lat = int(row['lat'])  # Convert from microdegrees to degrees
        gpsraw_msg.lon = int(row['lon'])  # Convert from microdegrees to degrees
        gpsraw_msg.alt = int(row['alt'])  # Convert from millimeters to meters
        
        # Fill in other GPSRAW fields
        gpsraw_msg.fix_type = int(row['fix_type'])
        gpsraw_msg.eph = int(row['eph'])
        gpsraw_msg.epv = int(row['epv'])
        gpsraw_msg.vel = int(row['vel'])
        gpsraw_msg.satellites_visible = int(row['satellites_visible'])

        self.publisher_.publish(gpsraw_msg)
        self.get_logger().info(f'Published GPSRAW message: {gpsraw_msg}')

        self.counter += 1  # Move to the next row

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSFilePublisher()

    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass
    
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()