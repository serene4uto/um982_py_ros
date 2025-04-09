import rclpy
from rclpy.node import Node 
import numpy as np
from rtcm_msgs.msg import Message as RTCM
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from src.um982 import UM982

class UM982Node(Node):
    def __init__(self):
        super().__init__('um982_node')
        self.get_logger().info('UM982 Node has been started')
        
        # Declare parameters
        # Port for GNSS data
        self.declare_parameter('port.gnss', '/dev/UM982')
        # Baudrate for the GNSS port
        self.declare_parameter('port.gnss_baudrate', 115200)
        # Port for RTCM input
        self.declare_parameter('port.rtcm', '/dev/UM982-RTCM')
        # Baudrate for the RTCM port
        self.declare_parameter('port.rtcm_baudrate', 115200)
        self.declare_parameter('heading_offset', 0.0)
        # Frame ID for the published messages
        self.declare_parameter('frame_id', 'gps')
        # Publish NMEA sentences
        self.declare_parameter('publish.nmea', True)
        # Add other publish parameters as needed
        # Coordinate system for the heading [enu, ned]
        self.declare_parameter('heading_system', 'enu')
        # Enable verbose logging
        self.declare_parameter('verbose', False)
        
        # Get parameters
        self.data_port = self.get_parameter('port.gnss').value
        self.data_port_baudrate = self.get_parameter('port.gnss_baudrate').value
        self.rtcm_port = self.get_parameter('port.rtcm').value
        self.rtcm_port_baudrate = self.get_parameter('port.rtcm_baudrate').value
        self.heading_offset = self.get_parameter('heading_offset').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_nmea = self.get_parameter('publish.nmea').value
        self.heading_system = self.get_parameter('heading_system').value
        self.verbose = self.get_parameter('verbose').value
        
        # Create publishers and subscribers
        self.rtcm_sub_ = self.create_subscription(
            RTCM,
            'rtcm',
            self.rtcm_callback,
            10
        )
        
        self.nmea_pub_ = self.create_publisher(Sentence, 'nmea', 10)
        self.navsat_pub_ = self.create_publisher(NavSatFix, 'fix', 10)
        self.heading_pub_ = self.create_publisher(Imu, 'heading', 10)
    
        # Timer for publishing data (adjusted from 0.5s to 0.1s for more responsive updates)
        self.pub_timer_ = self.create_timer(0.1, self.pub_timer_callback)
        
        # Initialize UM982 module
        self.um982 = UM982(
            data_port=self.data_port,
            data_port_baudrate=self.data_port_baudrate,
            rtcm_port=self.rtcm_port,
            rtcm_port_baudrate=self.rtcm_port_baudrate,
            heading_system=self.heading_system,
        )
        self.um982.open()
        
        # Cache for reused objects
        self._imu_msg = Imu()
        self._navsat_msg = NavSatFix()
        self._nmea_msg = Sentence()
        
    def rtcm_callback(self, msg):
        self.um982.write_rtcm(msg.message)
        
    def pub_timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        
        # Process position data
        bestpos = self.um982.get_bestpos()
        if bestpos:
            bestpos_type, bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = bestpos
            if self.verbose:
                self.get_logger().info(f'Bestpos: {bestpos_type}, {bestpos_hgt}, {bestpos_lat}, {bestpos_lon}')
            # Reuse NavSatFix message object
            self._navsat_msg.header.stamp = current_time
            self._navsat_msg.header.frame_id = self.frame_id
            self._navsat_msg.latitude = bestpos_lat
            self._navsat_msg.longitude = bestpos_lon
            self._navsat_msg.altitude = bestpos_hgt
            self._navsat_msg.position_covariance = [
                float(bestpos_latstd**2), 0.0, 0.0,
                0.0, float(bestpos_lonstd**2), 0.0,
                0.0, 0.0, float(bestpos_hgtstd**2)
            ]
            
            # Set status based on solution type
            if bestpos_type == 'SINGLE':
                self._navsat_msg.status.status = 0
            elif bestpos_type == 'PSRDIFF':
                self._navsat_msg.status.status = 1
            elif bestpos_type in ('NARROW_FLOAT', 'NARROW_INT'):  # Fixed bug in type checking
                self._navsat_msg.status.status = 2  # RTK Fixed
            
            self.navsat_pub_.publish(self._navsat_msg)
        
        # Process heading data
        heading = self.um982.get_heading()
        if heading:
            heading_type, heading_len, heading_deg, heading_pitch = heading
            if self.verbose:
                self.get_logger().info(f'Heading: {heading_type}, {heading_len}, {heading_deg}, {heading_pitch}')
            # Calculate quaternion from Euler angles
            # Only yaw is provided by the sensor, pitch and roll are set to 0
            yaw = np.deg2rad(heading_deg) + self.heading_offset
            
            # Optimized quaternion computation (half-angle)
            cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
            
            # For zero pitch and roll, simplified quaternion
            self._imu_msg.orientation.w = cy
            self._imu_msg.orientation.x = 0.0
            self._imu_msg.orientation.y = 0.0
            self._imu_msg.orientation.z = sy
            
            self._imu_msg.header.stamp = current_time
            self._imu_msg.header.frame_id = self.frame_id
            self._imu_msg.orientation_covariance = [
                0.0479, 0.0, 0.0,
                0.0, 0.020, 0.0,
                0.0, 0.0, 0.0041
            ]
            
            self.heading_pub_.publish(self._imu_msg)
        
        # Process NMEA data
        nmea = self.um982.get_nmea()
        if nmea and self.publish_nmea:
            # if self.verbose:
            #     self.get_logger().info(f'NMEA: {nmea}')
            self._nmea_msg.header.stamp = current_time
            self._nmea_msg.header.frame_id = 'nmea'
            self._nmea_msg.sentence = nmea
            self.nmea_pub_.publish(self._nmea_msg)

    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.destroy_node()
        if hasattr(self, 'um982'):
            self.um982.close()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        with UM982Node() as node:
            rclpy.spin(node)
    finally:
        rclpy.shutdown()