import rclpy
from rclpy.node import Node 

import numpy as np

from rtcm_msgs.msg import Message as RTCM
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

from .um982 import UM982

class UM982Node(Node):
    def __init__(self):
        super().__init__('um982_node')
        self.get_logger().info('UM982 Node has been started')
        
        # Declare parameters
        self.declare_parameter('port.gnss', '/dev/UM982')
        self.declare_parameter('port.gnss_baudrate', 115200)
        self.declare_parameter('port.rtcm', '/dev/UM982-RTCM')
        self.declare_parameter('port.rtcm_baudrate', 115200)
        self.declare_parameter('heading_offset', 0.0)
        
        # Get parameters
        self.data_port = self.get_parameter('port.gnss').value
        self.data_port_baudrate = self.get_parameter('port.gnss_baudrate').value
        self.rtcm_port = self.get_parameter('port.rtcm').value
        self.rtcm_port_baudrate = self.get_parameter('port.rtcm_baudrate').value
        self.heading_offset = self.get_parameter('heading_offset').value
        
        
        # Create publishers and subscribers
        self.rtcm_sub_ = self.create_subscription(
            RTCM,
            'rtcm',
            self.rtcm_callback,
            10
        )
        
        self.nmea_pub_ = self.create_publisher(
            Sentence,
            'nmea',
            10
        )
        
        self.navsat_pub_ = self.create_publisher(
            NavSatFix,
            'fix',
            10
        )
        
        self.heading_pub_ = self.create_publisher(
            Imu,
            'heading',
            10
        )
    
        self.pub_timer_ = self.create_timer(
            0.5,
            self.pub_timer_callback,
        )
        
        self.um982 = UM982(
            data_port = self.data_port,
            data_port_baudrate= self.data_port_baudrate,
            rtcm_port = self.rtcm_port,
            rtcm_port_baudrate= self.rtcm_port_baudrate
        )
        self.um982.open()
    
    def rtcm_callback(self, msg):
        self.um982.write_rtcm(msg.message)
        
    def pub_timer_callback(self):
        
        bestpos = self.um982.get_bestpos()
        if bestpos:
            bestpos_type, bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = bestpos
            navsat = NavSatFix()
            navsat.header.stamp = self.get_clock().now().to_msg()
            navsat.header.frame_id = 'gps'
            navsat.latitude = bestpos_lat
            navsat.longitude = bestpos_lon
            navsat.altitude = bestpos_hgt
            navsat.position_covariance = [
                float(bestpos_latstd**2), 0.0, 0.0,
                0.0, float(bestpos_lonstd**2), 0.0,
                0.0, 0.0, float(bestpos_hgtstd**2)
            ]
            
            if bestpos_type == 'SINGLE':
                navsat.status.status = 0
            elif bestpos_type == 'PSRDIFF':
                navsat.status.status = 1
            elif bestpos_type == 'NARROW_FLOAT' or 'NARROW_INT':
                navsat.status.status = 2 # RTK Fixed
            
            self.navsat_pub_.publish(navsat)
        
        heading = self.um982.get_heading()
        if heading:
            heading_type, heading_len, heading_deg, heading_pitch = heading
            imu = Imu()
            
            yaw  = np.deg2rad(heading_deg) + self.heading_offset
            pitch = 0
            roll = 0
            
            # Compute quaternion components
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)

            q_w = cr * cp * cy + sr * sp * sy
            q_x = sr * cp * cy - cr * sp * sy
            q_y = cr * sp * cy + sr * cp * sy
            q_z = cr * cp * sy - sr * sp * cy
            
            # Fill IMU message
            imu_msg = Imu()
            imu_msg.orientation.x = q_x
            imu_msg.orientation.y = q_y
            imu_msg.orientation.z = q_z
            imu_msg.orientation.w = q_w

            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            
            self.heading_pub_.publish(imu_msg)
        
        nmea = self.um982.get_nmea()
        if nmea:
            nmea_msg = Sentence()
            nmea_msg.header.stamp = self.get_clock().now().to_msg()
            nmea_msg.header.frame_id = 'nmea'
            nmea_msg.sentence = nmea
            self.nmea_pub_.publish(nmea_msg)
        

def main(args=None):
    rclpy.init(args=args)
    um982_node = UM982Node()
    rclpy.spin(um982_node)
    um982_node.um982.close()
    rclpy.shutdown()
    