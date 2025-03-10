import rclpy
from rclpy.node import Node 

from rtcm_msgs.msg import Message as RTCM
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix

from .um982 import UM982

class UM982Node(Node):
    def __init__(self):
        super().__init__('um982_node')
        self.get_logger().info('UM982 Node has been started')
        
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
        
        self.pub_timer_ = self.create_timer(
            1.0,
            self.pub_timer_callback,
        )
        
        self.um982 = UM982(data_port = "/dev/ttyUSB0", rtcm_port = "/dev/ttyUSB1")
        self.um982.open()
    
    def rtcm_callback(self, msg):
        self.um982.write_rtcm(msg.message)
        
    def pub_timer_callback(self):
        fix = self.um982.get_fix()
        if fix:
            bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = fix
            navsat = NavSatFix()
            navsat.header.stamp = self.get_clock().now().to_msg()
            navsat.header.frame_id = 'gps'
            navsat.latitude = bestpos_lat
            navsat.longitude = bestpos_lon
            navsat.altitude = bestpos_hgt
            # navsat.position_covariance = [
            #     bestpos_latstd**2, 0, 0,
            #     0, bestpos_lonstd**2, 0,
            #     0, 0, bestpos_hgtstd**2
            # ]
            self.navsat_pub_.publish(navsat)
        
        # orientation = self.um982.get_orientation()
        # if orientation:
        #     nmea = Sentence()
        #     nmea.header.stamp = self.get_clock().now().to_msg()
        #     nmea.header.frame_id = 'gps'
        #     nmea.sentence = orientation
        #     self.nmea_pub_.publish(nmea)
        
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
    