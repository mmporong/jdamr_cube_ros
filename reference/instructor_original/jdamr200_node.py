import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import math 
import serial 
import time 

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from jdamr200_lib import Jdamr200 

class JdamrControlNode(Node):

    def __init__(self):
        super().__init__('jdamr_control_node')
        # jsamr200 
        self.robot = Jdamr200("/dev/ttyUSB0")
        
        self.publisher_ = self.create_publisher(String, 'jdamr_control', 10)
         # Odometry 메시지를 퍼블리싱하는 퍼블리셔 생성
        self.pub_odom = self.create_publisher(Odometry, 'odom', 50)
        
        # Voltage 메시지를 퍼블리싱하는 퍼블리셔 생성
        self.pub_v = self.create_publisher(Int8, 'Voltage', 1000)
         # Create subscriber for '/cmd_vel' topic
        self.subscriber_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        odom_timer_period = 0.05  # seconds
        self.odom_timer = self.create_timer(odom_timer_period, self.odom_timer_callback)
        volt_timer_period = 1
        self.volt_timer = self.create_timer(volt_timer_period, self.volt_timer_callback)

    def odom_timer_callback(self):
        msg = Odometry()
        msg.header.frame_id = 'odom'
        # Reading jdamr200 sensor value 
        self.robot.readSpeed()
        # calculate odmetry 
        result = self.robot.update_odometry()
        # get odmetry data 
        msg.pose.pose.position.x = result[0]
        msg.pose.pose.position.y = result[1]
        msg.pose.pose.position.z = result[2]
        self.pub_odom.publish(msg)
        #self.get_logger().info('Publishing Odometry data')

    def volt_timer_callback(self):
        msg = Int8()
        # 메시지 데이터를 여기서 설정
        msg.data = 100  # 예시로 100을 설정
        self.pub_v.publish(msg)
        #self.get_logger().info('Publishing Voltage data')

    def cmd_vel_callback(self, msg):
        print("cmd_vel")
        go_back = msg.linear.x
        rotate = msg.angular.z
        speed = int(msg.linear.z)
        if go_back > 0:
            self.robot.move_run_mode(self.robot.GO_FORWARD, speed)
            print('go forward', go_back)
        elif go_back < 0:
            self.robot.move_run_mode(self.robot.GO_BACKWARD, speed)
            print('go_backward', go_back)
        elif rotate > 0:
            self.robot.move_run_mode(self.robot.TURN_LEFT, speed)
            print('left', rotate)
        elif rotate < 0:
            self.robot.move_run_mode(self.robot.TURN_RIGHT, speed)
            print('right', rotate)
        else:
            self.robot.move_run_mode(self.robot.STOP, 0)
            print('stop')
        # Perform actions based on the received velocities (e.g., control a robot)
        #print(f"Received linear velocity: {go_back}, angular velocity: {rotate}")

    def main_loop(self):
        while rclpy.ok():
            #self.get_logger().info('Running myAGV')
            rclpy.spin_once(self)
           
def main(args=None):
    rclpy.init(args=args)

    jdamr_node = JdamrControlNode()
    jdamr_node.main_loop()
    jdamr_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
