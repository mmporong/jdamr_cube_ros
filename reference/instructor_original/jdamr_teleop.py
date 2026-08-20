import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control myagv!
---------------------------
Moving around:
        w     
   a    s    d
    
space key, k : stop
w           : foward
s           : backward
a           : turn left
d           : turn right
x           : stop
u           : speed up
j           : speed down
space       : stop

CTRL-C to quit
"""

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        # Publisher to the '/cmd_vel' topic
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variable to hold key settings
        self.settings = termios.tcgetattr(sys.stdin)

        # Movement variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.speed = 50.0

        # Start listening for keyboard input
        self.get_logger().info(msg)
        self.run()

    def run(self):
        try:
            while True:
                key = self.get_key()

                if key == ' ' or key == 'x':
                    self.x = 0.0
                    self.y = 0.0
                    self.theta = 0.0
                elif key == 'w':
                    self.x = 0.5
                    self.y = 0.0
                    self.theta = 0.0
                elif key == 's':
                    self.x = -0.5
                    self.y = 0.0
                    self.theta = 0.0
                elif key == 'a':
                    self.x = 0.0
                    self.y = 0.5
                    self.theta = 1.0
                elif key == 'd':
                    self.x = 0.0
                    self.y = 0.0
                    self.theta = -1.0
                elif key == '\x03':  # CTRL-C
                    break
                elif key == 'u': # arrow up
                    self.speed += 10.0
                    if self.speed > 100.0:  # actual max is 255.
                        self.speed = 100.0
                elif key == 'j': # arrow down 
                    self.speed -= 10.0
                    if self.speed < 30.0:
                        self.speed = 30.0
             
                # Publish the Twist message
                twist = Twist()
                twist.linear.x = self.x
                twist.linear.y = self.y
                twist.linear.z = self.speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.theta
                self.pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

        finally:
            # Stop the robot before exiting
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


def main(args=None):
    rclpy.init(args=args)

    # Create the TeleopNode
    teleop_node = TeleopNode()

    # Spin the node (keep it alive)
    rclpy.spin(teleop_node)

    # Shutdown
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
