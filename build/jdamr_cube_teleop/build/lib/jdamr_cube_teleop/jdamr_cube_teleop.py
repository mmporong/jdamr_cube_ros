import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
JDAMR Control
---------------------------
Moving around:
        w
   a    s    d

Up/Down Arrow : Increase/Decrease Speed
Space Bar     : Emergency Stop
CTRL-C to quit
"""

class JdamrTeleop(Node):
    def __init__(self):
        super().__init__('jdamr_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.5  # 초기 속도 (0.0 ~ 1.0 비율)
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info(f"Teleop Node Started. Current Speed: {self.speed}")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1)
        if key == '\x1b': # 화살표 키 처리를 위한 이스케이프 시퀀스
            key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                twist = Twist()
                changed = False

                if key == 'w':
                    twist.linear.x = self.speed
                    changed = True
                elif key == 's':
                    twist.linear.x = -self.speed
                    changed = True
                elif key == 'a':
                    twist.angular.z = 1.0 # 좌회전 신호
                    changed = True
                elif key == 'd':
                    twist.angular.z = -1.0 # 우회전 신호
                    changed = True
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    changed = True
                elif key == '\x1b[A': # Up Arrow
                    self.speed = min(self.speed + 0.05, 1.0)
                    print(f"Speed Up: {self.speed:.2f}")
                elif key == '\x1b[B': # Down Arrow
                    self.speed = max(self.speed - 0.05, 0.05)
                    print(f"Speed Down: {self.speed:.2f}")
                elif key == '\x03': # CTRL-C
                    break

                if changed:
                    self.publisher_.publish(twist)

        except Exception as e:
            print(e)
        finally:
            twist = Twist()
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = JdamrTeleop()
    teleop.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()