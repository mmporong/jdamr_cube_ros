import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import struct

class JdamrCubeNode(Node):
    def __init__(self):
        super().__init__('jdamr_cube_node')
        
        # 1. 시리얼 설정 (8번/10번 핀 매핑된 /dev/ttyAMA0)
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
        
        # 2. Publisher: IMU 데이터 전달
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # 3. Subscriber: 상위 PC로부터 속도 명령 수신
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # 4. 수신 루프 타이머 (0.05초 간격으로 시리얼 읽기)
        self.timer = self.create_timer(0.05, self.read_serial)
        self.get_logger().info('jdamr_cube_node Started')

    def read_serial(self):
        # 버퍼에 데이터가 충분히 쌓일 때까지 기다리지 않고, 
        # 루프를 돌며 헤더를 찾습니다.
        while self.ser.in_waiting > 0:
            # 1. 헤더의 첫 번째 바이트 찾기
            if self.ser.read(1) == b'\xf5':
                # 2. 다음 바이트가 두 번째 헤더인지 확인
                if self.ser.read(1) == b'\xf5':
                    # 3. 헤더를 찾았으므로 나머지 27바이트를 읽음
                    data = self.ser.read(27)
                    if len(data) == 27:
                        self.parse_sensor_data(data)
                        # 한 번 읽었으면 루프 종료 (타이머 주기에 맞춰 다음 읽기 진행)
                        break

    def parse_sensor_data(self, data):
        # 29바이트 패킷 구조에 맞춰 파싱 (빅 엔디언)
        # buffer[2..3] 가속도 X, [4..5] 가속도 Y ...
        try:
            ax_raw = struct.unpack('>h', data[0:2])[0]
            ay_raw = struct.unpack('>h', data[2:4])[0]
            az_raw = struct.unpack('>h', data[4:6])[0]
            
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # ESP32에서 *1000 해서 보냈으므로 다시 복원
            imu_msg.linear_acceleration.x = ax_raw / 1000.0
            imu_msg.linear_acceleration.y = ay_raw / 1000.0
            imu_msg.linear_acceleration.z = az_raw / 1000.0
            
            self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f'Parsing Error: {e}')

    def cmd_vel_callback(self, msg):
        # ESP32 프로토콜: 0xf5 0xf5 0x51 [방향] [속도]
        # 방향 정의 예시: 1:전진, 2:후진, 3:좌회전, 4:우회전, 5:정지
        cmd_packet = bytearray([0xf5, 0xf5, 0x51, 5, 0])
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 1. 우선순위 결정 (회전과 직진 중 무엇을 먼저 처리할지)
        if angular_z > 0:   # 좌회전
            cmd_packet[3] = 3
            cmd_packet[4] = 150 # 회전 속도는 상수로 고정하거나 매핑
        elif angular_z < 0: # 우회전
            cmd_packet[3] = 4
            cmd_packet[4] = 150
        elif linear_x > 0:  # 전진
            cmd_packet[3] = 1
            cmd_packet[4] = min(int(abs(linear_x) * 255), 255)
        elif linear_x < 0:  # 후진
            cmd_packet[3] = 2
            cmd_packet[4] = min(int(abs(linear_x) * 255), 255)
        else:               # 정지
            cmd_packet[3] = 5
            cmd_packet[4] = 0
            
        self.ser.write(cmd_packet)
        self.get_logger().info(f'Send Cmd: Dir={cmd_packet[3]}, Spd={cmd_packet[4]}')

def main(args=None):
    rclpy.init(args=args)
    node = JdamrCubeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()