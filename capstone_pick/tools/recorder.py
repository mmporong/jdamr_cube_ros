"""삼각대 카메라 스폰 + 프레임 녹화 (GIF용)."""
import os
import subprocess
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

OUT = '/tmp/frames'
os.makedirs(OUT, exist_ok=True)
for f in os.listdir(OUT):
    os.remove(os.path.join(OUT, f))


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


svc('/world/room/remove', 'gz.msgs.Entity', 'name: "tripod" type: MODEL')
time.sleep(0.5)
# 쓰레기통 데모용: (1.35,0.55,0.60)에서 로봇 진행선(0.33, 0.2~1.0)을 옆에서 바라봄
cam = ('<sdf version="1.6"><model name="tripod"><static>true</static>'
       '<pose>1.35 0.55 0.60 0 0.32 2.98</pose>'
       '<link name="link">'
       '<sensor name="cam" type="camera"><always_on>1</always_on><update_rate>10</update_rate>'
       '<topic>tripod/image</topic>'
       '<camera><horizontal_fov>1.1</horizontal_fov>'
       '<image><width>640</width><height>480</height></image>'
       '<clip><near>0.05</near><far>10</far></clip></camera></sensor>'
       '</link></model></sdf>')
print('tripod:', svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + cam.replace('"', '\\"') + '"'), flush=True)
time.sleep(1)

bridge_proc = subprocess.Popen('ros2 run ros_gz_image image_bridge /tripod/image > /tmp/tripod_bridge.log 2>&1',
                               shell=True, executable='/bin/bash')
time.sleep(3)


class Rec(Node):
    def __init__(self):
        super().__init__('gif_recorder')
        self.bridge = CvBridge()
        self.n = 0
        self.last = 0.0
        self.create_subscription(Image, '/tripod/image', self.cb, 3)

    def cb(self, msg):
        now = time.time()
        if now - self.last < 0.30:
            return
        self.last = now
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(f'{OUT}/f_{self.n:04d}.png', img)
        self.n += 1


rclpy.init()
r = Rec()
t0 = time.time()
DURATION = float(os.environ.get('REC_SEC', '75'))
while time.time() - t0 < DURATION:
    rclpy.spin_once(r, timeout_sec=0.2)
print(f'녹화 종료: {r.n} 프레임', flush=True)
r.destroy_node()
rclpy.shutdown()
bridge_proc.terminate()
