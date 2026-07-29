"""바닥 픽 무대: 받침대 없이 파란 큐브를 바닥에 놓는다.

사용: python3 floor_stage.py [robot_x]  (기본 0.3 = 그립 단독 위치, E2E는 -0.3 권장)
큐브는 항상 (0.661, 0) — robot_x 0.3일 때 바닥 포켓(0.361)에 정확히 온다.
"""
import subprocess
import sys
import time

ROBOT_X = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


for n in ('pick_object', 'pick_object_green', 'pick_object_blue', 'pick_object_orange',
          'pick_table', 'pick_blue', 'pick_red'):
    svc('/world/room/remove', 'gz.msgs.Entity', f'name: "{n}" type: MODEL')
time.sleep(1)
c = ('<sdf version="1.6"><model name="pick_blue">'
     '<pose>0.661 0.0 0.015 0 0 0</pose>'
     '<link name="link"><inertial><mass>0.04</mass>'
     '<inertia><ixx>4e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>4e-6</iyy><iyz>0</iyz><izz>4e-6</izz></inertia></inertial>'
     '<collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
     '<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface></collision>'
     '<visual name="v"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
     '<material><ambient>0.1 0.2 0.9 1</ambient><diffuse>0.1 0.2 0.9 1</diffuse></material></visual></link></model></sdf>')
print('cube:', svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"'))
print('robot:', svc('/world/room/set_pose', 'gz.msgs.Pose',
                    f'name: "jdamr_cube" position {{x: {ROBOT_X} y: 0 z: 0.03}} orientation {{w: 1}}'))
time.sleep(2)
out = subprocess.run('gz model -m pick_blue -p', shell=True, capture_output=True, text=True).stdout
print(out[-110:])
