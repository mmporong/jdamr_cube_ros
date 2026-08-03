"""쓰레기통 투입 데모 무대.

큐브를 로봇과 쓰레기통 사이 직선상에 둔다 — 집은 뒤 회전 없이 직진만으로 운반되므로
운반 중 낙하(회전 관성이 주원인)를 피할 수 있다.
로봇 (0.3, 0.05) → 큐브 (0.33, 0.60) → 통 (0.34, 1.0), 모두 +y 방향 일직선.
"""
import math
import subprocess
import time


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


for n in ('pick_object', 'pick_object_green', 'pick_object_blue', 'pick_object_orange',
          'pick_table', 'pick_blue', 'pick_red', 'pick_green'):
    svc('/world/room/remove', 'gz.msgs.Entity', f'name: "{n}" type: MODEL')
time.sleep(1)

c = ('<sdf version="1.6"><model name="pick_blue">'
     '<pose>0.33 0.60 0.015 0 0 0</pose>'
     '<link name="link"><inertial><mass>0.04</mass>'
     '<inertia><ixx>4e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>4e-6</iyy><iyz>0</iyz><izz>4e-6</izz></inertia></inertial>'
     '<collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
     '<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface></collision>'
     '<visual name="v"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
     '<material><ambient>0.1 0.2 0.9 1</ambient><diffuse>0.1 0.2 0.9 1</diffuse></material></visual></link></model></sdf>')
print('cube:', svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"'))

# 통 위치도 무대가 책임진다 — 다른 시험이 통을 옮겨 두면 "일직선" 전제가
# 조용히 깨진다(실측: 통이 측후방에 남은 채 일직선 시험이 돌아 회전 운반이 됐다)
print('trash:', svc('/world/room/set_pose', 'gz.msgs.Pose',
                    'name: "trash_can" position {x: 0.34 y: 1.0 z: 0.0} orientation {w: 1}'))

# 로봇을 +y(통 방향)로 향하게 두고, 큐브 앞 파지 포켓(0.381) 거리에 배치
yaw = math.pi / 2
print('robot:', svc('/world/room/set_pose', 'gz.msgs.Pose',
                    f'name: "jdamr_cube" position {{x: 0.33 y: 0.219 z: 0.03}} '
                    f'orientation {{w: {math.cos(yaw / 2)} z: {math.sin(yaw / 2)}}}'))
time.sleep(2)
out = subprocess.run('gz model -m pick_blue -p', shell=True, capture_output=True, text=True).stdout
print(out[-110:])
print('배치: 로봇(0.33,0.219,+y향) → 큐브(0.33,0.60) → 통(0.34,1.0) 일직선')
