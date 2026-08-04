"""3색 무대: 파랑(받침대) + 빨강·초록(바닥) — 색을 골라 집는 데모용.

사용: python3 tricolor_stage.py [robot_x]  (기본 0.3)
"""
import subprocess
import sys
import time

ROBOT_X = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


def cube(name, rgba, pose):
    return ('<sdf version="1.6"><model name="' + name + '">'
            '<pose>' + pose + '</pose>'
            '<link name="link"><inertial><mass>0.04</mass>'
            '<inertia><ixx>4e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>4e-6</iyy><iyz>0</iyz><izz>4e-6</izz></inertia></inertial>'
            '<collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
            '<surface><friction><ode><mu>3.0</mu><mu2>3.0</mu2></ode><torsional><coefficient>1.0</coefficient><use_patch_radius>true</use_patch_radius><patch_radius>0.01</patch_radius></torsional></friction></surface></collision>'
            '<visual name="v"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
            '<material><ambient>' + rgba + '</ambient><diffuse>' + rgba + '</diffuse></material></visual></link></model></sdf>')


for n in ('pick_object', 'pick_object_green', 'pick_object_blue', 'pick_object_orange',
          'pick_table', 'pick_blue', 'pick_red', 'pick_green'):
    svc('/world/room/remove', 'gz.msgs.Entity', f'name: "{n}" type: MODEL')
time.sleep(1)
ped = ('<sdf version="1.6"><model name="pick_table"><static>true</static>'
       '<pose>1.3 -0.35 0.0575 0 0 0</pose>'
       '<link name="link"><collision name="c"><geometry><box><size>0.07 0.07 0.115</size></box></geometry></collision>'
       '<visual name="v"><geometry><box><size>0.07 0.07 0.115</size></box></geometry>'
       '<material><ambient>0.4 0.3 0.2 1</ambient><diffuse>0.4 0.3 0.2 1</diffuse></material></visual></link></model></sdf>')
print('table:', svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + ped.replace('"', '\\"') + '"'))
time.sleep(1)
print('blue(받침대):', svc('/world/room/create', 'gz.msgs.EntityFactory',
                        'sdf: "' + cube('pick_blue', '0.1 0.2 0.9 1', '1.3 -0.35 0.132 0 0 0').replace('"', '\\"') + '"'))
print('red(바닥):', svc('/world/room/create', 'gz.msgs.EntityFactory',
                      'sdf: "' + cube('pick_red', '0.9 0.1 0.1 1', '1.15 -0.55 0.015 0 0 0').replace('"', '\\"') + '"'))
print('green(바닥):', svc('/world/room/create', 'gz.msgs.EntityFactory',
                        'sdf: "' + cube('pick_green', '0.1 0.8 0.1 1', '1.05 0.05 0.015 0 0 0').replace('"', '\\"') + '"'))
print('robot:', svc('/world/room/set_pose', 'gz.msgs.Pose',
                    f'name: "jdamr_cube" position {{x: {ROBOT_X} y: 0 z: 0.03}} orientation {{w: 1}}'))
time.sleep(2)
print('배치 완료: 파랑=받침대(1.3,-0.35) 빨강=바닥(1.15,-0.55) 초록=바닥(1.05,0.05)')
