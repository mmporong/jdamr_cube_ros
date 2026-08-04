import subprocess
import time


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


for n in ('pick_object', 'pick_object_green', 'pick_object_blue', 'pick_object_orange',
          'pick_table', 'pick_blue', 'pick_red'):
    svc('/world/room/remove', 'gz.msgs.Entity', f'name: "{n}" type: MODEL')
time.sleep(1)
ped = ('<sdf version="1.6"><model name="pick_table"><static>true</static>'
       '<pose>1.3 0.0 0.0575 0 0 0</pose>'
       '<link name="link"><collision name="c"><geometry><box><size>0.07 0.07 0.115</size></box></geometry></collision>'
       '<visual name="v"><geometry><box><size>0.07 0.07 0.115</size></box></geometry>'
       '<material><ambient>0.4 0.3 0.2 1</ambient><diffuse>0.4 0.3 0.2 1</diffuse></material></visual></link></model></sdf>')
print('table:', svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + ped.replace('"', '\\"') + '"'))
time.sleep(1)
c = ('<sdf version="1.6"><model name="pick_blue">'
     '<pose>1.3 0.0 0.132 0 0 0</pose>'
     '<link name="link"><inertial><mass>0.04</mass>'
     '<inertia><ixx>4e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>4e-6</iyy><iyz>0</iyz><izz>4e-6</izz></inertia></inertial>'
     '<collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
     '<surface><friction><ode><mu>3.0</mu><mu2>3.0</mu2></ode><torsional><coefficient>1.0</coefficient><use_patch_radius>true</use_patch_radius><patch_radius>0.01</patch_radius></torsional></friction></surface></collision>'
     '<visual name="v"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
     '<material><ambient>0.1 0.2 0.9 1</ambient><diffuse>0.1 0.2 0.9 1</diffuse></material></visual></link></model></sdf>')
print('cube:', svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"'))
print('robot:', svc('/world/room/set_pose', 'gz.msgs.Pose',
                    'name: "jdamr_cube" position {x: 0.890 y: 0.005 z: 0.03} orientation {w: 1}'))
time.sleep(2)
out = subprocess.run('gz model -m pick_blue -p', shell=True, capture_output=True, text=True).stdout
print(out[-110:])
