import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jd/jdamr_cube_ws/src/jdamr_cube_ros/install/jdamr_cube_bringup'
