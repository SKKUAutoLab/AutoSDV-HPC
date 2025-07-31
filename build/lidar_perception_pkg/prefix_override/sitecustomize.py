import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sungbhin/hpc_v2/install/lidar_perception_pkg'
