import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sungbhin/AutoSDV_HPC/install/serial_communication_pkg'
