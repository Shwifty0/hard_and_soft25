import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shwifty/SOSE25/hard_soft/robocar_ws/src/install/robocar_simulation'
