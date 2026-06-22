import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sorbonne/ros2_jazzy/src/monPaquetCoVAPSYR/install/monPaquetCoVAPSYR'
