import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/beto/broverette_nav2_ws/install/broverette_nav2_bringup'
