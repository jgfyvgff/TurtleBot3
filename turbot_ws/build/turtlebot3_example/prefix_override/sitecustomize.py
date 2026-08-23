import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ljq/Desktop/ljq_zq/Turtlebot/TurtleBot3/turbot_ws/install/turtlebot3_example'
