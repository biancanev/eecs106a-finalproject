import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/rkwon/Documents/106/eecs106a-finalproject/install/turtlebot_interceptor'
