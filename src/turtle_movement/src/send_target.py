import os
import sys

try:
    x = float(sys.argv[1])
    y = 0.
    z = float(sys.argv[2])
except:
    print("Usage: send_target.py x y")
    exit(0)

os.system('rostopic pub /target geometry_msgs/Point "x: %f\ny: %f\nz: %f\n"' % (x, y, z))

