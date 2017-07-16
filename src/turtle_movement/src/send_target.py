#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from geometry_msgs.msg import Point

def talker():
    pointPublisher = rospy.Publisher('target', Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    message = Point()
    try:
        message.x = float(sys.argv[1])
        message.z = float(sys.argv[2])
    except:
        print "Usage: send_target.py x y"
        exit(1)
    pointPublisher.publish(message)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

