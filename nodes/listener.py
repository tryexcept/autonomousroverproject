#!/usr/bin/env python
import rospy
import roslib
from sensor_msgs.msg import Range
from std_msgs.msg import Empty

def cb(pub, data):
    rospy.loginfo('{}: {}'.format(rospy.get_name(), data.range))
    if data.range < 16:
	pub.publish(Empty())

def listener():
    pub = rospy.Publisher('/beep', Empty)
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/ultrasound", Range, lambda x: cb(pub, x))
    rospy.spin()

if __name__ == '__main__':
    roslib.load_manifest('arp')
    try:
        listener()
    except rospy.ROSInterruptException:
	pass
