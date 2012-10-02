#!/usr/bin/env python

import roslib; roslib.load_manifest('arp')
import rospy

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import LaserScan

class convert:

    def __init__(self):
        rospy.init_node("arp_scan")
        rospy.Subscriber("arp_base_scan", Int16MutliArray, self.Cb)
        self.scanPub = rospy.Publisher('scan', LaserScan)
        rospy.spin()

    def Cb(self, msg):
        n_ranges = list()
        pv = msg.data[0]
        for i in range(len(msg.data_length)):
            if pv>0.0 and msg.data[i]>0.0:
                n_ranges.append((pv+msg.data[i])/2)
            else:
                n_ranges.append(0.0)
            n_ranges.append(msg.data[i])
            pv = msg.data[i]

        laserMsg = LaserScan()
        laserMsg.ranges = n_ranges
        laserMsg.angle_increment = 0.2; 
        laserMsg.time_increment = 1/50
        self.scanPub.publish(msg)

if __name__ == "__main__":
    a = convert()

