#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive


def callback(data):
    callback.prev_time = rospy.get_time()

callback.prev_time = 0

rospy.init_node('emergency_traxxas', anonymous=True)
callback.prev_time = rospy.get_time()
rospy.Subscriber('drivecmd', AckermannDrive, callback, queue_size=1000)
pub = rospy.Publisher('drivecmd', AckermannDrive, queue_size=100)

while not rospy.is_shutdown():
    if rospy.get_time() - callback.prev_time > 1:
        msg = AckermannDrive()
        msg.steering_angle = 0
        msg.speed = 0
        pub.publish(msg)
        print('Emergency stop!')
