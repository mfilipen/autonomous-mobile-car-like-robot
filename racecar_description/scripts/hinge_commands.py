#!/usr/bin/env python
import rospy, math

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import geometry_msgs
from std_msgs.msg import String

import numpy as np

import tf

flag_move = 0
steerInner = 0
steerOuter = 0


def get_r(alpha):
    L = 0.325 #wheelBase
    r = L/math.tan(alpha)
    return r



def set_throttle_steer(data):
    global bf
    global flag_move
    global steerInner
    global steerOuter

    steer =  data.steering_angle * 0.145 * math.pi

    if (abs(steer) > 0.145 * math.pi):
        steet= steer/abs(steer) * 0.145 * math.pi

    if (steer != 0) :
        L = 0.325
        T = 0.2
        r = get_r(steer)

        steerInner =  math.atan(L / (r - T/2) )
        steerOuter =  math.atan(L / (r + T/2) )
    else:
        steerInner = steer
        steerOuter = steer



    #rospy.loginfo("SteerInner: %s , steerOuter: %s", steerInner, steerOuter)

if __name__ == '__main__':

    br = tf.TransformBroadcaster()

    rospy.init_node('hinge_commands', anonymous=True)
    rospy.Subscriber("/drivecmd", AckermannDrive, set_throttle_steer)

    rate = rospy.Rate(500)

    shift = np.pi / 2

    while True:
        #rospy.loginfo("i am live")


        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(steerOuter + shift, 0, 0 ),
                         rospy.Time.now(),
                         "left_front_wheel",
                         "left_steering_hinge")

        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(steerInner + shift,0, 0 ),
                         rospy.Time.now(),
                         "right_front_wheel",
                         "right_steering_hinge")
        rate.sleep()

    rospy.spin()



