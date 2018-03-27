#!/usr/bin/env python
import rospy, math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive

flag_move = 0

def get_r(alpha):
    L = 0.325 #wheelBase
    r = L/math.tan(alpha)
    return r



def set_throttle_steer(data):

    global flag_move

    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    #

    throttle = data.speed/0.01
    steer = (-1) * data.steering_angle * 0.145 * math.pi

    if (abs(steer) > 0.145 * math.pi):
        steet= steer/abs(steer) * 0.145 * math.pi

    if (steer != 0) :
        L = 0.325
        T = 0.2
        r = get_r(steer)

        steerInner = math.atan(L / (r - T/2) )
        steerOuter = math.atan(L / (r + T/2) )
    else:
        steerInner = steer
        steerOuter = steer

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steerInner)
    pub_pos_right_steering_hinge.publish(steerOuter)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("/drivecmd", AckermannDrive, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
