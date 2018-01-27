#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive

currentState = 0
lastSteering = 0
lastV = 0


# state -1 corresponds to the backward motion
# state 0 corresponds to the break
# state 1 corresponds to the forward motion

def stateUpdate(v, w):
    # (1, v+) -> (1)
    # (1, v-) -> (0)
    # (1, 0) -> (0)
    # (0, v+) -> (1)
    # (0, v-) -> (-1)
    # (0, 0) -> (0)
    # (-1, v+) -> (1)
    # (-1, v-) -> (-1)
    # (-1, 0) -> (0)

    global currentState
    global lastSteering
    global lastV

    delay = 0.25

    steering = 0
    power = 0

    if (v > 0):
        # (1, v+) -> (1)
        # (0, v+) -> (1)
        # (-1, v+) -> (1)

        if (currentState == 1 or currentState == 0 or currentState == -1):
            currentState = 1
            steering = (-1) * convert_trans_rot_vel_to_steering_angle(v, w, wheelbase)
            power = convert_speed_to_persantage(v)

    if (v < 0):
        # (1, v-) -> (0)
        # (0, v-) -> (-1)
        # (-1, v-) -> (-1)

        if (currentState == 1):
            currentState = 0

            steering = lastSteering
            power = -0.3
            # tsleep = lastV * 4
            publishAckermannMsg(power, steering)
            rospy.sleep(delay)
            publishAckermannMsg(0, steering)
            rospy.sleep(delay)

        if (currentState == 0 or currentState == -1):
            currentState = -1

            power = convert_speed_to_persantage(v)
            steering = convert_trans_rot_vel_to_steering_angle(v, w, wheelbase)

    if (v == 0):
        # (1, 0) -> (0)
        # (0, 0) -> (0)
        # (-1, 0) -> (0)
        previousState = currentState

        if (previousState == 1):
            power = -0.9

            publishAckermannMsg(power, steering)
            rospy.sleep(delay)

        currentState = 0
        power = 0
        steering = lastSteering

    lastSteering = steering
    lastV = v

    publishAckermannMsg(power, steering)


def publishAckermannMsg(power, steering):
    global currentState

    rospy.loginfo("Info: power: %s, steering: %s, State: %s", power, steering, currentState)

    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.drive.steering_angle = steering
    msg.drive.speed = power
    pub.publish(msg.drive)


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def convert_speed_to_persantage(speed):
    # At this point we don have feedback from sensors
    # We shoould to find some constants to compensate resistatnce of the system

    global currentState

    forwardVelocityOffset = 0.13
    backwarddVelocityOffset = -0.33

    if (currentState == 1):
        return 0.064552 * speed + forwardVelocityOffset

    if (currentState == -1):
        return speed * 0.3 + backwarddVelocityOffset

    return 0


def cmd_callback(data):
    global wheelbase
    global ackermann_cmd_topic
    global frame_id
    global pub

    v = data.linear.x
    w = data.angular.z

    stateUpdate(v, w)


if __name__ == '__main__':
    try:

        rospy.init_node('cmd_vel_to_ackermann_drive')

        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
        ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/drivecmd')
        wheelbase = rospy.get_param('~wheelbase', 0.4)
        frame_id = rospy.get_param('~frame_id', 'base_link')

        rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
        pub = rospy.Publisher(ackermann_cmd_topic, AckermannDrive, queue_size=1)

        rospy.loginfo(
            "Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f",
            "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
