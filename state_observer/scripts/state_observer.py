#!/usr/bin/env python

import rospy, math

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

poseMsgBefore=PoseWithCovarianceStamped()
getFirst=False


def parsePoseMsg(pose):

    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)

    (roll, pitch, yaw) = euler_from_quaternion(quaternion)

    return (pose.position.x, pose.position.y, yaw)

def getVelocities(x1, y1, yaw1, t1, x2, y2, yaw2, t2):

    #coordinates X2 in coordinate frame X1
    yaw = - (yaw2-yaw1)

    x = math.cos(yaw)*(x2-x1) - math.sin(yaw)*(y2-y1)
    y = math.sin(yaw)*(x2-x1) + math.cos(yaw)*(y2-y1)

    vx= x/(t2-t1)
    vy=y/(t2-t1)
    w=diff(yaw1,yaw2,t1,t2)

    return (vx,vy,w)

def diff(x1,x2,t1,t2):
    # x`= (x(i+1)- x(i))/ (t(i+1)-t(i))

    dx= (x2-x1)/(t2-t1)

    if (abs(dx)>3):
        dx=dx/abs(dx)*3

    return dx

def getTimeFromHeaderMsg(header):
    stamp = header.stamp
    time= stamp.to_sec()

    return time



def processPoseWithCovarianceStamped(poseMsg):
    global pubVelocity
    global getFirst
    global poseMsgBefore

    if (getFirst==False):
        getFirst = True
        return

    seq = poseMsg.header.seq
    stamp = poseMsg.header.stamp
    frame_id = poseMsg.header.frame_id

    t1=getTimeFromHeaderMsg(poseMsgBefore.header)
    t2 = getTimeFromHeaderMsg(poseMsg.header)
    (x1, y1, yaw1) = parsePoseMsg(poseMsgBefore.pose.pose)
    (x2, y2, yaw2) = parsePoseMsg(poseMsg.pose.pose)

    (vx,vy,w) = getVelocities(x1, y1, yaw1, t1, x2, y2, yaw2, t2)

    #rospy.loginfo("Info: x1: %s, y1: %s,  yaw1: %s, t1= %s", x1, y1, yaw1, t1)
    #rospy.loginfo("Info: x2: %s, y2: %s, yaw2: %s, t2= %s", x2, y2, yaw2, t2)
    #rospy.loginfo("Info: vx: %s, vy: %s,w: %s, t= %s", vx, vy, w, stamp.to_sec())

    msg = TwistStamped()
    msg.header.seq = seq
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.twist.linear.x=vx
    msg.twist.linear.x = vy
    msg.twist.angular.z = w

    pubVelocity.publish(msg)

    #save last state of system for computing speed
    poseMsgBefore = poseMsg



if __name__ == '__main__':
    try:

        rospy.init_node('state_observer')

        poseUpdateTopic = rospy.get_param('~poseUpdateTopic', '/poseupdate')
        velocityTopic = rospy.get_param('~poseTopic', '/state_observer/velocity')


        rospy.Subscriber(poseUpdateTopic, PoseWithCovarianceStamped, processPoseWithCovarianceStamped, queue_size=1)
        pubVelocity = rospy.Publisher(velocityTopic, TwistStamped, queue_size=1)

        rospy.loginfo("state_observer")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass