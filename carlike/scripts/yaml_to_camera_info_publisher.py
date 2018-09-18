#!/usr/bin/env python

"""
pointgrey_camera_driver (at least the version installed with apt-get) doesn't
properly handle camera info in indigo.
This node is a work-around that will read in a camera calibration .yaml
file (as created by the cameracalibrator.py in the camera_calibration pkg),
convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.

The yaml parsing is courtesy ROS-user Stephan:
    http://answers.ros.org/question/33929/camera-calibration-parser-in-python/

This file just extends that parser into a rosnode.
"""
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import rospkg

def yaml_to_CameraInfo(yaml_fname, header):
    """
    Parse a yaml file containing camera calibration data (as produced by
    rosrun camera_calibration cameracalibrator.py) into a
    sensor_msgs/CameraInfo msg.

    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data

    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header = header
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg


def pub_camera_info(im):
    global publisher
    global filename

    header = im.header
    camera_info = yaml_to_CameraInfo(filename, header)
    publisher.publish(camera_info)

if __name__ == "__main__":
    try:
        # Get fname from command line (cmd line input required)

	rospy.init_node("camera_info_publisher")

	rospack = rospkg.RosPack()

        # get the file path for sensor_fusion
        path = rospack.get_path('carlike')

        # Initialize publisher node

        camera_topic = rospy.get_param('~camera_topic')
        calibration_conf = rospy.get_param('~camera_conf')
        camera_info_topic = rospy.get_param('~camera_info_topic')

        filename = path + calibration_conf




        publisher = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=10)

        rospy.Subscriber(camera_topic, Image, pub_camera_info, queue_size=1)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass