#!/usr/bin/env python2

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import MultiArrayDimension, Float64MultiArray
from random import random
from math import sin, cos, pi

import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import tf
import os
import datetime
import numpy as np

#  ***************apriltag required ****************
from apriltag_ros.msg import AprilTagDetectionArray

global image_pub, bridge, server
global pose_list, pose_list_name, image_dir

global tags_index_list, tags_pose_list, model_points_list, calibrated, vins_T_model, calibration_list
global old_tags_index_list, old_tags_pose_list, tag_tolerance

global pub_tag_msg
print('finish import')


def tag_callback(tags_data):
    global tags_index_list, tags_pose_list, model_points_list, calibrated, pose_list, pose_list_name, model_T_vins, calibration_list
    global br, model_pose, pcd_publisher, pcd_msg, Timer1, vins_T_model, tag_tolerance
    global pub_tag_ids, pub_tag_locations
    # init sending msg "palmextras"
    palmextras = Float64MultiArray()
    dim = MultiArrayDimension()
    dim.label = "tag_id"
    dim.size = 1
    palmextras.layout.dim.append(dim)

    dim = MultiArrayDimension()
    dim.label = "tag_location"
    dim.size = 3
    palmextras.layout.dim.append(dim)

    mid_tag_output_msg = []
    ## check if record tag
    for item in tags_data.detections:
        print('item', item)
        # print('item.id',item.id)
        # tag_orient = item.pose.pose.pose.orientation
        tag_orient = item.pose.pose.pose.position
        tag_q = [tag_orient.x, tag_orient.y, tag_orient.z]
        # tag_euler = tf.transformations.euler_from_quaternion(tag_q, axes='sxyz')
        # print('tag_euler',tag_euler)
        tag_id = item.id[0]
        mid_tag_output_msg.append(tag_id)
        for i in range(len(tag_q)):
            mid_tag_output_msg.append(tag_q[i])
        print('tag_id', mid_tag_output_msg)
    palmextras.data = mid_tag_output_msg
    pub_tag_msg.publish(palmextras)  # n*4 msgs, each msg include "tag_id, tag_x,tag_y,tag_z"


if __name__ == "__main__":
    global model_pose, vins_T_model, model_T_vins, calibrated, pcd_msg, Timer1
    global pub_tag_ids, pub_tag_msg

    calibrated = False

    print('start node')
    rospy.init_node("basic_controls")
    rospy.sleep(0.5)
    br = TransformBroadcaster()

    rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_callback, queue_size=1)
    pub_tag_msg = rospy.Publisher("/pub_tag_msg", Float64MultiArray, queue_size=10)

    print('waiting for tag')
    # rospy.Subscriber("h20t_pub_GUI_topic", H20tPubGUI, H20T_callback,queue_size=10)

    # broadcast tag frames
    '''
    while(True):
        time = rospy.Time.now()
        br.sendTransform( vins_points[0], (0, 0, 0, 1),  time, str(tag_list[0]), "world" )
        br.sendTransform( vins_points[1], (0, 0, 0, 1),  time, str(tag_list[1]), "world" )
        br.sendTransform( vins_points[2], (0, 0, 0, 1),  time, str(tag_list[2]), "world" )
    '''

    rospy.spin()
