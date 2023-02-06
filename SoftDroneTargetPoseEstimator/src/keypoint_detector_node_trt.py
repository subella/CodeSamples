#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
import os
import zmq
import base64
from softdrone_target_pose_estimator.msg import Keypoints2D, Keypoint2D


def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    #buf = memoryview(msg)
    A = np.frombuffer(msg, dtype=md['dtype'])
    return A.reshape(md['shape'])

def image_callback(msg):
    try:
        img = bridge.imgmsg_to_cv2(msg, "rgb8")
        socket.send(img)
        kps = recv_array(socket)
        if np.any(kps):
            kps_msg = Keypoints2D()
            kps_msg.header.stamp = msg.header.stamp
            for kp in kps:
                kp_msg = Keypoint2D()
                kp_msg.x = kp[0]
                kp_msg.y = kp[1]
                kps_msg.keypoints_2D.append(kp_msg)
            kps_pub.publish(kps_msg)

            for kp in kps:
                img = cv2.circle(img, (kp[0], kp[1]), 2, (255,0,0),2)

            annotated_img_pub.publish(bridge.cv2_to_imgmsg(img))

    except CvBridgeError, e:
        print(e)

def main():
    image_topic = "/target_cam/color/image_raw"
    rospy.Subscriber(image_topic, ImageMsg, image_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('keypoint_detector')
    context = zmq.Context()
    bridge = CvBridge()
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://localhost:5555')
    kps_pub = rospy.Publisher('~keypoints_out', Keypoints2D)
    annotated_img_pub = rospy.Publisher('~annotated_img_out', ImageMsg)
    main()
