#!/usr/bin/env python

import socket
import time
import xml.etree.ElementTree as ET

import numpy as np

import rospy
import tf
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tesse.env import Env
from tesse.msgs import StepWithTransform

# TESSE ROS bridge coordinate transforms

# fmt: off
enu_T_unity = np.array([[1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]])

unity_T_enu = np.transpose(enu_T_unity)

# fmt: off
brh_T_blh = np.array([[1, 0, 0, 0],
                      [0, -1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

blh_T_brh = np.transpose(brh_T_blh)

gravity_enu = [0.0, 0.0, -9.81]  # in 'world' frame


def get_origin_odom_msg():
    """ Get ros odometry message at origin.

    Returns:
        Odometry: ROR odometry message with
            position = (0, 0, 0) and
            quaternion = (0, 0, 0, 1)
    """
    msg = Odometry()
    pose = PoseWithCovariance()
    pose.pose.orientation.w = 1
    msg.pose = pose
    return msg


def enu_brh_to_unity_blh(enu_brh):
    """ Convert an ENU right-handed frame to unity left-handed frame

    Args:
        enu_brh (np.ndarray): Shape-(4, 4) SE(3) matrix of
            pose in ENU right-handed frame.

    Returns:
        np.ndarray: Shape-(4, 4) SE(3) matrix
            in Unity left-handed frame.
    """
    # right handed coordinate frame to left-handed frame
    enu_T_blh = enu_brh.dot(brh_T_blh)

    # convert to unity frame
    unity_odom = unity_T_enu.dot(enu_T_blh)

    return unity_odom


def convert_pose_ros_to_unity(odom_msg):
    """ Convert pose from ROS to unity coordinate system.

    Takes the position and quaternion provided by an odometry message,
    converts those from ROS ENU right-handed to Unity left-handed Unity
    left-handed coordinate system, then returns position and quaternion
    components.

    Args:
        odom_msg (Odometry): ROS odometry message

    Returns:
        Tuple[np.ndarray, np.ndarray]: Shapes (3,) and (4,) arrays containing
            position and quaternion.
    """
    position = odom_msg.pose.pose.position
    quat = odom_msg.pose.pose.orientation

    # convert position and quaternion to 4x4 transformation matrix
    enu_brh = tf.transformations.quaternion_matrix(np.array([quat.x, quat.y, quat.z, quat.w]))
    enu_brh[:3, 3] = np.array([position.x, position.y, position.z])

    # ROS to Unity coordinate system transform
    unity_blh = enu_brh_to_unity_blh(enu_brh)

    position = unity_blh[:3, 3]
    quat = tf.transformations.quaternion_from_matrix(unity_blh)

    return position, quat


def metadata_from_odometry_msg(msg, gt_metadata):
    """ Turn odometry message into TESSE gt_metadata message

    Args:
        msg (Odometry): ROS odometry message.
        gt_metadata (string): Ground truth TESSE metadata used to
            populate collision and time information.

    Returns:
        str: Metadata message containing position and orientation
            information from `msg`.
    """
    position, quat = convert_pose_ros_to_unity(msg)

    timestamp = msg.header.stamp

    msg_root = ET.Element("TESSE_Agent_Metadata_v0.5")

    ET.SubElement(
        msg_root, "position", {"x": str(position[0]), "y": str(position[1]), "z": str(position[2])},
    )
    ET.SubElement(
        msg_root, "quaternion", {"x": str(quat[0]), "y": str(quat[1]), "z": str(quat[2]), "w": str(quat[3])},
    )
    t = ET.SubElement(msg_root, "time")
    t.text = str(timestamp.to_sec())
    # TODO velocity

    if gt_metadata is not None:
        gt_metadata_root = ET.fromstring(gt_metadata)
        gt_metadata_time = gt_metadata_root.find("time").text
        collision_msg = gt_metadata_root.find("collision")
        collider_msg = gt_metadata_root.find("collider")
        ET.SubElement(
            msg_root,
            "collision",
            {"status": collision_msg.attrib["status"], "name": collision_msg.attrib["name"], "time": gt_metadata_time,},
        )
        ET.SubElement(
            msg_root, "collider", {"status": collider_msg.attrib["status"], "time": gt_metadata_time},
        )

    # flag to indicate this is an estimate
    noisy_tag = ET.SubElement(msg_root, "noisy_pose")
    noisy_tag.text = "true"

    msg = ET.tostring(msg_root)
    return msg


def wait_for_initialization(data, vars_to_init):
    """ Wait for variables in `data` to be initialized.

    Args:
        data (TesseData): Object to store variables.
        vars_to_init (Tuple[str]): Requires these variables in
            `data` to be initialized.

    to be initialized, ensuring there is data to send back to
    clients upon request.
    """
    # If current time is 0, advance game time so initial data is published
    while not data.all_initialized(vars_to_init):
        time.sleep(2)  # wait for TESSE to start.
        rospy.loginfo("Sending empty step message to advance game time")
        try:
            Env().send(StepWithTransform(0, 0, 0))
        except socket.error:  # if TESSE is not initialized
            rospy.loginfo("TESSE connection refused")

def call_trigger_service(service_name, timeout=None):
    """
    Returns:
        TriggerResponse
    """
    rospy.wait_for_service(service_name, timeout=timeout)
    trigger_service = rospy.ServiceProxy(service_name, Trigger)
    return trigger_service()

class TesseData:
    """ Class to hold TESSE Data"""

    def __init__(self):
        self._rgb_left = None
        self._rgb_right = None
        self._segmentation_gt = None
        self._depth_gt = None
        self._metadata_gt = None

        self._segmentation_noisy = None
        self._depth_noisy = None
        self._metadata_noisy = None

    def all_initialized(self, vars):
        """ Check if all variables given in `vars`
        are initialized.

        Args:
            vars (Tuple[str]): Variables to check.

        Returns:
            bool: True if all variables in `vars` are
                initialized. False otherwise. """
        return not any([getattr(self, v) is None for v in vars])
    @property
    def rgb_left(self):
        return self._rgb_left

    @rgb_left.setter
    def rgb_left(self, value):
        self._rgb_left = value

    @property
    def rgb_right(self):
        return self._rgb_right

    @rgb_right.setter
    def rgb_right(self, value):
        self._rgb_right = value

    @property
    def segmentation_gt(self):
        return self._segmentation_gt

    @segmentation_gt.setter
    def segmentation_gt(self, value):
        self._segmentation_gt = value

    @property
    def depth_gt(self):
        return self._depth_gt

    @depth_gt.setter
    def depth_gt(self, value):
        self._depth_gt = value

    @property
    def metadata_gt(self):
        return self._metadata_gt

    @metadata_gt.setter
    def metadata_gt(self, value):
        self._metadata_gt = value

    @property
    def segmentation_noisy(self):
        return self._segmentation_noisy

    @segmentation_noisy.setter
    def segmentation_noisy(self, value):
        self._segmentation_noisy = value

    @property
    def depth_noisy(self):
        return self._depth_noisy

    @depth_noisy.setter
    def depth_noisy(self, value):
        self._depth_noisy = value

    @property
    def metadata_noisy(self):
        return self._metadata_noisy

    @metadata_noisy.setter
    def metadata_noisy(self, value):
        self._metadata_noisy = value
