#!/usr/bin/env python

###################################################################################################
# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
#
# This material is based upon work supported by the Under Secretary of Defense for Research and
# Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions
# or recommendations expressed in this material are those of the author(s) and do not necessarily
# reflect the views of the Under Secretary of Defense for Research and Engineering.
#
# (c) 2020 Massachusetts Institute of Technology.
#
# MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)
#
# The software/firmware is provided to you on an As-Is basis
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013
# or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work
# are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other
# than as specifically authorized by the U.S. Government may violate any copyrights that exist in
# this work.
###################################################################################################

import socket
import struct

import numpy as np

import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from tesse.env import Env
from tesse.msgs import StepWithTransform
from tesse_segmentation_ros.models import get_model
from tesse_segmentation_ros.utils import get_class_colored_image

from tesse_gym_bridge.utils import (
    TesseData,
    get_origin_odom_msg,
    metadata_from_odometry_msg,
    wait_for_initialization,
)

IMG_MSG_LENGTH = 12
VALID_MSG_TAGS = ["tIMG", "rIMG"]


class ImageServer:
    def __init__(self):
        self.image_port = rospy.get_param("~image_port", 9008)
        self.use_ground_truth = rospy.get_param("~use_ground_truth", True)
        self.far_clip_plane = rospy.get_param("~far_clip_plane", 50.0)
        self.image_height = rospy.get_param("~image_height", 240)
        self.image_width = rospy.get_param("~image_width", 320)

        # If set to true, run semantic segmentation only upon client request
        # this reduces unnecessary computation
        self.run_segmentation_on_demand = rospy.get_param("~run_segmentation_on_demand", True)
        self.publish_segmentation = rospy.get_param("~publish_segmentation", False)
        model_type = rospy.get_param("~model_type", "")
        weights = rospy.get_param("~weights", "")

        self.cv_bridge = CvBridge()

        # hold current data
        self.data = TesseData()

        if self.run_segmentation_on_demand:
            self.segmentation_model = get_model(model_type, weights)

        self.subscribers = [
            rospy.Subscriber("/left_cam/image_raw", Image, self.left_cam_callback),
            rospy.Subscriber("/right_cam/image_raw", Image, self.right_cam_callback),
            rospy.Subscriber("/segmentation/image_raw", Image, self.segmentation_cam_callback),
            rospy.Subscriber("/depth/image_raw", Image, self.depth_cam_callback),
            rospy.Subscriber(
                "/segmentation_estimate/image_raw", Image, self.segmentation_noisy_cam_callback
            ),
            rospy.Subscriber("/depth_noisy/image_raw", Image, self.depth_noisy_cam_callback),
            rospy.Subscriber("/metadata", String, self.metadata_callback),
            rospy.Subscriber("/kimera_vio_ros/odometry", Odometry, self.odometry_callback),
        ]

        self.data_source_service = rospy.Service(
            "use_ground_truth", SetBool, self.rosservice_change_data_source,
        )

        self.episode_reset_service = rospy.Service(
            "image_server_episode_reset", Trigger, self.episode_reset_service,
        )

        if self.publish_segmentation:
            self.segmentation_pubs = [
                rospy.Publisher("rgb_left", Image, queue_size=10),
                rospy.Publisher("segmentation", Image, queue_size=10),
            ]

        # set up image socket
        self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.image_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.image_socket.settimeout(None)
        self.image_socket.bind(("", self.image_port))

    def rosservice_change_data_source(self, request):
        """ Change between ground truth and noisy data modes.

        Default provides ground truth image and position information
        from TESSE. Upon request, noisy semantic segmentation,
        depth, and position estimates are given instead.

        Args:
            request (SetBool): ROS SetBool message.

        Returns:
            bool: True, indicating successful service call.
        """
        self.use_ground_truth = request.data
        return True

    def episode_reset_service(self, trigger):
        """ Called on episode reset.

        This will re-initialize the pose estimate to
            position = (0, 0, 0)
            quaternion = (0, 0, 0, 1)

        Args:
            trigger (Trigger): ROS Trigger service message.

        Returns:
            Tuple[bool, str]: True if reset was a success, 
                empty message (to fulfill Trigger interface).
        """
        self.data.metadata_noisy = metadata_from_odometry_msg(
            get_origin_odom_msg(), self.data.metadata_gt
        )

        return True, ""

    def on_shutdown(self):
        """ Close image server socket. """
        self.image_socket.close()

    def odometry_callback(self, msg):
        """ Form metadata message containing noisy pose.

        Args:
            msg (Odometry): Odometry message containing a noisy pose estimate.
        """
        self.data.metadata_noisy = metadata_from_odometry_msg(msg, self.data.metadata_gt)

    @staticmethod
    def encode_float_to_rgba(img):
        """ Encode 1 channel float image to 8 bit unsigned int RGBA image.

        Args:
            img (np.ndarray): Shape - (H, W) image.

        Returns:
            np.ndarray: shape=(H, W, 4) unsigned 8 bit RGBA image.
        """
        img = img[..., np.newaxis] * np.float32((1.0, 255.0, 255.0 ** 2, 255.0 ** 3))
        img -= np.floor(img)
        img -= img[..., (1, 2, 3, 3)] * np.float32((1 / 256.0, 1 / 256.0, 1 / 256.0, 0.0))

        return (255 * img).astype(np.uint8)

    def left_cam_callback(self, img):
        self.data.rgb_left = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]

    def right_cam_callback(self, img):
        self.data.rgb_right = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]

    def segmentation_cam_callback(self, img):
        self.data.segmentation_gt = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]

    def depth_cam_callback(self, img):
        # decoded image is in [0, far_clip_plane], map to [0, 1] for proper encoding
        depth_img = self.cv_bridge.imgmsg_to_cv2(img, "32FC1")[::-1] / self.far_clip_plane
        self.data.depth_gt = self.encode_float_to_rgba(depth_img)

    def segmentation_noisy_cam_callback(self, img):
        self.data.segmentation_noisy = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]

    def depth_noisy_cam_callback(self, img):
        depth_img = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]
        depth_img = np.nan_to_num(depth_img)
        depth_img /= self.far_clip_plane  # TODO(ZR) double check this step
        self.data.depth_noisy = self.encode_float_to_rgba(depth_img)

    def metadata_callback(self, metadata_msg):
        self.data.metadata_gt = metadata_msg.data

        # initialize pose estimate at origin
        if self.data.metadata_noisy is None:
            self.data.metadata_noisy = metadata_from_odometry_msg(
                get_origin_odom_msg(), self.data.metadata_gt
            )

    def unpack_img_msg(self, img_msg):
        """ Unpack an image request message.

        Args:
            img_msg (bytearray): Image request message.

        Returns:
            Tuple[int, int, int]: IDs for camera, compression, and number
                of channels.
        """
        assert len(img_msg) == 12, "recieved image message of length %d, require length 12" % len(
            img_msg
        )
        camera = struct.unpack("<i", img_msg[0:4])[0]
        compression = struct.unpack("<i", img_msg[4:8])[0]
        channels = struct.unpack("<i", img_msg[8:])[0]
        return camera, compression, channels

    def null_check(self, x, var_name):
        """ Checks for null values and logs error if found. """
        if x is None:
            rospy.logerr("Variable %s is none" % var_name)

    def _get_segmentation_img_response(self):
        """ Get semantic segmentation image and proper encoding. 
        
        Returns:
            Tuple[np.ndarray - shape=(H, W, 3), str]: Image as 
                numpy array and required encoding (RGB).
        """
        if self.use_ground_truth:
            segmentation_img = self.data.segmentation_gt
        else:
            # if requested, run segmentation inference on the latest rgb image
            # otherwise, just use the most recent segmentation estimate
            if self.run_segmentation_on_demand:
                self.data.segmentation_noisy = self.segmentation_inference(self.data.rgb_left)
            segmentation_img = self.data.segmentation_noisy.astype(np.uint8)

        self.null_check(segmentation_img, "Segmentation image")
        return (
            segmentation_img,
            "xRGB",
        )

    def _get_depth_img_response(self):
        """ Get semantic depth image and proper encoding. 
        
        Returns:
            Tuple[np.ndarray - shape=(H, W, 4), str]: Image as 
                numpy array and required encoding (RGB). Float32 
                depth image is encoded over uint8 RGBA image.
        """
        depth_img = self.data.depth_gt if self.use_ground_truth else self.data.depth_noisy
        if self.use_ground_truth:
            depth_img = self.data.depth_gt
        else:
            depth_img = self.data.depth_noisy

            # send empty depth image if estimate is not initialized
            if depth_img is None:
                depth_img = np.zeros((self.image_height, self.image_width, 4))  # depth is RGBA

        depth_img = depth_img.astype(np.float32)

        self.null_check(depth_img, "Depth img")
        return (
            depth_img,
            "xFLT",
        )

    def _get_metadata_response(self):
        return self.data.metadata_gt if self.use_ground_truth else self.data.metadata_noisy

    def unpack_data_request(self, message):
        """ Decode client data request

        Args:
            message (str): Client data request message.

        Returns:
            Tuple[List[Tuple[np.ndarray, str]], str]: Two element tuple of
                - List of image and image format pairs
                - metadata
        """
        message = bytearray(message)
        tag = message[0:4]
        data_response = []

        if tag in VALID_MSG_TAGS:
            for msg_index in np.arange(4, len(message), 12):
                camera, compression, channels = self.unpack_img_msg(
                    message[msg_index : msg_index + IMG_MSG_LENGTH]
                )
                if camera == 0:  # RGB left
                    self.null_check(self.data.rgb_left, "rgb left")
                    data_response.append((self.data.rgb_left, "xRGB"))
                if camera == 1:  # RGB right
                    self.null_check(self.data.rgb_right, "rgb right")
                    data_response.append((self.data.rgb_right, "xRGB"))
                if camera == 2:  # Semantic segmentation
                    data_response.append(self._get_segmentation_img_response())
                if camera == 3:  # Depth
                    data_response.append(self._get_depth_img_response())
            if tag == "tIMG":
                metadata = self._get_metadata_response()
            else:
                metadata = struct.pack("I", 0)
        else:
            rospy.logerr("Received unknown request tag: %s" % tag)

        return data_response, metadata

    def form_image_response(self, img_data, tag):
        """ Encode image data to message.

        Args:
            img_data (np.ndarray): Input image data.
            tag (str): Data format (xRGB or xFLT).

        Returns:
            bytearray: Encoded image and associated metadata.
        """
        img_data = img_data.astype(np.uint8)
        img_height, img_width, channels = img_data.shape

        img_data = img_data.tobytes()
        cam_id = 0

        img_type = tag
        img_payload_length = len(img_data)

        img_payload = bytearray()
        img_payload.extend(list("null"))  # 4 byte empty sequence
        img_payload.extend(struct.pack("I", img_payload_length))
        img_payload.extend(struct.pack("I", img_width))
        img_payload.extend(struct.pack("I", img_height))
        img_payload.extend(struct.pack("I", cam_id))
        img_payload.extend(img_type)
        img_payload.extend(2 * list("null"))  # 8 byte empty sequence
        img_payload.extend(img_data)

        return img_payload

    def form_multi_image_response(self, imgs, metadata):
        """ Compose message of multiple images.

        Args:
            imgs (Tuple[np.ndarray, str]): List of image/encoding pairs.
            metadata (str): Metadata to send.

        Returns:
            bytearray: Encoded images and metadata.
        """
        # define data
        img_payloads = [self.form_image_response(img, tag) for (img, tag) in imgs]

        # defined payload length
        img_payload_length = sum([len(img_payload) for img_payload in img_payloads])
        meta_payload_length = len(metadata)

        # form message below
        return_payload = bytearray()
        return_payload.extend("mult")

        # header
        return_payload.extend(struct.pack("I", img_payload_length))
        return_payload.extend(struct.pack("I", meta_payload_length))

        # images
        for img_payload in img_payloads:
            return_payload.extend(img_payload)

        # metadata
        return_payload.extend(metadata)

        return return_payload

    def segmentation_inference(self, input_img):
        """ Run semantic segmentation network """
        # flip columns (across vertical axis) to meet opencv convention
        input_img = input_img[::-1] / 255.0
        seg = get_class_colored_image(self.segmentation_model.infer(input_img).astype(np.uint8))
        seg = seg[::-1]  # flip columns back
        return seg

    def _send_image_message(self, return_payload):
        image_send_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        image_send_socket.connect(("", self.image_port))
        image_send_socket.send(return_payload)
        image_send_socket.close()

    def _publish_rgb_and_segmentation(self):
        self.segmentation_pubs[0].publish(
            self.cv_bridge.cv2_to_imgmsg(self.data.rgb_left[::-1].astype(np.uint8), "rgb8")
        )
        self.segmentation_pubs[1].publish(
            self.cv_bridge.cv2_to_imgmsg(
                self.data.segmentation_noisy[::-1].astype(np.uint8), "rgb8"
            )
        )

    def spin(self):
        """ Receive client image requests and send back data. """

        # wait for required data to be initialized before setting up server
        vars_to_init = ("rgb_left",)

        # if waiting for external noisy segmentation
        if not self.use_ground_truth and not self.run_segmentation_on_demand:
            vars_to_init += ("segmentation_noisy",)

        wait_for_initialization(self.data, vars_to_init)
        rospy.loginfo("Image server initialized")

        while not rospy.is_shutdown():
            message, address = self.image_socket.recvfrom(1024)

            requested_data, metadata = self.unpack_data_request(message)
            return_payload = self.form_multi_image_response(requested_data, metadata)

            self._send_image_message(return_payload)

            if self.publish_segmentation:
                self._publish_rgb_and_segmentation()


if __name__ == "__main__":
    rospy.init_node("ImageServer_node")
    node = ImageServer()
    node.spin()
