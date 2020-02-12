#!/usr/bin/env python

import socket
import struct

import numpy as np

import rospy
import tf
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tesse.utils import UdpListener

from gym_ros_interface.srv import DataSourceService
from gym_ros_interface.utils import metadata_from_odometry_msg


IMG_MSG_LENGTH = 12
VALID_MSG_TAGS = ["tIMG", "rIMG"]


class ImageServer:
    def __init__(self):
        self.image_port = rospy.get_param("~image_port", 9008)
        self.metadata_udp_port = rospy.get_param("~metadata_udp_port", 9004)
        self.use_ground_truth = rospy.get_param("~use_ground_truth", True)
        self.far_clip_plane = rospy.get_param("~far_clip_plane", 50.0)

        self.subscribers = [
            rospy.Subscriber("/left_cam/image_raw", Image, self.left_cam_callback),
            rospy.Subscriber("/right_cam/image_raw", Image, self.right_cam_callback),
            rospy.Subscriber("/segmentation/image_raw", Image, self.segmentation_cam_callback),
            rospy.Subscriber("/depth/image_raw", Image, self.depth_cam_callback),
            rospy.Subscriber("/segmentation_noisy/image_raw", Image, self.segmentation_noisy_cam_callback),
            rospy.Subscriber("/depth_noisy/image_raw", Image, self.depth_noisy_cam_callback),

            rospy.Subscriber("/kimera_vio_ros/odometry", Odometry, self.odometry_callback),
        ]

        self.data_source_service = rospy.Service(
            "~use_ground_truth", DataSourceService, self.rosservice_change_data_source,
        )

        self.cv_bridge = CvBridge()

        # hold most recent images
        self.data = {}

        # read UDP metadata broadcast from TESSE
        self.udp_listener = UdpListener(port=self.metadata_udp_port, rate=200)
        self.udp_listener.subscribe("catch_metadata", self.catch_udp_broadcast)
        self.udp_listener.start()

        # set up image socket
        self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.image_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.image_socket.settimeout(None)
        self.image_socket.bind(("", self.image_port))

        self.transform_listener = tf.TransformListener()

        self.initial_pose = None

    def rosservice_change_data_source(self, request):
        """ Change between ground truth and noisy data modes.

        Default provides ground truth image and position information
        from TESSE. Upon request, noisy semantic segmentation,
        depth, and position estimates are given instead.

        Args:
            request (DataSourceServie): ROS service data source.

        Returns:
            bool: True, indicating successful service call.
        """
        self.use_ground_truth = request.use_gt
        return True

    def on_shutdown(self):
        """ Close image server socket and udp listener. """
        self.image_socket.close()
        self.udp_listener.join()

    def odometry_callback(self, msg):
        """ Form metadata message containing noisy pose.

        Args:
            msg (Odometry): Odometry message containing a noisy pose estimate.
        """
        try:
            # TODO check
            self.data["metadata_noisy"] = metadata_from_odometry_msg(msg, self.data["metadata"])
        except Exception as ex:
            rospy.loginfo("Image server caught exception %s" % ex)
            self.data["metadata_noisy"] = ""

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
        """ Listen to left camera topic and save message. """
        self.data["left_cam"] = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]

    def right_cam_callback(self, img):
        """ Listen to right camera topic and save message. """
        self.data["right_cam"] = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]

    def segmentation_cam_callback(self, img):
        """ Listen to segmentation topic and save message. """
        self.data["segmentation"] = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]

    def depth_cam_callback(self, img):
        """ Listen to depth topic and save message. """
        # decoded image is in [0, far_clip_plane], map to [0, 1] for proper encoding
        depth_img = self.cv_bridge.imgmsg_to_cv2(img, "32FC1")[::-1] / self.far_clip_plane
        self.data["depth"] = self.encode_float_to_rgba(depth_img)

    def segmentation_noisy_cam_callback(self, img):
        """ Listen to segmentation topic and save message. """
        self.data["segmentation_noisy"] = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]

    def depth_noisy_cam_callback(self, img):
        """ Listen to depth topic and save message. """
        depth_img = self.cv_bridge.imgmsg_to_cv2(img, "32FC1")[::-1]
        self.data["depth_noisy"] = self.encode_float_to_rgba(depth_img)

    def catch_udp_broadcast(self, udp_metadata):
        """ Capture metadata messages broadcast by TESSE. """
        self.data["metadata"] = udp_metadata

    def unpack_img_msg(self, img_msg):
        """ Unpack an image request message.

        Args:
            img_msg (bytearray): Image request message.

        Returns:
            Tuple[int, int, int]: IDs for camera, compression, and number
                of channels.
        """
        assert len(img_msg) == 12, "recieved image message of length %d, require length 12" % len(img_msg)
        camera = struct.unpack("<i", img_msg[0:4])[0]
        compression = struct.unpack("<i", img_msg[4:8])[0]
        channels = struct.unpack("<i", img_msg[8:])[0]
        return camera, compression, channels

    def unpack_data_request(self, message):
        """ Decode client data request

        Args:
            message (str): Client data request message.

        Returns:
            Tuple[List[Tuple[np.ndarray, str]], str]: The first element is a
                list of image/format pairs. The second element is metadata.
        """
        message = bytearray(message)
        tag = message[0:4]
        data_response = []

        if tag in VALID_MSG_TAGS:
            for msg_index in np.arange(4, len(message), 12):
                camera, compression, channels = self.unpack_img_msg(message[msg_index : msg_index + IMG_MSG_LENGTH])
                if camera == 0:
                    data_response.append((self.data["left_cam"], "xRGB"))
                if camera == 1:
                    data_response.append((self.data["right_cam"], "xRGB"))
                if camera == 2:
                    data_response.append(  # TODO error check
                        (
                            self.data["segmentation"] if self.use_ground_truth else self.data["segmentation_noisy"],
                            "xRGB",
                        )
                    )
                if camera == 3:
                    data_response.append(
                        (self.data["depth"] if self.use_ground_truth else self.data["depth_noisy"], "xFLT",)
                    )

            if tag == "tIMG":
                metadata = self.data["metadata"] if self.use_ground_truth else self.data["metadata_noisy"]
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
        img_height, img_width, channels = img_data.shape
        img_data = list(img_data.reshape(-1))
        cam_id = 0

        img_type = tag
        img_data = struct.pack("B" * len(img_data), *img_data)
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

    def spin(self):
        """ Receive client image requests and send back data. """
        while not rospy.is_shutdown():
            message, address = self.image_socket.recvfrom(1024)
            requested_data, metadata = self.unpack_data_request(message)

            # send response
            return_payload = self.form_multi_image_response(requested_data, metadata)

            image_send_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            image_send_socket.connect(("", self.image_port))
            image_send_socket.send(return_payload)
            image_send_socket.close()


if __name__ == "__main__":
    rospy.init_node("ImageServer_node")
    node = ImageServer()
    node.spin()
