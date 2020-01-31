#! /usr/bin/python

import xml.etree.ElementTree as ET
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

import numpy as np
import struct
import socket

from gym_ros_interface.srv import DataSourceService


IMG_MSG_LENGTH = 12
VALID_MSG_TAGS = ["tIMG", "rIMG"]


class ImageServer:
    def __init__(self):
        self.image_port = rospy.get_param("~image_port", 9008)
        self.use_ground_truth = rospy.get_param("~use_ground_truth", True)
        rospy.loginfo("image port " + str(self.image_port))
        self.subscribers = [
            rospy.Subscriber("/left_cam/image_raw", Image, self.left_cam_callback),
            rospy.Subscriber("/right_cam/image_raw", Image, self.right_cam_callback),
            rospy.Subscriber("/segmentation/image_raw", Image, self.segmentation_cam_callback),
            rospy.Subscriber("/depth/image_raw", Image, self.depth_cam_callback),
            rospy.Subscriber("/segmentation_noisy/image_raw", Image, self.segmentation_noisy_cam_callback),
            rospy.Subscriber("/depth_noisy/image_raw", Image, self.depth_noisy_cam_callback),
            rospy.Subscriber("/metadata", String, self.metadata_callback),
            rospy.Subscriber("/kimera_vio_ros/odometry", Odometry, self.odometry_callback),
        ]

        self.data_source_service = rospy.Service(
            "/gym_ros_interface/data_source_request", DataSourceService, self.rosservice_change_data_source,
        )

        self.cv_bridge = CvBridge()

        # hold most recent images
        self.data = {}

        # set up image socket
        self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.image_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.image_socket.settimeout(None)
        self.image_socket.bind(("", self.image_port))

    def rosservice_change_data_source(self, request):
        self.use_ground_truth = request.use_gt
        return True

    def on_shutdown(self):
        self.img_server.image_socket.close()
        rospy.loginfo("Shutting down GymROS_node")

    def odometry_callback(self, msg):
        self.data["metadata_noisy"] = self.metadata_from_odometry_msg(msg)

    def metadata_from_odometry_msg(self, msg):
        pose = msg.pose.pose
        position = pose.position
        quat = pose.orientation
        timestamp = msg.header.stamp

        msg_root = ET.Element("TESSE_Agent_Metadata_v0.5")
        ET.SubElement(
            msg_root, "position", {"x": str(position.x), "y": str(position.y), "z": str(position.z)},
        )
        ET.SubElement(
            msg_root, "quaternion", {"x": str(quat.x), "y": str(quat.y), "z": str(quat.z), "w": str(quat.w)},
        )
        t = ET.SubElement(msg_root, "time")
        rospy.loginfo(type(timestamp))
        t.text = str(timestamp.to_sec())
        # TODO velocity

        if "metadata" in self.data.keys():
            metadata_root = ET.fromstring(self.data["metadata"])
            metadata_time = metadata_root.find("time").text
            collision_msg = metadata_root.find("collision")
            collider_msg = metadata_root.find("collider")
            ET.SubElement(
                msg_root,
                "collision",
                {
                    "status": collision_msg.attrib["status"],
                    "name": collision_msg.attrib["name"],
                    "time": metadata_time,
                },
            )
            ET.SubElement(
                msg_root, "collider", {"status": collider_msg.attrib["status"], "time": metadata_time},
            )

        msg = ET.tostring(msg_root)
        rospy.loginfo("Formed messge: %s" % (msg))
        return msg

    @staticmethod
    def encode_float_to_rgba(img):
        """ Encode 1 channel float image to 8 bit unsigned int RGBA image.

        Args:
            img (np.ndarray - shape=(H, W)): Input image.

        Returns:
            np.ndarray - shape=(H, W, 4)
                Unsigned 8 bit RGBA image.
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
        depth_img = self.cv_bridge.imgmsg_to_cv2(img, "32FC1")[::-1]
        self.data["depth"] = self.encode_float_to_rgba(depth_img)

    def segmentation_noisy_cam_callback(self, img):
        """ Listen to segmentation topic and save message. """
        self.data["segmentation_noisy"] = self.cv_bridge.imgmsg_to_cv2(img, "passthrough")[::-1]

    def depth_noisy_cam_callback(self, img):
        """ Listen to depth topic and save message. """
        depth_img = self.cv_bridge.imgmsg_to_cv2(img, "32FC1")[::-1]
        self.data["depth_noisy"] = self.encode_float_to_rgba(depth_img)

    def metadata_callback(self, msg):
        """ Listen to metadata topic and save message. """
        self.data["metadata"] = msg.data

    def unpack_img_msg(self, img_msg):
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
                    data_response.append(
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
