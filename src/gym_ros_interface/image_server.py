#! /usr/bin/python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import struct
import socket

IMG_MSG_LENGTH = 12
VALID_MSG_TAGS = ['tIMG', 'rIMG']


class ImageServer:
    def __init__(self):
        self.image_port = rospy.get_param("~image_port", 9008)
        rospy.loginfo("image port " + str(self.image_port))
        self.subscribers = [rospy.Subscriber("/left_cam/image_raw", Image, self.left_cam_callback),
                            rospy.Subscriber("/right_cam/image_raw", Image, self.right_cam_callback),
                            rospy.Subscriber("/segmentation/image_raw", Image, self.segmentation_cam_callback),
                            rospy.Subscriber("/depth/image_raw", Image, self.depth_cam_callback),
                            rospy.Subscriber("/metadata", String, self.metadata_callback)]

        # hold most recent images
        self.last_images = {}

        # set up image socket
        self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.image_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.image_socket.settimeout(None)
        self.image_socket.bind(('', self.image_port))

    def on_shutdown(self):
        self.img_server.image_socket.close()
        rospy.loginfo("Shutting down GymROS_node")

    def buffer_to_img(self, img, add_axis=False, n_channels=1):
        """ Decode ROS image message to image. """
        height, width = img.height, img.width
        img = np.frombuffer(img.data, dtype=np.uint8).reshape((height, width, n_channels))
        img = img[::-1]
        if add_axis:
            img = np.repeat(img, 3, axis=-1)
        return img

    def left_cam_callback(self, img):
        """ Listen to left camera topic and save message. """
        self.last_images['left_cam'] = self.buffer_to_img(img, n_channels=3)

    def right_cam_callback(self, img):
        """ Listen to right camera topic and save message. """
        self.last_images['right_cam'] = self.buffer_to_img(img, n_channels=3)

    def segmentation_cam_callback(self, img):
        """ Listen to segmentation topic and save message. """
        self.last_images['segmentation'] = self.buffer_to_img(img, n_channels=3)

    def depth_cam_callback(self, img):
        """ Listen to depth topic and save message. """
        self.last_images['depth'] = self.buffer_to_img(img, n_channels=4)
        self.last_images['depth'] = self.last_images['depth'][..., ::-1]  # switch bit order

    def metadata_callback(self, msg):
        """ Listen to metadata topic and save message. """
        self.last_images['meta'] = msg.data

    def unpack_img_msg(self, img_msg):
        assert len(img_msg) == 12, \
            "recieved image message of length %d, require length 12" % len(img_msg)
        camera = struct.unpack('<i', img_msg[0:4])[0]
        compression = struct.unpack('<i', img_msg[4:8])[0]
        channels = struct.unpack('<i', img_msg[8:])[0]
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
                camera, compression, channels = \
                    self.unpack_img_msg(message[msg_index:msg_index + IMG_MSG_LENGTH])
                if camera == 0:
                    data_response.append((self.last_images['left_cam'], 'xRGB'))
                if camera == 1:
                    data_response.append((self.last_images['right_cam'], 'xRGB'))
                if camera == 2:
                    data_response.append((self.last_images['segmentation'], 'xRGB'))
                if camera == 3:
                    data_response.append((self.last_images['depth'], 'xFLT'))

            metadata = self.last_images['meta'] if tag == 'tIMG' else struct.pack('I', 0)
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
        img_data = struct.pack('B' * len(img_data), *img_data)
        img_payload_length = len(img_data)

        img_payload = bytearray()
        img_payload.extend(list('null'))  # 4 byte empty sequence
        img_payload.extend(struct.pack('I', img_payload_length))
        img_payload.extend(struct.pack('I', img_width))
        img_payload.extend(struct.pack('I', img_height))
        img_payload.extend(struct.pack('I', cam_id))
        img_payload.extend(img_type)
        img_payload.extend(2*list('null'))  # 8 byte empty sequence
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
        return_payload.extend('mult')

        # header
        return_payload.extend(struct.pack('I', img_payload_length))
        return_payload.extend(struct.pack('I', meta_payload_length))

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
            image_send_socket.connect(('', self.image_port))
            image_send_socket.send(return_payload)
            image_send_socket.close()


if __name__ == "__main__":
    rospy.init_node("ImageServer_node")
    node = ImageServer()
    node.spin()
