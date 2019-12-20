#! /usr/bin/python

import rospy
from std_msgs.msg import String
import struct
import socket

IMG_MSG_LENGTH = 12


class MetadataServer:
    def __init__(self):
        self.image_port = rospy.get_param("~metadata_port", 9007)
        self.metadata_subscriber = rospy.Subscriber("/metadata", String, self.metadata_callback)

        # store last received metadata
        self.last_metadata = ''

        # initialize socket
        self.metadata_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.metadata_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.metadata_socket.settimeout(None)
        self.metadata_socket.bind(('', self.image_port))

    def metadata_callback(self, msg):
        """ Listens to the metadata topic and save last metadata message. """
        self.last_metadata = msg.data

    def spin(self):
        """ Receive client metadata request and response """
        while not rospy.is_shutdown():
            message, address = self.metadata_socket.recvfrom(1024)

            if message == 'rMET':
                # send response
                response = bytearray()
                response.extend('meta')

                metadata = self.last_metadata
                response.extend(struct.pack('I', len(metadata)))
                response.extend(self.last_metadata)

                image_send_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                image_send_socket.connect(('', self.image_port))
                image_send_socket.send(response)
                image_send_socket.close()
            else:
                rospy.logerror("Unknown tag: %s" % message)


if __name__ == "__main__":
    rospy.init_node("MetadataServer_node")
    node = MetadataServer()
    node.spin()
