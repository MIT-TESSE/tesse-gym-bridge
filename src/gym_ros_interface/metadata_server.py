#!/usr/bin/env python

import socket
import struct

import rospy
from nav_msgs.msg import Odometry
from tesse.utils import UdpListener

from gym_ros_interface.srv import DataSourceService
from gym_ros_interface.utils import metadata_from_odometry_msg


class MetadataServer:
    def __init__(self):
        self.use_ground_truth = rospy.get_param("~use_ground_truth", True)
        self.metadata_udp_port = rospy.get_param("~metadata_udp_port", 9004)
        self.metadata_port = rospy.get_param("~metadata_port", 9007)
        self.nosy_pose_subscriber = rospy.Subscriber("/kimera_vio_ros/odometry", Odometry, self.odometry_callback)

        self.data_source_service = rospy.Service(
            "~use_ground_truth", DataSourceService, self.rosservice_change_data_source,
        )

        # store last received metadata
        self.last_metadata = ""
        self.last_noisy_metadata = ""

        # read UDP metadata broadcast from TESSE
        self.udp_listener = UdpListener(port=self.metadata_udp_port, rate=200)
        self.udp_listener.subscribe("catch_metadata", self.catch_udp_broadcast)
        self.udp_listener.start()

        # initialize socket
        self.metadata_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.metadata_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.metadata_socket.settimeout(None)
        self.metadata_socket.bind(("", self.metadata_port))

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

    def odometry_callback(self, msg):
        """ Form metadata message containing noisy pose.

        Args:
            msg (Odometry): Odometry message containing a noisy pose estimate.
        """
        try:
            # TODO check
            self.last_noisy_metadata = metadata_from_odometry_msg(
                msg, self.last_metadata
            )
        except Exception as ex:
            rospy.loginfo("Metadata server caught exception %s" % ex)
            self.last_noisy_metadata = ""

    def catch_udp_broadcast(self, udp_metadata):
        """ Capture metadata messages broadcast by TESSE. """
        self.last_metadata = udp_metadata

    def on_shutdown(self):
        """ Close metadata server socket and udp listener. """
        self.metadata_socket.close()
        self.udp_listener.join()

    def spin(self):
        """ Receive client metadata request and response """
        while not rospy.is_shutdown():
            message, address = self.metadata_socket.recvfrom(1024)

            if message == "rMET":
                # send response
                response = bytearray()
                response.extend("meta")

                metadata = (
                    self.last_metadata
                    if self.use_ground_truth
                    else self.last_noisy_metadata
                )
                response.extend(struct.pack("I", len(metadata)))
                response.extend(metadata)

                image_send_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                image_send_socket.connect(("", self.metadata_port))
                image_send_socket.send(response)
                image_send_socket.close()
            else:
                rospy.logerror("Unknown tag: %s" % message)


if __name__ == "__main__":
    rospy.init_node("MetadataServer_node")
    node = MetadataServer()
    node.spin()
