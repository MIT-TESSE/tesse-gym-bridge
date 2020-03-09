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

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from tesse_gym_bridge.srv import DataSourceService
from tesse_gym_bridge.utils import TesseData, metadata_from_odometry_msg


class MetadataServer:
    def __init__(self):
        self.use_ground_truth = rospy.get_param("~use_ground_truth", True)
        self.metadata_port = rospy.get_param("~metadata_port", 9007)

        self.metadata_gt_subscriber = (rospy.Subscriber("/metadata", String, self.metadata_callback),)
        self.nosy_pose_subscriber = rospy.Subscriber("/kimera_vio_ros/odometry", Odometry, self.odometry_callback)

        self.data_source_service = rospy.Service(
            "/tesse_gym_bridge/data_source_request", DataSourceService, self.rosservice_change_data_source,
        )

        # store last received metadata
        self.data = TesseData()

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
            self.data.metadata_noisy = metadata_from_odometry_msg(msg, self.data.metadata_gt)
        except Exception as ex:
            rospy.loginfo("Metadata server caught exception %s" % ex)
            self.data.metadata_noisy = ""

    def metadata_callback(self, metadata_msg):
        self.data.metadata_gt = metadata_msg.data

    def on_shutdown(self):
        """ Close metadata server socket."""
        self.metadata_socket.close()

    def spin(self):
        """ Receive client metadata request and response """
        while not rospy.is_shutdown():
            message, address = self.metadata_socket.recvfrom(1024)

            if message == "rMET":
                # send response
                response = bytearray()
                response.extend("meta")

                metadata = self.data.metadata_gt  # TODO(ZR) enable when Kimera-VIO has reset flag
                # (
                #     self.data.metadata_gt
                #     if self.use_ground_truth
                #     else self.data.metadata_noisy
                # )
                response.extend(struct.pack("I", len(metadata)))
                response.extend(metadata)

                metadata_send_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                metadata_send_socket.connect(("", self.metadata_port))
                metadata_send_socket.send(response)
                metadata_send_socket.close()
            elif message == "sRES":
                rospy.loginfo("Setting new episode")
            else:
                rospy.logerror("Unknown tag: %s" % message)


if __name__ == "__main__":
    rospy.init_node("MetadataServer_node")
    node = MetadataServer()
    node.spin()
