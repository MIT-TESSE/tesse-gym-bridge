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

import errno
import socket
import struct

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from tesse_gym_bridge.utils import (
    TesseData,
    call_trigger_service,
    get_origin_odom_msg,
    metadata_from_odometry_msg,
    wait_for_initialization,
)


class MetadataServer:
    def __init__(self):
        self.use_ground_truth = rospy.get_param("~use_ground_truth", True)
        self.metadata_port = rospy.get_param("~metadata_port", 9007)
        self.vio_restart_service = rospy.get_param(
            "~vio_restart_service", rospy.get_param("~vio_restart")
        )

        self.metadata_gt_subscriber = (
            rospy.Subscriber("/metadata", String, self.metadata_callback),
        )
        self.nosy_pose_subscriber = rospy.Subscriber(
            "/kimera_vio_ros/odometry", Odometry, self.odometry_callback
        )

        self.data_source_service = rospy.Service(
            "use_ground_truth", SetBool, self.rosservice_change_data_source,
        )

        self.episode_reset_service = rospy.Service(
            "metadata_server_episode_reset", Trigger, self.episode_reset_service,
        )

        # store last received metadata
        self.data = TesseData()

        # initialize socket
        self.metadata_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.metadata_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.metadata_socket.settimeout(None)
        self.metadata_socket.bind(("", self.metadata_port))

    def rosservice_change_data_source(self, request):
        """ Change between ground truth and estimate data modes.

        Default provides ground truth image and position information
        from TESSE. Upon request, semantic segmentation,
        depth, and position estimates are given instead.

        Args:
            request (ChangeDataSource): ROS SetBool message.

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
        # TODO(ZR) add error checking for ground truth metadata
        self.data.metadata_noisy = metadata_from_odometry_msg(
            get_origin_odom_msg(), self.data.metadata_gt
        )

        return True, ""

    def odometry_callback(self, msg):
        """ Form metadata message containing pose estimate.

        Args:
            msg (Odometry): Odometry message containing a pose estimate.
        """
        try:
            self.data.metadata_noisy = metadata_from_odometry_msg(msg, self.data.metadata_gt)
        except Exception as ex:
            rospy.loginfo("Metadata server caught exception %s" % ex)
            self.data.metadata_noisy = ""

    def metadata_callback(self, metadata_msg):
        self.data.metadata_gt = metadata_msg.data

        # initialize noisy metadata at origin
        if self.data.metadata_noisy is None:
            self.data.metadata_noisy = metadata_from_odometry_msg(
                get_origin_odom_msg(), self.data.metadata_gt
            )

    def call_reset_episode_services(self):
        """ Call required services upon episode reset. 

        Calls
            - Kimera-VIO-ROS reset: Resets VIO state
            - tesse-gym-bridge episode reset: Resets the 
                pose estimate to origin
        """
        call_trigger_service(self.vio_restart_service)
        call_trigger_service("/tesse_gym_bridge/image_server_episode_reset")
        call_trigger_service("/tesse_gym_bridge/metadata_server_episode_reset")

    def on_shutdown(self):
        """ Close metadata server socket."""
        self.metadata_socket.close()

    def _form_metadata_respons(self):
        """ Create metadata response for client.

        Follows the TESSE network API.

        Returns:
            bytearry: Metadata response.
        """
        response = bytearray()
        response.extend("meta")
        metadata = self.data.metadata_gt if self.use_ground_truth else self.data.metadata_noisy
        response.extend(struct.pack("I", len(metadata)))
        response.extend(metadata)
        return response

    def _send_metadata_to_client(self, response):
        """ Send metadata message to client.

        Args:
            response (bytearra): Response to send to client.
        """
        try:
            metadata_send_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            metadata_send_socket.connect(("", self.metadata_port))
            metadata_send_socket.send(response)
        except socket.error as err:
            if err.errno != errno.ECONNREFUSED:
                raise err
            rospy.loginfo("Connection refused on metadata response")
        finally:
            metadata_send_socket.close()

    def spin(self):
        """ Receive client metadata request and response """

        # wait for required data to be initialized before setting up server
        vars_to_init = ("metadata_gt",)

        if not self.use_ground_truth:
            vars_to_init += ("metadata_noisy",)

        wait_for_initialization(self.data, vars_to_init)
        rospy.loginfo("Metadata server initialized")

        while not rospy.is_shutdown():
            message, address = self.metadata_socket.recvfrom(1024)

            if message == "rMET":
                response = self._form_metadata_respons()
                self._send_metadata_to_client(response)
            elif message == "sRES":
                self.call_reset_episode_services()
                rospy.loginfo("Setting new episode")
            else:
                rospy.logerror("Unknown tag: %s" % message)


if __name__ == "__main__":
    rospy.init_node("MetadataServer_node")
    node = MetadataServer()
    node.spin()
