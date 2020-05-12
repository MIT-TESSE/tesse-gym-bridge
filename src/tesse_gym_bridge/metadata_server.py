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
import shutil
import os

import rospy
from rospy.exceptions import ROSException
import rospkg
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tesse_gym_bridge.srv import DataSourceService
from tesse_gym_bridge.utils import (
    CollisionInfo,
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
        self.per_episode_kimera_log = rospy.get_param("~per_episode_kimera_log", True)

        self.metadata_gt_subscriber = (
            rospy.Subscriber("/metadata", String, self.metadata_callback),
        )
        self.nosy_pose_subscriber = rospy.Subscriber(
            "/kimera_vio_ros/odometry", Odometry, self.odometry_callback
        )

        self.data_source_service = rospy.Service(
            "data_source_request", DataSourceService, self.rosservice_change_data_source,
        )

        self.episode_reset_service = rospy.Service(
            "metadata_server_episode_reset", Trigger, self.episode_reset_service,
        )

        self.kimera_log_path = ""
        self.episode_count = 0
        if self.per_episode_kimera_log:
            try:
                self.kimera_log_path = rospkg.RosPack().get_path('kimera_vio_ros')
                self.kimera_log_path += '/output_logs/'
            except rospkg.common.ResourceNotFound:
                rospy.logerror("Kimera VIO is not installed, per episode vio logs will not be saved!")
                self.per_episode_kimera_log = False

        # store last received metadata
        self.data = TesseData()
        self.last_odom_msg = get_origin_odom_msg()

        # track collisions since last metadata query
        # TESSE only sends collision information once so that a
        # collision isn't recounted.
        self.last_collision_info = None

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
            request (DataSourceServie): ROS service data source.

        Returns:
            bool: True, indicating successful service call.
        """
        self.use_ground_truth = request.use_gt
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
        self.last_odom_msg = get_origin_odom_msg()
        if self.last_collision_info is not None:
            self.last_collision_info.set_collision_false()
        self.data.metadata_noisy = metadata_from_odometry_msg(
            self.last_odom_msg, self.last_collision_info
        )

        return True, ""

    def odometry_callback(self, msg):
        """ Form metadata message containing pose estimate.

        Args:
            msg (Odometry): Odometry message containing a pose estimate.
        """
        try:
            self.last_odom_msg = msg
            self.data.metadata_noisy = metadata_from_odometry_msg(
                self.last_odom_msg, self.last_collision_info
            )
        except Exception as ex:
            rospy.loginfo("Metadata server caught exception %s" % ex)

    def metadata_callback(self, metadata_msg):
        self.data.metadata_gt = metadata_msg.data

        if self.last_collision_info is None:
            self.last_collision_info = CollisionInfo(self.data.metadata_gt)
        else:
            self.last_collision_info.update_collision_info(self.data.metadata_gt)

        # initialize pose estimate at (x, y, yaw) = (0, 0, 0)
        if self.data.metadata_noisy is None:
            self.data.metadata_noisy = metadata_from_odometry_msg(
                get_origin_odom_msg(), self.last_collision_info
            )
        else:  # update noisy metadata with time and collision information 
            self.data.metadata_noisy = metadata_from_odometry_msg(
                self.last_odom_msg, self.last_collision_info
            )

    def call_reset_episode_services(self):
        """ Call required services upon episode reset. 

        Calls
            - Kimera-VIO-ROS reset: Resets VIO state
            - tesse-gym-bridge episode reset: Resets the 
                pose estimate to origin
        """
        try:
            call_trigger_service(self.vio_restart_service, timeout=1)

            if self.per_episode_kimera_log and self.episode_count > 0:
                episode_log_dir = self.kimera_log_path + "Tesse_%d" % self.episode_count
                
                # increment until we write to a new log dir
                while os.path.isdir(episode_log_dir):
                    self.episode_count += 1
                    episode_log_dir = self.kimera_log_path + "Tesse_episode_%d" % self.episode_count
                    rospy.loginfo("Incremented VIO logs episode count to %s" % episode_log_dir)

                rospy.loginfo("Moving current VIO logs to: %s" % episode_log_dir)
                shutil.copytree(self.kimera_log_path + "Tesse", 
                                episode_log_dir)
            
            self.episode_count += 1

        except ROSException:
            rospy.loginfo("Kimera VIO reset cannot be reached")
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
        self.last_collision_info.set_collision_false()

        response = bytearray()
        response.extend("meta")
        metadata = self.data.metadata_gt if self.use_ground_truth else self.data.metadata_noisy
        response.extend(struct.pack("I", len(metadata)))
        response.extend(metadata)
        return response

    def _send_metadata_to_client(self, response):
        """ Send metadata message to client.

        Args:
            response (bytearray): Response to send to client.
        """
        try:
            metadata_send_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            metadata_send_socket.connect(("", self.metadata_port))
            metadata_send_socket.send(response)
        except socket.error as err:
            if err.errno != errno.ECONNREFUSED:
                raise err
            rospy.logwarn("Connection refused on metadata response")
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
