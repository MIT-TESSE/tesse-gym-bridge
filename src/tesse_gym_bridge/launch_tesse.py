#! /usr/bin/python

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

import rospy
import subprocess


class LaunchTesse:
    """ Node to launch TESSE via a subprocess """

    def __init__(self):
        tesse_path = rospy.get_param("~tesse_path")
        listen_port = rospy.get_param("~position_port", "9000")
        send_port = rospy.get_param("~send_port", "9000")
        width = rospy.get_param("~width", 320)
        height = rospy.get_param("~height", 240)

        self.proc = subprocess.Popen(
            [
                tesse_path,
                "--listen_port",
                listen_port,
                "--send_port",
                send_port,
                "--set_resolution",
                str(width),
                str(height),
            ]
        )

        rospy.on_shutdown(self.on_exit)
        rospy.spin()

    def on_exit(self):
        """ Kill subprocess upon exit"""
        self.proc.kill()
        rospy.loginfo("Killed TESSE subprocess")


if __name__ == "__main__":
    rospy.init_node("LaunchTesse_node")
    node = LaunchTesse()
