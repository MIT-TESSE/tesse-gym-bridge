#! /usr/bin/python

import subprocess
import time

import rospy


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
