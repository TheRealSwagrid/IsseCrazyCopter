#!/usr/bin/env python
import math
import os
import socket
import time
from time import sleep

import numpy as np
import rospy
import tf
from AbstractVirtualCapability import VirtualCapabilityServer
from tf.transformations import quaternion_about_axis

from CrazyFly import CrazyFly
from visualization_msgs.msg import Marker
from tf import TransformListener
from pycrazyswarm.crazyflie import CrazyflieServer, Crazyflie


class CrazyFly_Ros_interface:

    def __init__(self):
        self.target = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        self.rotation = [0., 0., 0., 1.]
        self.scale = .1
        self.arming_status = False
        self.id = int(rospy.get_param('~cf_id'))
        self.transformListener = TransformListener()
        self.name = f"CrazyFly#{int(rospy.get_param('~cf_id'))}@{int(rospy.get_param('~semantix_port'))}"
        self.pub = rospy.Publisher("/robot", Marker, queue_size=1)
        self.br = tf.TransformBroadcaster()
        for crazyflie in rospy.get_param("/crazyflies"):
            rospy.logwarn(crazyflie["id"])
            rospy.logwarn(str(self.id) + "\n")
            if int(crazyflie["id"]) == self.id:
                self.cf = Crazyflie(self.id, crazyflie["initialPosition"], self.transformListener)
                break

    def set_rotation(self, quaternion: list):
        self.rotation = quaternion

    def get_rotation(self):
        return self.rotation

    def set_name(self, name):
        self.name = name

    def get_name(self):
        return self.name

    def rotate(self, axis, deg):
        axis = np.array(axis)
        theta = np.deg2rad(deg)
        self.rotation = list(quaternion_about_axis(theta, axis))
        return self.rotation

    def fly_to(self, p: list):
        # self.cf.setLEDColor(1., 1., 0.)
        self.cf.goTo(p, 0, 5)
        timer = time.time()
        rospy.logerr(f"Flying to {p}")
        pos = self.get_position()
        dist = math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2 + (p[2] - pos[2]) ** 2)
        while dist > 0.1:
            pos = self.get_position()
            dist = math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2 + (p[2] - pos[2]) ** 2)
            sleep(.01)
        rospy.logerr(f"Arrived at {p} after {time.time() - timer} seconds")
        # self.cf.setLEDColor(0., 1., 0.)

    def arm(self):
        self.arming_status = True
        # self.cf.setLEDColor(1., 1., 1.)
        self.cf.takeoff(1.0, 3.0)
        rospy.sleep(3)
        sleep(3)

    def disarm(self):
        self.arming_status = False
        # self.cf.setLEDColor(1., 0., 1.)
        self.cf.land(0., 3.0)
        rospy.sleep(3)
        sleep(3)

    def get_position(self):
        return self.cf.position().tolist()

    def get_arming_status(self):
        return self.arming_status

    def select_cf(self, cf_id: int):
        for crazyflie in rospy.get_param("/crazyflies"):
            rospy.logwarn(crazyflie["id"])
            rospy.logwarn(str(self.id) + "\n")
            if int(crazyflie["id"]) == self.id:
                self.cf = Crazyflie(self.id, crazyflie["initialPosition"], self.transformListener)
                break

    def hover(self):
        self.cf.takeoff(1.0, 3.0)
        rospy.sleep(3)
        self.cf.land(0., 3.0)
        rospy.sleep(3)

    def change_led(self, red, green, blue):
        self.cf.setLEDColor(red / 255., green / 255., blue / 255.)

    def publish_visual(self):
        # rospy.logwarn(f"Publishing {self.position}")
        marker = Marker()
        marker.id = int(rospy.get_param('~semantix_port'))
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"crazyfly"
        marker.lifetime = rospy.Duration(0)
        # marker.color.r = .1
        # marker.color.g = .15
        # marker.color.b = .3
        self.position = self.get_position()
        marker.mesh_use_embedded_materials = True
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = self.rotation[0]
        marker.pose.orientation.y = self.rotation[1]
        marker.pose.orientation.z = self.rotation[2]
        marker.pose.orientation.w = self.rotation[3]
        # Scale down
        marker.scale.x = self.scale
        marker.scale.y = self.scale
        marker.scale.z = self.scale
        marker.color.a = 1
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = r"package://isse_crazy/meshes/copter.dae"

        self.pub.publish(marker)

        # TF
        self.br.sendTransform(self.position,
                              self.rotation, rospy.Time.now(), self.name, "world")


if __name__ == '__main__':
    rospy.init_node('rosnode', xmlrpc_port=int(os.environ["xmlrpc_port"]), tcpros_port=int(os.environ["tcpros_port"]))
    rate = rospy.Rate(30)

    rospy.logwarn("Starting CrazyFly ROS")
    drone = CrazyFly_Ros_interface()

    rospy.logwarn("Starting server")

    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')), socket.gethostbyname(socket.gethostname()))

    rospy.logwarn("starting isse_copter semanticplugandplay")
    copter = CrazyFly(server)

    copter.functionality["arm"] = drone.arm
    copter.functionality["disarm"] = drone.disarm
    copter.functionality["setNeoPixelColor"] = drone.change_led
    copter.functionality["get_arming"] = lambda: drone.arming_status

    copter.functionality["set_pos"] = drone.fly_to
    copter.functionality["get_pos"] = drone.get_position

    copter.functionality["set_name"] = drone.set_name
    copter.functionality["get_name"] = drone.get_name

    copter.functionality["get_rot"] = drone.get_rotation
    copter.functionality["set_rot"] = drone.set_rotation
    copter.functionality["rotate"] = drone.rotate

    copter.start()
    # signal.signal(signal.SIGTERM, handler)

    drone.publish_visual()
    while not rospy.is_shutdown():
        drone.publish_visual()
        drone.br.sendTransform(drone.position,
                               np.array(drone.rotation), rospy.Time.now(), drone.name, "world")

        rate.sleep()
        # rospy.logwarn(f"Server status: {server.running}, {copter}")