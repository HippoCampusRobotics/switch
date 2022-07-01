#!/usr/bin/env python

import rospy
from hippocampus_common.node import Node
from mavros_msgs.srv import CommandLong, CommandBool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import pigpio


class SwitchNode(Node):
    def __init__(self, name, anonymous=False, disable_signals=False):
        super(SwitchNode, self).__init__(name,
                                         anonymous=anonymous,
                                         disable_signals=disable_signals)

        self.gpios = self.get_param("~gpios")
        self.namespaces = self.get_param("~namespaces")
        tmp = self.get_param("~gpio_map")
        self.gpio_map = dict()
        for key in tmp:
            self.gpio_map[int(key)] = tmp[key]

        if self.gpios is None:
            rospy.logerr(
                "Could not read gpio config from param server. Exiting...")
            exit(0)
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("Could not connect to pigpiod. Exiting...")
            exit(0)
        self.times = dict()
        self.distance_control_flag = False
        self.services = self.init_services()
        for gpio in self.gpios:
            self.times[gpio] = rospy.get_time()
            self.pi.set_pull_up_down(gpio, pigpio.PUD_UP)
            self.pi.callback(gpio, pigpio.FALLING_EDGE, self.on_falling_edge)

    def init_services(self):
        s = dict()
        s["arm"] = {
            0:
            rospy.ServiceProxy(
                "{}/mavros/cmd/arming".format(self.namespaces[0]), CommandBool),
            1:
            rospy.ServiceProxy(
                "{}/mavros/cmd/arming".format(self.namespaces[1]), CommandBool)
        }
        s["command_long"] = {
            0:
            rospy.ServiceProxy(
                "{}/mavros/cmd/command".format(self.namespaces[0]),
                CommandLong),
            1:
            rospy.ServiceProxy(
                "{}/mavros/cmd/command".format(self.namespaces[1]), CommandLong)
        }
        s["enable_distance_control"] = {
            0:
            rospy.ServiceProxy(
                "{}/enable_distance_control".format(self.namespaces[0]),
                SetBool),
            1:
            rospy.ServiceProxy(
                "{}/enable_distance_control".format(self.namespaces[1]),
                SetBool),
        }
        return s

    def on_falling_edge(self, gpio, *_):
        now = rospy.get_time()
        if now - self.times[gpio] < 1.0:
            return
        self.times[gpio] = now
        name = self.gpio_map[gpio]
        rospy.logwarn("{} pressed".format(self.gpio_map[gpio]))
        if name == "arm0":
            self.call_arm(0)
        elif name == "disarm0":
            self.call_disarm(0)
        elif name == "reboot0":
            self.call_reboot(0)
        elif name == "arm1":
            self.call_arm(1)
        elif name == "disarm1":
            self.call_disarm(1)
        elif name == "reboot1":
            self.call_reboot(1)
        elif name == "toggle_distance_control0":
            self.call_toggle_distance_control(0)
        elif name == "toggle_distance_control1":
            self.call_toggle_distance_control(1)
        else:
            rospy.logwarn("Unknown button pressed.")

    def call_toggle_distance_control(self, index):
        try:
            self.services["enable_distance_control"][index](
                self.distance_control_flag)
        except rospy.ServiceException:
            rospy.logerr("Failed to set distance_control to '{}'".format(
                self.distance_control_flag))
        else:
            self.distance_control_flag = not self.distance_control_flag

    def call_disarm(self, index):
        try:
            self.services["command_long"][index](command=400,
                                                 param1=0,
                                                 param2=21196)
        except rospy.ServiceException:
            rospy.logerr("Failed to call disarm service.")

    def call_arm(self, index):
        try:
            r = self.services["command_long"][index](command=400, param1=1, param2=21196)
        except rospy.ServiceException:
            rospy.logerr("Failed to call arm service.")
        else:
            rospy.logwarn(r)

    def call_reboot(self, index):
        try:
            self.services["command_long"][index](command=246,
                                                 confirmation=0,
                                                 param1=1)
        except rospy.ServiceException:
            rospy.logerr("Failed to call reboot service")


def main():
    node = SwitchNode("switch")
    node.run()


if __name__ == "__main__":
    main()
