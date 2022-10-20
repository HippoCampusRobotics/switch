#!/usr/bin/env python
import rospy
from hippocampus_common.node import Node
import pigpio
from switch.msg import Button
from typing import List
import os


class ButtonNode(Node):

    def __init__(self, name, anonymous=False, disable_signals=False):
        super().__init__(name, anonymous, disable_signals)
        self.button_pub_ = rospy.Publisher("button_pressed",
                                           Button,
                                           queue_size=10)
        self.gpios: List = self.get_param("~gpios")
        self.gpio_state: int = 0

        if self.gpios is None:
            rospy.logerr(
                "Could not read gpio config from param server. Exiting.")
            exit(1)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("Could not connect to gpiod. Exiting.")
            exit(1)

        self.times = dict()
        for gpio in self.gpios:
            self.times[gpio] = rospy.get_time()
            self.pi.set_pull_up_down(gpio, pigpio.PUD_UP)
            self.pi.callback(gpio, pigpio.FALLING_EDGE, self.on_falling_edge)

    def on_falling_edge(self, gpio, *_):
        now = rospy.get_time()
        if now - self.times[gpio] < 1.0:
            return
        self.times[gpio] = now
        try:
            index = self.gpios.index(gpio)
        except ValueError:
            rospy.logerr(
                "Button at GPIO %d pressed, but not in list of known GPIOs.",
                gpio)
            return
        msg = Button()
        msg.header.frame_id = os.path.basename(rospy.get_name())
        msg.button = index
        self.button_pub_.publish(msg)


def main():
    node = ButtonNode(name="button")
    node.run()


if __name__ == "__main__":
    main()