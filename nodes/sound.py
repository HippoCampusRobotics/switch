#!/usr/bin/env python

import rospy
from hippocampus_common.node import Node
import pigpio
import time
from switch.msg import Button

BUZZER_PIN_DEFAULT = 18


class Buzzer():

    def __init__(self, pin):
        self.pin = pin
        self.pi = pigpio.pi()

    def blip(self):
        self.pi.write(self.pin, pigpio.HIGH)
        self.pi.write(self.pin, pigpio.LOW)

    def high_pitch(self):
        self.pi.set_PWM_frequency(self.pin, 4000)
        self.pi.set_PWM_dutycycle(self.pin, 128)

    def low_pitch(self):
        self.pi.set_PWM_frequency(self.pin, 2000)
        self.pi.set_PWM_dutycycle(self.pin, 128)

    def happy(self, time_per_tone: float):
        self.low_pitch()
        time.sleep(time_per_tone)
        self.high_pitch()
        time.sleep(time_per_tone)
        self.off()

    def sad(self, time_per_tone: float):
        self.high_pitch()
        time.sleep(time_per_tone)
        self.low_pitch()
        time.sleep(time_per_tone)
        self.off()

    def off(self):
        self.pi.write(self.pin, pigpio.LOW)


class SoundNode(Node):

    def __init__(self, name, anonymous=False, disable_signals=False):
        super().__init__(name, anonymous, disable_signals)
        self.pin = self.get_param("~gpio", BUZZER_PIN_DEFAULT)
        self.buzzer = Buzzer(self.pin)
        rospy.on_shutdown(self.on_shutdown)
        self.buzzer.happy(0.2)
        self.button_sub_ = rospy.Subscriber("button_pressed", Button, self.on_button_pressed)

    def on_button_pressed(self, _):
        self.buzzer.blip()

    def on_shutdown(self):
        self.buzzer.sad(0.2)


def main():
    node = SoundNode("sound")
    node.run()


if __name__ == "__main__":
    main()
