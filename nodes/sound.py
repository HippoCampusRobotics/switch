#!/usr/bin/env python

import rospy
from hippocampus_common.node import Node
import pigpio
import time
from switch.msg import Button, BatteryState
import threading

BUZZER_PIN_DEFAULT = 18


def synced(func):

    def wrap(s, *args):
        with s.lock:
            func(s, *args)

    return wrap


class Buzzer():

    def __init__(self, pin):
        self.pin = pin
        self.pi = pigpio.pi()
        self.lock = threading.RLock()

    @synced
    def blip(self):
        self.pi.write(self.pin, pigpio.HIGH)
        self.pi.write(self.pin, pigpio.LOW)

    @synced
    def high_pitch(self, duration=0.0):
        self.pi.set_PWM_frequency(self.pin, 4000)
        self.pi.set_PWM_dutycycle(self.pin, 128)
        if duration > 0:
            time.sleep(duration)
            self.off()

    @synced
    def low_pitch(self, duration=0.0):
        self.pi.set_PWM_frequency(self.pin, 2000)
        self.pi.set_PWM_dutycycle(self.pin, 128)
        if duration > 0:
            time.sleep(duration)
            self.off()

    @synced
    def happy(self, time_per_tone: float):
        self.low_pitch()
        time.sleep(time_per_tone)
        self.high_pitch()
        time.sleep(time_per_tone)
        self.off()

    @synced
    def double_low(self, length=0.1, delay=0.1):
        self.low_pitch(length)
        time.sleep(delay)
        self.low_pitch(length)

    @synced
    def sad(self, time_per_tone: float):
        self.high_pitch(time_per_tone)
        self.low_pitch(time_per_tone)

    @synced
    def off(self):
        self.pi.write(self.pin, pigpio.LOW)


class SoundNode(Node):

    def __init__(self, name, anonymous=False, disable_signals=False):
        super().__init__(name, anonymous, disable_signals)
        self.pin = self.get_param("~gpio", BUZZER_PIN_DEFAULT)
        self.buzzer = Buzzer(self.pin)
        rospy.on_shutdown(self.on_shutdown)
        self.buzzer.happy(0.2)
        self.low_battery = False
        self.button_sub_ = rospy.Subscriber("button_pressed", Button,
                                            self.on_button_pressed)
        self.battery_sub_ = rospy.Subscriber("battery_state", BatteryState,
                                             self.on_battery_state)

    def on_button_pressed(self, _):
        if not self.low_battery:
            self.buzzer.blip()

    def on_battery_state(self, msg: BatteryState):
        self.low_battery = msg.state == BatteryState.BAD

    def on_shutdown(self):
        self.buzzer.sad(0.2)

    def run(self):
        r = rospy.Rate(10)
        t_last = rospy.get_time()
        while not rospy.is_shutdown():
            r.sleep()
            now = rospy.get_time()
            if now - t_last > 2.0:
                if self.low_battery:
                    t_last = now
                    self.buzzer.low_pitch(0.5)


def main():
    node = SoundNode("sound")
    node.run()


if __name__ == "__main__":
    main()
