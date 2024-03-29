#!/usr/bin/env python
from rpi_ws281x import PixelStrip, Color
import time
from hippocampus_common.node import Node
import threading
import rospy
from std_msgs.msg import Float64, Bool
from switch.msg import BatteryState

LED_COUNT = 4
LED_PIN = 10
LED_FREQ_HZ = 900000
LED_DMA = 10
LED_INVERT = False
LED_CHANNEL = 0


class Strip():
    STATUS_INDEX = 0
    ARMING_INDEX = 1
    BATTERY_INDEX = 2

    COLOR_ARMED = Color(254, 0, 0)
    COLOR_DISARMED = Color(0, 254, 0)

    COLOR_BATTERY_GOOD = Color(0, 254, 0)
    COLOR_BATTERY_MEDIUM = Color(254, 100, 0)
    COLOR_BATTERY_BAD = Color(254, 0, 0)

    COLOR_STATE_GOOD = Color(0, 254, 0)
    COLOR_STATE_BAD = Color(254, 0, 0)
    COLOR_OFF = Color(0, 0, 0)
    COLOR_UNDEFINED = Color(0, 0, 254)

    def __init__(self):
        self.lock = threading.RLock()
        self.strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA,
                                LED_INVERT, 255, LED_CHANNEL)
        self.strip.begin()
        self.armed = False
        self.t_arming_blinked = 0.0

    def blink_arming(self):
        with self.lock:
            if self.armed:
                self.strip.setPixelColorRGB(self.ARMING_INDEX, 254, 0, 0)
                self.strip.show()
                time.sleep(0.1)
                self.strip.setPixelColorRGB(self.ARMING_INDEX, 0, 0, 0)
                self.strip.show()

    def color_wipe(self, color: Color, duration: float):
        with self.lock:
            if duration > 0:
                delay_per_pixel = 1.0 / duration
            else:
                delay_per_pixel = 0
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, color)
                self.strip.show()
                time.sleep(delay_per_pixel)

    def switch_off(self):
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColorRGB(i, 0, 0, 0)
        self.strip.show()

    def set_status(self, good=False):
        with self.lock:
            self.strip.setPixelColorRGB(self.STATUS_INDEX,
                                        (1 - int(good)) * 254,
                                        int(good) * 254, 0)
            self.strip.show()

    def set_arming(
        self,
        time: float,
        armed=False,
    ):
        with self.lock:
            if armed != self.armed:
                self.t_arming_blinked = time
                self.armed = armed
                if armed:
                    self.just_armed()
                else:
                    self.just_disarmed()
            else:
                if not self.armed:
                    self.just_disarmed()
                if time - self.t_arming_blinked >= 1.0:
                    self.blink_arming()

    def set_arming_state_undefined(self):
        with self.lock:
            self.strip.setPixelColor(self.ARMING_INDEX, self.COLOR_UNDEFINED)

    def just_armed(self):
        with self.lock:
            self.blink_arming()

    def just_disarmed(self):
        with self.lock:
            self.strip.setPixelColor(self.ARMING_INDEX, self.COLOR_DISARMED)
            self.strip.show()

    def set_battery_good(self):
        with self.lock:
            self.strip.setPixelColor(self.BATTERY_INDEX,
                                     self.COLOR_BATTERY_GOOD)
            self.strip.show()

    def set_battery_medium(self):
        with self.lock:
            self.strip.setPixelColor(self.BATTERY_INDEX,
                                     self.COLOR_BATTERY_MEDIUM)
            self.strip.show()

    def set_battery_low(self):
        with self.lock:
            self.strip.setPixelColor(self.BATTERY_INDEX, self.COLOR_BATTERY_BAD)
            self.strip.show()

    def set_battery_undefined(self):
        with self.lock:
            self.strip.setPixelColor(self.BATTERY_INDEX, self.COLOR_UNDEFINED)
            self.strip.show()


class LedNode(Node):

    def __init__(self, name, anonymous=False, disable_signals=False):
        super().__init__(name, anonymous, disable_signals)
        self.lock = threading.RLock()
        self.strip = Strip()
        rospy.on_shutdown(self.on_shutdown)
        self.vehicle_namespace = self.get_param("~vehicle_name")
        if not self.vehicle_namespace:
            rospy.logfatal("No namespace provided. Shutting down!")
            rospy.signal_shutdown("No namespace provided.")
            exit(1)

        self.battery = dict(value=BatteryState.UNAVAILABLE,
                            stamp=0.0,
                            updated=False)
        self.arming = dict(value=False, stamp=0.0, updated=False)

        self.battery_sub = rospy.Subscriber("battery_state", BatteryState,
                                            self.on_battery_state)

        topic = f"/{self.vehicle_namespace}/arming_state"
        self.arming_state_sub = rospy.Subscriber(topic, Bool,
                                                 self.on_arming_state)

    def on_battery_state(self, msg: BatteryState):
        with self.lock:
            self.battery["value"] = msg.state
            self.battery["stamp"] = rospy.get_time()
            self.battery["updated"] = True

    def on_arming_state(self, msg: Bool):
        with self.lock:
            self.arming["value"] = msg.data
            self.arming["stamp"] = rospy.get_time()
            self.arming["updated"] = True

    def update_battery(self):
        if self.battery["value"] == BatteryState.GOOD:
            self.strip.set_battery_good()
        elif self.battery["value"] == BatteryState.MEDIUM:
            self.strip.set_battery_medium()
        elif self.battery["value"] == BatteryState.BAD:
            self.strip.set_battery_low()
        elif self.battery["value"] == BatteryState.UNSET:
            self.strip.set_battery_undefined()
            rospy.logwarn("BatteryState is UNSET")
        elif self.battery["value"] == BatteryState.UNAVAILABLE:
            self.strip.set_battery_undefined()
            rospy.logwarn("BatteryState is UNAVAILABLE")
        else:
            rospy.logwarn("Unhandled BatteryState: %d", self.battery["value"])

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            timed_out = False
            with self.lock:
                if self.battery["updated"]:
                    self.battery["updated"] = False
                    self.update_battery()
                else:
                    if now - self.battery["stamp"] > 5.0:
                        rospy.logwarn_throttle(2.0, "BatteryState timed out.")
                        self.strip.set_battery_undefined()
                        timed_out = True

                if self.arming["updated"]:
                    self.arming["updated"] = False
                    self.strip.set_arming(now, self.arming["value"])
                else:
                    if now - self.arming["stamp"] > 5.0:
                        rospy.logwarn_throttle(2.0, "ArmingState timed out.")
                        self.strip.set_arming_state_undefined()
                        timed_out = True
            self.strip.set_status(not timed_out)
            rate.sleep()

    def on_shutdown(self):
        print("Test")
        self.strip.switch_off()
        time.sleep(0.1)


def main():
    node = LedNode("led")
    node.run()


if __name__ == "__main__":
    main()
