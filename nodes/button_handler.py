#!/usr/bin/env python
import rospy
from switch.msg import Button
from hippocampus_common.node import Node
from std_srvs.srv import SetBool


class ButtonHandlerNode(Node):
    ARM_BUTTON = 0
    DISARM_BUTTON = 1

    def __init__(self, name, anonymous=False, disable_signals=False):
        super().__init__(name, anonymous, disable_signals)
        self.button_sub_ = rospy.Subscriber("button_pressed", Button,
                                            self.on_button_pressed)
        self.vehicle_name = self.get_param("~vehicle_name")
        if not self.vehicle_name:
            rospy.signal_shutdown("No vehicle name provided.")
            exit(1)
        self.arm_name = f"/{self.vehicle_name}/arm"
        rospy.loginfo("Waiting for service [%s]", self.arm_name)
        rospy.wait_for_service(self.arm_name)
        rospy.loginfo("Service available [%s]", self.arm_name)

    def arm_vehicle(self, arm: bool):
        try:
            arm = rospy.ServiceProxy(self.arm_name, SetBool)
            response = arm(True)
            return response
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call arming service: %s", e)
            return None

    def on_button_pressed(self, msg: Button):
        if msg.button == self.ARM_BUTTON:
            self.arm_vehicle(True)
        elif msg.button == self.DISARM_BUTTON:
            self.arm_vehicle(False)
        else:
            rospy.logwinfo("Unhandled button pressed")


def main():
    node = ButtonHandlerNode("button_handler")
    node.run()


if __name__ == "__main__":
    main()