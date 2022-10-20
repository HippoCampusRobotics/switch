#!/usr/bin/env python
import rospy
from hippocampus_common.node import Node
from std_msgs.msg import Float64
from switch.msg import BatteryState


class BatteryWatcherNode(Node):
    V_CELL_MEDIUM = 3.55
    V_CELL_LOW = 3.3

    def __init__(self, name, anonymous=False, disable_signals=False):
        super().__init__(name, anonymous, disable_signals)
        self.n_cells = self.get_param("~n_cells")
        self.vehicle_name = self.get_param("~vehicle_name")
        if not self.n_cells or not self.vehicle_name:
            rospy.logfatal("Required params not provided.")
            rospy.signal_shutdown("Required params not provided.")
            exit(1)
        topic_name = f"/{self.vehicle_name}/battery_voltage"
        self.state_pub = rospy.Publisher("battery_state",
                                         BatteryState,
                                         queue_size=10)
        self.battery_sub = rospy.Subscriber(topic_name, Float64,
                                            self.on_battery_voltage)

    def on_battery_voltage(self, msg: Float64):
        voltage = msg.data
        out = BatteryState()
        if voltage > self.V_CELL_MEDIUM * self.n_cells:
            out.state = BatteryState.GOOD
        elif voltage > self.V_CELL_LOW * self.n_cells:
            out.state = BatteryState.MEDIUM
        else:
            out.state = BatteryState.BAD
        self.state_pub.publish(out)


def main():
    node = BatteryWatcherNode("battery_watcher")
    node.run()


if __name__ == "__main__":
    main()
