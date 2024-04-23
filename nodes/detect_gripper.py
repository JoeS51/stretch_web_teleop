#! /usr/bin/env python3

import cv2
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import threading
from rclpy.executors import MultiThreadedExecutor
import os
# - 0.49236512252811776 open

# - 0.01482612063635088 close



class GripperCloseDetectorNode(Node):
    def __init__(self):
        super().__init__('gripper_close_detector_node')

        # creating subscriber to joint states
        self.subscription = self.create_subscription(JointState, '/joint_states', self.closed_gripper_callback, 15)
        self.subscription

        # creating publisher for when the gripper is closed
        self.publisher = self.create_publisher(Bool, '/gripper_closed', 15)

        # queue that stores the last 20 values of the gripper state
        self.queue = []
        self.idx = 0
    
    def closed_gripper_callback(self, msg):
        if self.idx == 20:
            self.idx = 0
            self.queue.append(msg.position[12])
            if len(self.queue) == 20:
                self.queue.pop(0)
                self.check_gripper_state()
        self.idx += 1
    
    def check_gripper_state(self):
        print(self.queue[0] - self.queue[-1])
        # checking if the gripper is closed
        if self.queue[0] - self.queue[-1] > 0 and self.queue[-1] == self.queue[-2]:
            msg = Bool()
            msg.data = True
            self.publisher.publish(msg)
        else:
            msg = Bool()
            msg.data = False
            self.publisher.publish(msg)

        

def main(args=None):
    rclpy.init(args=args)

    gripper_close_detector_node = GripperCloseDetectorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(gripper_close_detector_node)
    try:
        rclpy.spin(gripper_close_detector_node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        gripper_close_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
