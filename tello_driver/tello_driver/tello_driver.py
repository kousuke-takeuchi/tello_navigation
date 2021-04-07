import os, sys
import time

import tellopy
import av
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tello_msgs.msg import TelloState
from tello_msgs.srv import TelloAction


class TelloDriver(Node):
    # skip first 300 frames
    frame_skip = 300

    def __init__(self):
        super().__init__('tello_driver')
        
        self.drone = tellopy.Tello()
        self.drone.subscribe(self.drone.EVENT_FLIGHT_DATA, self.flight_data_handler)

        self.drone.connect()
        self.drone.wait_for_connection(60.0)

        # self.container = av.open(self.drone.get_video_stream())
        
        self.cmd_vel_sub_ = self.create_subscription(Twist,
                                                    "cmd_vel",
                                                    self.cmd_vel_callback,
                                                    qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.image_pub_ = self.create_publisher(Image, "image_raw", 1)
        self.state_pub_ = self.create_publisher(TelloState, "tello/state", 1)
        self.action_srv_ = self.create_service(TelloAction, 'tello/action', self.action_callback)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
       
    def flight_data_handler(self, event, sender, data, **args):
        if event is self.drone.EVENT_FLIGHT_DATA:
            pass

    def cmd_vel_callback(self, cmd_vel):
        self.robot.go(robot_dir*FWD_SPEED,robot_rot*ROT_SPEED)
        # self.cmd_vel_pub_.publish(cmd_vel)

    def action_callback(self, command, response):
        # Arbitrary Tello command:
        # string cmd
        # ---
        # # Initial response code:
        # uint8 OK=1                    # Command sent
        # uint8 ERROR_NOT_CONNECTED=2   # Can't communicate with drone
        # uint8 ERROR_BUSY=3            # There's already an active command
        # uint8 rc
        # response.rc = -1
        if command.cmd == 'takeoff':
            self.drone.takeoff()
            response.rc = response.OK
        elif command.cmd == 'land':
            self.drone.land()
            response.rc = response.OK
        elif command.cmd == 'flip':
            self.drone.clockwise()
            response.rc = response.OK
        return response

    def timer_callback(self):
        pass
        # self.get_image_stream()

    def get_image_stream(self):
        print("hello")
        print(self.frame_skip)
        for frame in self.container.decode(video=0):
            if 0 < self.frame_skip:
                self.frame_skip = self.frame_skip - 1
                continue
            image = self.bridge.cv2_to_imgmsg(np.array(frame.to_image()), "bgr8")
            self.image_pub_.publish(image)

    def on_destroy(self):
        self.drone.quit()


def main(args=None):
    rclpy.init(args=args)

    tello_driver = TelloDriver()

    while rclpy.ok():
        rclpy.spin_once(tello_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tello_driver.on_destroy()
    tello_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()