import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction
from ros2_aruco_interfaces.msg import ArucoMarkers


# ARuCoの追跡
class Test2(Node):

    def __init__(self):
        super().__init__('test2')
        self.declare_parameter("aruco_topic", "/aruco_markers")

        aruco_topic = self.get_parameter("aruco_topic").get_parameter_value().string_value

        self.cmd_vel_pub_ = self.create_publisher(Twist, "cmd_vel", 1)
        self.tello_client_ = self.create_client(TelloAction, "tello_action")

        self.info_sub = self.create_subscription(ArucoMarkers,
                                                 aruco_topic,
                                                 self.aruco_callback,
                                                 qos_profile_sensor_data)

        while not self.tello_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()

    
    def takeoff(self):
        self.req.cmd = "takeoff"
        self.future = self.tello_client_.call_async(self.req)
    
    def land(self):
        self.req.cmd = "land"
        self.future = self.tello_client_.call_async(self.req)

    def fly(self, cmd_vel):
        self.cmd_vel_pub_.publish(cmd_vel)

    def aruco_callback(self, aruco_markers):
        if len(aruco_markers.poses) == 0:
            return
        # 一つ目のマーカーの姿勢を取り出す
        # pose.position.x = tvecs[i][0][0]
        # pose.position.y = tvecs[i][0][1]
        # pose.position.z = tvecs[i][0][2]
        pose = aruco_markers.poses[0]

        twist_msg = Twist()
        twist_msg.linear.x = pose.linear.z - 1.0 # [m]
        twist_msg.linear.y = pose.linear.x # [m]
        twist_msg.linear.z = pose.linear.y # [m]
        twist_msg.angular.z = math.atan2(y, x) # [rad]

        self.fly(twist_msg)



def main(args=None):
    rclpy.init(args=args)

    test2 = Test2()

    # 離陸
    test2.get_logger().info('take off')
    test2.takeoff()
    while rclpy.ok():
        rclpy.spin_once(test2)
        if test2.future.done():
            try:
                response = test2.future.result()
            except Exception as e:
                test2.get_logger().info(
                    'Service call failed %r' % (e,))
            break
    
    # 10秒待つ
    time.sleep(10)

    while True:
        try:
            rclpy.spin_once(test2)
        except KeyboardInterrupt:
            break
    
    # 着陸
    test2.get_logger().info('land')
    test2.land()
    while rclpy.ok():
        rclpy.spin_once(test2)
        if test2.future.done():
            try:
                response = test2.future.result()
                print(response)
            except Exception as e:
                test2.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()