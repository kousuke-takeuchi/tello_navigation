import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction


# 上下左右前後に0.5mずつ移動
class Test1(Node):

    def __init__(self):
        super().__init__('test1')
        self.cmd_vel_pub_ = self.create_publisher(Twist, "cmd_vel", 1)
        self.tello_client_ = self.create_client(TelloAction, "tello_action")

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


def main(args=None):
    rclpy.init(args=args)

    test1 = Test1()

    # 離陸
    test1.get_logger().info('take off')
    test1.takeoff()
    while rclpy.ok():
        rclpy.spin_once(test1)
        if test1.future.done():
            try:
                response = test1.future.result()
            except Exception as e:
                test1.get_logger().info(
                    'Service call failed %r' % (e,))
            break
    
    # 10秒待つ
    time.sleep(10)

    # 上に1m移動
    test1.get_logger().info('up')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.2 # [m/s]
    test1.fly(twist_msg)
    time.sleep(5)
    
    # 静止
    test1.get_logger().info('stop')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(1)

    # 下に1m移動
    test1.get_logger().info('down')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = -0.2 # [m/s]
    test1.fly(twist_msg)
    time.sleep(5)
    
    # 静止
    test1.get_logger().info('stop')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(1)

    # 右に1m移動
    test1.get_logger().info('right')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.2 # [m/s]
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(5)
    
    # 静止
    test1.get_logger().info('stop')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(1)

    # 左に1m移動
    test1.get_logger().info('left')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = -0.2 # [m/s]
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(5)
    
    # 静止
    test1.get_logger().info('stop')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(1)

    # 前に1m移動
    test1.get_logger().info('front')
    twist_msg = Twist()
    twist_msg.linear.x = 0.2 # [m/s]
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(5)
    
    # 静止
    test1.get_logger().info('stop')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(1)

    # 後に1m移動
    test1.get_logger().info('back')
    twist_msg = Twist()
    twist_msg.linear.x = -0.2 # [m/s]
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(5)
    
    # 静止
    test1.get_logger().info('stop')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(1)

    # 右回転
    test1.get_logger().info('rot+')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.z = 0.4
    test1.fly(twist_msg)
    time.sleep(5)

    # 後に1m移動
    test1.get_logger().info('rot-')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.z = -0.4
    test1.fly(twist_msg)
    time.sleep(5)
    
    # 静止
    test1.get_logger().info('stop')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    test1.fly(twist_msg)
    time.sleep(1)
    
    # 着陸
    test1.get_logger().info('land')
    test1.land()
    while rclpy.ok():
        rclpy.spin_once(test1)
        if test1.future.done():
            try:
                response = test1.future.result()
                print(response)
            except Exception as e:
                test1.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()