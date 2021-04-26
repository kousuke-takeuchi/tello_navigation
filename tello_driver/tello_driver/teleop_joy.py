import os, sys
import time
import threading

import tellopy
import av
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
import camera_info_manager as cim

from sensor_msgs.msg import Image, Imu, CameraInfo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tello_msgs.msg import TelloState
from tello_msgs.srv import TelloAction


class TelloDriver(Node):
    # skip first 300 frames
    frame_skip = 300
    vel_cmd_scale = 10

    def __init__(self):
        super().__init__('tello_driver')

        self.declare_parameter("camera_calibration", "tello_driver/config/960x720.yaml")
        self.declare_parameter("camera_frame", "camera_front")

        calib_path = self.get_parameter("camera_calibration").get_parameter_value().string_value
        camera_frame = self.get_parameter("camera_front").get_parameter_value().string_value
        
        self.drone = tellopy.Tello()
        self.drone.subscribe(self.drone.EVENT_LOG_DATA, self.flight_data_handler)

        self.drone.connect()
        self.drone.wait_for_connection(60.0)

        # self.container = av.open(self.drone.get_video_stream())
        
        self._cmd_vel_sub = self.create_subscription(Twist,
                                                    "cmd_vel",
                                                    self.cmd_vel_callback,
                                                    qos_profile_sensor_data)
        self.bridge = CvBridge()
        self._image_pub = self.create_publisher(Image, "image_raw", 10)
        self._imu_pub = self.create_publisher(Imu, "imu", 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 1)
        self._state_pub = self.create_publisher(TelloState, "tello/state", 1)
        self._action_srv = self.create_service(TelloAction, 'tello/action', self.action_callback)

        self.caminfo = cim.loadCalibrationFile(calib_path, camera_frame)
        self.caminfo.header.frame_id = camera_frame
        self._caminfo_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=1, latch=True)                
        self._caminfo_pub.publish(self.caminfo)

        self.frame_thread = threading.Thread(target=self.framegrabber_loop)
        self.frame_thread.start()

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def framegrabber_loop(self):
        for frame in self.container.decode(video=0):
            if 0 < self.frame_skip:
                self.frame_skip = self.frame_skip - 1
                continue
            image = self.bridge.cv2_to_imgmsg(np.array(frame.to_image()), "bgr8")
            image.header.frame_id = self.caminfo.header.frame_id
            self._image_pub.publish(image)
            self._caminfo_pub.publish(self.caminfo)   

        # import av #Import here as 'hack' to prevent troublesome install of PyAV when not used
        # # Repeatedly try to connect
        # vs = self.drone.get_video_stream()
        # while self.drone.state != self.drone.STATE_QUIT:
        #     try:
        #         container = av.open(vs)
        #         break
        #     except BaseException as err:
        #         print('fgrab: pyav stream failed - %s' % str(err))
        #         time.sleep(1.0)

        # # Once connected, process frames till drone/stream closes
        # while self.drone.state != self.drone.STATE_QUIT:
        #     # vs blocks, dies on self.stop
        #     for frame in container.decode(video=0):
        #         img = np.array(frame.to_image())
        #         img_msg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
        #         # try:
        #         #     img_msg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
        #         #     img_msg.header.frame_id = self.caminfo.header.frame_id
        #         # except CvBridgeError as err:
        #         #     rospy.logerr('fgrab: cv bridge failed - %s' % str(err))
        #         #     continue
        #         self.pub_image_raw.publish(img_msg)
        #         # self.pub_caminfo.publish(self.caminfo)  
       
    def flight_data_handler(self, event, sender, data, **args):
        current_time = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'base_link'

        # Height from MVO received as negative distance to floor
        odom_msg.pose.pose.position.z = -data.mvo.pos_z #self.height #-data.mvo.pos_z
        odom_msg.pose.pose.position.x = data.mvo.pos_x
        odom_msg.pose.pose.position.y = data.mvo.pos_y
        odom_msg.pose.pose.orientation.w = data.imu.q0
        odom_msg.pose.pose.orientation.x = data.imu.q1
        odom_msg.pose.pose.orientation.y = data.imu.q2
        odom_msg.pose.pose.orientation.z = data.imu.q3
        # Linear speeds from MVO received in dm/sec
        odom_msg.twist.twist.linear.x = data.mvo.vel_y/10
        odom_msg.twist.twist.linear.y = data.mvo.vel_x/10
        odom_msg.twist.twist.linear.z = -data.mvo.vel_z/10
        odom_msg.twist.twist.angular.x = data.imu.gyro_x
        odom_msg.twist.twist.angular.y = data.imu.gyro_y
        odom_msg.twist.twist.angular.z = data.imu.gyro_z
                
        self._odom_pub.publish(odom_msg)
        
        imu_msg = Imu()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.header.stamp = current_time
        
        imu_msg.orientation.w = data.imu.q0
        imu_msg.orientation.x = data.imu.q1
        imu_msg.orientation.y = data.imu.q2
        imu_msg.orientation.z = data.imu.q3        
        imu_msg.angular_velocity.x = data.imu.gyro_x
        imu_msg.angular_velocity.y = data.imu.gyro_y
        imu_msg.angular_velocity.z = data.imu.gyro_z
        imu_msg.linear_acceleration.x = data.imu.acc_x
        imu_msg.linear_acceleration.y = data.imu.acc_y
        imu_msg.linear_acceleration.z = data.imu.acc_z
        
        self._imu_pub.publish(imu_msg)
        
    def __scale_vel_cmd(self, cmd_val):
        return self.vel_cmd_scale * cmd_val

    def cmd_vel_callback(self, cmd_vel):
        self.drone.set_pitch( self.drone.__scale_vel_cmd(cmd_vel.linear.y) )
        self.drone.set_roll( self.drone.__scale_vel_cmd(cmd_vel.linear.x) )
        self.drone.set_yaw( self.drone.__scale_vel_cmd(cmd_vel.angular.z) )
        self.drone.set_throttle( self.drone.__scale_vel_cmd(cmd_vel.linear.z) )
        
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
        elif command.cmd == 'throw_and_go':
            self.drone.throw_and_go()
            response.rc = response.OK
        elif command.cmd == 'land':
            self.drone.land()
            response.rc = response.OK
        elif command.cmd == 'flip_forward':
            self.drone.flip_forward()
            response.rc = response.OK
        elif command.cmd == 'flip_back':
            self.drone.flip_back()
            response.rc = response.OK
        elif command.cmd == 'flip_left':
            self.drone.flip_left()
            response.rc = response.OK
        elif command.cmd == 'flip_right':
            self.drone.flip_right()
            response.rc = response.OK
        else:
            response.rc = response.ERROR_UNKNOWN
        return response

    def timer_callback(self):
        pass
        # self.get_image_stream()

    # def get_image_stream(self):
    #     for frame in self.container.decode(video=0):
    #         if 0 < self.frame_skip:
    #             self.frame_skip = self.frame_skip - 1
    #             continue
    #         image = self.bridge.cv2_to_imgmsg(np.array(frame.to_image()), "bgr8")
    #         self._image_pub.publish(image)

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