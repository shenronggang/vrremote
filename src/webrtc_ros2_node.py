#!/usr/bin/env python
import threading
from time import sleep

import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image
from .mocap2robot_src.webcam import webcam
from .mocap2robot_src.webcam.webcam import main_simple
from .mocap2robot_src.common.config import get_config
from rclpy.executors import MultiThreadedExecutor



class WebrtcROS2Node(Node):
    def __init__(self):
        super().__init__('WebrtcROS2Node')
        self.bridge = CvBridge()
        self.img_subscription = self.create_subscription(Image, '/femto_cam/color/image_raw', self.image_callback, 10)

        self.img_publisher_ = self.create_publisher(Image, 'camera/image', 10)

        self.frame=np.zeros((720,1280,3),dtype=np.uint8)

        #self.img_timer = self.create_timer(1, self.publish_image)  # Publish at 10 Hz

        # thread1=threading.Thread(target=self.publish_image)
        # thread1.start()

        # main_simple()


    def publish_image(self):
        
        # while rclpy.ok():
        # Capture a frame from the camera
        self.frame+=1
        # Convert the frame to a ROS Image message and publish it
        print("send frame")
        msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        self.img_publisher_.publish(msg)
        # self.get_logger().info('Published image frame')
        # sleep(1)


    def image_callback(self,msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            webcam.frame_buffer[:] = cv_image[:]
            #print("ros cb",webcam.frame_buffer.shape)

            # self.data_provider.set_image(cv_image)

            # Display the image using OpenCV
            # cv2.imshow("Image window", cv_image)
            # cv2.waitKey(1)  # Refresh display at 1ms interval
        except CvBridgeError as e:
            print(e)
            rclpy.logerr(e)


def ros_task():
    node = WebrtcROS2Node()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    thread1=threading.Thread(target=ros_task)
    thread1.start()

    # ros_task()
    
    main_simple() # aiohttp app

    #sleep(3)

    rclpy.shutdown()  # Stop the node from the main thread
    thread1.join()


if __name__ == '__main__':
    main()

    
