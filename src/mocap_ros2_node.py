import rclpy
import asyncio
from multiprocessing import shared_memory
import threading
from time import sleep

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
# from my_custom_msgs.msg import MyCustomMessage
from .mocap2robot_src.HelpFuction import is_active
from .mocap2robot_src.transformations import quaternion_from_euler, euler_from_matrix, quaternion_from_matrix
from geometry_msgs.msg import TransformStamped

from .mocap2robot_src.Mocap2Robot_quest3Controller2Qinlong import Quest3Ctrller2Qinlong

# from .mocap2robot_src.webcam.webcam import main_simple


# from my_custom_msgs.srv import MocapCalibrate, MocapCalibrateResponse
# from loong_msgs.msg import VpControlData
from loong_msgs.msg import VpControl
# from loong_msgs.msg import VrPose

# from loong_msgs.msg import NeckControl

# import vrremote

# import .mocap2robot_src

# print(vrremote.__file__)

from .mocap2robot_src.common.config import get_config


def create_tf_mat(tf, parent, child):
    tf_msg = TransformStamped()

    # Set the frame IDs
    tf_msg.header.frame_id = parent  # Parent frame
    tf_msg.child_frame_id = child  # Child frame

    q = quaternion_from_matrix(tf)

    # Set the initial position (x, y, z)
    tf_msg.transform.translation.x = tf[0, -1]
    tf_msg.transform.translation.y = tf[1, -1]
    tf_msg.transform.translation.z = tf[2, -1]

    # Set the initial orientation (roll, pitch, yaw)

    tf_msg.transform.rotation.x = q[0]
    tf_msg.transform.rotation.y = q[1]
    tf_msg.transform.rotation.z = q[2]
    tf_msg.transform.rotation.w = q[3]

    tf_msg.header.stamp = rclpy.Time.now()

    return tf_msg




class VRMocapManagerNode(Node):
    def __init__(self):
        super().__init__('mocap_manager_node')

        # start vuer website
        self.data_provider = Quest3Ctrller2Qinlong(
            get_config("udp")["host"],
            get_config("udp")["port"],
            get_config("broadcast")["host"],
            get_config("broadcast")["port"],
        )

        self.mocap2robot = self.data_provider

        self.vp_control_pub = self.create_publisher(VpControl, '/vp_control', 10)
        # self.vr_pose_pub = self.create_publisher(VrPose, '/VrPose', 10)
        # self.neck_ctrl_pub = self.create_publisher(NeckControl, '/NeckControl', 10)
        
        self.msg = VpControl()
        self.str_timer = self.create_timer(0.060, self.run)  


    def run(self):


        # print("start main loop")

        # while is_active():

        data = self.data_provider.get_data()
        if data is not None:
            zyx_left_robot, left_wrist_t, zyx_right_robot, right_wrist_t, joint_hand_left, joint_hand_right,head_pose,left_hand_pose,right_hand_pose = self.mocap2robot.process(data)


            

            # vr_pose_msg=VrPose()
            # vr_pose_msg.pos_head=[*head_pose]
            # vr_pose_msg.pos_left_hand=[*left_hand_pose]
            # vr_pose_msg.pos_right_hand=[*right_hand_pose]
            
            # self.vr_pose_pub.publish(vr_pose_msg)


            zyx_rad=(head_pose[:3])

            pan=  zyx_rad[0]
            tilt=zyx_rad[1]

            # neck_ctrl_msg=NeckControl()
            # neck_ctrl_msg.neck_q=[pan,tilt]
            # self.neck_ctrl_pub.publish(neck_ctrl_msg)

            
            self.msg.filt_level =1
            self.msg.arm_mode =4
            self.msg.finger_mode =3
            self.msg.neck_mode =3

            self.msg.arx_pos_left = [*left_wrist_t]
            self.msg.arx_rot_left=[zyx_left_robot[0], zyx_left_robot[1], zyx_left_robot[2]]


            self.msg.arx_pos_right = [*right_wrist_t]
            self.msg.arx_rot_right=[ zyx_right_robot[0], zyx_right_robot[1], zyx_right_robot[2]]


            # msg.vcap = [joint_hand_left[2], joint_hand_right[2]]
            # print(joint_hand_left)
            self.msg.hand_q_left = [*joint_hand_left]
            self.msg.hand_q_right = [*joint_hand_right]
            self.msg.neck_q=[pan,tilt]

        self.vp_control_pub.publish(self.msg )


def main(args=None):
    rclpy.init(args=args)
    node = VRMocapManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
