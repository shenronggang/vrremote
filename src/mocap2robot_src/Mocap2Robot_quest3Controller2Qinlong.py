from datetime import datetime
from enum import Enum
from multiprocessing import shared_memory
import socket
import struct
import threading
from collections import deque

# from .net_utils import UdpIkSender
from .loong_mani_sdk_udp import maniSdkCtrlDataClass, maniSdkClass, maniSdkSensDataClass


try:

    import rclpy
    from .transformations import quaternion_from_matrix, quaternion_from_euler, euler_from_matrix, quaternion_matrix, \
        quaternion_from_matrix, \
        euler_matrix
    from .Mocap2Robot import Mocap2Robot
    from .log_utils import print1

    from .HelpFuction import matrix3d_to_euler_angles_zyx, xyz_quaternion_to_homogeneous, rpy2rotation_matrix, \
    rotation_matrix_to_rpy, \
    find_axis_angle, calc_dist, rpy2rotation_matrix, fast_mat_inv, is_active, log

    print("rospy successfully imported.")
except ImportError as e:

    print(e.msg)
    from transformations import quaternion_from_matrix, quaternion_from_euler, euler_from_matrix, quaternion_matrix, \
        quaternion_from_matrix, \
        euler_matrix
    from Mocap2Robot import Mocap2Robot
    from log_utils import print1

    from HelpFuction import matrix3d_to_euler_angles_zyx, xyz_quaternion_to_homogeneous, rpy2rotation_matrix, \
        rotation_matrix_to_rpy, \
        find_axis_angle, calc_dist, rpy2rotation_matrix, fast_mat_inv,is_active

import numpy as np
import struct

def rotx(theta):
    theta = np.radians(theta) 
    Rx = np.array([[1, 0, 0],
               [0, np.cos(theta), -np.sin(theta)],
               [0, np.sin(theta), np.cos(theta)]])
    return Rx
    
def roty(theta):
    theta = np.radians(theta) 
    Ry = np.array([[np.cos(theta), 0, np.sin(theta)],
               [0, 1, 0],
               [-np.sin(theta), 0, np.cos(theta)]])
    return Ry
    
def rotz(theta):
    theta = np.radians(theta)
    Rz = np.array([[np.cos(theta), -np.sin(theta), 0],
               [np.sin(theta), np.cos(theta), 0],
               [0, 0, 1]])
    return Rz


def lim_angle(angle):
    if angle < 0:
        angle = 0
    if angle > 90:
        angle = 90

    return int(angle)


class Quest3ControllerDataProcess:
    def __init__(self):
        self.ee_init_value_l = np.array([0.0, 0.0, 0.0, 0.400, 0.300, -0.060, 0.0])
        self.ee_init_value_r = np.array([0.0, 0.0, 0.0, 0.400, -0.300, -0.060, 0.0])

        # self.ee_init_value_l = np.array([1.44961, -0.186147, 2.61344, -0.164, 0.310, 0.168, 1])
        # self.ee_init_value_r = np.array([1.84076, 0.1608, -0.114557, -0.250, -0.1500, 0.150, -1])

        self.ee_init_rt_l = np.eye(4)
        self.ee_init_rt_l = euler_matrix(*self.ee_init_value_l[:3], 'szyx')
        self.ee_init_rt_l[:3, -1] = self.ee_init_value_l[3:6]

        self.ee_init_rt_r = np.eye(4)
        self.ee_init_rt_r = euler_matrix(*self.ee_init_value_r[:3], 'szyx')
        self.ee_init_rt_r[:3, -1] = self.ee_init_value_r[3:6]

        self.ee_cur_rt_l = self.ee_init_rt_l.copy()
        self.ee_cur_rt_r = self.ee_init_rt_r.copy()

        self.ee_last_rt_l = self.ee_init_rt_l.copy()
        self.ee_last_rt_r = self.ee_init_rt_r.copy()

    def handle_raw_data(self, data_bytes):
        '''
        if l == 4 * 48 * 7:
                       ...
                       fun(data_bytes)
        '''
        float_array = struct.unpack(f'{48 * 7}f', data_bytes)
        xyzqwqxqyqz = np.array(float_array).reshape((48, 7))
        return xyzqwqxqyqz

    def process(self, xyzqwqxqyqz):
        # global ee_cur_rt_l, ee_cur_rt_r, ee_last_rt_l, ee_last_rt_r

        # if save_once:
        #     np.save('/home/jyw/posedata.npy',xyzqwqxqyqz)
        #     save_once=False
       
        cmd = xyzqwqxqyqz[1, :].copy()
        cmd2 = xyzqwqxqyqz[2, :].copy()
        cmd3 = xyzqwqxqyqz[3, :].copy()
        zoom=cmd2[0]
        print(cmd)
        
        # print(cmd)
        # 0 left grasp
        # 1 right grasp
        # 2 left move
        # 3 right move
        # 4 left reset
        # 5 right reset
        if cmd[2] == 0:
            self.ee_cur_rt_l = self.ee_last_rt_l.copy()
        if cmd[3] == 0:
            self.ee_cur_rt_r = self.ee_last_rt_r.copy()

        if cmd[4] == 1:
            self.ee_cur_rt_l = self.ee_init_rt_l.copy()
            self.ee_last_rt_l = self.ee_init_rt_l.copy()
        if cmd[5] == 1:
            self.ee_cur_rt_r = self.ee_init_rt_r.copy()
            self.ee_last_rt_r = self.ee_init_rt_r.copy()

        left_grasp = cmd[0] * 50
        right_grasp = cmd[1] * 50

        # unity left frame to right frame
        xyzqwqxqyqz[:, 3] *= -1

        xyzqwqxqyqz = xyzqwqxqyqz[:, [0, 2, 1, 3, 4, 6, 5]]

        xyz = xyzqwqxqyqz[:, :3]
        # qwqxqyqz=xyzqwqxqyqz[:,3:]
        qxqyqzqw = xyzqwqxqyqz[:, [4, 5, 6, 3]]

        rt_list = []
        cmd_list = [1, 2, 3]
        for i in range(48):
            if i in cmd_list:
                rt_list.append(np.eye(4))
            else:
                rt_base_quest_2_part_quest = quaternion_matrix(qxqyqzqw[i, :])
                rt_base_quest_2_part_quest[:3, -1] = xyz[i, :]
                rt_list.append(rt_base_quest_2_part_quest)

        rt_l_hand_quest_2_l_hand_robot = np.eye(4)
        # rt_l_hand_quest_2_l_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(180),0,np.deg2rad(0))
        rt_l_hand_quest_2_l_hand_robot[:3, :3] = rpy2rotation_matrix(0, np.deg2rad(-90), 0)

        rt_r_hand_quest_2_r_hand_robot = np.eye(4)
        # rt_r_hand_quest_2_r_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(90),0,np.deg2rad(180))
        # rt_r_hand_quest_2_r_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(0),np.deg2rad(180),0)
        rt_r_hand_quest_2_r_hand_robot[:3, :3] = rpy2rotation_matrix(np.deg2rad(180), np.deg2rad(-90), 0)

        if cmd[2] > 0:
            rt_l_ee1_quest_2_l_ee2_quest = rt_list[0]
            ee_new_rt_l = np.eye(4)
            delta_ee_l = fast_mat_inv(
                rt_l_hand_quest_2_l_hand_robot) @ rt_l_ee1_quest_2_l_ee2_quest @ rt_l_hand_quest_2_l_hand_robot

            # result correct but method not clear
            ee_new_rt_l[:3, -1] = self.ee_cur_rt_l[:3, -1] + delta_ee_l[:3, -1]
            ee_new_rt_l[:3, :3] = delta_ee_l[:3, :3] @ self.ee_cur_rt_l[:3, :3]

            # update last
            self.ee_last_rt_l = ee_new_rt_l.copy()

        # print(cmd[3])
        if cmd[3] > 0:
            rt_r_ee1_quest_2_r_ee2_quest = rt_list[24 + 0]
            # print(rt_r_ee1_quest_2_r_ee2_quest)
            ee_new_rt_r = np.eye(4)
            delta_ee_r = fast_mat_inv(
                rt_r_hand_quest_2_r_hand_robot) @ rt_r_ee1_quest_2_r_ee2_quest @ rt_r_hand_quest_2_r_hand_robot
            ee_new_rt_r[:3, -1] = self.ee_cur_rt_r[:3, -1] + delta_ee_r[:3, -1]
            ee_new_rt_r[:3, :3] = delta_ee_r[:3, :3] @ self.ee_cur_rt_r[:3, :3]
            # update last
            self.ee_last_rt_r = ee_new_rt_r.copy()

        # print(ee_new_rt_l[:3,-1].flatten(),delta_ee_l[:3,-1].flatten())
        # print(ee_new_rt_r[:3,-1].flatten(),delta_ee_r[:3,-1].flatten())

        # here we use last result

        homo_waist_shoulder_left=np.array([
            [0,1,0,0.004],
            [0,0,1,0.1616],
            [1,0,0,0.3922],
            [0,0,0,1]
            ])

        homo_waist_shoulder_right=np.array([
            [0,-1,0,0.004],
            [0,0,-1,-0.1616],
            [1,0,0,0.3922],
            [0,0,0,1]
            ])
        
        
        # homo_waist_shoulder_left=np.array([
        #     [1,0,0,0.004],
        #     [0,1,0,0.1616],
        #     [0,0,1,0.3922],
        #     [0,0,0,1]
        #     ])

        # homo_waist_shoulder_right=np.array([
        #     [1,0,0,0.004],
        #     [0,1,0,-0.1616],
        #     [0,0,1,0.3922],
        #     [0,0,0,1]
        #     ])

        homo_waist_hand_left=homo_waist_shoulder_left @ self.ee_last_rt_l
        home_waist_hand_right=homo_waist_shoulder_right@ self.ee_last_rt_r


        # homo_waist_hand_left[:3,:3]=homo_waist_hand_left[:3,:3]@ rotx(-90)




        zyx_left_robot = matrix3d_to_euler_angles_zyx(homo_waist_hand_left)
        zyx_right_robot = matrix3d_to_euler_angles_zyx(home_waist_hand_right)

        left_wrist_t = homo_waist_hand_left[:3, -1] #* 1000
        right_wrist_t = home_waist_hand_right[:3, -1] #* 1000
        # print("left_theta_rad",self.left_theta_rad)
        # print(left_data,left_wrist_t)
        # print([zyx_left_robot,
        #         left_wrist_t,
        #         zyx_right_robot,
        #         right_wrist_t,
        #         left_grasp,0,0,0,0,0,
        #         right_grasp,0,0,0,0,0,])

        idxRightHandDelta = 24
        idxLeftHand = 25
        idxRightHand = 26
        idxHead = 27

        head_pose = np.zeros(6)
        left_hand_pose = np.zeros(6)
        right_hand_pose = np.zeros(6)

        head_pose[3:] = rt_list[idxHead][:3, -1]
        left_hand_pose[3:] = rt_list[idxLeftHand][:3, -1]
        right_hand_pose[3:] = rt_list[idxRightHand][:3, -1]

        head_pose[:3] = matrix3d_to_euler_angles_zyx(rt_list[idxHead])
        left_hand_pose[:3] = matrix3d_to_euler_angles_zyx(rt_list[idxLeftHand])
        right_hand_pose[:3] = matrix3d_to_euler_angles_zyx(rt_list[idxRightHand])

        # print(xyzqwqxqyqz[idxHead,:])
        

        return [zyx_left_robot,
                left_wrist_t,
                zyx_right_robot,
                right_wrist_t,
                [80., left_grasp, left_grasp, left_grasp, left_grasp, left_grasp],
                [80., right_grasp, right_grasp, right_grasp, right_grasp, right_grasp],
                head_pose,
                left_hand_pose,
                right_hand_pose]


class Quest3ControllerDataProcess2(Quest3ControllerDataProcess):
    def __init__(self):
        
        # self.ee_init_value_r = np.array([0, 0, 0.0, -0.282, 0.217, 0.58, -0.5233])
        # rz ry rx x y z
        self.ee_init_value_l = np.array([0.0, 0.0, 0.0, 0.400, 0.300, -0.060, 0.0])
        self.ee_init_value_r = np.array([0.0, 0.0, 0.0, 0.400, -0.300, -0.060, 0.0])
        self.ee_init_rt_l = np.eye(4)
        self.ee_init_rt_l = euler_matrix(*self.ee_init_value_l[:3], 'szyx')
        self.ee_init_rt_l[:3, -1] = self.ee_init_value_l[3:6]

        self.ee_init_rt_r = np.eye(4)
        self.ee_init_rt_r = euler_matrix(*self.ee_init_value_r[:3], 'szyx')
        self.ee_init_rt_r[:3, -1] = self.ee_init_value_r[3:6]

        self.ee_cur_rt_l = self.ee_init_rt_l.copy()
        self.ee_cur_rt_r = self.ee_init_rt_r.copy()

        self.ee_last_rt_l = self.ee_init_rt_l.copy()
        self.ee_last_rt_r = self.ee_init_rt_r.copy()

        # self.sdk=maniSdkClass("0.0.0.0", 8003, 19, 6)

        # self.net = UdpIkSender("192.168.1.155",8003)

    def handle_raw_data(self, data_bytes):
        '''
        if l == 4 * 48 * 7:
                       ...
                       fun(data_bytes)
        '''
        float_array = struct.unpack(f'{48 * 7}f', data_bytes)
        xyzqwqxqyqz = np.array(float_array).reshape((48, 7))
        return xyzqwqxqyqz

    def process(self, xyzqwqxqyqz):
        # global ee_cur_rt_l, ee_cur_rt_r, ee_last_rt_l, ee_last_rt_r

        # if save_once:
        #     np.save('/home/jyw/posedata.npy',xyzqwqxqyqz)
        #     save_once=False

        cmd = xyzqwqxqyqz[1, :].copy()
        cmd2 = xyzqwqxqyqz[2, :].copy()
        cmd3 = xyzqwqxqyqz[3, :].copy()
        # print(cmd)
        # 0 left grasp
        # 1 right grasp
        # 2 left move
        # 3 right move
        # 4 left reset
        # 5 right reset
        if cmd[2] == 0:
            self.ee_cur_rt_l = self.ee_last_rt_l.copy()
        if cmd[3] == 0:
            self.ee_cur_rt_r = self.ee_last_rt_r.copy()

        if cmd[4] == 1:
            self.ee_cur_rt_l = self.ee_init_rt_l.copy()
            self.ee_last_rt_l = self.ee_init_rt_l.copy()
        if cmd[5] == 1:
            self.ee_cur_rt_r = self.ee_init_rt_r.copy()
            self.ee_last_rt_r = self.ee_init_rt_r.copy()

        left_grasp = cmd[0] * 80
        right_grasp = cmd2[2] * 80

        # unity left frame to right frame
        xyzqwqxqyqz[:, 1] *= -1
        # xyzqwqxqyqz[:, 5] *= -1
        xyzqwqxqyqz[:, 4] *= -1
        xyzqwqxqyqz[:, 6] *= -1

        # xyzqwqxqyqz = xyzqwqxqyqz[:, [0, 2, 1, 3, 4, 6, 5]]

        xyz = xyzqwqxqyqz[:, :3]
        # qwqxqyqz=xyzqwqxqyqz[:,3:]
        qxqyqzqw = xyzqwqxqyqz[:, [4, 5, 6, 3]]

        rt_list = []
        cmd_list = [1, 2, 3]
        for i in range(48):
            if i in cmd_list:
                rt_list.append(np.eye(4))
            else:
                rt_base_quest_2_part_quest = quaternion_matrix(qxqyqzqw[i, :])
                rt_base_quest_2_part_quest[:3, -1] = xyz[i, :]
                rt_list.append(rt_base_quest_2_part_quest)

        rt_l_hand_quest_2_l_hand_robot = np.eye(4)
        # rt_l_hand_quest_2_l_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(180),0,np.deg2rad(0))
        rt_l_hand_quest_2_l_hand_robot[:3, :3] = rpy2rotation_matrix(0, np.deg2rad(-90),np.deg2rad(90))

        rt_r_hand_quest_2_r_hand_robot = np.eye(4)
        # rt_r_hand_quest_2_r_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(90),0,np.deg2rad(180))
        # rt_r_hand_quest_2_r_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(0),np.deg2rad(180),0)
        rt_r_hand_quest_2_r_hand_robot[:3, :3] = rpy2rotation_matrix(0, np.deg2rad(-90),np.deg2rad(90))

        if cmd[2] > 0:
            rt_l_ee1_quest_2_l_ee2_quest = rt_list[0]
            ee_new_rt_l = np.eye(4)
            delta_ee_l = fast_mat_inv(
                rt_l_hand_quest_2_l_hand_robot) @ rt_l_ee1_quest_2_l_ee2_quest @ rt_l_hand_quest_2_l_hand_robot

            # result correct but method not clear
            ee_new_rt_l[:3, -1] = self.ee_cur_rt_l[:3, -1] + delta_ee_l[:3, -1]
            ee_new_rt_l[:3, :3] = delta_ee_l[:3, :3] @ self.ee_cur_rt_l[:3, :3]

            idxLeftHand = 25

            print(delta_ee_l[:3, -1], ee_new_rt_l[:, -1], rt_list[idxLeftHand][:3, -1])

            # update last
            self.ee_last_rt_l = ee_new_rt_l.copy()

        # print(cmd[3])
        if cmd[3] > 0:
            rt_r_ee1_quest_2_r_ee2_quest = rt_list[24 + 0]
            # print(rt_r_ee1_quest_2_r_ee2_quest)
            ee_new_rt_r = np.eye(4)
            delta_ee_r = fast_mat_inv(
                rt_r_hand_quest_2_r_hand_robot) @ rt_r_ee1_quest_2_r_ee2_quest @ rt_r_hand_quest_2_r_hand_robot


            ee_new_rt_r[:3, -1] = self.ee_cur_rt_r[:3, -1] + delta_ee_r[:3, -1]
            ee_new_rt_r[:3, :3] = delta_ee_r[:3, :3] @ self.ee_cur_rt_r[:3, :3]

            print(delta_ee_r[:3, -1],ee_new_rt_r[:,-1])

            idxRightHand = 26

            right_hand_pose = np.zeros(6)
            right_hand_pose[3:] = rt_list[idxRightHand][:3, -1]
            print(delta_ee_r[:3, -1], ee_new_rt_r[:, -1],rt_list[idxRightHand][:3, -1])

            # update last
            self.ee_last_rt_r = ee_new_rt_r.copy()

        # print(ee_new_rt_l[:3,-1].flatten(),delta_ee_l[:3,-1].flatten())
        # print(ee_new_rt_r[:3,-1].flatten(),delta_ee_r[:3,-1].flatten())

        # here we use last result

        # zyx_left_robot = matrix3d_to_euler_angles_zyx(self.ee_last_rt_l)
        # zyx_right_robot = matrix3d_to_euler_angles_zyx(self.ee_last_rt_r)
        zyx_left_robot=euler_from_matrix(self.ee_last_rt_l,"rzyx")
        zyx_right_robot=euler_from_matrix(self.ee_last_rt_r,"rzyx")

        left_wrist_t = self.ee_last_rt_l[:3, -1]
        right_wrist_t = self.ee_last_rt_r[:3, -1]

        left_wrist_q = quaternion_from_matrix(self.ee_last_rt_l)[[3, 0, 1, 2]]  # xyzw

        right_wrist_q = quaternion_from_matrix(self.ee_last_rt_r)[[3, 0, 1, 2]]

        idxRightHandDelta = 24
        idxLeftHand = 25
        idxRightHand = 26
        idxHead = 27

        head_pose = np.zeros(6)
        left_hand_pose = np.zeros(6)
        right_hand_pose = np.zeros(6)

        head_pose[3:] = rt_list[idxHead][:3, -1]
        left_hand_pose[3:] = rt_list[idxLeftHand][:3, -1]
        right_hand_pose[3:] = rt_list[idxRightHand][:3, -1]

        head_pose[:3] = matrix3d_to_euler_angles_zyx(rt_list[idxHead])
        left_hand_pose[:3] = matrix3d_to_euler_angles_zyx(rt_list[idxLeftHand])
        right_hand_pose[:3] = matrix3d_to_euler_angles_zyx(rt_list[idxRightHand])

        # print(xyzqwqxqyqz[idxHead,:])

        # return [left_wrist_q,
        #         left_wrist_t,
        #         right_wrist_q,
        #         right_wrist_t,
        #         (80, left_grasp, left_grasp, left_grasp, left_grasp, left_grasp),
        #         (80, right_grasp, right_grasp, right_grasp, right_grasp, right_grasp),
        #         0,
        #         0,
        #         0]

        # ctrl=maniSdkCtrlDataClass()
        

        # ctrl.inCharge  =1
        # ctrl.filtLevel =4
        # ctrl.armMode   =4
        # ctrl.fingerMode=0
        # ctrl.neckMode  =5
        # ctrl.lumbarMode=0
        # ctrl.armCmd   =np.array([[0.4, 0.4, 0.3,
        #                             -np.pi/2,-np.pi/2,0,   
        #                             #  0,0,0,
        #                             np.pi/4],
        #                         [0.4,-0.4, 0.3,   0,0,0,   np.pi/4]],np.float32)
        # ctrl.armFM    =np.zeros((2,6),np.float32)
        # ctrl.fingerCmd=np.zeros((2,6),np.float32)
        # ctrl.neckCmd  =np.zeros(2,np.float32)
        # ctrl.lumbarCmd=np.zeros(3,np.float32)

        # ctrl.armCmd[0][0:3]=left_wrist_t
        # ctrl.armCmd[0][3:6]=np.array(zyx_left_robot)[[2,1,0]]

        # ctrl.armCmd[1][0:3]=right_wrist_t
        # ctrl.armCmd[1][3:6]=np.array(zyx_right_robot)[[2,1,0]]

        # self.sdk.send(ctrl)
        # sens=self.sdk.recv()

        return [zyx_left_robot,
            left_wrist_t,
            zyx_right_robot,
            right_wrist_t,
            [left_grasp, left_grasp, left_grasp, left_grasp, left_grasp, left_grasp],
            [right_grasp, right_grasp, right_grasp, right_grasp, right_grasp, right_grasp],
            head_pose,
            left_hand_pose,
            right_hand_pose]


class Quest3Ctrller2Qinlong(Mocap2Robot):
    def __init__(self, local_udp_ip, local_udp_port, broadcast_ip, broadcast_port):
        super().__init__()

        self.local_udp_ip = local_udp_ip
        self.local_udp_port = local_udp_port
        self.broadcast_ip = broadcast_ip
        self.broadcast_port = broadcast_port
        # start one thread to receive data

        # resolution = (720, 1280)
        # crop_size_w = 340  # (resolution[1] - resolution[0]) // 2
        # crop_size_h = 270
        # resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)  # 450 * 600
        # self.img_shape = (2 * resolution_cropped[0], resolution_cropped[1], 3)  # 900 * 600
        # img_height, img_width = resolution_cropped[:2]  # 450 * 600
        #
        # try:
        #     rospy.loginfo("try close shared memory image_shared_memory")
        #     self.shm = shared_memory.SharedMemory(name='image_shared_memory')
        #     self.shm.close()
        #     self.shm.unlink()
        # except Exception as e:
        #     rospy.loginfo(e)
        #     rospy.loginfo("if image_shared_memory not found, it's ok because we need to create one")
        #
        # self.shm = shared_memory.SharedMemory(name='image_shared_memory', create=True,
        #                                       size=np.prod(self.img_shape) * np.uint8().itemsize)
        #
        # self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)

        self.data_process = Quest3ControllerDataProcess2()
        self.shared_resource_lock = threading.Lock()

        # Define a deque with a maximum size of 2
        self.data_queue = deque(maxlen=2)

        t1 = threading.Thread(target=self.receive_data)
        t1.start()

    def receive_data(self):
        # server_ip = "0.0.0.0"  # IP address to listen on
        # server_port = 5015  # Port to listen on
        BUFFER_SIZE = 8888  # Buffer size for incoming messages

        # Create UDP socket
        # broadcast_ip = '255.255.255.255'
        # broadcast_ip = '192.168.1.110'
        # port = 4032

        # Broadcast a message
        message = b"Hello, devices!"
        # Create a UDP socket
        max_retries = 5  # 设定一个最大重试次数
        retry_count = 0

        # 创建新的套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        sock.bind(('', self.local_udp_port))
        sock.settimeout(1.0)  # Set timeout for recvfrom (in seconds)

        # 初始化并启动接收循环
        # log(f"Listening for broadcasts on port {self.local_udp_port}...")

        class ConnectState(Enum):
            CONNECT_STATE = 1
            DISCONNECT_STATE = 2
            ERROR_STATE = 3

        connect_state = ConnectState.DISCONNECT_STATE
        while is_active():
            try:
                # 如果达到重试次数上限，重新发送广播消息
                if connect_state == ConnectState.DISCONNECT_STATE:
                    log("重新广播消息以重新连接...")
                    sock.sendto(message, (self.broadcast_ip, self.broadcast_port))
                    retry_count = 0
                    # print("Broadcast message sent!")
                    data, addr = sock.recvfrom(4024)  # Buffer size is 1024 bytes
                    log(f"Received message: {data} from {addr}")
                    connect_state = ConnectState.CONNECT_STATE
                else:
                    data_bytes, addr = sock.recvfrom(BUFFER_SIZE)

                    data_length = len(data_bytes)
                    if data_length == 4 * 48 * 7:
                        with self.shared_resource_lock:
                            self.data_queue.append(self.data_process.handle_raw_data(data_bytes))

                retry_count = 0  # 成功接收数据后重置重试计数
            except socket.timeout:
                # Get the current time with milliseconds
                current_time = datetime.now()

                # Format and print the time with milliseconds
                log("no data")
                retry_count += 1
                # 如果达到重试次数上限，重新发送广播消息
                if retry_count >= max_retries:
                    connect_state = ConnectState.DISCONNECT_STATE

                continue
            except socket.error:
                print("socket.error")
                break

        sock.close()
        print("UDP receiver stopped")

        print("sub thread exit")

    def get_data(self):
        """
        retrive data
        :return:
        """
        data_out = None
        with self.shared_resource_lock:
            if self.data_queue:
                data_out = self.data_queue.popleft()  # Get the oldest item

                # print(f"Consumed: {item}, Queue: {list(data_queue)}")
            # else:
            #     print("Queue is empty, waiting for data...")
        # self.shared_resource_lock.acquire()
        # data_out= self.xyzqwqxqyqz.copy()
        # self.shared_resource_lock.release()
        return data_out

    def process(self, xyzqwqxqyqz):
        return self.data_process.process(xyzqwqxqyqz)

    def set_image(self, frame):
        # shared memory way
        image = cv2.resize(frame, (self.img_shape[1], self.img_shape[0]))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # cv2.imshow("canvas",frame)
        # cv2.waitKey(30)

        np.copyto(self.img_array, image)

        return 0


import asyncio
import cv2


async def periodic_task():
    data_provider = Quest3Ctrller2Qinlong()
    cap = cv2.VideoCapture(0)
    p_min, p_max = -2000, 2000
    # t  -900 900
    t_min, t_max = -300, 950

    pan_filtered = 0
    tilt_filtered = 0
    t = 0

    while True:
        # Your task code here
        # print("Task executed.")
        data = data_provider.get_data()

        ret, frame = cap.read()

        if not ret:
            break

        cv2.imshow("frame", frame)
        cv2.waitKey(20)

        if data is not None:
            zyx_left_robot, left_wrist_t, zyx_right_robot, right_wrist_t, joint_hand_left, joint_hand_right, head_pose, left_hand_pose, right_hand_pose = data_provider.process(
                data)

            # zyx=euler_from_matrix(head_pose,'szyx')
            zyx_degree = np.rad2deg(head_pose[:3])

            pan = 90 * np.sin(t)  #zyx_degree[0]
            t = t + 0.02
            tilt = zyx_degree[2]
            k = 0.3
            pan_filtered = ((1 - k) * pan_filtered + k * pan)
            tilt_filtered = ((1 - k) * tilt_filtered + k * tilt)
            print(pan_filtered, tilt_filtered)

            pan_final = np.clip(int(pan_filtered * 10), p_min, p_max)
            tilt_final = np.clip(int(tilt_filtered * 10), t_min, t_max)
            print(pan_final, tilt_final)
            res = cap.set(cv2.CAP_PROP_PAN, pan_final)
            # res = cap.set(cv2.CAP_PROP_TILT, tilt_final)
            print(zyx_degree)

        # Wait for 30 milliseconds
        await asyncio.sleep(0.02)


async def periodic_task2():
    def calculate_checksum(data1, data2, data3, data4):
        # 计算校验和 sum = AA + data1 + data2 + data3 + data4
        return (0xAA + data1 + data2 + data3 + data4) & 0xFF  # 确保是1字节

    def send_data_to_gimbal(serial_port, servo1, servo2):
        # 分别获取舵机1和舵机2的数据高8位和低8位
        data1 = (servo1 >> 8) & 0xFF  # 舵机1 高8位
        data2 = servo1 & 0xFF  # 舵机1 低8位
        data3 = (servo2 >> 8) & 0xFF  # 舵机2 高8位
        data4 = servo2 & 0xFF  # 舵机2 低8位

        # 计算校验和
        checksum = calculate_checksum(data1, data2, data3, data4)

        # 构建发送的数据包
        data_packet = [0xAA, data1, data2, data3, data4, checksum]

        # 发送数据包
        serial_port.write(bytearray(data_packet))
        # serial_port.write(b'\r')
        # serial_port.write(b'\n')
        print(f"Sent: {data_packet}")

    import serial.tools.list_ports

    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(port.device)

    # 打开串口，设置波特率为9600
    ser = serial.Serial('COM10', baudrate=9600, timeout=1)

    delta1 = 50
    delta2 = 50
    t = 0
    dt = 0.02
    # 循环发送数据，周期为10-20ms

    servo1_position = 1500  # 900 - 2100
    servo2_position = 1200  # 600-1800

    data_provider = Quest3Ctrller2Qinlong()

    p_min, p_max = -2000, 2000
    # t  -900 900
    t_min, t_max = -300, 950

    pan_filtered = 0
    tilt_filtered = 0
    t = 0

    while True:
        # Your task code here
        # print("Task executed.")
        data = data_provider.get_data()

        if data is not None:
            zyx_left_robot, left_wrist_t, zyx_right_robot, right_wrist_t, joint_hand_left, joint_hand_right, head_pose, left_hand_pose, right_hand_pose = data_provider.process(
                data)

            # zyx=euler_from_matrix(head_pose,'szyx')
            zyx_degree = np.rad2deg(head_pose[:3])

            pan = zyx_degree[0]
            t = t + 0.02
            tilt = zyx_degree[2]
            k = 0.3
            pan_filtered = ((1 - k) * pan_filtered + k * pan)
            tilt_filtered = ((1 - k) * tilt_filtered + k * tilt)
            print(pan_filtered, tilt_filtered)

            pan_final = np.clip(int(pan_filtered * 10), p_min, p_max)
            tilt_final = np.clip(int(tilt_filtered * 10), t_min, t_max)
            print(pan_final, tilt_final)

            # res = cap.set(cv2.CAP_PROP_TILT, tilt_final)
            # print(zyx_degree)

            send_data_to_gimbal(ser, int(1500 + np.clip(pan_final, -300, 300)),
                                int(1200 + np.clip(tilt_final, -300, 300)))

        # Wait for 30 milliseconds
        await asyncio.sleep(0.02)


async def periodic_task3():
    delta1 = 50
    delta2 = 50
    t = 0
    dt = 0.02
    # 循环发送数据，周期为10-20ms

    servo1_position = 1500  # 900 - 2100
    servo2_position = 1200  # 600-1800

    data_provider = Quest3Ctrller2Qinlong()

    p_min, p_max = -2000, 2000
    # t  -900 900
    t_min, t_max = -300, 950

    pan_filtered = 0
    tilt_filtered = 0
    t = 0

    # net = UdpIkSender("192.168.113.172",8010)
    net = UdpIkSender("192.168.1.155",8003)

    while True:
        # Your task code here
        # print("Task executed.")
        data = data_provider.get_data()

        if data is not None:
            zyx_left_robot, left_wrist_t, zyx_right_robot, right_wrist_t, joint_hand_left, joint_hand_right, head_pose, left_hand_pose, right_hand_pose = data_provider.process(
                data)

            # zyx=euler_from_matrix(head_pose,'szyx')
            zyx_degree = np.rad2deg(head_pose[:3])

            pan = zyx_degree[0]
            tilt = zyx_degree[2]
            # print(pan,tilt)

            '''
                struct Motion_Data_Recieve
                {
                    float left_arm_rz;/* data */
                    float left_arm_ry;
                    float left_arm_rx;
                    float left_arm_px;/* data */
                    float left_arm_py;
                    float left_arm_pz;
                    float left_arm_belt;
            
                    float right_arm_rz;/* data */
                    float right_arm_ry;
                    float right_arm_rx;
                    float right_arm_px;/* data */
                    float right_arm_py;
                    float right_arm_pz;
                    float right_arm_belt;
            
                    float hand_data[2][DOF_HAND];//2*6
                    float vcap_data[2];
                    float agv_speed[3];
            
                    float waist[3];
                    float head[2];
                };
            '''
            print(left_wrist_t,right_wrist_t)
            message = [
                #
                *zyx_left_robot,
                # -1.5708, 1.5708, 0,
                *left_wrist_t, 1.0,
                # -500.0, 300, 100.0, 0.5233,
                # 1.0,
                # right
                # 0.0, 1.5708, 0.0,
                # 100.0, -200, 500.0, 0.0,
                #  0.0True
                *zyx_right_robot,
                *right_wrist_t, -1.0,
                # 0.0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                joint_hand_left[1]/50,joint_hand_right[1]/50,
                0, 0, 0,
                0,0,0,
                pan, tilt
            ]

            net.send(message)

        # Wait for 30 milliseconds
        await asyncio.sleep(0.02)

import numpy as np
async def periodic_task4():
    

    ctrl=maniSdkCtrlDataClass()
    sdk=maniSdkClass("0.0.0.0", 8003, 19, 6)

    ctrl.inCharge  =1
    ctrl.filtLevel =4
    ctrl.armMode   =4
    ctrl.fingerMode=0
    ctrl.neckMode  =5
    ctrl.lumbarMode=0
    ctrl.armCmd   =np.array([[0.4, 0.4, 0.3,
                                -np.pi/2,-np.pi/2,0,   
                                #  0,0,0,
                                np.pi/4],
                            [0.4,-0.4, 0.3,   0,0,0,   np.pi/4]],np.float32)
    ctrl.armFM    =np.zeros((2,6),np.float32)
    ctrl.fingerCmd=np.zeros((2,6),np.float32)
    ctrl.neckCmd  =np.zeros(2,np.float32)
    ctrl.lumbarCmd=np.zeros(3,np.float32)

    ctrl.armCmd[0][0:3]=left_wrist_t
    ctrl.armCmd[0][3:6]=zyx_left_robot

    ctrl.armCmd[1][0:3]=right_wrist_t
    ctrl.armCmd[1][3:6]=zyx_right_robot

    data_provider = Quest3Ctrller2Qinlong(
        "0.0.0.0",5015,
        "192.168.113.187",4032
        )

    # info=maniSdkInfoDataClass()
    # # info.print()
    # print(info.getFmts())
    # # # print(info.__dict__)
    # setattr(info,"tgtTipFM2B[0]",6)
    # info.print()
    while True:
        data = data_provider.get_data()

        if data is not None:
            zyx_left_robot, left_wrist_t, zyx_right_robot, right_wrist_t, joint_hand_left, joint_hand_right, head_pose, left_hand_pose, right_hand_pose = data_provider.process(
                data)
            


            ctrl.armCmd[0][0:3]=left_wrist_t
            ctrl.armCmd[0][3:7]=zyx_left_robot

            ctrl.armCmd[1][0:3]=right_wrist_t
            ctrl.armCmd[1][3:7]=zyx_right_robot


        sdk.send(ctrl)
        sens=sdk.recv()
        # sens.print()
        # print((sens.actTipPRpy2B*1000).astype(int))
        # print((sens.tgtTipPRpy2B*1000).astype(int))


        await asyncio.sleep(0.02)


async def main():
    # Start the periodic task
    await periodic_task4()

if __name__ == "__main__":
    asyncio.run(main())
