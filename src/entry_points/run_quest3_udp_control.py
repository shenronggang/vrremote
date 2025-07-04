import asyncio

import numpy as np

from src.mocap2robot_src.Mocap2Robot_quest3Controller2Qinlong import Quest3Ctrller2Qinlong
from src.mocap2robot_src.net_utils import UdpIkSender

from src.mocap2robot_src.common.config import get_config

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
    # net = UdpIkSender("192.168.177.64", 8010)
    net = UdpIkSender(get_config("host")["ip"], get_config("host")["port"])

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
            print(left_wrist_t, right_wrist_t)
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
                joint_hand_left[1] / 50, joint_hand_right[1] / 50,
                0, 0, 0,
                0, 0, 0,
                pan, tilt
            ]

            net.send(message)

        # Wait for 30 milliseconds
        await asyncio.sleep(0.02)


async def main():
    # Start the periodic task
    await periodic_task3()


if __name__ == "__main__":
    asyncio.run(main())