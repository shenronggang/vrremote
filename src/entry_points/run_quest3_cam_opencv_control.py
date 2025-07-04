import asyncio
import cv2
import numpy as np

from mocap2robot_src.common.config import get_config
from mocap2robot_src.HelpFuction import is_active
from mocap2robot_src.Mocap2Robot_quest3Controller2Qinlong import Quest3Ctrller2Qinlong


async def periodic_task():
    data_provider = Quest3Ctrller2Qinlong(
        get_config("udp")["host"],
        get_config("udp")["port"],
        get_config("broadcast")["host"],
        get_config("broadcast")["port"],

    )
    cap = cv2.VideoCapture(0)
    p_min, p_max = -2000, 2000
    # t  -900 900
    t_min, t_max = -300, 950

    pan_filtered = 0
    tilt_filtered = 0
    t=0



    while is_active():
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
            zyx_degree=np.rad2deg(head_pose[:3])

            pan=  90*np.sin(t)#zyx_degree[0]
            t=t+0.02
            tilt=zyx_degree[2]
            k=0.3
            pan_filtered=((1-k)*pan_filtered+k*pan)
            tilt_filtered=((1-k)*tilt_filtered+k*tilt)
            print(pan_filtered, tilt_filtered)

            pan_final = np.clip(int(pan_filtered*10), p_min, p_max)
            tilt_final = np.clip(int(tilt_filtered*10), t_min, t_max)
            print(pan_final,tilt_final)
            res = cap.set(cv2.CAP_PROP_PAN, pan_final)
            # res = cap.set(cv2.CAP_PROP_TILT, tilt_final)
            print(zyx_degree)


        # Wait for 30 milliseconds
        await asyncio.sleep(0.02)


async def main():
    # Start the periodic task
    await periodic_task()


if __name__=="__main__":
    asyncio.run(main())