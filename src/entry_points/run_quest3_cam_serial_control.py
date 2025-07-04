import asyncio
import cv2
import numpy as np

from src.vrremote.src.mocap2robot_src.HelpFuction import is_active
from src.vrremote.src.mocap2robot_src.Mocap2Robot_quest3Controller2Qinlong import Quest3Ctrller2Qinlong


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
    t=0



    while is_active():
        # Your task code here
        # print("Task executed.")
        data = data_provider.get_data()


        if data is not None:
            zyx_left_robot, left_wrist_t, zyx_right_robot, right_wrist_t, joint_hand_left, joint_hand_right, head_pose, left_hand_pose, right_hand_pose = data_provider.process(
                data)

            # zyx=euler_from_matrix(head_pose,'szyx')
            zyx_degree=np.rad2deg(head_pose[:3])

            pan=  zyx_degree[0]
            t=t+0.02
            tilt=zyx_degree[2]
            k=0.3
            pan_filtered=((1-k)*pan_filtered+k*pan)
            tilt_filtered=((1-k)*tilt_filtered+k*tilt)
            print(pan_filtered, tilt_filtered)

            pan_final = np.clip(int(pan_filtered*10), p_min, p_max)
            tilt_final = np.clip(int(tilt_filtered*10), t_min, t_max)
            print(pan_final,tilt_final)

            # res = cap.set(cv2.CAP_PROP_TILT, tilt_final)
            # print(zyx_degree)

            send_data_to_gimbal(ser, int(1500+np.clip(pan_final,-300,300)),
                                int(1200+np.clip(tilt_final,-300,300)))


        # Wait for 30 milliseconds
        await asyncio.sleep(0.02)


async def main():
    # Start the periodic task
    await periodic_task2()


if __name__=="__main__":
    asyncio.run(main())