''''
@Description :
@File        :controlYaw.py
@Author      :lvzenan
@Date        :2023-07-27
@versions    :
@usage       :
@email       :zenanlv@163.com
Copyright (c) 2023. All rights reserved.Created by lvzenan
'''
'''
workflow:
1、读取rosbag
2、提取出control_debug中的姿态信息
3、将其中的偏航角提取出来
4、转换为弧度值
5、输出带时间戳的二维曲线
'''

import sys
import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import math


def fromQuaternion2yaw(x, y, z, w):
    return np.arctan2(2 * (x*y + w*z), w*w + x*x - y*y - z*z)



def main():
    if len(sys.argv) < 2:
        print("Usage: python script_name.py path_to_rosbag")
        return
    
    bag_file = sys.argv[1]
    if not os.path.isfile(bag_file):
        print(f"Error: The specified file '{bag_file}' does not exist.")
        return

    try:
        bag = rosbag.Bag(bag_file, 'r')
        topics = [
            "/debugPx4ctrl",
            "/fusion/odom"
        ]

        yaw_data = []  # 存储提取的偏航角数据
        roll_data = [] # 存储期望的横滚角数据
        timestamps = []  # 存储对应的时间戳 
        roll_timestamps = [] #横滚角对应的时间戳

        acc = [0, 0]    # 加速度信息

        
        for topic, msg, t in bag.read_messages(topics=topics):
            if topic == "/debugPx4ctrl":
                acc[0] = msg.des_a_x
                acc[1] = msg.des_a_y
                x = msg.des_q_x
                y = msg.des_q_y
                z = msg.des_q_z
                w = msg.des_q_w
                yaw = np.arctan2(2 * (x*y + w*z), w*w + x*x - y*y - z*z)
                yaw_data.append(yaw)
                timestamps.append(msg.header.stamp.to_sec())
            if topic == "/fusion/odom":
                yaw = fromQuaternion2yaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                sin = math.sin(yaw)
                cos = math.cos(yaw)
                roll = (acc[0] * sin - acc[1] * cos )/ 9.81;
                roll_data.append(roll)
                roll_timestamps.append(msg.header.stamp.to_sec())

        bag.close()

        # import pdb;pdb.set_trace()

        # 绘制第一张图
        plt.figure(1)
        plt.plot(timestamps, yaw_data)
        plt.xlabel('Time (seconds)')
        plt.ylabel('Yaw (radians)')
        plt.title('Yaw over time')
        plt.grid(True)


        # 绘制第二张图
        plt.figure(2)
        plt.plot(roll_timestamps, roll_data)
        plt.xlabel('Time (seconds)')
        plt.ylabel('Roll (radians)')
        plt.title('Roll over time')
        plt.grid(True)

        # 显示两张图
        plt.show()


    except rosbag.ROSBagException as e:
        print(f"Error while reading bag file: {str(e)}")
        return       



if __name__ == '__main__':
    main()
