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
import queue
import sys
import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import math




def smoothfilter(data, window):
    window.put(data)
    queue_length = window.qsize()
    if ( queue_length == 5 ):
        window.get()

    sum_elements = 0
    my_queue = window
    while not my_queue.empty():
        element = my_queue.get()
        sum_elements += element
    average = sum_elements / queue_length
    return window, average
    


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
        ]

        
        timestamps = []  # 存储对应的时间戳 
        acc_data_x = []
        acc_data_y = []
        acc_pre_x = 9.8
        SmoothWindow = queue.Queue()
        for topic, msg, t in bag.read_messages(topics=topics):
            if topic == "/debugPx4ctrl":
                ax = msg.des_a_z
                print(ax)
                if abs(acc_pre_x - 9.8) < 1 and abs(ax - acc_pre_x) > 2.5:
                    ax = acc_pre_x
                else :
                    SmoothWindow, ax = smoothfilter(ax, SmoothWindow)
                    acc_pre_x = ax
                
                acc_data_x.append(ax)

                timestamps.append(msg.header.stamp.to_sec())
            

        bag.close()

        # import pdb;pdb.set_trace()

        # # 绘制第一张图
        # plt.figure(1)
        # plt.plot(roll_timestamps, roll_data)
        # plt.xlabel('Time (seconds)')
        # plt.ylabel('Roll (radians)')
        # plt.title('Roll over time')
        # plt.grid(True)


        # # 绘制第二张图
        # plt.figure(2)
        # plt.plot(roll_timestamps, odom_yaw)
        # plt.xlabel('Time (seconds)')
        # plt.ylabel('yaw (radians)')
        # plt.title('yaw over time')
        # plt.grid(True)

        # # 显示两张图
        # plt.show()

        # 绘制在同一个画布上
        plt.figure()

        # 绘制第一条曲线
        plt.plot(timestamps, acc_data_x, label='acc_data_x', color='blue', linestyle='-')
        # 设置第一条曲线的标签，颜色为蓝色，线条样式为实线

        # 绘制第二条曲线
        #plt.plot(timestamps, acc_data_y, label='acc_data_y', color='red', linestyle='--')
        # 设置第二条曲线的标签，颜色为红色，线条样式为虚线

        # 设置横纵轴标签和标题
        plt.xlabel('Time (seconds)')
        plt.ylabel('acc')
        plt.title('Acc over time')

        # 显示图例
        plt.legend()

        # 显示网格
        plt.grid(True)

        # 显示画布
        plt.show()


    except rosbag.ROSBagException as e:
        print(f"Error while reading bag file: {str(e)}")
        return       



if __name__ == '__main__':
    main()

