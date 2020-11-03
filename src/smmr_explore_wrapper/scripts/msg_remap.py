#!/usr/bin/env python
#coding=utf-8
import rospy
#导入自定义的数据类型
from cartographer_ros_msgs.msg import SubmapList

class Msg_Remaper:

    def __init__(self):
        self.pub = rospy.Publisher("msg_out", SubmapList, queue_size=1)
        #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
        self.sub = rospy.Subscriber("msg_in", SubmapList, self.callback)

    #回调函数输入的应该是msg
    def callback(self, data):
        self.pub.publish(data)


def main():
    rospy.init_node('msg_remap', anonymous=True)
    mr = Msg_Remaper()
    rospy.spin()

if __name__ == '__main__':
    main()