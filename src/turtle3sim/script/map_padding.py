#!/usr/bin/python3.8
from numpy.lib.function_base import _median_dispatcher
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2

class MapPadding:
    def __init__(self, robot_name) -> None:
        print(robot_name)
        self.map_pub = rospy.Publisher(
            robot_name+"/map", OccupancyGrid, queue_size=10)
        # self.pose_pub = rospy.Publisher(
        #     robot_name+"/testpose", PoseStamped, queue_size=10)
        
        rospy.Subscriber(
            robot_name+"/map_origin", OccupancyGrid, self.map_callback, queue_size=1)
    
    def map_callback(self, map):
        # print(map.info.origin.position)
        map_message = OccupancyGrid()
        map_message.header = map.header
        map_message.info = map.info
        # print("map orientation::", map.info.origin)
        padding = 200
        shape = (map.info.height, map.info.width)
        mapdata = np.asarray(map.data).reshape(shape)
        # cv2.imwrite("/home/zzl/zzlWorkspace/debug/beforepad.jpg", mapdata)
        localMap = np.full((shape[0]+padding*2, shape[1]+padding*2), -1).astype(np.int8)
        localMap[padding:shape[0]+padding, padding:shape[1]+padding] = mapdata
        # cv2.imwrite("/home/zzl/zzlWorkspace/debug/paddedmap.jpg", localMap)
        map_message.data = tuple(localMap.flatten())
        map_message.info.height += padding*2
        map_message.info.width += padding*2
        map_message.info.origin.position.x -= padding*map.info.resolution
        map_message.info.origin.position.y -= padding*map.info.resolution
        self.map_pub.publish(map_message)
        # before_send = np.asarray(map_message.data).reshape((map_message.info.height, map_message.info.width))
        # cv2.imwrite("/home/zzl/zzlWorkspace/debug/globalmapbegfore.jpg", before_send)

        # pose = PoseStamped()
        # pose.header.frame_id = map_message.header.frame_id
        # pose.pose.position = map_message.info.origin.position
        # pose.pose.orientation.z = 0
        # pose.pose.orientation.w = 1
        # self.pose_pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node("map_padding")
    robot_name = rospy.get_param("~robot_name")
    node = MapPadding(robot_name)
    rospy.spin()