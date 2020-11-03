#!/usr/bin/env python
#coding=utf-8
import rospy
import threading
import numpy as np
import struct
#导入自定义的数据类型
from dslam_sp.msg import MatchStamp
from cartographer_ros_msgs.msg import SubmapList, SubmapEntry
from cartographer_ros_msgs.srv import OccupancyGridQuery
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from multirobot_map_merge.srv import mapPair2tf
from icp_registration.srv import mapdata, Laserdata
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PoseArray, Transform
import PyKDL as kdl
from tf_conversions import posemath
import tf
import sys, getopt

def trans2pose(trans):
    pose = Pose()
    pose.orientation = trans.rotation
    pose.position = trans.translation
    return pose

def pose2trans(pose):
    trans = Transform()
    trans.rotation = pose.orientation
    trans.translation = pose.position
    return trans
class MultiRobotTF_Publisher:

    def __init__(self, method):
        # assert (method == "submap_img" or method == "submap_pcl" or method == "laser_scan")
        self.method = method
        rospy.wait_for_service('GetMapTransform_submap_img')
        self.img_client = rospy.ServiceProxy('GetMapTransform_submap_img', mapPair2tf)
        if self.method == "submap_pcl":
            rospy.wait_for_service('GetMapTransform_submap_pcl')
            self.pcl_client = rospy.ServiceProxy('GetMapTransform_submap_pcl', mapdata)
        elif self.method == "laser_scan":
            rospy.wait_for_service('GetMapTransform_laser_scan')
            self.scan_client = rospy.ServiceProxy('GetMapTransform_laser_scan', Laserdata)
        
        self.pointcloud_pub = rospy.Publisher("pointcloud_sub", PointCloud2, queue_size=1)
        self.occupancy_grid_pub = rospy.Publisher("occupancy_grid_sub", OccupancyGrid, queue_size=1)
        #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
        self.list_sub = rospy.Subscriber("submap_list", SubmapList, self.listcallback)
        self.match_sub = rospy.Subscriber("match_pair", MatchStamp, self.matchcallback)
        self.submap_list = {}
        self.keyframe_list = {}
        self.keyFrame_locks = {}
        self.br = tf.TransformBroadcaster()
        

    #回调函数输入的应该是msg
    def listcallback(self, data):
        FrameIDList = data.header.frame_id.split('/')
        current_robot_id = FrameIDList[1] if FrameIDList[0]=='' else FrameIDList[0]
        
        if not self.submap_list.has_key(current_robot_id):
            self.submap_list[current_robot_id] = SubmapList()
            self.keyframe_list[current_robot_id] = []
            self.keyFrame_locks[current_robot_id] = threading.Lock()
        
        self.keyFrame_locks[current_robot_id].acquire()
        if len(self.submap_list[current_robot_id].submap)<len(data.submap):
            self.keyframe_list[current_robot_id].append(data.header)
        self.submap_list[current_robot_id] = data
        # assert(len(self.submap_list[current_robot_id].submap) == len(self.keyframe_list[current_robot_id])), "submap_len:"+str(len(self.submap_list[current_robot_id].submap))+" keyframe_len:"+str(len(self.keyframe_list[current_robot_id]))
        self.keyFrame_locks[current_robot_id].release()
        
        print("update submap list")


    def matchcallback(self, data):
        print("New match")
        if self.submap_list.has_key(data.robotid1) and self.submap_list.has_key(data.robotid2):
        # if self.submap_list.has_key(data.robotid1):
            rospy.wait_for_service('/{}/get_occupancy_grid'.format(data.robotid1))
            submap_client1 = rospy.ServiceProxy('/{}/get_occupancy_grid'.format(data.robotid1), OccupancyGridQuery)
            submap_info1 = SubmapList()
            submap_info1.header.frame_id = '{}/map'.format(data.robotid1)
            submap_info1.submap.append(self.find_submap_info(data.robotid1, data.stamp1))
            # submap_info1 = self.submap_list[data.robotid1]
            submap_result1 = submap_client1(submap_info1)
            # submap_pointcloud1 = self.OccupancyGrid2PointCloud(submap_result1.map)
            # self.occupancy_grid_pub.publish(submap_result1.map)
            pub = rospy.Publisher("/{}/occupancy_grid_sub".format(data.robotid1), OccupancyGrid, queue_size=1)
            pub.publish(submap_result1.map)
            
        # if self.submap_list.has_key(data.robotid2):
            rospy.wait_for_service('/{}/get_occupancy_grid'.format(data.robotid2))
            submap_client2 = rospy.ServiceProxy('/{}/get_occupancy_grid'.format(data.robotid2), OccupancyGridQuery)
            submap_info2 = SubmapList()
            submap_info2.header.frame_id = '{}/map'.format(data.robotid2)
            submap_info2.submap.append(self.find_submap_info(data.robotid2, data.stamp2))
            # submap_info2 = self.submap_list[data.robotid2]
            submap_result2 = submap_client2(submap_info2)
            # submap_pointcloud2 = self.OccupancyGrid2PointCloud(submap_result2.map)
            # self.occupancy_grid_pub.publish(submap_result2.map)
            pub = rospy.Publisher("/{}/occupancy_grid_sub".format(data.robotid2), OccupancyGrid, queue_size=1)
            pub.publish(submap_result2.map)

            map2_to_map1_msg = Transform()
            # if self.method == "submap_img":
            print("img_tf_client start")
            print(submap_result1.map)
            print(submap_result2.map)
            tf_result = self.img_client.call(submap_result1.map, submap_result2.map)
            print(tf_result.transform)
            map2_to_map1_msg = tf_result.transform

            if self.method == "submap_pcl":
                pointcloud1 = self.OccupancyGrid2PointCloud(submap_result1.map)
                pub = rospy.Publisher("/{}/pointcloud_sub".format(data.robotid1), PointCloud2, queue_size=1)
                pub.publish(pointcloud1)
                pointcloud2 = self.OccupancyGrid2PointCloud(submap_result2.map)
                pub = rospy.Publisher("/{}/pointcloud_sub".format(data.robotid2), PointCloud2, queue_size=1)
                pub.publish(pointcloud2)
                print("icp_tf_client start")
                tf_result = self.pcl_client.call(pointcloud1, pointcloud2, map2_to_map1_msg)
                print(tf_result.transformation)
                map2_to_map1_msg = tf_result.transformation
            # map1_to_sub1_tf = posemath.fromMsg(submap_info1.submap[-1].pose)
            # map2_to_sub2_tf = posemath.fromMsg(submap_info2.submap[-1].pose)
            # sub2_to_sub1_tf = posemath.fromMsg(trans2pose(tf_result.transform))
            # map1_to_map2_tf = map1_to_sub1_tf * sub2_to_sub1_tf.Inverse() * map2_to_sub2_tf.Inverse()

            # map1_to_map2_msg = pose2trans(posemath.toMsg(map1_to_map2_tf))
            # pub_tf = TransformStamped()
            # pub_tf.header = submap_result1.map.header
            # pub_tf.child_frame_id = '{}/map'.format(data.robotid2)
            # pub_tf.transform = map1_to_map2_msg
            # self.br.sendTransformMessage(pub_tf)
            
            pub_tf = TransformStamped()
            pub_tf.header.stamp = rospy.Time.now()
            pub_tf.header.frame_id = '{}/map'.format(data.robotid2)
            pub_tf.child_frame_id = '{}/map'.format(data.robotid1)
            pub_tf.transform = map2_to_map1_msg
            self.br.sendTransformMessage(pub_tf)
            print(pub_tf)


    def find_submap_info(self, robotid, stamp):
        output_submap_info = SubmapEntry()
        output_submap_stamp = rospy.Time(0)
        
        self.keyFrame_locks[robotid].acquire()
        # assert(len(self.submap_list[robotid].submap) == len(self.keyframe_list[robotid])), "submap_len:"+str(len(self.submap_list[robotid].submap))+" keyframe_len:"+str(len(self.keyframe_list[robotid]))
        for submap_info, keyframe_header in zip(self.submap_list[robotid].submap, self.keyframe_list[robotid]):
            if stamp>keyframe_header.stamp and (stamp-keyframe_header.stamp)<(stamp-output_submap_stamp):
                output_submap_info = submap_info
        self.keyFrame_locks[robotid].release()
        
        return output_submap_info

    def OccupancyGrid2PointCloud(self, occupancy_grid):
        map = occupancy_grid.data
        map = np.array(map)
        map[map==255] = 50
        max_grid = np.max(map)
        min_grid = np.min(map)
        map = map.reshape((occupancy_grid.info.height, occupancy_grid.info.width))
        
        buffer = []
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i,j]>((max_grid-min_grid)*0.67+min_grid) and map[i,j]>50:
                    z = 0
                    x = j * occupancy_grid.info.resolution + occupancy_grid.info.origin.position.x
                    y = i * occupancy_grid.info.resolution + occupancy_grid.info.origin.position.y
                    buffer.append(struct.pack('ffff', x, y, z, 1))

        pcl_msg = PointCloud2()

        pcl_msg.header = occupancy_grid.header
        print( pcl_msg.header )

        pcl_msg.height = 1
        pcl_msg.width = len(buffer)

        pcl_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        pcl_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        pcl_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        pcl_msg.fields.append(PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1))

        pcl_msg.is_bigendian = False
        pcl_msg.point_step = 16
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width * pcl_msg.height
        pcl_msg.is_dense = False
        pcl_msg.data = "".join(buffer)

        return pcl_msg

def main(argv):
    method = "submap_img"
    opts, args = getopt.getopt(argv,"m:")
    for opt, arg in opts:
        if opt in ("-m"):
            method = arg
    rospy.init_node('match_stamps_to_TF', anonymous=True)
    tfp = MultiRobotTF_Publisher(method)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv[1:])