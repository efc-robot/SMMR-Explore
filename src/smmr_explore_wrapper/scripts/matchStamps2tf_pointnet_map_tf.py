#!/usr/bin/python3.8
#coding=utf-8
import rospy
import threading
import numpy as np
import struct
#导入自定义的数据类型
#import data structures
# from dslam_sp.msg import MatchStamp
from cartographer_ros_msgs.msg import SubmapList, SubmapEntry
from cartographer_ros_msgs.srv import OccupancyGridQuery
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from multirobot_map_merge.srv import mapPair2tf
from icp_registration.srv import mapdata, Laserdata
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PoseArray, Transform
# import PyKDL as kdl
from tf_conversions import posemath
import tf
import sys, getopt
import torch
import torch.nn as nn
import PointNetVlad as PNV
import random
import pickle
import math
import threading
import time

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

    def __init__(self, method, weightpath, self_id):
        self.robot_id = self_id
        assert (method == "submap_img" or method == "submap_pcl" or method == "laser_scan")
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
        # self.match_sub = rospy.Subscriber("match_pair", MatchStamp, self.matchcallback)
        self.submap_list = {}
        self.feature_record = {}
        self.submap_locks = {}
        self.matched_list = []
        self.br = tf.TransformBroadcaster()
        self.tf_between_robots_dict = {}
        
        self.nn_input_size = 1024
        self.nn_output_size = 256

        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        # device = torch.device("cpu")
        self.model = PNV.PointNetVlad(global_feat=True, feature_transform=True, max_pool=False,
                                output_dim=self.nn_output_size, num_points=self.nn_input_size)
        self.model = self.model.to(device)
        resume_filename = weightpath
        print("Resuming From ", resume_filename)
        checkpoint = torch.load(resume_filename)
        # checkpoint = torch.load(resume_filename,map_location=lambda storage, loc: storage)
        saved_state_dict = checkpoint['state_dict']
        self.model.load_state_dict(saved_state_dict)
        
        tf_thread = threading.Thread(target=self.TFThread)
        tf_thread.setDaemon(True)
        tf_thread.start()
        
    def __del__(self):
        f = open('{}_matched_list.txt'.format(self.robot_id),'w')
        print(self.matched_list)
        pickle.dump(self.matched_list,f)
        f.close()
        print("__del__")
        
    def TFThread(self):
        while not rospy.is_shutdown():
            small_frame_list=[]
            big_frame_list=[]
            self_frame = self.robot_id + "/map"
            for tf_frame,tf in self.tf_between_robots_dict.items():
                if tf_frame[:len(self_frame)] == self_frame :
                    big_frame_list.append(tf.child_frame_id)
                elif tf_frame[-len(self_frame):] == self_frame :
                    small_frame_list.append(tf.header.frame_id)
            
            small_frame_list.sort()
            for i in range(1,len(small_frame_list)):
                tf1 = self.tf_between_robots_dict[small_frame_list[i-1] + ' to ' + self_frame]
                tf2 = self.tf_between_robots_dict[small_frame_list[i] + ' to ' + self_frame]
                pose1 = posemath.fromMsg(trans2pose(tf1))
                pose2 = posemath.fromMsg(trans2pose(tf2))
                tf = TransformStamped()
                tf.header.stamp = rospy.Time.now()
                tf.header.frame_id = tf1.header.frame_id
                tf.child_frame_id = tf2.header.frame_id
                tf.transform = pose2trans(posemath.toMsg(pose1*pose2.Inverse()))
                print("publish tf: " + tf.header.frame_id + " to " + tf.child_frame_id)
                self.br.sendTransformMessage(tf)
            if len(small_frame_list)>0 :
                print("publish tf: " + small_frame_list[-1] + ' to ' + self_frame)
                self.br.sendTransformMessage(self.tf_between_robots_dict[small_frame_list[-1] + ' to ' + self_frame])
            
            big_frame_list.sort()
            if len(big_frame_list)>0 :
                print("publish tf: " + self_frame + ' to ' + big_frame_list[0])
                self.br.sendTransformMessage(self.tf_between_robots_dict[self_frame + ' to ' + big_frame_list[0]])
            for i in range(1,len(big_frame_list)):
                tf1 = self.tf_between_robots_dict[self_frame + ' to ' + big_frame_list[i-1]]
                tf2 = self.tf_between_robots_dict[self_frame + ' to ' + big_frame_list[i]]
                pose1 = posemath.fromMsg(trans2pose(tf1))
                pose2 = posemath.fromMsg(trans2pose(tf2))
                tf = TransformStamped()
                tf.header.stamp = rospy.Time.now()
                tf.header.frame_id = tf1.child_frame_id
                tf.child_frame_id = tf2.child_frame_id
                tf.transform = pose2trans(posemath.toMsg(pose1.Inverse()*pose2))
                print("publish tf: " + tf.header.frame_id + " to " + tf.child_frame_id)
                self.br.sendTransformMessage(tf)
            time.sleep(1)
        

    #回调函数输入的应该是msg
    def listcallback(self, data):
        FrameIDList = data.header.frame_id.split('/')
        current_robot_id = FrameIDList[1] if FrameIDList[0]=='' else FrameIDList[0]
        
        if not self.submap_list.__contains__(current_robot_id): # 如果没有current_robot，则新建一个key           
            self.submap_list[current_robot_id] = SubmapList()
            self.feature_record[current_robot_id] = np.zeros((0,self.nn_output_size),dtype=float)
            self.submap_locks[current_robot_id] = threading.Lock()
        
        self.submap_locks[current_robot_id].acquire()
        if len(self.submap_list[current_robot_id].submap)<len(data.submap) and len(data.submap)>2: # 有新来的子图，且子图已不再更新（倒数两个会继续更新）
            newsub = len(data.submap) - max(len(self.submap_list[current_robot_id].submap),2)
            for submap in data.submap[-newsub-2:-2]: # 遍历每个新的稳定的子图             
                # 通过submap-info获得submap
                rospy.wait_for_service('/{}/get_occupancy_grid'.format(current_robot_id))
                submap_client = rospy.ServiceProxy('/{}/get_occupancy_grid'.format(current_robot_id), OccupancyGridQuery)
                submap_info = SubmapList()
                submap_info.header.frame_id = '{}/map'.format(current_robot_id)
                submap_info.submap.append(submap)
                submap_result = submap_client(submap_info)

                # 计算submap的feature（nn）             
                feature = self.map2feature(submap_result.map, self.nn_input_size)
                self.feature_record[current_robot_id] = np.vstack(( self.feature_record[current_robot_id] , np.array(feature) ) )
                
                if current_robot_id == self.robot_id:
                    # 自己robot的新来的子图 与 其他robot的所有子图 匹配
                    for robot_id in self.submap_list: # 遍历所有robot（遍历key）
                        if robot_id != current_robot_id:
                            self.findMatches(submap_result.map, submap, current_robot_id, robot_id)
                else:
                    # 其他robot的新子图 与 自己robot的所有子图 匹配
                    self.findMatches(submap_result.map, submap, current_robot_id, self.robot_id)
        self.submap_list[current_robot_id] = data
        self.submap_locks[current_robot_id].release()
        
        print("update submap list")


    def findMatches(self, self_submap, submap_info, self_id, other_id):
        print("New matching")
        if self.feature_record[other_id].shape[0]<1:
            return
        print("self.feature_record[{}].shape:".format(self_id)+str(self.feature_record[self_id].shape) )
        print("self.feature_record[{}].shape:".format(other_id)+str(self.feature_record[other_id].shape) )
        results = np.matmul(self.feature_record[other_id], self.feature_record[self_id][-1])
        print("results:"+str(results))
        validmatch = results>0.3
        # if ( validmatch.any() ):
        if True:
            rospy.wait_for_service('/{}/get_occupancy_grid'.format(other_id))
            submap_client2 = rospy.ServiceProxy('/{}/get_occupancy_grid'.format(other_id), OccupancyGridQuery)

            # map2_to_map1_tf_list = []
            confidence_list= []
            for i in range(len(results)):
                # if validmatch[i]:
                    submap_info2 = SubmapList()
                    submap_info2.header.frame_id = '{}/map'.format(other_id)
                    submap_info2.submap.append(self.submap_list[other_id].submap[i])
                    submap_result2 = submap_client2(submap_info2)

                    map2_to_map1_msg = Transform()
                    print("img_tf_client start")
                    try:
                        tf_result1 = self.img_client.call(self_submap, submap_result2.map)
                        print("transform:" + str(tf_result1.transform) + "confidence:" + str(tf_result1.confidence))
                    except Exception as e:
                        outstr = "self_id: " + self_id + "\n others_id: " + other_id
                        print(outstr)
                        print("self_len: " + str(len(self.submap_list[self_id].submap ) ) )
                        print("OTHER_len: " + str(len(self.submap_list[other_id].submap ) ) )
                        print(e)
                        confidence_list.append(0.)
                        continue
                    confidence_list.append(tf_result1.confidence)
                    if tf_result1.confidence < 0.5:
                        continue
                    map2_to_map1_msg = tf_result1.transform

                    if self.method == "submap_pcl":
                        pointcloud1 = self.OccupancyGrid2PointCloud(self_submap)
                        pub = rospy.Publisher("/{}/pointcloud_sub".format(self_id), PointCloud2, queue_size=1)
                        pub.publish(pointcloud1)
                        pointcloud2 = self.OccupancyGrid2PointCloud(submap_result2.map)
                        pub = rospy.Publisher("/{}/pointcloud_sub".format(other_id), PointCloud2, queue_size=1)
                        pub.publish(pointcloud2)
                        print("icp_tf_client start")
                        try:
                            tf_result2 = self.pcl_client.call(pointcloud1, pointcloud2, map2_to_map1_msg)
                        except Exception as e:
                            print(e)
                            continue
                        print(tf_result2.transformation)
                        map2_to_map1_msg = tf_result2.transformation

                    # map2_to_map1_tf_list.append(map2_to_map1_msg)
                    # confidence_list.append(tf_result1.confidence)
                    matched_info = {}
                    matched_info['robot1_id'] = self_id
                    matched_info['robot2_id'] = other_id
                    matched_info['robot1_submap_index'] = submap_info.submap_index
                    matched_info['robot2_submap_index'] = self.submap_list[other_id].submap[i].submap_index
                    matched_info['nn_result'] = results[i]
                    matched_info['confidence_result'] = tf_result1.confidence
                    matched_info['tf'] = map2_to_map1_msg
                    self.matched_list.append(matched_info)
                    
                    print("self.matched_list"+str(self.matched_list))
                    tf = {'x':[], 'y':[], 'z':[], 'a':[], 'b':[]}
                    for matched_info in self.matched_list: # 统计tf
                        if matched_info['robot1_id'] == self_id and matched_info['robot2_id'] == other_id:
                            tf['x'].append(matched_info['tf'].translation.x)
                            tf['y'].append(matched_info['tf'].translation.y)
                            tf['z'].append(matched_info['tf'].translation.z)
                            tf['a'].append(math.sin(2*math.atan(matched_info['tf'].rotation.z/matched_info['tf'].rotation.w)))
                            tf['b'].append(math.cos(2*math.atan(matched_info['tf'].rotation.z/matched_info['tf'].rotation.w)))
                        if matched_info['robot2_id'] == self_id and matched_info['robot1_id'] == other_id:
                            tf_tmp = pose2trans(posemath.toMsg(posemath.fromMsg(trans2pose(matched_info['tf'])).Inverse()))
                            tf['x'].append(tf_tmp.translation.x)
                            tf['y'].append(tf_tmp.translation.y)
                            tf['z'].append(tf_tmp.translation.z)
                            tf['a'].append(math.sin(2*math.atan(tf_tmp.rotation.z/tf_tmp.rotation.w)))
                            tf['b'].append(math.cos(2*math.atan(tf_tmp.rotation.z/tf_tmp.rotation.w)))
                    print("before pop tf:"+ str(tf))
                    if len(tf['x'])<3: 
                        continue
                    # 计算平均值、标准差，剔除离群点
                    mean_x = np.mean(tf['x'])
                    mean_y = np.mean(tf['y'])
                    mean_z = np.mean(tf['z'])
                    mean_a = np.mean(tf['a'])
                    mean_b = np.mean(tf['b'])
                    std_x = np.std(tf['x'])
                    std_y = np.std(tf['y'])
                    std_z = np.std(tf['z'])
                    std_a = np.std(tf['a'])
                    std_b = np.std(tf['b'])
                    sigma = 0.9
                    i = 0
                    while(i<len(tf['x'])):
                        if abs(tf['x'][i]-mean_x) > sigma*std_x or abs(tf['y'][i]-mean_y) > sigma*std_y or abs(tf['z'][i]-mean_z) > sigma*std_z or abs(tf['a'][i]-mean_a) > sigma*std_a or abs(tf['b'][i]-mean_b) > sigma*std_b:
                            tf['x'].pop(i)
                            tf['y'].pop(i)
                            tf['z'].pop(i)
                            tf['a'].pop(i)
                            tf['b'].pop(i)
                        else:
                            i += 1
                    print("after pop tf:"+ str(tf))
                    if len(tf['x'])>=1:
                        pub_tf = TransformStamped()
                        pub_tf.transform.translation.x = np.mean(tf['x'])
                        pub_tf.transform.translation.y = np.mean(tf['y'])
                        pub_tf.transform.translation.z = np.mean(tf['z'])
                        pub_tf.transform.rotation.z = math.sin(math.atan2(np.mean(tf['a']),np.mean(tf['b']))/2)
                        pub_tf.transform.rotation.w = math.cos(math.atan2(np.mean(tf['a']),np.mean(tf['b']))/2)
                        pub_tf.header.stamp = rospy.Time.now()
                        print("before inverse:"+str(pub_tf))
                        if self_id > other_id:
                            pub_tf.header.frame_id = '{}/map'.format(other_id)
                            pub_tf.child_frame_id = '{}/map'.format(self_id)
                        else:
                            pub_tf.header.frame_id = '{}/map'.format(self_id)
                            pub_tf.child_frame_id = '{}/map'.format(other_id)
                            pub_tf.transform = pose2trans(posemath.toMsg(posemath.fromMsg(trans2pose(pub_tf.transform)).Inverse()))
                        self.br.sendTransformMessage(pub_tf)
                        self.tf_between_robots_dict[pub_tf.header.frame_id + ' to ' + pub_tf.child_frame_id] = pub_tf
                        print("after inverse:"+str(pub_tf))
        
            print("nn_results:"+str(results))
            print("confidence_list:"+str(confidence_list))

            # print("confidence_list: "+ str(confidence_list))
            # if(len(confidence_list)>0):
                
                    # pub_tf = TransformStamped()
                    # pub_tf.header.stamp = rospy.Time.now()
                    # if self_id > other_id:
                        # pub_tf.header.frame_id = '{}/map'.format(other_id)
                        # pub_tf.child_frame_id = '{}/map'.format(self_id)
                        # pub_tf.transform = map2_to_map1_tf_list[confidence_list.index(max(confidence_list))]
                    # else:
                        # pub_tf.header.frame_id = '{}/map'.format(self_id)
                        # pub_tf.child_frame_id = '{}/map'.format(other_id)
                        # tf = map2_to_map1_tf_list[confidence_list.index(max(confidence_list))]
                        # pub_tf.transform = pose2trans(posemath.toMsg(posemath.fromMsg(trans2pose(tf)).Inverse()))
                    # self.br.sendTransformMessage(pub_tf)
                    # print(pub_tf)
                    
            f = open('{}_matched_list.txt'.format(self.robot_id),'wb')
            print(self.matched_list)
            pickle.dump(self.matched_list,f)
            f.close()
            print("saved")

    def map2feature(self, submap, input_dim):
        self.model.eval()
        scaled_grid = self.msg2scaled(submap, input_dim)
        with torch.no_grad():
            feature = np.array(self.model(torch.from_numpy(scaled_grid).float().view((-1, 1, input_dim, 2)).cuda()).cpu())
        return feature

    def msg2scaled(self, map_msg, NUM_POINTS):
        map = map_msg.data
        map = np.array(map)
        map[map==255] = 50
        map = map.reshape((map_msg.info.height, map_msg.info.width))
        points = []
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i][j] > 51 and map[i][j] < 255:
                    points.append(np.array([i*map_msg.info.resolution, j*map_msg.info.resolution]))
        if len(points) == 0:
            return np.zeros((NUM_POINTS, 2))
        points = np.array(points)
        # downsample
        output = np.zeros((NUM_POINTS, 2))
        if len(points) >= NUM_POINTS:
            idx = np.random.rand(len(points))
            idx = np.argsort(idx)
            output = points[idx[:NUM_POINTS]]
        else:
            idx = np.random.randint(0, len(points), NUM_POINTS - len(points))
            output = np.concatenate((points, points[idx]))
        # transform
        centroid = np.mean(output, axis=0) 
        d = np.sum(np.sqrt(np.sum(np.square(output-centroid),axis=1)))/len(output)
        s = 0.5/d
        T = np.array([[s, 0, -s*centroid[0]],[0, s, -s*centroid[1]],[0, 0, 1]])
        scaled_output = np.dot(T, np.transpose(np.concatenate((output, np.ones((len(output), 1))), axis=1)))
        scaled_output = np.transpose(scaled_output[:2])
        if np.min(scaled_output) > -1 and np.max(scaled_output) < 1:
            return scaled_output
        else:
            idx = (np.min(scaled_output,axis=1) < -1)+(np.max(scaled_output,axis=1) > 1)
            idx = np.where(idx==True)[0]
            for i in range(len(idx)):
                while True:
                    new_point = points[random.randint(0,len(points)-1)]
                    new_point=np.dot(T, np.transpose(np.concatenate((new_point, np.ones(1)), axis=0)))[:2]
                    if np.min(new_point) > -1 and np.max(new_point) < 1:
                        scaled_output[idx[i]] = new_point
                        break
            return scaled_output

    def find_submap_info(self, robotid, stamp):
        output_submap_info = SubmapEntry()
        output_submap_stamp = rospy.Time(0)
        
        self.keyFrame_locks[robotid].acquire()
        assert(len(self.submap_list[robotid].submap) == len(self.keyframe_list[robotid])), "submap_len:"+str(len(self.submap_list[robotid].submap))+" keyframe_len:"+str(len(self.keyframe_list[robotid]))
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
    weightpath = ""
    self_id = "robot1"
    opts, args = getopt.getopt(argv,"m:w:i:")
    for opt, arg in opts:
        if opt in ("-m"):
            method = arg
        if opt in ("-w"):
            weightpath = arg
        if opt in ("-i"):
            self_id = arg
    rospy.init_node('match_stamps_to_TF', anonymous=True)
    tfp = MultiRobotTF_Publisher(method, weightpath, self_id)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv[1:])