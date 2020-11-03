#!/usr/bin/env python

""" odom_ekf.py - Version 0.1 2012-07-08
    Republish the /robot_pose_ekf/odom_combined topic which is of type 
    geometry_msgs/PoseWithCovarianceStamped as an equivalent message of
    type nav_msgs/Odometry so we can view it in RViz.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib
#roslib.load_manifest('rbx1_nav')
import rospy
import getopt
import sys
from sensor_msgs.msg import Imu

class ImuChange():
    def __init__(self, frame_id = 'base_footprint'):
        # Give the node a name
        rospy.init_node('ImuChangeHeader', anonymous=False)
        self.frame_id = frame_id

        # Publisher of type nav_msgs/Odometry
        self.Imu_pub = rospy.Publisher('output', Imu,queue_size=10)
        
        # Wait for the /odom_combined topic to become available
        rospy.wait_for_message('input', Imu)
        
        # Subscribe to the /odom_combined topic
        rospy.Subscriber('input', Imu, self.pub_Imu)
        
        rospy.loginfo("Publishing combined odometry on /ImuChange")
        
    def pub_Imu(self, msg):
        imu_data = msg
        imu_data.header.frame_id = self.frame_id
        
        self.Imu_pub.publish(imu_data)
        
if __name__ == '__main__':
    # try:
    opts, args = getopt.getopt(sys.argv[1:],"c:")
    for opt, arg in opts:
        if opt in ("-c"):
            self_frame_id = str(arg)
    ImuChange(self_frame_id)
    rospy.spin()
    # except:
    #     pass
        
