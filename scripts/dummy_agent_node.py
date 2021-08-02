#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import MotionPlanner.module_7 as module7
import numpy as np
import time
#from geometry_msgs.msg import Point

module_7= module7.Autonomous()
def to_eularian_angles(q):
    z = q.z
    y = q.y
    x = q.x
    w = q.w
    ysqr = y * y

    # roll (x-axis rotation)
    t0 = +2.0 * (w*x + y*z)
    t1 = +1.0 - 2.0*(x*x + ysqr)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w*y - z*x)
    if (t2 > 1.0):
        t2 = 1
    if (t2 < -1.0):
        t2 = -1.0
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w*z + x*y)
    t4 = +1.0 - 2.0 * (ysqr + z*z)
    yaw = math.atan2(t3, t4)

    return (pitch, roll, yaw)

def rad2deg(rad):
    pi = math.pi
    return rad * 180 / pi

def deg2rad(deg):
    pi = math.pi
    return deg / 180 * pi

class Agent(object):
    def __init__(self):
        #mapdata.data = OccupancyGrid.data
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.path_pub = rospy.Publisher('/paths', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/marker', Marker, queue_size=10)
        self.markerarray_pub = rospy.Publisher('/markerarray', MarkerArray, queue_size=10)

        #self.mapdata = rospy.Subscriber('/map', OccupancyGrid, self.callback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.waypoint_x = [7.05, 9.53, 5.29, 6.38, 3.24, 1.69, -2.26, -0.2, -1.73, -5.58, -2.76, 0.0]
        self.waypoint_y = [-2.39, -8.13, -16.2, -23.2, -23.1, -14.3, -9.0, -5.88, -4.56, -5.14, -1.08, 0.0] 
        self.point = 0
        self.current_time = time.time()
        self.prev_time = 0
        self.times = []
        self.times2 = []
        self.scan_data = []
        self.ego_state=[0,0,0]

    def transform(self, ego_state, scan_data):
        transformed_scan = []
        for scan in scan_data:

            x_transformed=(ego_state[0] + scan[0]*math.cos(ego_state[2]) - \
                                                scan[1]*math.sin(ego_state[2]))
            y_transformed=(ego_state[1] + scan[0]*math.sin(ego_state[2]) + \
                                                scan[1]*math.cos(ego_state[2]))
            #t_transformed.append(scan[2][i] + ego_state[2])

            transformed_scan.append([x_transformed, y_transformed])

        return transformed_scan

    def scan_callback(self, scan_msg):
        scan_data = []
        data = scan_msg.ranges
        maxi = scan_msg.range_max
        data = data[359:719]
        #print(len(data))
        for idx in range(0,len(data)):
            theta = -(179-idx)*0.25  
            theta = math.radians(theta)
            x= data[idx]*math.cos(theta)
            y = data[idx]*math.sin(theta)
            scan_data.append([x,y])
        #print(len(self.scan_data), max(data))
        self.scan_data= self.transform(self.ego_state,scan_data)
        #time.sleep(10)



    def odom_callback(self, odom_msg):
        #
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        velocity = odom_msg.twist.twist.linear 
        _,_,yaw_rad = to_eularian_angles(orientation)

        self.ego_state = [position.x, position.y, yaw_rad]


        
        

        now = rospy.get_rostime()

        self.prev_time = self.current_time
        ptime = time.time()
        
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

        speed = np.linalg.norm([velocity.x,velocity.y,velocity.z])
        paths, cmd_throttle,cmd_steer,cmd_brake, best_index = module_7.exec_waypoint_nav_demo(position, yaw_rad, now.nsecs,speed,self.scan_data)
        paths_np = np.array(paths)
        drive = AckermannDriveStamped()
        drive.drive.speed = cmd_throttle
        drive.drive.steering_angle = cmd_steer
        #print("cmd_throttle",cmd_throttle)
        #print("cmd_steer",cmd_steer)
        path_draw = Path()
        path_draw.header.frame_id = "map"
        path_draw.header.stamp = rospy.Time.now()
        for i in range(len(paths)):
            for j in range(len(paths[i][0])):
                pose = PoseStamped()
                pose.pose.position.x = paths[i][0][j]
                pose.pose.position.y = paths[i][1][j]
                pose.pose.position.z = 0
                path_draw.poses.append(pose)
        path_marker = Marker()
        path_marker.header.frame_id = 'map'
        path_marker.header.stamp = rospy.get_rostime()
        path_marker.ns = "my"
        path_marker.id = 0
        path_marker.type = 2
        path_marker.action = 0
        path_marker.pose.position.x = paths[best_index][0][-1]
        path_marker.pose.position.y = paths[best_index][1][-1]
        path_marker.pose.position.z = 0
        path_marker.pose.orientation.x = 0
        path_marker.pose.orientation.y = 0
        path_marker.pose.orientation.z = 0
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.1
        path_marker.scale.y = 0.1
        path_marker.scale.z = 0.1
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0


        scan_marker = MarkerArray()
        for i in range(len(self.scan_data)):
            marker  = Marker() 
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.get_rostime()
            marker.ns = "my"+str(i)
            marker.id = 0
            marker.type = 2
            marker.action = 0
            marker.pose.position.x = float(self.scan_data[i][0])
            marker.pose.position.y = float(self.scan_data[i][1])
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            scan_marker.markers.append(marker)

        self.drive_pub.publish(drive)

        #self.current_time = time.time()
        #ctime = time.time()
        #print("fps : ", 1/(self.current_time - self.prev_time))
        # self.times.append(self.current_time - self.prev_time)
        # self.times2.append(ctime - ptime)
        # if len(self.times) % 1000 == 0:
        #     tt = 0
        #     ttt = 0
        #     for t in self.times:
        #         tt += t
        #     for t in self.times2:
        #         ttt += t
        #     tt /= len(self.times)
        #     ttt /= len(self.times2)
        #     print("****************************average fps +ROS: ********************************", 1/tt)
        #     print("****************************average fps : ********************************", 1/ttt)

        self.path_pub.publish(path_draw)
        self.marker_pub.publish(path_marker)
        self.markerarray_pub.publish(scan_marker)
        #time.sleep(0.05)

if __name__ == '__main__':
    rospy.init_node('dummy_agent')
    #time.sleep(10)
    dummy_agent = Agent()
    rospy.spin()
