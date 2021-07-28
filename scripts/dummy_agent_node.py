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
import math
import MotionPlanner.module_7 as module7
import numpy as np
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

        #self.mapdata = rospy.Subscriber('/map', OccupancyGrid, self.callback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.waypoint_x = [7.05, 9.53, 5.29, 6.38, 3.24, 1.69, -2.26, -0.2, -1.73, -5.58, -2.76, 0.0]
        self.waypoint_y = [-2.39, -8.13, -16.2, -23.2, -23.1, -14.3, -9.0, -5.88, -4.56, -5.14, -1.08, 0.0] 
        self.point = 0
        

    def scan_callback(self, scan_msg):
        # print('got scan, now plan')
        #print(len(self.mapdata.data))
        drive = AckermannDriveStamped()

    def odom_callback(self, odom_msg):
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        velocity = odom_msg.twist.twist.linear 
        _,_,yaw = to_eularian_angles(orientation)
        car_yaw_degree = rad2deg(yaw)
        #print(position)
        # tan_x = self.waypoint_x[self.point] - position.x
        # tan_y = self.waypoint_y[self.point] - position.y
        # waypoint_yaw_degree = rad2deg(math.atan2(tan_y, tan_x))
        # to_waypoint_yaw = waypoint_yaw_degree - car_yaw_degree
        # if to_waypoint_yaw < -180:
        #     to_waypoint_yaw = 360 + to_waypoint_yaw
        # elif to_waypoint_yaw > 180:
        #     to_waypoint_yaw = -360 + to_waypoint_yaw
        # print("TO waypoint yaw : ", to_waypoint_yaw)
        

        now = rospy.get_rostime()
        #rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

        speed = np.linalg.norm([velocity.x,velocity.y,velocity.z])
        paths, cmd_throttle,cmd_steer,cmd_brake = module_7.exec_waypoint_nav_demo(position, car_yaw_degree, now.nsecs,speed)

        drive = AckermannDriveStamped()
        drive.drive.speed = cmd_throttle
        drive.drive.steering_angle = cmd_steer
        print("cmd_throttle",cmd_throttle)
        print("cmd_steer",cmd_steer)
        path_draw = Path()
        path_draw.header.frame_id = "path"
        path_draw.header.stamp = rospy.Time.now()
        for i in range(len(paths)):
            for j in range(len(paths[i][0])):
                pose = PoseStamped()
                pose.pose.position.x = paths[i][0][j]
                pose.pose.position.y = paths[i][1][j]
                pose.pose.position.z = 0
                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 0
                pose.pose.orientation.z = 0
                pose.pose.orientation.w = 1
                path_draw.poses.append(pose)
            
        print('len path poses', len(path_draw.poses))
        #print(path_draw.poses)
        self.drive_pub.publish(drive)
        self.path_pub.publish(path_draw)

if __name__ == '__main__':
    rospy.init_node('dummy_agent')
    dummy_agent = Agent()
    rospy.spin()
