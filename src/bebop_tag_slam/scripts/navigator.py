#!/usr/bin/env python3

import rospy
import math
import tf.transformations as transformations
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion

class WallFollower:
    def __init__(self):
        self.map_data = None
        self.scan_data = None
        self.current_pose = None
        self.exploring = False
        self.rate = rospy.Rate(10)  # 10 Hz

        # Set default values for ROS parameters
        self.min_obstacle_distance = rospy.get_param("~min_obstacle_distance", 0.8)

        # Subscribe to topics
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Create action client for move_base
        self.goal_client = SimpleActionClient('move_base', MoveBaseAction)

        # Wait for move_base action server to become available
        if not self.goal_client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("Timeout waiting for move_base action server")
            rospy.signal_shutdown("Timeout waiting for move_base action server")

    def map_callback(self, msg):
        self.map_data = msg

    def scan_callback(self, msg):
        self.scan_data = msg

        # Only publish a new goal if not exploring
        if not self.exploring:
            self.publish_goal()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose


    def generate_goal(self):
        if not self.scan_data or not self.current_pose:
            return None

        regions = {
            'right':  min(self.scan_data.ranges[0:143]) if self.scan_data.ranges[0:143] else 10,
            'fright': min(self.scan_data.ranges[144:287]) if self.scan_data.ranges[144:287] else 10,
            'front':  min(self.scan_data.ranges[288:431]) if self.scan_data.ranges[288:431] else 10,
            'fleft':  min(self.scan_data.ranges[432:575]) if self.scan_data.ranges[432:575] else 10,
            'left':   min(self.scan_data.ranges[576:713]) if self.scan_data.ranges[576:713] else 10,
        }

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()

        current_orientation = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

        if regions['front'] > self.min_obstacle_distance and regions['fleft'] > self.min_obstacle_distance and regions['fright'] > self.min_obstacle_distance:
            # No obstacles detected, switch to exploration mode
            rospy.loginfo("Goal is outside the map, switching to exploration mode")
            self.exploring = True
            rospy.set_param('/explore_lite/explore', True)
            return None

        elif regions['front'] < self.min_obstacle_distance and regions['fleft'] > self.min_obstacle_distance and regions['fright'] > self.min_obstacle_distance:
            # Obstacles in front of the robot, turn left
            goal_pose.pose.position.x = self.current_pose.position.x
            goal_pose.pose.position.y = self.current_pose.position.y
            yaw += math.pi / 2
        elif regions['front'] > self.min_obstacle_distance and regions['fleft'] > self.min_obstacle_distance and regions['fright'] < self.min_obstacle_distance:
            # Obstacle detected only on the right side of the robot, follow the wall
            goal_pose.pose.position.x = self.current_pose.position.x + 1.0 * math.cos(yaw)
            goal_pose.pose.position.y = self.current_pose.position.y + 1.0 * math.sin(yaw)
        else:
            # Other cases, switch to exploration mode
            rospy.loginfo("Goal is outside the map, switching to exploration mode")
            self.exploring = True
            rospy.set_param('/explore_lite/explore', True)
            return None

        # Check if the goal is in the map
        map_width = self.map_data.info.width * self.map_data.info.resolution
        map_height = self.map_data.info.height * self.map_data.info.resolution
        if goal_pose.pose.position.x < 0 or goal_pose.pose.position.x > map_width or goal_pose.pose.position.y < 0 or goal_pose.pose.position.y > map_height:
            # Goal is outside the map, switch to exploration mode
            rospy.loginfo("Goal is outside the map, switching to exploration mode")
            self.exploring = True
            rospy.set_param('/explore_lite/explore', True)
            return None

        quat = transformations.quaternion_from_euler(0, 0, yaw)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = goal_pose

        return move_base_goal


    def publish_goal(self):
        if self.exploring:
            return
        goal = self.generate_goal()
        if goal:
            self.goal_client.send_goal(goal)

    def run(self):
        while not rospy.is_shutdown():
            if not self.exploring:
                self.publish_goal()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()

