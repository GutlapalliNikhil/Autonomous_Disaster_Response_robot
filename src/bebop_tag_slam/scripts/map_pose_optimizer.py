#!/usr/bin/env python3

import rospy
import gtsam
from gtsam import Pose2
from gtsam import BetweenFactorPose2 as BetweenFactor
from gtsam import PriorFactorPose2 as PriorFactor
from gtsam import NonlinearFactorGraph, LevenbergMarquardtOptimizer, Values
from nav_msgs.msg import Odometry, OccupancyGrid
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from geometry_msgs.msg import TransformStamped



class GtsamSLAMNode:
    def __init__(self):
        rospy.init_node("gtsam_slam_node")
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_callback)
        self.pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=10)

        # Add a subscriber for the Cartographer-generated map
        self.map_sub = rospy.Subscriber("/map_updater", OccupancyGrid, self.map_callback)
        self.updated_map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)


        self.factor_graph = NonlinearFactorGraph()
        self.initial_estimates = Values()

        # Set up noise models for odometry and AprilTag observations
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas([0.1, 0.1, 0.1])
        self.apriltag_noise = gtsam.noiseModel.Diagonal.Sigmas([0.1, 0.1, 0.1])

        # Initialize pose counter, landmark set, and map data
        self.pose_counter = 0
        self.landmark_set = set()
        self.map_data = None

    # Add a callback for the Cartographer-generated map
    def map_callback(self, msg):
        self.map_data = msg

    def odom_callback(self, msg):
        # Process odometry data and add odometry factors to the factor graph
        current_pose = self.pose_counter
        next_pose = current_pose + 1

        dx = msg.twist.twist.linear.x
        dy = msg.twist.twist.linear.y
        dtheta = msg.twist.twist.angular.z

        odometry = Pose2(dx, dy, dtheta)

        self.factor_graph.add(BetweenFactor(X(current_pose), X(next_pose), odometry, self.odom_noise))
        self.initial_estimates.insert(X(next_pose), self.initial_estimates.atPose2(X(current_pose)).compose(odometry))

        self.pose_counter += 1



    def apriltag_callback(self, msg):
        # Process AprilTag detections and add observation factors to the factor graph
        for detection in msg.detections:
            tag_id = detection.id[0]

            if tag_id not in self.landmark_set:
                self.landmark_set.add(tag_id)
                self.initial_estimates.insert(L(tag_id), Pose2())

            relative_pose = Pose2(detection.pose.pose.pose.position.x,
                                  detection.pose.pose.pose.position.y,
                                  detection.pose.pose.pose.orientation.z)  # Assuming yaw angle only

            self.factorgraph.add(BetweenFactor(X(self.pose_counter), L(tag_id), relative_pose, self.apriltag_noise))

    def optimize(self):
        # Optimize the factor graph using GTSAM's optimization algorithms
        optimizer = LevenbergMarquardtOptimizer(self.factor_graph, self.initial_estimates)
        result = optimizer.optimize()
        return result

    def update_map(self, optimized_poses):
        if self.map_data is None:
            return

        # Transform the occupancy grid based on the changes in the robot pose
        map_data = self.map_data
        robot_pose = optimized_poses.atPose2(X(self.pose_counter))

        map_data.header.stamp = rospy.Time.now()

        # Update the map's origin according to the robot's optimized pose
        map_data.info.origin.position.x += robot_pose.x()
        map_data.info.origin.position.y += robot_pose.y()

        # Assuming yaw angle only, convert to quaternion and update map's origin orientation
        yaw = robot_pose.theta()
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        map_data.info.origin.orientation.x = q[0]
        map_data.info.origin.orientation.y = q[1]
        map_data.info.origin.orientation.z = q[2]
        map_data.info.origin.orientation.w = q[3]

        # Publish the transformed map
        self.updated_map_pub.publish(map_data)

    def update_pose(self, optimized_pose):
        # Publish the updated pose of the robot
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = optimized_pose.x()
        pose_msg.pose.position.y = optimized_pose.y()
        pose_msg.pose.position.z = 0

        # Assuming yaw angle only, convert to quaternion
        yaw = optimized_pose.theta()
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        # Publish the updated tf transform
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = optimized_pose.x()
        transform.transform.translation.y = optimized_pose.y()
        transform.transform.translation.z = 0
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        tf_broadcaster.sendTransform(transform)
        self.pose_pub.publish(pose_msg)

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            optimized_poses = self.optimize()
            self.update_pose(optimized_poses.atPose2(X(self.pose_counter)))  # Update robot pose
            self.update_map(optimized_poses)  # Update the map
            rate.sleep()

def X(key):
    if not isinstance(key, int):
        raise ValueError("The 'key' argument must be an integer.")
    return gtsam.symbol(ord('x'), key)

def L(key):
    return gtsam.symbol(ord('l'), key)

if __name__ == "__main__":
    try:
        node = GtsamSLAMNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
