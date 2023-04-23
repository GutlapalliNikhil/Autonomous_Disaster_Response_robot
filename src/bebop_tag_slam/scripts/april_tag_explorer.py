#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AprilTagExplorer:
    def __init__(self):
        rospy.init_node('april_tag_explorer')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/scan', LaserScan, self.get_scan_data, queue_size=10)
        rospy.Subscriber('/cmd_vel_update', Twist, self.get_command, queue_size=10)

        self.move_start_time = rospy.Time.now()
        self.move_duration = rospy.Duration(20.0)
        self.rotation_duration = rospy.Duration(20)

        self.scan_data = None

    def get_scan_data(self, msg):
        self.scan_data = msg.ranges

    def get_command(self, msg):
        if rospy.Time.now() >= self.move_start_time + self.move_duration:
            msg_new = Twist()
            msg_new.linear.x = 0
            msg_new.linear.y = 0
            msg_new.angular.z = 0.3125

            rotation_start_time = rospy.Time.now()
            rotation_end_time = rotation_start_time + self.rotation_duration

            while rospy.Time.now() <= rotation_end_time:
                rospy.loginfo("Rotating to search for tags.")
                self.cmd_pub.publish(msg_new)
                rospy.sleep(0.1)

            self.move_start_time = rospy.Time.now()

        else:
            self.cmd_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        explorer = AprilTagExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass
