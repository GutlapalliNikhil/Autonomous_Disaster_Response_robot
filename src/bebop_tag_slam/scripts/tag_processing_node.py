#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from datetime import datetime
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class TagTracker:
    def __init__(self):
        # Global Variables
        self.filepath = None
        self.DT = 0.1
        self.tags = {}
        self.TF_ORIGIN = 'map'
        self.TF_CAMERA = 'camera_link'

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = TransformBroadcaster()

        # Generate filepath for tags
        dt = datetime.now()
        run_id = dt.strftime("%Y-%m-%d-%H-%M-%S")
        self.filepath = "tags_" + str(run_id) + ".txt"

        # Subscribers and Timer
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tag_detection, queue_size=1)
        rospy.Timer(rospy.Duration(self.DT), self.timer_callback)

    def get_tag_detection(self, tag_msg):
        if len(tag_msg.detections) == 0:
            return

        for i in range(len(tag_msg.detections)):
            tag_id = tag_msg.detections[i].id[0]
            tag_pose = tag_msg.detections[i].pose.pose.pose
            t = [tag_pose.position.x, tag_pose.position.y, tag_pose.position.z]
            q = [tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w]
            r = R.from_quat(q).as_matrix()

            T_AC = np.array([[r[0][0], r[0][1], r[0][2], t[0]],
                             [r[1][0], r[1][1], r[1][2], t[1]],
                             [r[2][0], r[2][1], r[2][2], t[2]],
                             [0, 0, 0, 1]])

            if T_AC is None:
                print("Found tag, but cannot create global transform.")
                return

            T_AO = self.get_transform(TF_TO=self.TF_CAMERA, TF_FROM=self.TF_ORIGIN) @ T_AC 
            # T_AO = T_AC

            if tag_id in self.tags.keys():
                print('UPDATING TAG: ', tag_id)
                L = 0.9
                self.tags[tag_id] = np.add(L * self.tags[tag_id], (1 - L) * T_AO)
            else:
                print('FOUND NEW TAG: ', tag_id)
                self.tags[tag_id] = T_AO

    def get_transform(self, TF_TO, TF_FROM):
        try:
            pose = self.tf_buffer.lookup_transform(TF_FROM, TF_TO, rospy.Time(0), rospy.Duration(4))
        except Exception as e:
            print("Transform from " + TF_FROM + " to " + TF_TO + " not found.")
            print("Exception: ", e)
            return None

        transformT = [pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z]
        transformQ = (pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w)
        r = R.from_quat(transformQ).as_matrix()

        return np.array([[r[0][0], r[0][1],r[0][2], transformT[0]],
                            [r[1][0], r[1][1], r[1][2], transformT[1]],
                            [r[2][0], r[2][1], r[2][2], transformT[2]],
                            [0, 0, 0, 1]])

    def timer_callback(self, event):
        self.publish_tf()
        self.save_tags_to_file(self.tags)

    def publish_tf(self):
        for tag_id, T_AO in self.tags.items():
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.TF_ORIGIN
            # t.header.frame_id = self.TF_CAMERA
            t.child_frame_id = "april_tag_" + str(tag_id)
            t.transform.translation.x = T_AO[0, 3]
            t.transform.translation.y = T_AO[1, 3]
            t.transform.translation.z = T_AO[2, 3]
            quat = R.from_matrix(T_AO[:3, :3]).as_quat()
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(t)

    def save_tags_to_file(self, tags):
        if not tags:
            return
        data_for_file = []
        for id in tags.keys():
            # print(id, tags[id])
            data_for_file.append("id: " + str(id))
            for row in tags[id]:
                data_for_file.append(list(row))
            data_for_file.append("---------------------------------------")
        np.savetxt(self.filepath, data_for_file, fmt="%s", delimiter=",")

def main():
    rospy.init_node('tag_tracking_node')
    tag_tracker = TagTracker()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
