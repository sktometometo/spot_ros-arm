#!/usr/bin/env python

import message_filters
import PyKDL
import rospy
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
from spot_msgs.msg import GraphNavGraph, GraphNavLocalization


def convert_to_transform(frame: PyKDL.Frame):
    transform = Transform()
    transform.translation.x = frame.p[0]
    transform.translation.y = frame.p[1]
    transform.translation.z = frame.p[2]
    transform.rotation.x = frame.M.GetQuaternion()[0]
    transform.rotation.y = frame.M.GetQuaternion()[1]
    transform.rotation.z = frame.M.GetQuaternion()[2]
    transform.rotation.w = frame.M.GetQuaternion()[3]
    return transform


class GraphPublisher:

    def __init__(self):

        self.frame_id_graph_reference = rospy.get_param(
            "~frame_id_graph_reference", "graph_reference"
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        sub_odom = message_filters.Subscriber(
            "/spot/odometry",
            Odometry,
        )
        sub_graph_nav_localization = message_filters.Subscriber(
            "/spot/graph_nav_localization_state",
            GraphNavLocalization,
        )
        self.ts_graph_nav_localization = message_filters.ApproximateTimeSynchronizer(
            [sub_odom, sub_graph_nav_localization],
            10,
            0.1,
            allow_headerless=True,
        )
        self.ts_graph_nav_localization.registerCallback(self._cb_odom)

        self._sub_graph_nav_graph = rospy.Subscriber(
            "/spot/graph_nav_graph",
            GraphNavGraph,
            self._cb_graph_nav_graph,
        )

    def _cb_odom(self, msg_odom, msg_graph_nav_localization):
        frame_id_odom = msg_odom.header.frame_id
        frame_odom_to_body = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                msg_odom.pose.pose.orientation.x,
                msg_odom.pose.pose.orientation.y,
                msg_odom.pose.pose.orientation.z,
                msg_odom.pose.pose.orientation.w,
            ),
            PyKDL.Vector(
                msg_odom.pose.pose.position.x,
                msg_odom.pose.pose.position.y,
                msg_odom.pose.pose.position.z,
            ),
        )
        frame_graph_reference_to_body = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                msg_graph_nav_localization.body_pose_in_reference_frame.orientation.x,
                msg_graph_nav_localization.body_pose_in_reference_frame.orientation.y,
                msg_graph_nav_localization.body_pose_in_reference_frame.orientation.z,
                msg_graph_nav_localization.body_pose_in_reference_frame.orientation.w,
            ),
            PyKDL.Vector(
                msg_graph_nav_localization.body_pose_in_reference_frame.position.x,
                msg_graph_nav_localization.body_pose_in_reference_frame.position.y,
                msg_graph_nav_localization.body_pose_in_reference_frame.position.z,
            ),
        )
        frame_odom_to_graph_reference = (
            frame_odom_to_body * frame_graph_reference_to_body.Inverse()
        )
        transform = TransformStamped()
        transform.header.stamp = msg_odom.header.stamp
        transform.header.frame_id = frame_id_odom
        transform.child_frame_id = self.frame_id_graph_reference
        transform.transform = convert_to_transform(frame_odom_to_graph_reference)
        self.tf_broadcaster.sendTransform(transform)

    def _cb_graph_nav_graph(self, msg):
        frames_graph = {}
        for anchor in msg.anchoring.anchors:
            frames_graph["waypoint_" + anchor.id] = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(
                    anchor.seed_tform_waypoint.orientation.x,
                    anchor.seed_tform_waypoint.orientation.y,
                    anchor.seed_tform_waypoint.orientation.z,
                    anchor.seed_tform_waypoint.orientation.w,
                ),
                PyKDL.Vector(
                    anchor.seed_tform_waypoint.position.x,
                    anchor.seed_tform_waypoint.position.y,
                    anchor.seed_tform_waypoint.position.z,
                ),
            )
        for obj in msg.anchoring.objects:
            frames_graph["object_" + obj.id] = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(
                    obj.seed_tform_object.orientation.x,
                    obj.seed_tform_object.orientation.y,
                    obj.seed_tform_object.orientation.z,
                    obj.seed_tform_object.orientation.w,
                ),
                PyKDL.Vector(
                    obj.seed_tform_object.position.x,
                    obj.seed_tform_object.position.y,
                    obj.seed_tform_object.position.z,
                ),
            )
        for name, frame in frames_graph.items():
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = self.frame_id_graph_reference
            transform.child_frame_id = name
            transform.transform = convert_to_transform(frame)
            self.tf_broadcaster.sendTransform(transform)

if __name__ == "__main__":
    rospy.init_node("graph_publisher")
    node = GraphPublisher()
    rospy.spin()
