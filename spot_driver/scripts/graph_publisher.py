#!/usr/bin/env python

from geometry_msgs.msg import TransformStamped, Transform
from nav_msgs.msg import Odometry
from spot_msgs.msg import GraphNavGraph, GraphNavLocalization
import PyKDL
import copy
import rospy
import tf2_ros
import threading

def convert_to_transform(frame):
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

        self._frame_odom_to_body = PyKDL.Frame()
        self._frame_id_odom = None
        self._lock_frame_odom_to_body = threading.Lock()

        self._frame_odom_to_graph_reference = PyKDL.Frame()
        self._lock_frame_odom_to_graph_reference = threading.Lock()
        self._frame_id_graph_reference = rospy.get_param(
                '~frame_id_graph_reference',
                'graph_reference'
                )

        self._frames_graph = {} # id -> PyKDL.Frame
        self._lock_frames_graph = threading.Lock()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._sub_odom = rospy.Subscriber(
                '/spot/odometry',
                Odometry,
                self._cb_odom,
                )
        self._sub_graph_nav_localization = rospy.Subscriber(
                '/spot/graph_nav_localization_state',
                GraphNavLocalization,
                self._cb_graph_nav_localization,
                )
        self._sub_graph_nav_graph = rospy.Subscriber(
                '/spot/graph_nav_graph',
                GraphNavGraph,
                self._cb_graph_nav_graph,
                )

    @property
    def frame_odom_to_body(self):
        with self._lock_frame_odom_to_body:
            return copy.deepcopy(self._frame_odom_to_body)

    @property
    def frame_id_odom(self):
        with self._lock_frame_odom_to_body:
            return copy.deepcopy(self._frame_id_odom)

    @property
    def frame_odom_to_graph_reference(self):
        with self._lock_frame_odom_to_graph_reference:
            return copy.deepcopy(self._frame_odom_to_graph_reference)

    @property
    def frame_id_graph_reference(self):
        return self._frame_id_graph_reference

    @property
    def frames_graph(self):
        with self._lock_frames_graph:
            return copy.deepcopy(self._frames_graph)

    def _cb_odom(self, msg):
        with self._lock_frame_odom_to_body:
            self._frame_id_odom = msg.header.frame_id
            self._frame_odom_to_body = PyKDL.Frame(
                    PyKDL.Rotation.Quaternion(
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w,
                        ),
                    PyKDL.Vector(
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                        ),
                    )

    def _cb_graph_nav_localization(self, msg):
        with self._lock_frame_odom_to_graph_reference:
            frame_graph_reference_to_body = PyKDL.Frame(
                    PyKDL.Rotation.Quaternion(
                        msg.body_pose_in_reference_frame.orientation.x,
                        msg.body_pose_in_reference_frame.orientation.y,
                        msg.body_pose_in_reference_frame.orientation.z,
                        msg.body_pose_in_reference_frame.orientation.w,
                        ),
                    PyKDL.Vector(
                        msg.body_pose_in_reference_frame.position.x,
                        msg.body_pose_in_reference_frame.position.y,
                        msg.body_pose_in_reference_frame.position.z,
                        ),
                    )
            self._frame_odom_to_graph_reference = self.frame_odom_to_body * frame_graph_reference_to_body.Inverse()

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
        with self._lock_frames_graph:
            self._frames_graph = frames_graph

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.frame_id_odom is None:
                continue
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = self.frame_id_odom
            transform.child_frame_id = self.frame_id_graph_reference
            transform.transform = convert_to_transform(self.frame_odom_to_graph_reference)
            self.tf_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node('graph_publisher')
    node = GraphPublisher()
    node.spin()

