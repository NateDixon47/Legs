#!/usr/bin/env python3
"""Broadcast the odom -> base_link TF from MuJoCo's floating-base odometry.

mujoco_ros2_control publishes the free joint pose as nav_msgs/Odometry
(frame_id=odom, child=the MJCF 'base' body) on /simulator/floating_base_state.
The MJCF 'base' body frame and the URDF base_link frame share the same orientation:
both orient the torso mesh by the same rotation (the MJCF torso geom quat was
generated from the URDF base_to_body rpy 1.5708,0,1.5708 -> [[0,0,1],[1,0,0],[0,1,0]]).
So the odometry pose maps straight onto base_link with no extra rotation. (The R from
the footPose==Leg::FK cross-check lives in the leg kinematic chain, not the base frame.)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, "/simulator/floating_base_state", self.on_odom, 10
        )

    def on_odom(self, msg):
        t = TransformStamped()
        # Reuse the message's sim-time stamp so TF lines up under use_sim_time.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        p = msg.pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z

        # Base frames share orientation, so copy the odometry orientation directly.
        t.transform.rotation = msg.pose.pose.orientation

        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
