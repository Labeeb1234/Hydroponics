#!/usr/bin/env python3
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import amr_mani
import numpy as np



def main():
    rclpy.init()
    try:
        node = Node("pose_plan_node")
        callback_grp = ReentrantCallbackGroup()

        position = [0.0, 0.0, 0.5]
        quat = [0.0, 0.0, 0.0, 1.0]

        moveit2 = MoveIt2(
            node=node,
            joint_names=amr_mani.joint_names(),
            base_link_name=amr_mani.base_link_name(),
            end_effector_name=amr_mani.end_effector_name(),
            group_name=amr_mani.MOVE_GROUP_ARM, 
            callback_group=callback_grp,
        )

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        node.get_logger().info( f"Moving to position: {position}, quat_xyzw: {quat}")
        moveit2.move_to_pose(position=position, quat_xyzw=quat)
        moveit2.wait_until_executed()
    except KeyboardInterrupt:
        node.get_logger().info("Execution interrupted by user.")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        # Ensure proper cleanup
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()