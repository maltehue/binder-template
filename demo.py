import os
import time

import numpy as np
import rclpy
from pycram.datastructures.pose import PoseStamped

from pycram.datastructures.dataclasses import Context

from pycram.language import SequentialPlan

from pycram.datastructures.enums import Arms
from pycram.motion_executor import simulated_robot, real_robot

from pycram.robot_plans import (
    ParkArmsActionDescription,
    SetGripperActionDescription,
    MoveTorsoActionDescription,
    TransportActionDescription,
)
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.ros.pose_publisher import PosePublisher

from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.tfwrapper import TFWrapper
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.definitions import GripperState, TorsoState
from semantic_digital_twin.robots.tiago import TiagoMujoco
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from rclpy.executors import SingleThreadedExecutor
import threading


def start_node_spin(node) -> None:
    """Spin the underlying ROS 2 node in a background thread.

    Ensures timers, subscriptions, and publishers that rely on callbacks are processed
    without blocking the main thread.
    """
    _executor = SingleThreadedExecutor()
    _executor.add_node(node)
    _spin_thread = threading.Thread(target=_executor.spin, daemon=True)
    _spin_thread.start()

rclpy.init()
rclpy_node = rclpy.create_node("test_node")
start_node_spin(rclpy_node)

world = fetch_world_from_service(
        rclpy_node,
    )

tiago = TiagoMujoco.from_world(world=world)




context = Context(world, tiago, ros_node=rclpy_node)
description = ParkArmsActionDescription([Arms.BOTH])
plan = SequentialPlan(
    context,
    description,
)

with real_robot:
    plan.perform()



