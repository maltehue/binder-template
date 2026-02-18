#!/bin/bash

# Launch the ROS2
source ${ROS_PATH}/setup.bash
source ${MV_ROOT}/Multiverse-Launch/src/multiverse_connectors/multiverse_ros_connector/ros_ws/multiverse_ws2/install/setup.bash
source ${MV_ROOT}/../giskard_ws/install/setup.bash
# Add other startup programs here

# The following line will allow the binderhub start Jupyterlab, should be at the end of the entrypoint.
exec "$@"