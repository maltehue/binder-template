# Dockerfile of the base image: https://github.com/IntEL4CoRo/jupyter-ros2/blob/main/Dockerfile
FROM intel4coro/jupyter-ros2:jazzy-py3.12

USER root
#============= Install extra software packages =============#
# RUN apt update && apt install -y curl

# Install python packages
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt
#===========================================================#
# ------------------------------------------------------------
# Required
# ------------------------------------------------------------
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-control-msgs \
    ros-jazzy-rqt-robot-steering \
    graphviz graphviz-dev \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Install demo repository and cram code
# ------------------------------------------------------------

RUN pip install poetry
RUN set -ex && \
    poetry config virtualenvs.create false && \
    mkdir -p ${HOME}/libs && cd ${HOME}/libs && \
    git clone https://github.com/maltehue/semantic_digital_twin_demo.git && \
    git clone https://github.com/cram2/cognitive_robot_abstract_machine.git &&\
    cd cognitive_robot_abstract_machine && \
    sed -i '/^[[:space:]]*random_events[[:space:]]*=.*$/d' pyproject.toml && \
    poetry install

# ------------------------------------------------------------
# Multiverse installation
# ------------------------------------------------------------

ENV MV_ROOT=${HOME}/Multiverse
ENV MULTIVERSE_REPO=https://github.com/Multiverse-Framework/Multiverse
ENV MULTIVERSE_BRANCH=main

RUN set -ex && \
    mkdir -p ${MV_ROOT} && \
    cd ${HOME} && \
    git clone --recurse-submodules --depth 1 --branch ${MULTIVERSE_BRANCH} ${MULTIVERSE_REPO} ${MV_ROOT} || \
    (cd ${MV_ROOT} && git submodule update --init --recursive) && \
    cd ${MV_ROOT} && \
    if [ -f "./Multiverse-Launch/requirements.txt" ]; then \
    pip install --no-cache-dir -r ./Multiverse-Launch/requirements.txt; fi && \
    if [ -f "./Multiverse-Utilities/requirements.txt" ]; then \
    pip install --no-cache-dir -r ./Multiverse-Utilities/requirements.txt; fi && \
    if [ -f "./Multiverse-Launch/src/multiverse_connectors/multiverse_simulators_connector/src/mujoco_connector/requirements.txt" ]; then \
    pip install --no-cache-dir -r ./Multiverse-Launch/src/multiverse_connectors/multiverse_simulators_connector/src/mujoco_connector/requirements.txt; fi && \
    if [ -d "./Multiverse-Launch/src/multiverse_connectors/multiverse_ros_connector/ros_ws/multiverse_ws2" ]; then \
    cd ./Multiverse-Launch/src/multiverse_connectors/multiverse_ros_connector/ros_ws/multiverse_ws2 && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"; \
    cd ${MV_ROOT}; \
    fi


# ------------------------------------------------------------
# ROS2 workspace: giskardpy_ros (clone, deps, build)
# ------------------------------------------------------------
RUN pip install xacro

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}

# Create workspace, fetch sources, install Python reqs and system deps, then build
RUN set -ex && \
    mkdir -p ${HOME}/giskard_ws/src && cd ${HOME}/giskard_ws/src && \
    git clone https://github.com/SemRoCo/giskardpy_ros.git -b tiago_velocity_semdt && \
    git clone https://github.com/code-iai/iai_tiago_description.git -b ros2-main && \
    pip install -r giskardpy_ros/requirements.txt && \
    cd ${HOME}/giskard_ws && \
    vcs import src < src/giskardpy_ros/$ROS_DISTRO.repos && \
    rosdep update --rosdistro=$ROS_DISTRO && \
    apt-get update && rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} && \
    /bin/bash -lc "set -ex; source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${HOME}/giskard_ws && colcon build"


RUN set -ex && \
    cd ${HOME}/giskard_ws/src && git clone https://github.com/code-iai/iai_hsr.git -b ros2-humble && cd iai_hsr && rm -rf hsr_description hsr_navigation iai_hsr_bringup && \
    cd ${HOME}/giskard_ws && /bin/bash -lc "set -ex; source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${HOME}/giskard_ws && colcon build --cmake-args -DBUILD_TESTING=OFF"
# Note: Sourcing the workspace at runtime should be done in the entrypoint or shell:
#   source ~/giskard_ws/install/setup.bash

RUN pip install "setuptools<81"

USER root
# Copy content to the image
ENV REPO_DIR=/home/repo
RUN mkdir -p ${REPO_DIR}
COPY . ${REPO_DIR}/
RUN git config --global --add safe.directory ${REPO_DIR}

# Set JupyterLab default directory
WORKDIR ${REPO_DIR}

# Set VSCODE default directory
ENV CODE_WORKING_DIRECTORY=${REPO_DIR}

# The entrypoint of the docker image
COPY binder/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
