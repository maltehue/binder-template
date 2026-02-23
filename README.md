# binder-template

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/maltehue/binder-template/fera-stable)

This repo configures a Virtual Research Lab (VRL) of the Virtual Research Building (VRB) developed by the AICOR Intitute for Artifical Intelligence (https://ai.uni-bremen.de/).
It contains a ros2 controllable Mujoco simulation of a UR5e arm with a Robotiq Gripper, and a sliding door. THis setup is created in collaboration with the FERA research project.

The minimal capabilities of the VRL can be seen in this Video. The VRL can be entered by clicking the button above. 

[docs/video.mp4](docs/video.mp4)

In general, a VRL can be considered as a developement plattform in the web. Every user obtains an individual instance that is reset to the initial state after a restart. During interaction with the VRL the user can do whatever he wants. The initial state is defined by this repo. This repo can be forked to build your own VRL with extended functionality and to share it with the community.

## Running the Example
The commands to run the example from the video are:

```bash
/home/jovyan/Multiverse/Multiverse-Launch/bin/multiverse_launch /home/repo/multiverse_configs/fera/fera.muv
```
This creates a Multiverse Server (https://multiverseframework.readthedocs.io/en/latest/) that acts as a connector between ros2 processes and the mujoco simulation. The file `fera.muv` also configures thr ros2 controller manager as a Multiverse client. The file can be edited to load different ros2 controller. We use a custom controller that accepts velocity commands and writes position values into the hardware inteface of the simulated robot, as we configured the mujoco simulation with position actuators. This repo also contains configuration for standard position trajectory controllers that could be used instead.

```bash
/home/jovyan/libs/semantic_digital_twin_demo/mujoco/bin/simulate /home/jovyan/libs/semantic_digital_twin_demo/assets/fera/scene.xml
```
This starts the mujoco simulations as a Multiverse Client. This is achieved by a plugin configuration at the end of the `fera/scene.xml` file.
The configuration of the .muv file and the plugin in the .xml file allow the ros2 controllers to write into the custom multiverse hardware interface of the mujoco simulation


```bash
ros2 launch giskardpy_ros ur5_velocity.launch.py
```
This launches the ros interfaces of the giskardpy high level motion controller (https://github.com/SemRoCo/giskardpy_ros/tree/tiago_velocity_semdt).
It is part of the cognitive robot abstract machine (CRAM)(https://github.com/cram2/cognitive_robot_abstract_machine), which is also installed in this VRL, and that can be used to create a semantic digital twin of the simulated environment which is the basis for writing complex robot plans. 

## Development

### Run and build docker image Locally (Under repo directory)

First edit the docker-compose.yml to have `user: root`. This eases the interaction with the command line when inside the lab.

- Build and run docker image:

  ```bash
  docker compose -f ./binder/docker-compose.yml up --build
  ```

- Open Web browser and go to http://localhost:8888/

- To stop and remove container:

  ```bash
  docker compose -f ./binder/docker-compose.yml down
  ```
