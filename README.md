[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/maltehue/binder-template/garmi)

This repo configures a Virtual Research Lab (VRL) of the Virtual Research Building (VRB - https://vrb.ease-crc.org/) developed by the AICOR Intitute for Artifical Intelligence (https://ai.uni-bremen.de/).
It contains a ros2 controllable Mujoco simulation of the GARMI Robot developed by the TUM Campus Geratronik (https://www.mirmi.tum.de/geriatronics/startseite/)


In general, a VRL can be considered as a developement plattform in the web. Every user obtains an individual instance that is reset to the initial state after a restart. During interaction with the VRL the user can do whatever he wants. The initial state is defined by this repo. This repo can be forked to build your own VRL with extended functionality and to share it with the community.

The VRL can be entered by clicking the button at the top of the README. 

## Running the Example
Make sure to open a Virtual Desktop in the VRL before running the following.

The command to run the example setups are:

```bash
bash /home/repo/run_demo_block_stacking.bash
```
or

```bash
bash /home/repo/run_demo_simplified_apartment.bash
```
The second command starts a simplfied Mujoco environment of the real world lab environment replicated in the VR Demo below. Currently we are adapting the exisitng models for a smooth Mujoco Simulation of the whole apartment.



https://github.com/user-attachments/assets/23bb63e7-949d-4dd3-9402-f5e301ccfc49



Both of the above commands use tmux to start multiple processes simultaneously. It creates a Multiverse Server (https://multiverseframework.readthedocs.io/en/latest/) that acts as a connector between ros2 processes and the mujoco simulation. It also configures the ros2 controller manager as a Multiverse client. We use a standard joint trajectory controller for each arm, the torso and the head.

It also starts the mujoco simulations as a Multiverse Client. This is achieved by a plugin configuration at the end of the `/home/jovyan/libs/RIG2026/demo/assets/mjcf/scene_position_with_multiverse.xml` file.
The configuration of the ros2 processes and the plugin in the .xml file allow the ros2 controllers to write into the custom multiverse hardware interface of the mujoco simulation.

**Note**: Rendering of the Mujoco Gui can be very slow in the docker based VRL. To increase the framerate, use the menu on the left side of the GUI, scroll down to rednering and disable all the OpenGL Effects except for culling faces.

For interactive control of the robot run

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
This is a standard ros2 tool to command the activated controllers. Other tools can be installed during runtime using the terminal or offline by altering the Dockerfile.


## Development

All the software installed in this VRL is listed in `binder/Dockerfile`.

To build a new VRL from a new repo use this website: https://binder.intel4coro.de/

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
