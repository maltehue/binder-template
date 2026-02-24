[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/maltehue/binder-template/armar7-stable)

This repo configures a Virtual Research Lab (VRL) of the Virtual Research Building (VRB) developed by the AICOR Intitute for Artifical Intelligence (https://ai.uni-bremen.de/).
It contains a ros2 controllable Mujoco simulation of the Armar7 Robot developed by the H2T group at KIT (https://h2t.iar.kit.edu/)

The minimal capabilities of the VRL can be seen in this Video. The VRL can be entered by clicking the button above. 
The video shows the entry point of the VRL and how to enter the Virtual Desktop and the Web IDE (VSCode). The commands to replicate the demo shown in the video can befound below.

TODO:  Upload video

In general, a VRL can be considered as a developement plattform in the web. Every user obtains an individual instance that is reset to the initial state after a restart. During interaction with the VRL the user can do whatever he wants. The initial state is defined by this repo. This repo can be forked to build your own VRL with extended functionality and to share it with the community.

## Running the Example
The commands to run the example from the video are:

```bash
/home/jovyan/Multiverse/Multiverse-Launch/bin/multiverse_launch /home/repo/multiverse_configs/armar7/armar.muv
```
This creates a Multiverse Server (https://multiverseframework.readthedocs.io/en/latest/) that acts as a connector between ros2 processes and the mujoco simulation. The file `armar.muv` also configures thr ros2 controller manager as a Multiverse client. The file can be edited to load different ros2 controller. We use a standard joint trajectory controller for each arm and the torso.

```bash
/home/jovyan/libs/semantic_digital_twin_demo/mujoco/bin/simulate /home/jovyan/libs/semantic_digital_twin_demo/assets/armar7_scene.xml
```
This starts the mujoco simulations as a Multiverse Client. This is achieved by a plugin configuration at the end of the `armar7_scene.xml` file.
The configuration of the .muv file and the plugin in the .xml file allow the ros2 controllers to write into the custom multiverse hardware interface of the mujoco simulation

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
This is a standard ros2 tool to command the activated controllers. Other tools can be isnatlled during runtime using the terminal or offline by altering the Dockerfile.

All the software installed in this VRL is listed in `binder/Dockerfile`.

To build a new VRL from a new repo use this website: https://binder.intel4coro.de/

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
