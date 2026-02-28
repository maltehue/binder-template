# binder-template

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/maltehue/binder-template/HEAD)

This repo configures a Virtual Research Lab (VRL) of the Virtual Research Building (VRB - https://vrb.ease-crc.org/) developed by the AICOR Intitute for Artifical Intelligence (https://ai.uni-bremen.de/).
It contains a ros2 controllable Mujoco simulation of the TiaGo Robot.

The minimal capabilities of the VRL can be seen in this Video. The VRL can be entered by clicking the button above. 
The video shows the entry point of the VRL and how to enter the Virtual Desktop and the Web IDE (VSCode). The commands to replicate the demo shown in the video can befound below.



https://github.com/user-attachments/assets/387addb1-3f52-485e-909a-83673e57c238



## Running the Example
Make sure to open a Virtual Desktop in the VRL before running the following.

The commands to run the example from the video are:

bash to start tiago multiverse server and ros control
```bash
/home/jovyan/Multiverse/Multiverse-Launch/bin/multiverse_launch /home/repo/multiverse_configs/tiago/tiago_position.muv
````
This creates a Multiverse Server (https://multiverseframework.readthedocs.io/en/latest/) that acts as a connector between ros2 processes and the mujoco simulation. The .muv file also configures the ros2 controller manager as a Multiverse client. The file can be edited to load different ros2 controller. We use a custom velocity controller for both arms and the torso. The base is moved by a `/cmd_vel` topic.

bash to start tiago mujoco simulation
```bash
/home/jovyan/libs/semantic_digital_twin_demo/mujoco/bin/simulate /home/jovyan/libs/semantic_digital_twin_demo/assets/apartment_with_tiago_dual.xml
```
This starts the mujoco simulations as a Multiverse Client. This is achieved by a plugin configuration at the end of the `apartment_with_tiago_dual.xml` file. The configuration of the .muv file and the plugin in the .xml file allow the ros2 controllers to write into the custom multiverse hardware interface of the mujoco simulation.

bash for launchging giskardpy tiago
```bash
ros2 launch giskardpy_ros tiago_velocity.launch.py
```
This launches the Giskardpy whole-body motion controller of the CRAM Architecture (https://github.com/cram2/cognitive_robot_abstract_machine). This also runs an Interactive Marker that can be used vie the Rviz2 GUI to dirrect command endeffector positions of the robot.

bash for running a demo
```bash
python /home/repo/demo.py
```
This execute a minimal CRAM plan for the TiaGo. By opening the file in the Web VSCode application and configuring the correct venv (as shown in the video) you obtain code suggestions to faccilitate writing your own robot plans. Documentation on the CRAM system can be found here (https://cram2.github.io/cognitive_robot_abstract_machine/).

## Development
All the software installed in this VRL is listed in binder/Dockerfile.

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
