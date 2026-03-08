[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/maltehue/binder-template/armar7-stable)

This repo configures a Virtual Research Lab (VRL) of the Virtual Research Building (VRB - https://vrb.ease-crc.org/) developed by the AICOR Intitute for Artifical Intelligence (https://ai.uni-bremen.de/).
It contains a ros2 controllable Mujoco simulation of the Armar7 Robot developed by the H2T group at KIT (https://h2t.iar.kit.edu/)

The minimal capabilities of the VRL can be seen in this Video. The VRL can be entered by clicking the button above. 
The video shows the entry point of the VRL and how to enter the Virtual Desktop and the Web IDE (VSCode). The commands to replicate the demo shown in the video can befound below.



https://github.com/user-attachments/assets/349aec30-6c9a-4ef9-bb22-863bfa95d254



In general, a VRL can be considered as a developement plattform in the web. Every user obtains an individual instance that is reset to the initial state after a restart. During interaction with the VRL the user can do whatever he wants. The initial state is defined by this repo. This repo can be forked to build your own VRL with extended functionality and to share it with the community.

## Running the Example
Make sure to open a Virtual Desktop in the VRL before running the following.

The commands to run the example from the video are:

```bash
/home/jovyan/Multiverse/Multiverse-Launch/bin/multiverse_launch /home/repo/multiverse_configs/armar7/armar.muv
```
This creates a Multiverse Server (https://multiverseframework.readthedocs.io/en/latest/) that acts as a connector between ros2 processes and the mujoco simulation. The file `armar.muv` also configures thr ros2 controller manager as a Multiverse client. The file can be edited to load different ros2 controller. We use a standard joint trajectory controller for each arm and the torso.

```bash
/home/jovyan/libs/semantic_digital_twin_demo/mujoco/bin/simulate /home/jovyan/libs/semantic_digital_twin_demo/assets/armar7_scene.xml
```
This starts the mujoco simulations as a Multiverse Client. This is achieved by a plugin configuration at the end of the `armar7_scene.xml` file.
The configuration of the .muv file and the plugin in the .xml file allow the ros2 controllers to write into the custom multiverse hardware interface of the mujoco simulation.

**Note**: Rendering of the Mujoco Gui is very slow in the docker based VRL. To increase the Framerate, use the menu on the left side of the GUI, scroll down to rednering and disable all the OpenGL Effects except for culling faces.

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
This is a standard ros2 tool to command the activated controllers. Other tools can be isnatlled during runtime using the terminal or offline by altering the Dockerfile.

### Running an Example with CRAM and Physics Simulation
CRAM (https://github.com/cram2/cognitive_robot_abstract_machine) is a cognitve architecture for robot control programms. It represents semantic world knowledge, represents and executes generalized robot manipulation plans, utilizie whole-body robot motion control and much more. If you are interested to see the code in the VRL, open VSCode and open the folder `/home/jovyan/libs/cognitive_robot_abstract_machine/`,

To use CRAM with the simulated robot we need to change the control interface from a position controller to a velocity controller, because Giskard (the whole-body motion controller in CRAM) outputs joint velocities. Currently the mujoco simulations are setup with position based actuators, therefore we employ a custom ros2 controller (https://github.com/code-iai/iai_hsr/tree/ros2-jazzy/hsr_velocity_controller) that accepts velocity commands from Giskards and translates them into position commands for the ros2 Hardware Interface.

To change that interface in the VRL open the file `armar.muv` and change this block

```bash
controllers:
  spawner:
    - joint_state_broadcaster
      right_arm_joint_trajectory_controller
      left_arm_joint_trajectory_controller
      torso_joint_trajectory_controller
```
to this block

```bash
controllers:
  spawner:
    - joint_state_broadcaster
      realtime_body_controller_real
```

the feedforward and PID gains of the velocity controller can be tuned in the file `ros2_control.yaml`.

After doing the desired changes reuse the first teo comamnds from above to start the robot simulation and the ros2 interfaces.
Afterwards start Giskard with the command. When it fails, just retry.

```bash
ros2 launch giskardpy_ros armar_velocity.launch.py
```

Now Giskard is running and publishes an Interactive Marker that can be visualized in Rviz2 and be used to command end effector goals.
Alternatively run the file `demo.py` by calling

```bash
python /home/repo/demo.py
```

To execute a minimal CRAM plan for the ARMAR7. By opening the file in the Web VSCode application and configuring the correct venv (as shown in the video) you obtain code suggestions to faccilitate writing your own robot plans. Documentation on the CRAM system can be found here (https://cram2.github.io/cognitive_robot_abstract_machine/).

### Running an Example with CRAM without Physics Simulation

To start RViz2 open a new terminal and run
```bash
rviz2
```
Add a MarkerArray to RViz, which listens to the topic `/semworld/viz_marker`, and hast the durability policy set to `Transient Local`.

Then open another terminal and run

```bash
python /home/repo/no_simulation_demo.py
```

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
