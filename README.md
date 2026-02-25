# binder-template

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.intel4coro.de/v2/gh/maltehue/binder-template/main)



## Quick Start

bash to start tiago multiverse server and ros control
```bash
/home/jovyan/Multiverse/Multiverse-Launch/bin/multiverse_launch /home/repo/multiverse_configs/tiago/tiago_position.muv
````

bash to start tiago mujoco simulation
```bash
/home/jovyan/libs/semantic_digital_twin_demo/mujoco/bin/simulate /home/jovyan/libs/semantic_digital_twin_demo/assets/apartment_with_tiago_dual.xml
```

bash for launchging giskardpy tiago
```bash
ros2 launch giskardpy_ros tiago_velocity.launch.py
```

bash for running a demo
```bash
python /home/repo/demo.py
```

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
