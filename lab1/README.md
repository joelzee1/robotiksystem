## Demos

ROS 2 demo nodes covering topics, services, parameters, and actions.

## Build

```bash
colcon build
source install/setup.bash
```

## Run

Each pair runs in separate terminals.

### Topics

```bash
ros2 run demos talker.py
ros2 run demos listener.py
```

### Services

```bash
ros2 run demos service_server.py
ros2 run demos service_client.py
```

### Parameters

```bash
ros2 run demos param_talker.py --ros-args -p message:="Hello ROS" -p timer_period:=0.5
ros2 run demos config_reader.py --ros-args --params-file src/demos/config/params.yaml
ros2 param set /config_reader message "Updated during runtime"
```

### Actions

```bash
ros2 run demos action_server.py
ros2 run demos action_client.py
```

## Docker

```bash
make build   # build image + workspace
make up      # start container
make shell   # open a shell (one per terminal)
make down    # stop
make clean   # remove everything
```
