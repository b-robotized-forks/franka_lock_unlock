# Franka Lock Unlock

Locking and unlocking of Franka Emika Panda and Franka Research 3 (FR3) joint brakes programmatically. Open or close all joint locks either from the command-line, or from any Python program.

Also supports the activation of the Franka Control Interface (FCI) and other options.

## Problem Description

While the Franka Panda is a great robot for research and industrial use cases, it lacks the option of unlocking or locking its joints from a different source other than the Franka Desk Web UI. However, this is crucial if you want to automate the entire startup and shutdown phase of the robot. That is now possible with the introduction of this package.

## Command-Line Usage

Ensure to have python3 installed.

To start the robot, run the following:

```
usage: ./franka_lock_unlock.py [-h] [-u] [-l] [-w] [-r] [-p] [-c] [-i] hostname username password

positional arguments:
  hostname          The Franka Desk IP address or hostname, for example "1.2.3.4".
  username          The Franka Desk username, usually "admin".
  password          The Franka Desk password.

optional arguments:
  -h, --help        show this help message and exit
  -u, --unlock      Unlock the brakes. Otherwise, lock them.
  -l, --relock      Relock the brakes on exit.
  -w, --wait        Wait in case the robot web UI is currently in use.
  -r, --request     Request control by confirming physical access to the robot in case the robot web UI is currently in use.
  -p, --persistent  Keep the connection to the robot open persistently.
  -c, --fci         Activate the FCI.
  -i, --home        Home the gripper.
```

To shut the robot down, run the following:

```
usage: ./franka_shutdown.py [-h] [-w] [-r] hostname username password

Shutdown the Franka Emika Panda programmatically.

positional arguments:
  hostname       The Franka Desk IP address or hostname, for example "1.2.3.4".
  username       The Franka Desk username, usually "admin".
  password       The Franka Desk password.

options:
  -h, --help     show this help message and exit
  -w, --wait     Wait in case the robot web UI is currently in use.
  -r, --request  Request control by confirming physical access to the robot in case the robot web UI is currently in use.
```

## ROS Package

This repository exports a ROS package. This is useful for starting up the robot and FCI by means of a single launch file that also spawns the `franka_ros` interface at the same time. It will speed up your entire robot workflow dramatically.

The ROS package can be installed and invoked by the following commands.

### Installation

```sh
git clone https://github.com/jk-ethz/franka_lock_unlock.git
colcon build --packages-up-to franka_lock_unlock
```

### Simple Usage

```sh
ros2 run franka_lock_unlock franka_lock_unlock <PARAMS>
ros2 run franka_lock_unlock franka_shutdown <PARAMS>
```

### Advanced Usage

If you want to launch the controller or gripper while unlocking the joints, ensure to add the `respawn=true` parameter to the node tag, such that the control node retries to connect to the robot until all joints have been fully unlocked and the FCI has been activated.

```Python
Node(
    package='franka_gripper',
    executable='franka_gripper_node',
    name='panda_gripper',
    respawn=True,
)
```

The following launch file unlocks the joints, activates the FCI and connects to the robot via ROS. Also, it allows to re-lock the brakes automatically as soon as the launch file is exited via `SIGINT` or `SIGTERM` (`CTRL+C`).

```sh
ros2 launch franka_lock_unlock start.launch.xml hostname:=$HOSTNAME_OR_IP username:=$USERNAME password:=$PASSWORD
```

If you want to shut the robot down, you can use the related launch file with

```sh
ros2 launch franka_lock_unlock shutdown.launch.xml hostname:=<HOSTNAME> username:=<USERNAME> password:=<PASSWORD>
```

### ROS2 Lifecycle Usage

If you want to launch the controller as a ROS2 Lifecycle Node, follow the below commands.

```bash
ros2 launch franka_lock_unlock franka_lock_unlock.launch.xml hostname:=$HOSTNAME_OR_IP username:=$USERNAME password:=$PASSWORD

# configure
ros2 lifecycle set /franka_lock_unlock_node configure

# activate
ros2 lifecycle set /franka_lock_unlock_node activate

# deactivate
ros2 lifecycle set /franka_lock_unlock_node deactivate

# cleanup
ros2 lifecycle set /franka_lock_unlock_node cleanup

# shutdown
ros2 lifecycle set /franka_lock_unlock_node shutdown
```

#### Launch Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `hostname` | `localhost` | The Franka Desk IP address or hostname. |
| `username` | `user` | The Franka Desk username. |
| `password` | `strongpwd` | The Franka Desk password. |
| `robot_type` | `fr3` | Robot type: `fr3` or `panda`. |
| `enable_relock` | `true` | Relock the brakes on exit. |
| `wait_web_ui` | `true` | Wait in case the robot web UI is currently in use. |
| `request_physical_access` | `true` | Request control by confirming physical access to the robot in case the robot web UI is currently in use. Requires `wait_web_ui:=true`. |
| `enable_fci` | `true` | Activate the Franka Control Interface (FCI). |
| `log_level` | `debug` | Log level of the node. |
| `namespace` | `""` | Namespace of the node, e.g. `robot1` or `cell_a/robot1`. |

### Namespaces

All launch files accept an optional `namespace` argument. This allows running one node per robot, e.g. in a multi-robot setup. Without it, the node runs in the root namespace as before.

```bash
ros2 launch franka_lock_unlock franka_lock_unlock.launch.xml namespace:=robot1 hostname:=$HOSTNAME_OR_IP username:=$USERNAME password:=$PASSWORD
```

The node is then available as `/robot1/franka_lock_unlock_node`, so the lifecycle commands have to use the namespaced name:

```bash
ros2 lifecycle set /robot1/franka_lock_unlock_node configure
ros2 lifecycle set /robot1/franka_lock_unlock_node activate
```

Two robots can be handled side by side by launching the file once per robot with a different `namespace` and `hostname`:

```bash
ros2 launch franka_lock_unlock franka_lock_unlock.launch.xml namespace:=robot1 hostname:=$ROBOT1_IP username:=$USERNAME password:=$PASSWORD
ros2 launch franka_lock_unlock franka_lock_unlock.launch.xml namespace:=robot2 hostname:=$ROBOT2_IP username:=$USERNAME password:=$PASSWORD
```

The `log_level` argument is applied to the namespaced logger (e.g. `robot1.franka_lock_unlock_node`), so it keeps working with any namespace.

The same `namespace` argument is available for `start.launch.xml` and `shutdown.launch.xml`:

```sh
ros2 launch franka_lock_unlock start.launch.xml namespace:=robot1 hostname:=$HOSTNAME_OR_IP username:=$USERNAME password:=$PASSWORD
```

To include the launch file from another XML launch file, pass the argument as usual:

```xml
<include file="$(find-pkg-share franka_lock_unlock)/launch/franka_lock_unlock.launch.xml">
    <arg name="namespace" value="robot1" />
    <arg name="hostname" value="$(var robot1_ip)" />
    <arg name="username" value="$(var username)" />
    <arg name="password" value="$(var password)" />
</include>
```

> **Note:** to run without a namespace, omit the argument. `ros2 launch` rejects an empty value such as `namespace:=`.

### Lifecycle Stress Test

`test/hard_test.sh` cycles the lifecycle node through configure/activate/deactivate/cleanup/shutdown. Pass the namespace as the first argument if the node is namespaced:

```bash
./test/hard_test.sh           # /franka_lock_unlock_node
./test/hard_test.sh robot1    # /robot1/franka_lock_unlock_node
```
