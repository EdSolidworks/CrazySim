# CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor
This Simulator is based on the work in the ICRA 2024 paper "CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor" [1]. CrazySim is a simulator platform that runs Crazyflie firmware in a simulation state on a desktop machine with integrated communication with Gazebo sensors and physics engine. The simulated Crazyflie firmware is intended to communicate with a custom Crazyflie Python library ([CFLib](https://github.com/bitcraze/crazyflie-lib-python)) provided in this code. This enables simulating the behavior of CFLib scripts that are intended to control single or multiple Crazyflies in a real hardware demonstration. With CFLib communication capabilities, users can choose to use [CrazySwarm2](https://github.com/IMRCLab/crazyswarm2) with CFLib as the backend for a ROS 2 interface with the simulator.

![](16cfs.gif)

## Citation
If you use CrazySim for an academic publication, then please cite [ICRA2024 paper](https://coogan.ece.gatech.edu/papers/pdf/llanes2024crazysim.pdf) as


```bibtex
@INPROCEEDINGS{LlanesICRA2024,
author = {Llanes, Christian and Kakish, Zahi and Williams, Kyle and Coogan, Samuel},
booktitle = {2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
title = {CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor},
year = {2024}
}
```

# CrazySim Setup

## Supported Platforms
This simulator is supported on Ubuntu 22.04.
The following are instructions for installing the software on a fresh install of Ubuntu 22.04.

Need to install tools used throughout the installation
```bash
sudo apt install git python-is-python3 pip
```

For testing purposes to see if the simulator has been installed correctly and works, the crazyflie client will be installed, see the [documentation](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/) for information on the use of the client.
```bash
git clone https://github.com/llanesc/crazyflie-clients-python
cd crazyflie-clients-python
git checkout sitl-release
pip install -e .
cd ..
```
This code will clone the repository containing all the required code for the client, then move to that newly downloaded directory. It will change it to the custom simulation version and install the software before exiting back to the initial directory

To install this repository use the recursive command as shown below for HTTPS:
```bash
git clone https://github.com/EdSolidworks/CrazyflieSimulator.git --recursive
cd CrazyflieSimulator
```
This code downloads all the code from the github repository and unpacks it.

## crazyflie-lib-python
```bash
cd CrazySim/crazyflie-lib-python
pip install -e .
cd ..
```
This code installs the python library needed to control the crazyflie

### Dependencies
Run the following commands to install dependencies.
```bash
sudo apt install cmake build-essential
pip install Jinja2
```

To download all the prerequisites of gazebo garden run the following code
```bash
sudo apt-get update
sudo apt-get install lsb-release curl gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
```

Then to install gazebo garden, run the following
```bash
sudo apt-get install gz-garden
```

Run the following to build the firmware and Gazebo plugins.
```bash
cd crazyflie-firmware
mkdir -p sitl_make/build && cd $_
cmake ..
make all
```
All the necessary tools should now be installed and the simulator should be ready to use. To use the crazyflie client, open a terminal and type in:
```bash
cfclient
```

## How to use
Currently, users have to restart Gazebo after each CFLib connect and disconnect cycle. Supporting a restart cycle without restarting Gazebo is on the list of things to do.

### Start up SITL
Open a terminal and run
```bash
cd crazyflie-firmware
```

We can then run the firmware instance and spawn the models with Gazebo using a launch script. All launch scripts require a model argument `-m`. All currently implemented models are tabulated below.

| Models | Description |
| --- | --- |
| crazyflie | The default Crazyflie 2.1. |
| crazyflie_thrust_upgrade | The Crazyflie 2.1 with thrust upgrade bundle. |

#### Option 1: Spawning a single crazyflie model with initial position (x = 0, y = 0)
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m crazyflie -x 0 -y 0
```

#### Option 2: Spawning 8 crazyflie models to form a perfect square
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 8 -m crazyflie
```

#### Option 3: Spawning multiple crazyflie models with positions defined in the *agents.txt* file. New vehicles are defined by adding a new line with comma deliminated initial position *x,y*.
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh -m crazyflie
```

Now you can run any CFLib Python script with URI `udp://0.0.0.0:19850`. For drone swarms increment the port for each additional drone.

You can also test a single crazyflie using the custom client if you installed it from the crazyflie-clients-python section.

First start up the custom client.
```bash
cfclient
```

Click on the scan button, select the UDP interface, and connect. Once it's connected you can take off and fly using the command based flight controls.

### PID Tuning Example
One use case for simulating a crazyflie with the client is real time PID tuning. If you created a custom crazyflie with larger batteries, multiple decks, and upgraded motors, then it would be useful to tune the PIDs in a simulator platform before tuning live on hardware. An example of real time PID tuning is shown below.

https://github.com/gtfactslab/Llanes_ICRA2024/assets/40842920/b865127c-1b0d-4f49-941d-e57aecda9a54




# Crazyswarm2 and Model Predictive Control Case Study
This section follows the setup of CrazySwarm2 with CrazySim and demonstrating a case study that uses a model predictive controller (MPC) with Acados to track a set of predefined temporally parametrized trajectories.

Make sure you have ROS 2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). 

Install the following for Crazyswarm2:
```bash
sudo apt install libboost-program-options-dev libusb-1.0-0-dev
pip3 install rowan transforms3d
sudo apt install ros-humble-tf-transformations
```

If you want to run the MPC code then you will need Acados. Acados can be installed by following their [documentation](https://docs.acados.org/installation/index.html).

Then build the ROS 2 workspace.
```bash
cd ros2_ws
colcon build --symlink-install
```

### Configuration
The crazyswarm2  configuration files can be found in 
```bash
ros2_ws/src/crazyswarm2/crazyflie/config/
```
The crazyflies.yaml describes the robots currently being used. If a robot is not in the simulator or hardware, then it can be disabled by setting the enabled parameter to false. A more detailed description for crazyswarm2 configurations can be found [here](https://imrclab.github.io/crazyswarm2/usage.html).

The main code for the MPC script is in the following:
```bash
ros2_ws/crazyflie_mpc/crazyflie_mpc/crazyflie_multiagent_mpc.py
```
The trajectory type can be changed to a horizontal circle, vertical circle, helix, or a lemniscate trajectory by changing the variable "trajectory_type" in the CrazyflieMPC class. There is also a motors variable in the CrazyflieMPC class that can be changed based on if you defined the crazyflie or crazyflie_thrust_upgrade model.

### Start up the Firmware
Start up the firmware with any of the 3 launch script options. Below we demonstrate 4 Crazyflies in a square formation.
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 4 -m crazyflie
```

### Start Crazyswarm2
Make sure that `cf_1`, `cf_2`, `cf_3`, and `cf_4` are enabled in the CrazySwarm2 configuration YAML file. Launch the Crazyswarm2 services with CFLib backend.
```bash
ros2 launch crazyflie launch.py backend:=cflib
```

### Start MPC code
### 
Run the Crazyflie MPC demonstration with the code below. The argument `n_agents` can be modified for the number of agents in your environment. Additionally, the argument `--build_acados` can be defined to compile the Acados optimal control problem.
```bash
ros2 run crazyflie_mpc crazyflie_multiagent_mpc --n_agents=4 --build_acados
```

Using the command line publisher we can command all vehicles to take off using MPC.
```bash
ros2 topic pub -t 1 /all/mpc_takeoff std_msgs/msg/Empty
```

Using the command line publisher we can command all vehicles to start the trajectory.
```bash
ros2 topic pub -t 1 /all/mpc_trajectory std_msgs/msg/Empty
```

Using the command line publisher we can command all vehicles to stop the trajectory and hover.
```bash
ros2 topic pub -t 1 /all/mpc_hover std_msgs/msg/Empty
```

We also implemented a MPC land feature, but it's still experimental and may result in crashing the drone.

## Versions
| Version | Description |
| --- | --- |
| 1.0 | Initial release |
| 1.1 | Added receiver thread for CFLib UdpDriver, new thrust upgrade model to Gazebo, and a seperate MPC solver thread with a queue for storing the controls. |
| 1.2 | Merge crazyflie-firmware with commits up to [dbb09b5](https://github.com/bitcraze/crazyflie-firmware/commit/dbb09b5ca16f0ddf63e98d2c44d247a3aa15f056), update submodule motion_capture_tracking to version 1.0.5, fixed Gazebo sending external pose to firmware (wasn't receiving orientation), cleaned up launch scripts, removed some firmware module copies for sitl. |
