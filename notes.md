# Krabi

## Todo list

- [x] add gz plugin for odometry
- [x] add gz sensor for lidar
- [ ] correct lidar specs in urdf (laserscan or lidar)
- [ ] add gz plugin for magnetic grabber
- [ ] add elevator in urdf
- [ ] add controller for elevator
- [ ] add default spawn locations as an argument to the sim

## Description

Contains urdf, properties and gazebo plugins

Utilities launch files:

- `display.launch.xml`: to visualize the urdf
- `spawn.launch.py`: to spawn the robot in a running gazebo simulation
- `gz_bridge.launch.xml`: to connect the robot gz topic to ros and vice-versa
- `spawn_and_bridge.launch.py`: a launch to both spawn and the bridge

## Simulation

To run the simulation you can open any gazebo world using `gz sim world_name.sdf`

### 2025 Cup simulation

We're taking the simulation from the open-source [ezbot](https://github.com/VincidaB/ezBotV2.git).
We've added the ament_hooks so that by simply sourcing the workspace
you can run the simulation using:

```bash
gz sim Table_2025_with_cans_planks.sdf
```

> **note** *it is better to run the simulation with a smaller time step for
> better accuracy*

### krabby_gazebo

This is a custom package that contains everything needed to run the simulation.
It depends on krabby_description and the ezBotV2 package.

To launch the simulation with krabby use:

```bash
ros2 launch krabby_gazebo eurobot2025.launch.py
# and in another terminal
gz sim -g
```

> **remark** that the launch will run the simulation without the gui

## Controlling the robot

The robot is expecting twist commands on the `/krabby/cmd_vel` topic.

(TODO) A `krabby_teleop` pkg contains the relevant launch files to control
krabby using teleop tools (gui or joystick)

## Stack
