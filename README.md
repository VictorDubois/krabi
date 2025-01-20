<!--toc:start-->
- [Description](#description)
- [Documentation](#documentation)
- [Clone](#clone)
- [Update](#update)
- [Build](#build)
  - [Install dependencies](#install-dependencies)
  - [Compile workspace](#compile-workspace)
- [Run](#run)
<!--toc:end-->

# Description

This is a meta-package, containing the various packages needed
for the Krabi/Kraboss robot.
These packages are git submodules for this packages

# Documentation

[Have a look at the wiki](https://github.com/VictorDubois/krabi/wiki)

# Clone

- go to your catkin workspace's src folder
- `git clone git@github.com:VictorDubois/krabi.git --recurse-submodules`

If you have just cloned it regularly (without --recurse-submodules),
you can still init the submodules:

- `git submodule init`
- `git submodule update`

# Update

To update the submodules, once initialized:

- `git submodule update`

# Build

## Install dependencies

```shell
rosdep install --from-paths src -iry
```

## Compile workspace

```shell
colcon build --merge-install --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' -Wall -Wextra -Wpedantic
```

# Run

To run the robot within the simulation do:

- `source devel/setup.zsh`
- `roslaunch krabi_bringup kraboss.launch isBlue:=false`
