# Description

This is a meta-package, containing the various packages needed for the Krabi/Kraboss robot.
These packages are git submodules for this packages

# Documentation
[Have a look at the wiki](https://github.com/VictorDubois/krabi/wiki)

# Clone 
- go to your catkin workspace's src folder
- `git clone git@github.com:VictorDubois/krabi.git --recurse-submodules`

If you have just cloned it regularly (without --recurse-submodules), you can still init the submodules:
- `git submodule init`
- `git submodule update`

# Update
To update the submodules, once initialized:
- `git submodule update`

# Build
To build all the projects:
- `catkin build`
To build in debug, run this command before:
- `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug`

# Run
To run the robot within the simulation do:
- `source devel/setup.zsh`
- `roslaunch krabi_bringup kraboss.launch isBlue:=false`
