# Description

This is a meta-package, containing the various packages needed for the Krabi/Kraboss robot.
These packages are git submodules for this packages

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
The "krabi_mgs" package is needed for all the other packages. To build it first, do:
- `catkin_make krabi_msgs_generate_messages_cpp && catkin_make`
