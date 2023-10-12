# Steps to run Snowman

```sh
# Open a terminal and start turtlesim node
ros2 run turtlesim turtlesim_node

# Open another terminal and cd into repo, if you run `ls` you should see `src` directory
# build the snowman ros package
colcon build --packages-select snowman

# source it so you can run in ros
source install/setup.bash   # if on bash
source install/setup.zsh    # if on zsh

# Run the snowman
ros2 run snowman move

# now you should see on turtlesim node, enjoy :)
```

# Steps to run task1_b base

```sh
# install required packages
sudo apt install ros-humble-tf-transformations
sudo pip3 install transforms3d
sudo apt install -f ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-joint-state-publisher

# build the packages
colcon build

# source it so you can run in ros
source install/setup.bash   # if on bash
source install/setup.zsh    # if on zsh

# Launch gazebo
ros2 launch hb_task_1b gazebo.launch.py
```
