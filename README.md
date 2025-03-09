# rover_bot
---

```
this is a six drive wheel robot with a rocker-bogie mechanism.
It has it own drive controller and can climb rough uneven terrain
easily

clone (git clone git@github.com:samuko-things/mobo_bot.git) or Download
the repo or in your ROS2 workspace, build, and source it.
```
## Prequisite
- install ros humble if you haven't - [ros humble PC installation](https://robocre8.gitbook.io/robocre8/tutorials/how-to-install-ros2-humble-desktop-on-pc-full-install)

- install gazebo classic and all it's ros packages (Pls note gazebo classic will reach its EOL by January 2025)
> ```shell
> sudo apt install ros-humble-gazebo-ros-pkgs
> sudo apt install ros-humble-gazebo-ros2-control
> ```

## Create ROS Workspace And Download and Setup mobo_bot Packages
- install python pynput library for the mobo_bot_teleop
  ```shell
  pip3 install pynput
  ```
  
- create your <ros_ws> in the home dir. (replace <ros_ws> with your workspace name)
  ```shell
  mkdir -p ~/<ros_ws>/src
  cd ~/<ros_ws>
  colcon build
  source ~/<ros_ws>/install/setup.bash
  ```

- cd into the src folder of your <ros_ws> and download the **rover_bot** packages
  ```shell
  cd ~/<ros_ws>/src
  git clone https://github.com/samuko-things/rover_bot.git
  ```

- build your <ros_ws>
  ```shell
  cd ~/<ros_ws>/
  colcon build --symlink-install
  ```

- don't forget to source your <ros_ws> in any new terminal
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ```

## Basic Launch
- to spawn robot in gazebo:
  ```shell
    ros2 launch rover_bot_description spawn.launch.py
  ```

![rover_bot](https://github.com/samuko-things/rover_bot/blob/main/rover_bot_pics1.png)
- to spawn robot in gazebo:
  ```shell
    ros2 launch rover_bot_description spawn.launch.py
  ```


![rover_bot](https://github.com/samuko-things/rover_bot/blob/main/rover_bot_pics2.png)
- to spawn robot in gazebo with a test terrain:
  ```shell
    ros2 launch rover_bot_description spawn_with_terrain.launch.py
  ```

- to spawn robot in gazebo and view in RVIZ simultaneously (although with some transorm issues):
  ```shell
    ros2 launch rover_bot_description spawn_and_view.launch.py
  ```

you can drive it around using the teleop_twist_keyboard package or you can 
also use the [pynput_teleop_twist_keyboard](https://github.com/samuko-things/pynput_teleop_twist_keyboard.git) package I wrote.


### How to drive rover_bot 
- before you can drive the **rover_bot**, you'll need to first start the drive controller after launching the simulation

- to launch the drive controller node, open a new terminal, source your workspace and run the following
  ```shell
    ros2 run rover_bot_drive_controller drive_controller
  ```  

- you should now be able to drive the **rover_bot** with the telop package

- I recommend driving the **rover_bot** with the [**arrow_key_teleop_drive**](https://github.com/samuko-things/arrow_key_teleop_drive) package I created to make driving robot easy using just your keyboard arrow keys.

- if you,ve downloaded and build the **arrow_key_teleop_drive**, run this command to drive the robot. don't forget to source your workspace.
  ```shell
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive
  ```
  OR
  ```shell
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive <v in m/s> <w in rad/sec>
  ```
