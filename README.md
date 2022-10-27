# rover_bot
---

![rover_bot](https://github.com/samuko-things/rover_bot/blob/main/rover_bot_pics1.png)

![rover_bot video](https://github.com/samuko-things/rover_bot/blob/main/rover_bot_vid1.mp4)

```
clone (git clone git@github.com:samuko-things/mobo_bot.git) or Download
the repo or in your ROS2 workspace, build, and source it.
```
## Basic Launch
to view robot in RVIZ:
```shell
$ ros2 launch rover_bot_description view.launch.py
```

to spawn robot in gazebo:
```shell
$ ros2 launch rover_bot_description spawn.launch.py
```

to spawn robot in gazebo and view in RVIZ simultaneously (although with some transorm issues):
```shell
$ ros2 launch rover_bot_description spawn_and_view.launch.py
```

you can drive it around using the teleop_twist_keyboard package or you can 
also use the [pynput_teleop_twist_keyboard](https://github.com/samuko-things/pynput_teleop_twist_keyboard.git) package I wrote.
