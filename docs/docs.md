# Installing/configuring/testing lidar06

## Enviroment

 - Raspberry Pi 4 Gb

 - Ubuntu 24 lts server
 
 - lidar06

 ## Installation

### Enviroment

 In ` /boot/firmware/cmdline.txt ` remove `console=serial0,115200,`  as we are going to use Serial0.

 The file is one liner.




> [!TIP]
>if using kitty terminal on the host some commands(nano, top e. t .c.) may not work, can be fixed by:
`export TERM=xterm-256color`

### Tools

minicom is a program that communictes with devices connected to serial ports. It can help to check the connection. I used it to identify whether TX and PWM are not swapped.

```
sudo apt install minicom
 sudo minicom -D /dev/ttyS0 -b 230400
 ```
if everything is correct it must show random characters.

install ROS2 jazzy:
```
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
```

now, install the driver:
```
https://deepwiki.com/ldrobotSensorTeam/ldlidar_stl_ros2/2.1-installation-and-build
```
this link gives instructions on instalation, follow them, below there are changes that ive apllied for those instructions to be sucsessful:

For some reason the code of the driver gave me problems. I suppose that there are differences in versions of compilator/old packages, here is how to edit files to make them work:

Here is what ive got at first launch with colcon, it shows the problem:

```
 colcon build
Starting >>> ldlidar_stl_ros2
[Processing: ldlidar_stl_ros2]                             
--- stderr: ldlidar_stl_ros2                               
/home/rpi/ldlidar_ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp: In member function ‘void LogModule::InitLock()’:
/home/rpi/ldlidar_ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp:172:3: error: ‘pthread_mutex_init’ was not declared in this scope; did you mean ‘pthread_mutex_t’?
  172 |   pthread_mutex_init(&mutex_lock_,NULL);
      |   ^~~~~~~~~~~~~~~~~~
      |   pthread_mutex_t
/home/rpi/ldlidar_ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp: In member function ‘void LogModule::RealseLock()’:
/home/rpi/ldlidar_ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp:180:9: error: ‘pthread_mutex_unlock’ was not declared in this scope; did you mean ‘pthread_mutex_t’?
  180 |         pthread_mutex_unlock(&mutex_lock_);
      |         ^~~~~~~~~~~~~~~~~~~~
      |         pthread_mutex_t
/home/rpi/ldlidar_ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp: In member function ‘void LogModule::Lock()’:
/home/rpi/ldlidar_ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp:188:9: error: ‘pthread_mutex_lock’ was not declared in this scope; did you mean ‘pthread_mutex_t’?
  188 |         pthread_mutex_lock(&mutex_lock_);
      |         ^~~~~~~~~~~~~~~~~~
      |         pthread_mutex_t
/home/rpi/ldlidar_ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp: In member function ‘void LogModule::UnLock()’:
/home/rpi/ldlidar_ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp:196:9: error: ‘pthread_mutex_unlock’ was not declared in this scope; did you mean ‘pthread_mutex_t’?
  196 |         pthread_mutex_unlock(&mutex_lock_);
      |         ^~~~~~~~~~~~~~~~~~~~
      |         pthread_mutex_t
```

fix:

`vim ~/ldlidar_ros2_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp` 

and add :

`#include <pthread.h>` 
in the beginning.

In the bottom of the page there are two verifications, the first one didnt work for me, tho the driver itself worked. 

```
Build Verification
Verify successful compilation by checking for the main executable:
ls ~/ldlidar_ros2_ws/install/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node
```

I have got:

```
ls: cannot access '/home/rpi/ldlidar_ros2_ws/install/lib/ldlidar_stl_ros2/': No such file or directory
```

Though, the node was build and exists:
```
ros2 pkg executables | grep ldlidar
ldlidar_stl_ros2 ldlidar_stl_ros2_node
```

We can try to find it:
```
find ~/ldlidar_ros2_ws/install -name "ldlidar_stl_ros2_node"
/home/rpi/ldlidar_ros2_ws/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node
```
The name has changed.

dont forget to change the port in the launch file as in
```
https://deepwiki.com/ldrobotSensorTeam/ldlidar_stl_ros2/2.2-quick-start-guide
 
```
to the one that lidar is connected.

## Launching
Read the docs from the link above.
### Lauching vith rviz
From my experience, with gnome installed on rpi with Ubuntu 24 GUI is very slow, it may be a problem with sd card, because the one i have is not the best one; so that having rviz2 on it is impossible.
I connected rpi and my laptop(also Ubuntu with ros2 and rviz2) to one wifi(phone's hotspot in my case, which, has not influenced the connection) and then used `ssh` to connect to rpi from laptop. 
(From docs above):
On rpi:
```
# Launch only the LiDAR node
ros2 launch ldlidar_stl_ros2 ld06.launch.py
```
On laptop:
`rviz2`
 In rviz2 seach for lidar's topics and display them. 
