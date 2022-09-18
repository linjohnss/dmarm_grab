# tmarm_grab
> ROS base TM Arm grabbing useing AruCo
## Environment
1. Ubuntu 20.04
2. RealSense D415(EIH)
3. TM Robot arm

<img src="https://user-images.githubusercontent.com/61956056/172375264-828dbe29-4e45-4e5b-bbbe-3e3420fca347.jpg" width="500">

## tmr_ros1
```shell=
git clone https://github.com/TechmanRobotInc/tmr_ros1.git
```
## robotiq_2finger
```shell=
git clone https://github.com/linjohnss/robotiq_2finger.git
```
## tmarm_grab
```shell=
git clone https://github.com/linjohnss/tmarm_grab.git
```
## ROS Services
You can use ros services to communicate with robot
```
uint16 id
bool isput

---

bool end
```
## Detection Image
This node subscribe ros compressed image topic
```
/output/image_raw/compressed
```

## Strat Server
```shell=
rosrun tmarm_grab arm_grab.py
```
## Start Client
```shell=
rosrun tmarm_grab client_test.py True
```

## Demo
Demo for grabbing bottle (Target and Destination must stick AruCo)

https://user-images.githubusercontent.com/61956056/172375447-83b8e4c1-cb4d-427c-9487-ae7778200e02.mov




