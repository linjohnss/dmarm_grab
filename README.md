# tmarm_grab
> ROS base TM Arm grabbing useing AruCo
## Environment
Ubuntu 20.04
RealSense D415(EIH)
TM Robot arm
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
```
uint16 id
bool isput

---

bool end
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

