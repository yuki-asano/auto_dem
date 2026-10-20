# auto_dem
## Setup
```
git@github.com:yuki-asano/auto_dem.git
```


## Experiment procedure
### Robot setup
- Set fr3 robot to the initial placement.
- Setup fr3 using Desk interface (unlock joints and activate FCI) 

### Robot machine (tiger)
```
[TERMINAL1]
roscore

[TERMINAL2]  # fr3 control
roslaunch auto_dem fr3_dielec.launch
```

### Operation machine (lion)
```
[TERMINAL1]
roslaunch auto_dem auto_dem.launch  # rosbridge_server, robot_manip

[TERMINAL2]  # for press process combination and roscore separation (due to collision of joint_controllers)
roslaunch wsl2_ros_bridge connect.launch remote_host:=192.168.0.6 connections_file:="/home/utokyo/catkin_ws/src/auto_dem/config/connections_fr3.csv" local_port:=9080 # 192.168.0.6: koala (PC for press process operation)
```

- dilect command to manip node  
    ```
    rostopic pub -1 /start_dielectrometer_manip std_msgs/Bool "data: true"  # publish topic to manip node
    ```

otherwise, only for robot manip
```
roscd auto_dem/euslisp
roseus dielectrometer-manip.l
```

### Dielectrometer machine (WSL)
```
[GUI]
ShockLine.exeを起動

[TERMINAL1]
roscore

[TERMINAL2]
roslaunch wsl2_ros_bridge connect.launch remote_host:=192.168.0.24 connections_file:="/home/utokyo/catkin_ws/src/wsl2_ros_bridge/example/connections_mod.csv"  # remote_host -> lion

[TERMINAL3]　<- 管理者権限で実行
rosrun aet_instrument control.py
```


### Experiment commands (benchmark function)
Below should be executed on the operation machine 
```
[exp for sample1]
(send *ri* :start-grasp :rarm)
(measure-10-28-40-ntimes-each :measure-times 3 :thickness 0.355 :sample-name "sample1" :file-path "C:/Users/test/Desktop/asano/robot_exp.csv" :insert-until-contact t)
(send *ri* :stop-grasp :rarm)

[exp for sample2]
(send *ri* :start-grasp :rarm)
(measure-10-28-40-ntimes-each :measure-times 3 :thickness 0.343 :sample-name "sample2" :file-path "C:/Users/test/Desktop/asano/robot_exp.csv" :insert-until-contact t)
(send *ri* :stop-grasp :rarm)

[exp for sample3]
(send *ri* :start-grasp :rarm)
(measure-10-28-40-ntimes-each :measure-times 3 :thickness 0.320 :sample-name "sample3" :file-path "C:/Users/test/Desktop/asano/robot_exp.csv" :insert-until-contact t)
(send *ri* :stop-grasp :rarm)
```
