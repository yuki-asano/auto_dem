# auto_dem
## Install & Setup
- https://github.com/yuki-asano/auto_dem/wiki/Install-&-Setup


## Experiment procedure
### Hardware setup
**Robot**
- Set fr3 robot to the initial placement.
- Setup fr3 using Desk interface (unlock joints and activate FCI) 

**Dielectrometer**
- Turn on

### Software setup
**Robot PC (tiger)**
```
[TERMINAL1]
roscore

[TERMINAL2]  # fr3 control
roslaunch auto_dem fr3_dielec.launch
```

**Manager PC (lion)**
```
[TERMINAL1]
roslaunch auto_dem auto_dem.launch  # rosbridge_server, robot_manip
```

- backup for press process combination
    roscore separation (due to collision of joint_controllers)
    ```
    [TERMINAL2]
    roslaunch wsl2_ros_bridge connect.launch remote_host:=192.168.0.6 connections_file:="/home/utokyo/catkin_ws/src/auto_dem/config/connections_fr3.csv" local_port:=9080 # 192.168.0.6: koala (PC for press process operation)
    ```

**Measurement PC (WSL)**
```
[GUI]
ShockLine.exeを管理者権限で起動

[TERMINAL1]
roscore

[TERMINAL2]
roslaunch wsl2_ros_bridge connect.launch remote_host:=192.168.0.44 connections_file:="/home/utokyo/catkin_ws/src/wsl2_ros_bridge/example/connections_mod.csv"  # remote_host -> lion

[TERMINAL3]　<- 管理者権限で実行
rosrun aet_instrument control.py
```

### Automation demo
```
rostopic pub -1 /start_dielectrometer_manip std_msgs/Bool "data: true"  # publish topic to manip node
```

### Demo commands (benchmark function)
ロボット動作単独テスト. Manager PCで以下を実行
```
roscd auto_dem/euslisp
roseus dielectrometer-manip.l
```

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

## Citation
```
@INPROCEEDINGS{auto_dem:Asano:IROS2024,
  author={Yuki Asano and Kei Okada and Junichiro Shiomi},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Robotic Measurement for Electrical Property of Polymers by Force-Sensing Robot toward Materials Lab-Automation}, 
  year={2024},
  pages={2319-2324},
  doi={10.1109/IROS58592.2024.10802186}
}
```
