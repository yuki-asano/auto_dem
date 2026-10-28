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
roslaunch auto_dem auto_dem_embrio.launch  # embrio connection

or
roslaunch auto_dem auto_dem.launch  # only auto_dem (rosbridge_server, robot_manip)
```

**Measurement PC (WSL)**
```
[GUI]
ShockLine.exeを起動 <-管理者権限

[TERMINAL1]　<- 管理者権限で実行
roslaunch aet_instrument dielectric_calculator_server.launch
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
