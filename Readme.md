# RMHardwearInterface

## 支持的电机&&电调

GM6020 C610 C620

## ros2_control urdf 参数定义

```xml

<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

        <ros2_control name="dji_motor_hardwear_interface_test_robot_control" type="system">
            <hardware>
                <plugin>RM_hardware_interface/RM_DJIMotorHardwareInterface</plugin>
                <param name="command_frequence">1000</param>
                <param name="read_times">5</param>
                <param name="can_port">can0</param>
            </hardware>

            <joint name="GM6020_test_joint">

                <param name="can_id">1</param>
                <param name="motor_type">GM6020</param>

                <command_interface name="effort"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
                <state_interface name="effort"/>
                <state_interface name="temperature"/>

            </joint>

        </ros2_control>
</robot>

```

+ command_frequence 本插件采用的是在一次循环中读入n次，然后写入一次，这里设置的是 这一个循环进行的最高频率 
+ read_times 单个循环内尝试读入的次数
+ can_port Can网口的名字
+ can_id 电机手册中对应的id，不是标识符，标识符会根据不同的电机自动生成
+ motor_type 电机种类

现在暂时将电流和力矩混着来（没确定怎么搞）

对joint的min，max，初始值等不生效

## 电机资料

[GM6020](https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E20231013.pdf)

[M2006&C610](https://rm-static.djicdn.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf)

[C620](https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E4%B8%AD%E8%8B%B1%E6%97%A5%EF%BC%89V1.01.pdf)