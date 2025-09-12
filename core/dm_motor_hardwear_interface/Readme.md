# dm_motor_hardwear_interface

## 支持的电机

+ [DM-J8009-2EC](https://gitee.com/kit-miao/DM-J8009-2EC/raw/master/%E8%AF%B4%E6%98%8E%E4%B9%A6/DM-J8009-2EC%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%E8%AF%B4%E6%98%8E%E4%B9%A6V1.0.pdf)
+ [DM-J4310-2EC](https://gitee.com/kit-miao/DM-J4310-2EC/raw/master/%E8%AF%B4%E6%98%8E%E4%B9%A6/DM-J4310-2EC%20V1.1%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%E8%AF%B4%E6%98%8E%E4%B9%A6V1.0.pdf)
+ 控制参数和以上电机相同的电机

## 支持的控制模式

+ MIT模式 (MIT)
+ 速度模式 (V or velocity)
+ 位置模式 (P or position)
+ 只读模式 (R or readonly) 就是没有输出

## 控制与指令接口输出

这里不支持自定义，按照设定的控制模式，会暴露出不同的command interface
+ MIT模式 (MIT) position_des, velocity_des, kp, kd, torque_ff
+ 速度模式 velocity
+ 位置模式 position, velocity
+ 只读模式 无命令

以上所有的模式都会给出以下的state

+ position
+ velcity
+ effort
+ temperature_mos
+ temperature_rotor
+ error_label

## ros2_control urdf 参数定义

```xml

<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

        <ros2_control name="dm_motor_hardwear_interface_test_robot_control" type="system">
            <hardware>
                <plugin>RM_hardware_interface/RM_DMMotorHardwearInterface</plugin>
                <param name="read_times">5</param>
                <param name="can_port">can0</param>
                <param name="control_model">V</param>
            </hardware>

            <joint name="Joint">

                <param name="can_id">1</param>
                <param name="MST_ID">0x13</param>
                <param name="reverse">false</param>
                <param name="control_model">V</param>

                <!-- 设置数值上下界 -->
                <param name="max_position">3.14</param>
                <param name="min_position">-3.14</param>
                <param name="max_velocity">3.14</param>
                <param name="min_velocity">-3.14</param>
                <param name="max_torque">-3.14</param>
                <param name="min_torque">3.14</param>
                <!-- 如果不用MIT，可以不设置这下面的 -->
                <param name="max_kp">3.14</param>
                <param name="min_kp">-3.14</param>
                <param name="max_kd">3.14</param>
                <param name="min_kd">-3.14</param>


            </joint>

        </ros2_control>
</robot>

```

+ read_times 单个循环内尝试读入的次数
+ can_port Can网口的名字
+ can_id 电机手册中对应的id，不是标识符，标识符会根据不同的电机自动生成
+ MST_ID 电机手册中的 master_id
+ reverse 是否反转
+ control_model 控制模式，可以在 hardware 标签里面设置， 也可以再 joint 标签中设置。其中hardware设置全体的控制模式，joint单独设置，并且 joint 优先级高。如果都不做设置，默认为 readonly

## 特殊功能

### 置零模式

HardwearInterface 将会监听 `hardware_info.name + "_zero_position"` topic， 其类型为 `std_msgs::msgs:Bool`。 当消息内容为 `True` 时，会使得电机下一次的发送指令的消息变为置零帧，然后消除标记

## 电机资料

[GM6020](https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E20231013.pdf)

[M2006&C610](https://rm-static.djicdn.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf)

[C620](https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E4%B8%AD%E8%8B%B1%E6%97%A5%EF%BC%89V1.01.pdf)