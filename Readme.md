# DJIMotorHardwearInterface

## 支持的电机&&电调

GM6020

## ros2_control urdf 参数定义

```xml

<ros2_control name="${name}" type="system" command_frequence= "1000" can_port="can0">
    <hardware>
        <plugin>mock_components/GenericSystem</plugin>
    </hardware>
    <joint name="joint_name" can_id = "1" motor_type = "GM6020">
        <!-- 电流转成力矩 -->
        <command_interface name="moment"/>

        <!-- 有些电机有这些，最多就是这些 -->

        <!-- 单位是弧度 -->
        <state_interface name="position"/>
        <!-- 单位是弧度每秒 -->
        <state_interface name="velocity"/>
        <!-- 单位是 N*m -->
        <state_interface name="moment"/>
        <!-- 单位未知，暂时直接转发电机给出的值 -->
        <state_interface name="temperature"/>

    </joint>
</ros2_control>

```

现在暂时将电流和力矩混着来（没确定怎么搞）

对joint的min，max，初始值等不生效

## 电机资料

[GM6020](https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E20231013.pdf)

[M2006&C610](https://rm-static.djicdn.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf)

[C620](https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E4%B8%AD%E8%8B%B1%E6%97%A5%EF%BC%89V1.01.pdf)