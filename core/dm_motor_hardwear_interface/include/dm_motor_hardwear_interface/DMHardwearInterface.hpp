#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "Candriver/candriver.hpp"
#include "dm_motor_hardwear_interface/DMCanframeprocessor.hpp"
#include <atomic>
#include <realtime_tools/realtime_publisher.hpp>

#ifndef RM_DM_MOTOR_HARDWARE_INTERFACE_HPP
#define RM_DM_MOTOR_HARDWARE_INTERFACE_HPP

#define DEBUG


#ifdef DEBUG
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rm_interface/msg/raw_can.hpp"
#include "rm_interface/msg/motor_state.hpp"
#endif

namespace RM_hardware_interface{

using realtime_tools::RealtimePublisher;
using StateInterface = hardware_interface::StateInterface;
using HardwareInfo = hardware_interface::HardwareInfo;
using CallbackReturn = hardware_interface::CallbackReturn;
using CommandInterface = hardware_interface::CommandInterface;
using return_type = hardware_interface::return_type;
using Candriver = RM_communication::CanDriver;

class RM_DMMotorHardwearInterface : public hardware_interface::SystemInterface{

public:
    RM_DMMotorHardwearInterface();
    ~RM_DMMotorHardwearInterface() = default;

    CallbackReturn on_init(const HardwareInfo & hardware_info) override;

    std::vector<StateInterface> export_state_interfaces() override;

    std::vector<CommandInterface> export_command_interfaces() override;

    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

private:

    // 每个电机的CanFrameProcessor
    std::vector<std::shared_ptr<DMCanframeprocessor>> can_frame_processors_;
    // CanDriver
    std::shared_ptr<Candriver> can_driver_=nullptr;

    // Canport
    std::string can_port_ = "can0";

    // 电机属性，从HardwareInfo中获取
    std::vector<PortAttribute> motor_attributes_;

    // 监控电机的每个电机的can反馈帧丢失情况
    std::vector<int> motor_back_frame_flage_cnt_;

    // 单个周期中尝试读入的次数
    int read_times_ = 0;

    // 检测ros2心跳线程
    std::shared_ptr<std::thread> ros2_heartbeat_thread_=nullptr;

    // ros2心跳线程停止标志
    std::atomic<bool> ros2_heartbeat_thread_stop_{false};

    // 用于logger以及
    rclcpp::Node::SharedPtr nh_ = nullptr;

    #ifdef DEBUG

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr write_time_interval_publishers_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr read_time_interval_publishers_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_info_publisher_ = nullptr;
    rclcpp::Publisher<rm_interface::msg::RawCan>::SharedPtr can_frame_publisher_ = nullptr;

    // 电机的state调试信息
    std::vector<rclcpp::Publisher<rm_interface::msg::MotorState>::SharedPtr> motor_state_debug_info_publishers_;

    rclcpp::Time last_write_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);
    rclcpp::Time last_read_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);

    #endif

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_position_sub_ = nullptr;

    void zero_position_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief 将字符串转换为控制模式
     * 
     * @param control_model_str  控制模式字符串
     * @return ControlModel 如果有对应的控制模式，返回对应的控制模式，否则返回READONLY
     */
    ControlModel _str_to_control_model(const std::string& control_model_str);

}; // class RM_DMMotorHardwearInterface


} // namespace RM_hardware_interface;

#endif  // RM_DJI_MOTOR_HARDWARE_INTERFACE_HPP