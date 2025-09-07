#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "Candriver/candriver.hpp"
#include "dji_motor_hardwear_interface/Canframeprocessor.hpp"
#include <atomic>

#ifndef RM_DJI_MOTOR_HARDWARE_INTERFACE_HPP
#define RM_DJI_MOTOR_HARDWARE_INTERFACE_HPP

// #define DEBUG


#ifdef DEBUG
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rm_interface/msg/raw_can.hpp"
#endif

namespace RM_hardware_interface{

using StateInterface = hardware_interface::StateInterface;
using HardwareInfo = hardware_interface::HardwareInfo;
using CallbackReturn = hardware_interface::CallbackReturn;
using CommandInterface = hardware_interface::CommandInterface;
using return_type = hardware_interface::return_type;
using Candriver = RM_communication::CanDriver;

bool operator<(const CanFramePosition& lhs, const CanFramePosition& rhs) {
    if (lhs.identifier != rhs.identifier) {
        return lhs.identifier < rhs.identifier;
    }
    return lhs.position < rhs.position;
}

bool operator==(const CanFramePosition& lhs, const CanFramePosition& rhs) {
    return lhs.identifier == rhs.identifier && lhs.position == rhs.position;
}

// 解析关节属性
struct PortAttribute{
    const std::string joint_name;
    const std::string motor_type;
    const u_int8_t can_id;
    bool reverse;
    std::vector<std::string> state_names;
    std::string command_name = "";
    std::vector<std::shared_ptr<double>> state_interface_ptrs;
    std::shared_ptr<double> command_interface_ptr = nullptr;
};

class RM_DJIMotorHardwareInterface : public hardware_interface::SystemInterface{
public:
    RM_DJIMotorHardwareInterface();
    ~RM_DJIMotorHardwareInterface() = default;

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

protected:

    // 每个电机的CanFrameProcessor
    std::vector<std::shared_ptr<CanFrameProcessor>> can_frame_processors_;
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

    //要发送的can帧
    std::vector<std::shared_ptr<can_frame>> can_frames_to_send_;

    // 检测ros2心跳线程
    std::shared_ptr<std::thread> ros2_heartbeat_thread_=nullptr;

    // ros2心跳线程停止标志
    std::atomic<bool> ros2_heartbeat_thread_stop_{false};

private:
    // 支持的电机种类
    static const std::vector<std::string> supported_motor_types_;

// Debug部分


#ifdef DEBUG

private:

    rclcpp::Node::SharedPtr debug_node_ = nullptr;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> debug_publishers_;
    rclcpp::Publisher<rm_interface::msg::RawCan>::SharedPtr debug_can_publishers_;

#endif

};

// 初始化支持的电机电调类型
const std::vector<std::string> RM_DJIMotorHardwareInterface::supported_motor_types_ = {
    "GM6020", "C620"
};

} // namespace RM_hardware_interface

#endif  // RM_DJI_MOTOR_HARDWARE_INTERFACE_HPP