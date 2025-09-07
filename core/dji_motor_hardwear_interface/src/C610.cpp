#include "dji_motor_hardwear_interface/Canframeprocessor.hpp"
#include <cmath>
#include <map>
#include <rclcpp/rclcpp.hpp>

namespace RM_hardware_interface {

C610::C610(const int canid, const std::string& name, bool reverse): CanFrameProcessor(name, canid, 0x200){

    // 初始化
    state_interfaces_.resize(3,nullptr);
    command_interface_ = nullptr;
    reverse_ = reverse;

}

C610::~C610() {}

bool C610::processFrame(const can_frame& frame) {
    if(frame.can_id != this->identifier) {
        // 跳过
        return false;
    }

    try{
        double position = combine_bytes_to_int16(frame.data[0], frame.data[1]) / 8191.0 * 2 * M_PI;
        double velocity = combine_bytes_to_int16(frame.data[2], frame.data[3]) * 2.0 * M_PI / (3600.0);
        double torque = _current_to_torque(_get_current(frame));
        if(reverse_){
            position = -position;
            velocity = -velocity;
            torque = -torque;
        }

        // 位置 单位弧度
        if(state_interfaces_[POSITION_INDEX] != nullptr) *state_interfaces_[POSITION_INDEX] = position;

        // 速度 单位弧度每秒
        if(state_interfaces_[VELOCITY_INDEX] != nullptr) *state_interfaces_[VELOCITY_INDEX] = velocity;

        // 力矩 单位牛米
        if(state_interfaces_[TORQUE_INDEX] != nullptr) *state_interfaces_[TORQUE_INDEX] = torque;

    }
    catch(std::exception & e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "Failed to process CAN frame: " << e.what());
        return false;
    }

    return true;

}

bool C610::setCommandInterface(std::string command_name, std::shared_ptr<double> command_interface) {

    static const std::string supported_command = "effort";

    if(command_name == ""){
        RCLCPP_WARN_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "Empty command name, skipping setting command interface");
        return true;
    }

    if(command_name != supported_command) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "Unsupported command name: " << command_name);
        throw std::invalid_argument("Unsupported command name: " + command_name);
    }

    if(command_interface == nullptr) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "Command interface is null");
        throw std::invalid_argument("Command interface is null");
        return false;
    }

    command_interface_ = command_interface;
    return true;
}

bool C610::setStateInterfaces(std::vector<std::shared_ptr<double>> state_interfaces, std::vector<std::string> state_names) {
    const static std::vector<std::string> supported_port={"effort", "position", "velocity"};

    std::map<std::string,int> port_index_map = {
        {"effort", TORQUE_INDEX},
        {"position", POSITION_INDEX},
        {"velocity", VELOCITY_INDEX},
    };

    if(state_interfaces.size() != state_names.size()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "State interfaces and names size mismatch");
        throw std::invalid_argument("State interfaces and names size mismatch");
        return false;
    }

    for(size_t i=0; i<state_interfaces.size(); i++) {
        auto it = port_index_map.find(state_names[i]);
        if(it == port_index_map.end()) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "Unsupported state name: " << state_names[i]);
            throw std::invalid_argument("Unsupported state name: " + state_names[i]);
            return false;
        }

        if(state_interfaces[i] == nullptr) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "State interface is null: " << state_names[i]);
            throw std::invalid_argument("State interface is null: " + state_names[i]);
            return false;
        }

        if(state_interfaces_[it->second] != nullptr){
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "State interface already set: " << state_names[i]);
            throw std::invalid_argument("State interface already set: " + state_names[i]);
            return false;
        }

        state_interfaces_[it->second] = state_interfaces[i];

    }

    return true;
}

CanFramePosition C610::getCanFramePosition(){

    auto position = CanFramePosition();

    if(canid<=4){
        position.identifier = 0x200;
        position.position = static_cast<u_int8_t>(canid-1);
        return position;
    }
    if(4< canid && canid <= 8){
        position.identifier = 0x1FF;
        position.position = static_cast<u_int8_t>(canid-5);
        return position;
    }

    position.identifier = 0xFF;
    position.position = 0xFF;

    return position;
}

double C610::_get_current(can_frame frame){
    int16_t current_int = (int16_t(frame.data[4])<<8) + int16_t(frame.data[5]);

    return current_int;
}

int16_t C610::_get_current_reverse(double current){
    if(current > 10) current = 10;
    if(current < -10) current = -10;
    return static_cast<int16_t>(current / 10.0 * 10000);
}

double C610::_current_to_torque(double current){
    return current;
}

} // namespace RM_hardware_interface