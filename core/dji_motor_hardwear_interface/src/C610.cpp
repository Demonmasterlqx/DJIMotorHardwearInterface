#include "dji_motor_hardwear_interface/Canframeprocessor.hpp"
#include <cmath>
#include <map>
#include <rclcpp/rclcpp.hpp>

namespace RM_hardware_interface {

C610::C610(const int canid, const std::string& name): CanFrameProcessor(name, canid, 0x200){

    // 初始化
    state_interfaces_.resize(3,nullptr);
    state_buffers_.resize(3, realtime_tools::RealtimeBuffer<double>(0));
    command_interface_ = nullptr;
    command_buffer_ = realtime_tools::RealtimeBuffer<double>(0);

}

C610::~C610() {}

bool C610::processFrame(const can_frame& frame) {
    if(frame.can_id != this->identifier) {
        // 跳过
        return false;
    }

    try{
        // 位置 单位弧度
        state_buffers_[POSITION_INDEX].writeFromNonRT(combine_bytes_to_int16(frame.data[0], frame.data[1]) / 8191.0 * 2 * M_PI);

        // 速度 单位弧度每秒
        state_buffers_[VELOCITY_INDEX].writeFromNonRT(combine_bytes_to_int16(frame.data[2], frame.data[3]) * 2 * M_PI / (3600.0));

        // 力矩 单位牛米
        state_buffers_[TORQUE_INDEX].writeFromNonRT(_current_to_torque(_get_current(frame)));
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

bool C610::read(){
    try{
        for(size_t i=0; i<state_interfaces_.size(); i++) {
            if(state_interfaces_[i] != nullptr) {
                *(state_interfaces_[i]) = *(state_buffers_[i].readFromRT());
            }
        }

    }
    catch(std::exception & e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "Failed to write state interfaces: " << e.what());
        return false;
    }
    return true;
}

bool C610::write(){
    try{
        if(command_interface_ != nullptr) {
            command_buffer_.writeFromNonRT(_current_to_torque(*command_interface_));
        }
    }
    catch(std::exception & e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(name + "CanFrameProcessor"), "Failed to read command interface: " << e.what());
        return false;
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