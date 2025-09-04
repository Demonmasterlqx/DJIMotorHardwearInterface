#include "dji_motor_hardwear_interface/Canframeprocessor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace RM_hardware_interface {

CanFrameProcessor::CanFrameProcessor(const std::string& name,const u_int8_t canid, const u_int16_t preidentifier):canid(canid), identifier(preidentifier + canid), name(name) {
    if(canid <=0 || canid > 7){
        throw std::invalid_argument("canid must be between 0 and 7");
    }
}

CanFrameProcessor::~CanFrameProcessor() {}

double CanFrameProcessor::_current_to_torque(double current) {
    return current;
}

double CanFrameProcessor::_torque_to_current(double torque) {
    return torque;
}

bool CanFrameProcessor::setCanCommandInterface(u_int8_t* first_command_interface) {
    this->first_command_interface = first_command_interface;
    return true;
}

bool CanFrameProcessor::writeIntoCanInterface() {
    if (!first_command_interface) {
        RCLCPP_ERROR(rclcpp::get_logger("CanFrameProcessor"), "First command interface is not set");
        return false;
    }
    double command = *command_buffer_.readFromRT();

    int16_t current = _get_current_reverse(command);

    *first_command_interface = (current >> 8) & 0xFF;
    *(first_command_interface + 1) = current & 0xFF;

    return true;
}

} // namespace RM_hardware_interface