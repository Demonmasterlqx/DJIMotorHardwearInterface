#include "dm_motor_hardwear_interface/DMCanframeprocessor.hpp"
#include "dm_motor_hardwear_interface/utils.hpp"

namespace RM_hardware_interface{

DMCanframeprocessor::DMCanframeprocessor(const PortAttribute& attribute){
    // 初始化赋值
    can_id_ = attribute.can_id;
    MST_ID_ = attribute.MST_ID;
    control_model_ = attribute.control_model;
    reverse_ = attribute.reverse;
    joint_name_ = attribute.joint_name;

    // 根据控制模式开始分配内存，并且设置 COMMAND_CAN_ID

    switch (control_model_){
    case ControlModel::MIT:
        COMMAND_CAN_ID = can_id_;
        command_name_pos_ref = MIT_comand_name_pos_vec;
        break;
    case ControlModel::POSITION:
        COMMAND_CAN_ID = static_cast<u_int8_t>(0x100 + can_id_);
        command_name_pos_ref = POSITION_comand_name_pos_vec;
        break;
    case ControlModel::VELOCITY:
        COMMAND_CAN_ID = static_cast<u_int8_t>(0x200 + can_id_);
        command_name_pos_ref = VELOCITY_comand_name_pos_vec;
        break;
    case ControlModel::READONLY:
        COMMAND_CAN_ID = 0;
        command_name_pos_ref = READ_comand_name_pos_vec;
        break;
    default:
        throw std::runtime_error("Unknown control model");
        return;
        break;
    }

    // 分配内存
    state_interface_ptrs_.resize(state_name_pos_vec.size(), nullptr);
    command_interface_ptrs_.resize(command_name_pos_ref.size(), nullptr);
    for (size_t i = 0; i < state_interface_ptrs_.size(); ++i) {
        state_interface_ptrs_[i] = std::make_shared<double>(0.0);
    }
    for (size_t i = 0; i < command_interface_ptrs_.size(); ++i) {
        command_interface_ptrs_[i] = std::make_shared<double>(0.0);
    }

    // 开始为每种控制类型，确定控制函数

    switch (control_model_){
    case ControlModel::MIT:
        command_function_ = std::bind(&DMCanframeprocessor::MITCommandFunction, this);
        break;
    case ControlModel::POSITION:
        command_function_ = std::bind(&DMCanframeprocessor::POSITIONCommandFunction, this);
        break;
    case ControlModel::VELOCITY:
        command_function_ = std::bind(&DMCanframeprocessor::VELOCITYCommandFunction, this);
        break;
    case ControlModel::READONLY:
        command_function_ = std::bind(&DMCanframeprocessor::READCommandFunction, this);
        break;
    default:
        throw std::runtime_error("Unknown control model");
        return;
        break;
    }

}

std::string DMCanframeprocessor::getJointName() const {
    return joint_name_;
}

can_frame DMCanframeprocessor::getCommandFrame(){
    if (COMMAND_CAN_ID == 0){
        // 只读模式
        can_frame frame = {};
        frame.can_id = 0;
        frame.can_dlc = 0;
        return frame;
    }
    if(is_set_position_zero_){
        return getPositionZeroFrame();
    }
    return command_function_();
}

can_frame DMCanframeprocessor::MITCommandFunction(){
    const double & velocity_des = *command_interface_ptrs_[COMMAND_VELOCITYDES];
    const double & position_des = *command_interface_ptrs_[COMMAND_POSITIONDES];
    const double & kp   = *command_interface_ptrs_[COMMAND_KP];
    const double & kd   = *command_interface_ptrs_[COMMAND_KD];
    const double & torque_ff = *command_interface_ptrs_[COMMAND_TORQUEFF];

    can_frame frame = {};
    frame.can_id = COMMAND_CAN_ID;
    frame.can_dlc = 8;

    MITControlFrameData data = {};
    data.position_des = float_to_uint(position_des, limits_.MinPosition, limits_.MaxPosition, 16);
    data.velocity_des = float_to_uint(velocity_des, limits_.MinVelocity, limits_.MaxVelocity, 12);
    data.kp = float_to_uint(kp, limits_.MinKp, limits_.MaxKp, 12);
    data.kd = float_to_uint(kd, limits_.MinKd, limits_.MaxKd, 12);
    data.torque_ff = float_to_uint(torque_ff, limits_.MinTorque, limits_.MaxTorque, 12);

    // 复制
    memcpy(&frame.data[0], &data, sizeof(MITControlFrameData));

    return frame;
}

can_frame DMCanframeprocessor::VELOCITYCommandFunction(){
    const float & velocity_des = *command_interface_ptrs_[COMMAND_VELOCITYDES];
    can_frame frame = {};
    frame.can_id = COMMAND_CAN_ID;
    frame.can_dlc = 8;
    memcpy(&frame.data[0], &velocity_des, sizeof(float));
    return frame;
}

can_frame DMCanframeprocessor::POSITIONCommandFunction(){
    const float & position_des = *command_interface_ptrs_[COMMAND_POSITIONDES];
    const float & velocity_des = *command_interface_ptrs_[COMMAND_VELOCITYDES];

    can_frame frame = {};
    frame.can_id = COMMAND_CAN_ID;
    frame.can_dlc = 8;
    memcpy(&frame.data[0], &position_des, sizeof(float));
    memcpy(&frame.data[4], &velocity_des, sizeof(float));
    return frame;
}

can_frame DMCanframeprocessor::READCommandFunction(){
    can_frame frame = {};
    frame.can_id = 0;
    frame.can_dlc = 0;
    return frame;
}

can_frame DMCanframeprocessor::getPositionZeroFrame(){

    can_frame frame = {};
    frame.can_id = COMMAND_CAN_ID;
    frame.can_dlc = 8;
    memcpy(&frame.data[0], SaveZeroPosition_Frame, 8);

    return frame;
}

can_frame DMCanframeprocessor::getDisableFrame(){
    can_frame frame = {};
    frame.can_id = COMMAND_CAN_ID;
    frame.can_dlc = 8;
    memcpy(&frame.data[0], Disable_Frame, 8);
    return frame;
}

can_frame DMCanframeprocessor::getEnableFrame(){
    can_frame frame = {};
    frame.can_id = COMMAND_CAN_ID;
    frame.can_dlc = 8;
    memcpy(&frame.data[0], Enable_Frame, 8);
    return frame;
}

can_frame DMCanframeprocessor::getClearErrorFrame(){
    can_frame frame = {};
    frame.can_id = COMMAND_CAN_ID;
    frame.can_dlc = 8;
    memcpy(&frame.data[0], ClearError_Frame, 8);
    return frame;
}

CANFrameProcessorError DMCanframeprocessor::processReceivedFrame(const can_frame& frame){
    if(frame.can_id != MST_ID_){
        return CANFrameProcessorError::ERR_CANID;
    }
    if(frame.can_dlc != 8){
        return CANFrameProcessorError::ERR_CANID;
    }
    MotorFeedBackTypeDef feedback = {};
    memcpy(&feedback, &frame.data[0], sizeof(MotorFeedBackTypeDef));

    // 赋值

    double position = uint_to_float(feedback.PositionFdb, limits_.MinPosition, limits_.MaxPosition, 16);
    double velocity = uint_to_float(feedback.SpeedFdb, limits_.MinVelocity, limits_.MaxVelocity, 12);
    double torque = uint_to_float(feedback.TorqueFdb, limits_.MinTorque, limits_.MaxTorque, 12);
    double tem_mos = static_cast<double>(feedback.TemMOS);
    double tem_rotor = static_cast<double>(feedback.TemRotor);
    CANFrameProcessorError err = feedback.ERR;
    double error_label = static_cast<double>(err);
    if (reverse_){
        position = -position;
        velocity = -velocity;
        torque = -torque;
    }
    *state_interface_ptrs_[STATE_POSITION] = position;
    *state_interface_ptrs_[STATE_VELOCITY] = velocity;
    *state_interface_ptrs_[STATE_EFFORT] = torque;
    *state_interface_ptrs_[STATE_TEMPMOS] = tem_mos;
    *state_interface_ptrs_[STATE_TEMPROTOR] = tem_rotor;
    *state_interface_ptrs_[STATE_ERRORLABEL] = error_label;

    return err;
}

void DMCanframeprocessor::getStateInterfaces(std::vector<StateInterface>& state_interfaces){

    for (const auto& state : state_name_pos_vec) {
        state_interfaces.emplace_back(StateInterface(
            joint_name_, state.first, state_interface_ptrs_[state.second].get()
        ));
    }
}

void DMCanframeprocessor::getCommandInterfaces(std::vector<CommandInterface>& command_interfaces){

    for (const auto& command : command_name_pos_ref) {
        command_interfaces.emplace_back(CommandInterface(
            joint_name_, command.first, command_interface_ptrs_[command.second].get()
        ));
    }
}



} // namespace RM_hardware_interface