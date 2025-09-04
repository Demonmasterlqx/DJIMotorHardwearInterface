#include "dji_motor_hardwear_interface/DJIHardwearInterface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace RM_hardware_interface{

CallbackReturn RM_DJIMotorHardwareInterface::on_init(const HardwareInfo & hardware_info){

    // can port
    try{
        can_port_ = hardware_info.hardware_parameters.at("can_port");
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Can port not specified in hardware info");
        return CallbackReturn::ERROR;
    }

    if(can_port_ == ""){
        RCLCPP_ERROR(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Can port is miss or empty");
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Can port set to: %s", can_port_.c_str());

    // frequency
    try{
        write_to_can_frequence_ = std::stoi(hardware_info.hardware_parameters.at("command_frequence"));
    }
    catch(const std::exception & e){
        RCLCPP_WARN(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "command_frequence not specified in hardware info, set to 1000");
        write_to_can_frequence_ = 1000;
    }
    
    if(write_to_can_frequence_ <= 0){
        RCLCPP_WARN_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "command_frequence is not valid with value "<< write_to_can_frequence_ << ". set to 1000");
        write_to_can_frequence_ = 1000;
    }
    // 得到每个周期的最短时间
    period_ = rclcpp::Duration::from_seconds(1.0 / write_to_can_frequence_);

    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Command frequency set to: %d", write_to_can_frequence_);


    // read_times
    try{
        read_times_ = std::stoi(hardware_info.hardware_parameters.at("read_times"));
    }
    catch(const std::exception & e){
        RCLCPP_WARN(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "read_times not specified in hardware info, set to 1000");
        read_times_ = 5;
    }
    
    if(read_times_ <= 0){
        RCLCPP_WARN_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "read_times is not valid with value "<< read_times_ << ". set to 5");
        read_times_ = 5;
    }
    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Read times set to: %d", read_times_);


    // 为每个接口分配内存
    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "number of joints: %zu", hardware_info.joints.size());
    try{
        for(const auto & motor : hardware_info.joints){

            // 输出电机的信息
            RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Motor name: %s", motor.name.c_str());
            RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "CAN ID: %d", std::stoi(motor.parameters.at("can_id")));
            RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Motor type: %s", motor.parameters.at("motor_type").c_str());
            // 输出所有的 command 和 state 接口
            {
                for(const auto & command : motor.command_interfaces){
                    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Command interface: %s", command.name.c_str());
                }
                for(const auto & state : motor.state_interfaces){
                    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "State interface: %s", state.name.c_str());
                }
            }

            PortAttribute motor_attr={
                .joint_name = motor.name,
                .motor_type = [&motor]{
                    return motor.parameters.at("motor_type");
                }(),
                .can_id = [&motor]{
                    int id = std::stoi(motor.parameters.at("can_id"));
                    if(id <=0 || id > 7){
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "can_id must be between 1 and 7 for joint " << motor.name);
                        throw std::invalid_argument("can_id must be between 1 and 7 for joint " + motor.name);
                    }
                    return static_cast<u_int8_t>(id);
                }(),
                .state_names = [&motor]{
                    std::vector<std::string> state_names;
                    for(const auto & state : motor.state_interfaces){
                        state_names.push_back(state.name);
                    }
                    return state_names;
                }(),
                .command_name = [&motor]{
                    if(motor.command_interfaces.size() > 1){
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Only one command interface is supported for joint " << motor.name);
                        throw std::invalid_argument("Only one command interface is supported for joint " + motor.name);
                    }
                    if(motor.command_interfaces.size() == 1){
                        return motor.command_interfaces[0].name;
                    }
                    return std::string("");
                }(),
                .state_interface_ptrs = [&motor]{
                    std::vector<std::shared_ptr<double>> state_ptrs;
                    for(const auto & state : motor.state_interfaces){
                        (void)state;
                        state_ptrs.push_back(std::make_shared<double>(0.0));
                    }
                    return state_ptrs;
                }(),
                .command_interface_ptr = [&motor]{
                    if(motor.command_interfaces.size() > 1){
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Only one command interface is supported for joint " << motor.name);
                        throw std::invalid_argument("Only one command interface is supported for joint " + motor.name);
                    }
                    if(motor.command_interfaces.size() == 1){
                        return std::make_shared<double>(0.0);
                    }
                    std::shared_ptr<double> null_ptr = nullptr;
                    return null_ptr;
                }()

            };

            motor_attributes_.emplace_back(motor_attr);

        }
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to parse hardware info: " << e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Successfully parsed hardware info");

    return CallbackReturn::SUCCESS;

}


CallbackReturn RM_DJIMotorHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state){

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "RM_DJIMotorHardwareInterface on_configure with previous_state id: " << previous_state.id() << " label: " << previous_state.label());

    // 设置CAN通信

    can_driver_ = std::make_shared<Candriver>(can_port_);

    this->can_ok.store(can_driver_->isCanOk());
    if(!this->can_ok.load()){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to initialize CAN communication, will reopen later");
    }

    // 设置电机
    for(const auto & motor : motor_attributes_){
        if(motor.motor_type == "GM6020"){
            auto can_frame_processor = std::make_shared<GM6020>(motor.can_id, motor.joint_name);
            can_frame_processors_.emplace_back(can_frame_processor);
        }
        else{
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Unsupported motor type: " << motor.motor_type);
            throw std::invalid_argument("Unsupported motor type: " + motor.motor_type);
        }
    }

    // 检查反馈id是否有重复
    {
        std::map<u_int16_t, std::string> id_map;
        for(const auto & processor : can_frame_processors_){
            if(id_map.find(processor->identifier) != id_map.end()){
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Duplicate identifier: " << static_cast<int>(processor->identifier) << " for joints " << id_map[processor->canid] << " and " << processor->name);
                throw std::invalid_argument("Duplicate identifier: " + std::to_string(static_cast<int>(processor->identifier)));
            }
            else id_map[processor->identifier] = processor->name;
        }
    }

    // 设置端口

    try{
        for(std::size_t i=0; i<motor_attributes_.size(); i++){
            can_frame_processors_[i]->setCommandInterface(motor_attributes_[i].command_name, motor_attributes_[i].command_interface_ptr);
            can_frame_processors_[i]->setStateInterfaces(motor_attributes_[i].state_interface_ptrs, motor_attributes_[i].state_names);
        }
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to set command or state interfaces to can_frame_processors_: " << e.what());
        return CallbackReturn::ERROR;
    }

    // 设置向CAN口写入命令时的地址，也就是canframe的地址
    try{
        std::map<CanFramePosition, std::string> can_frame_positions;
        std::map<u_int16_t, std::shared_ptr<can_frame>> can_frame_map;
        for(const auto & processor : can_frame_processors_){
            auto pos = processor->getCanFramePosition();
            if(can_frame_positions.find(pos) != can_frame_positions.end()){
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Duplicate CAN frame position: identifier " << static_cast<int>(pos.identifier) << " position " << pos.position << " for joints " << can_frame_positions[pos] << " and " << processor->name);
                return CallbackReturn::ERROR;
                // throw std::invalid_argument("Duplicate CAN frame position: identifier " + std::to_string(static_cast<int>(pos.identifier)) + " position " + std::to_string(pos.position));
                // continue;
            }
            can_frame_positions[pos] = processor->name;
            if(can_frame_map.find(pos.identifier) == can_frame_map.end()){
                auto frame = std::make_shared<can_frame>();
                memset(frame.get(), 0, sizeof(can_frame));
                frame->can_dlc = 8;
                frame->can_id = pos.identifier;
                can_frame_map[pos.identifier] = frame;
                can_frames_to_send_.push_back(frame);
            }
            processor->setCanCommandInterface(can_frame_map[pos.identifier]->data + pos.position * 2);
        }
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to set CAN command interface addresses: " << e.what());
        return CallbackReturn::ERROR;
        // throw;
    }

    // 开启CAN通信的线程
    if(!start_can_thread()){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to start CAN thread");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "on_configure finish!");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DJIMotorHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state){

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "RM_DJIMotorHardwareInterface on_cleanup with previous_state id: " << previous_state.id() << " label: " << previous_state.label());

    end_can_thread();

    can_frame_processors_.clear();
    can_frames_to_send_.clear();

    this->can_ok.store(false);
    this->can_thread_stop.store(true);

    can_driver_ = nullptr;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "on_cleanup finish!");

    return CallbackReturn::SUCCESS;
    
}

bool RM_DJIMotorHardwareInterface::start_can_thread(){

    if (can_thread_){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "CAN thread is already running");
        return false;
    }

    auto func = [this](){
        while(!this->can_thread_stop.load() && rclcpp::ok()){
            auto t_start = rclcpp::Clock().now();
            
            if(this->can_ok.load()){
                
                std::vector<can_frame> recived_frames;
                for(int i=0; i<this->read_times_; i++){
                    can_frame recived_frame = {};
                    if (!can_driver_->receiveMessage(recived_frame)) {
                        if(can_driver_->isCanOk()){
                            this->can_ok.store(true);
                            // RCLCPP_WARN_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Don't receive frame in Canport and Canport is OK");
                            continue;
                        }
                        else{
                            this->can_ok.store(false);
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to receive CAN frame Canport Error");
                        }
                        std::this_thread::sleep_for(std::chrono::microseconds(100));
                    }
                    else{
                        recived_frames.push_back(recived_frame);
                    }
                }
                if(recived_frames.size() == 0){
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "No CAN frame received in this cycle");
                }

                for(const auto & processor : this->can_frame_processors_){
                    processor->write();
                    processor->writeIntoCanInterface();
                    for (const auto & recived_frame : recived_frames){
                        processor->processFrame(recived_frame);
                    }
                }
                if(this->is_activated.load()){
                    for(const auto & frame : this->can_frames_to_send_){
                        bool ok = can_driver_->sendMessage(*frame);
                        if(!ok && !can_driver_->isCanOk()){
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to send CAN frame with id " << std::hex << frame->can_id);
                            this->can_ok.store(false);
                        }
                        else{
                            this->can_ok.store(true);
                        }
                    }
                }
            }

            if(!this->can_ok.load()){
                RCLCPP_WARN(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "CAN bus error detected, try reopen");
                if (can_driver_->reopenCanSocket()){
                    can_ok.store(true);
                    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "CAN bus reopened successfully");
                }
                else{
                    this->can_ok.store(false);
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to reopen CAN bus");
                }
            }

            auto t_end = rclcpp::Clock().now();
            auto duration = t_end - t_start;

            if(duration < this->period_){
                auto sleep_time = this->period_ - duration;
                std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time.nanoseconds()));
            }

        }
    };

    can_thread_stop.store(false);
    can_thread_ = std::make_shared<std::thread>(func);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "CAN thread started");

    return true;

}

void RM_DJIMotorHardwareInterface::end_can_thread(){

    can_thread_stop.store(true);
    if(can_thread_ && can_thread_->joinable()){
        can_thread_->join();
    }
    can_thread_ = nullptr;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "CAN thread ended");

}

std::vector<StateInterface> RM_DJIMotorHardwareInterface::export_state_interfaces(){

    std::vector<StateInterface> state_interfaces;
    for(const auto & motor : this->motor_attributes_){
        for(std::size_t i=0; i<motor.state_names.size(); i++){
            state_interfaces.push_back(StateInterface(motor.joint_name, motor.state_names[i], motor.state_interface_ptrs[i].get()));
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Exporting state interfaces with %ld state_interfaces", state_interfaces.size());
    return state_interfaces;
}

std::vector<CommandInterface> RM_DJIMotorHardwareInterface::export_command_interfaces(){
    std::vector<CommandInterface> command_interfaces;
    for(const auto & motor : this->motor_attributes_){
        if(motor.command_name != ""){
            command_interfaces.push_back(CommandInterface(motor.joint_name, motor.command_name, motor.command_interface_ptr.get()));
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Exporting command interfaces with %ld command_interfaces", command_interfaces.size());
    return command_interfaces;
}

CallbackReturn RM_DJIMotorHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
    is_activated.store(true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "RM_DJIMotorHardwareInterface on_activate with previous_state id: " << previous_state.id() << " label: " << previous_state.label());
    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DJIMotorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    is_activated.store(false);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "RM_DJIMotorHardwareInterface deactivated with previous_state id: " << previous_state.id() << " label: " << previous_state.label());
    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DJIMotorHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "RM_DJIMotorHardwareInterface shutting down called with previous_state id: " << previous_state.id() << " label: " << previous_state.label() << " No thing happen.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DJIMotorHardwareInterface::on_error(const rclcpp_lifecycle::State & previous_state){

    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "RM_DJIMotorHardwareInterface on_error called with previous_state id: " << previous_state.id() << "label: " << previous_state.label() << "try to restart all hardware interface.");

    return CallbackReturn::SUCCESS;
}

return_type RM_DJIMotorHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){
    (void)time;
    (void)period;
    try{    
        for(auto can_processor : can_frame_processors_){
            if(!can_processor->read()){
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Error occurred while reading ROS2 state port " << can_processor->name);
                return return_type::ERROR;
            }
        }    
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Error occurred while reading ROS2 state ports: %s", e.what());
        return return_type::ERROR;
    }
    return return_type::OK;

}

return_type RM_DJIMotorHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
    (void)time;
    (void)period;
    try{
        for(auto can_processor : can_frame_processors_){
            if(!can_processor->write()){
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Error occurred while writing ROS2 command port " << can_processor->name);
                return return_type::ERROR;
            }
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Error occurred while writing ROS2 command ports: %s", e.what());
        return return_type::ERROR;
    }
    return return_type::OK;
}

} // namespace RM_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RM_hardware_interface::RM_DJIMotorHardwareInterface, hardware_interface::SystemInterface);