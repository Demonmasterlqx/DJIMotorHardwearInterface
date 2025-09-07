#include "dji_motor_hardwear_interface/DJIHardwearInterface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace RM_hardware_interface{

RM_DJIMotorHardwareInterface::RM_DJIMotorHardwareInterface():hardware_interface::SystemInterface(){}

CallbackReturn RM_DJIMotorHardwareInterface::on_init(const HardwareInfo & hardware_info){

    #ifdef DEBUG

    debug_node_ = rclcpp::Node::make_shared("dji_motor_hardware_interface_debug_node");

    debug_can_publishers_ = debug_node_->create_publisher<rm_interface::msg::RawCan>("debug_can", 10);

    debug_read_time_interval_publishers_ = debug_node_->create_publisher<std_msgs::msg::Float32>("debug_read_time_interval", 10);
    debug_write_time_interval_publishers_ = debug_node_->create_publisher<std_msgs::msg::Float32>("debug_write_time_interval", 10);

    #endif

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

    // read_times
    try{
        read_times_ = std::stoi(hardware_info.hardware_parameters.at("read_times"));
    }
    catch(const std::exception & e){
        RCLCPP_WARN(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "read_times not specified in hardware info, set to 5");
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
                .reverse = [&motor]{
                    bool reverse = false;
                    try{
                        reverse = (motor.parameters.at("reverse") == "true" || motor.parameters.at("reverse") == "True");
                    }
                    catch(const std::exception & e){
                        reverse = false;
                    }
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Motor " << motor.name << " reverse set to " << (reverse ? "true" : "false"));
                    return reverse;
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

            RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Motor " << motor.name << " initialized successfully");

            motor_attributes_.emplace_back(motor_attr);

        }
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to parse hardware info: " << e.what());
        return CallbackReturn::ERROR;
    }


    // 初始化电机反馈丢失计数器
    try{
        motor_back_frame_flage_cnt_.resize(motor_attributes_.size(), 0);
    }
    catch (const std::exception & e){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to initialize motor_back_frame_flage_cnt_: " << e.what());
        return CallbackReturn::ERROR;
    }

    // ros2 心跳线程
    try{
        ros2_heartbeat_thread_stop_.store(false);
        ros2_heartbeat_thread_ = std::make_shared<std::thread>([this](){
            rclcpp::Rate rate(1.0);
            while(rclcpp::ok()){
                rate.sleep();
                if(ros2_heartbeat_thread_stop_.load()) break;
            }
            if(can_driver_){
                for(int i=0; i<10; i++){
                    for(auto & motor_can_frame : can_frames_to_send_){
                        can_frame can_frame_zero = {};
                        can_frame_zero.can_id = motor_can_frame->can_id;
                        can_frame_zero.can_dlc = motor_can_frame->can_dlc;
                        can_driver_->sendMessage(can_frame_zero);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            RCLCPP_WARN(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "ROS2 is shutting down, try send 0.0 to CAN");
        });
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to start ros2 heartbeat thread: " << e.what());
        return CallbackReturn::ERROR;
    }

    #ifdef DEBUG
    
    for(const auto & motor : motor_attributes_){
        for(const auto & state_name : motor.state_names){
            std::string topic_name = "debug_" + motor.joint_name + "/state/" + state_name;
            debug_publishers_[topic_name] = debug_node_->create_publisher<std_msgs::msg::Float32>(topic_name, 10);
        }
        std::string topic_name = "debug_" + motor.joint_name + "/command/" + motor.command_name;
        debug_publishers_[topic_name] = debug_node_->create_publisher<std_msgs::msg::Float32>(topic_name, 10);
    }
    
    #endif

    RCLCPP_INFO(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Successfully parsed hardware info");

    return CallbackReturn::SUCCESS;

}


CallbackReturn RM_DJIMotorHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state){

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "RM_DJIMotorHardwareInterface on_configure with previous_state id: " << previous_state.id() << " label: " << previous_state.label());

    // 设置CAN通信

    can_driver_ = std::make_shared<Candriver>(can_port_);


    // 设置电机
    for(const auto & motor : motor_attributes_){
        if(motor.motor_type == "GM6020"){
            auto can_frame_processor = std::make_shared<GM6020>(motor.can_id, motor.joint_name, motor.reverse);
            can_frame_processors_.emplace_back(can_frame_processor);
        }
        else if(motor.motor_type == "C620"){
            auto can_frame_processor = std::make_shared<C620>(motor.can_id, motor.joint_name, motor.reverse);
            can_frame_processors_.emplace_back(can_frame_processor);
        }
        else if(motor.motor_type == "C610"){
            auto can_frame_processor = std::make_shared<C610>(motor.can_id, motor.joint_name, motor.reverse);
            can_frame_processors_.emplace_back(can_frame_processor);
        }
        else{
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Unsupported motor type: " << motor.motor_type);
            throw std::invalid_argument("Unsupported motor type: " + motor.motor_type);
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Motor " << can_frame_processors_.back()->name << " Can identifier: " << static_cast<int>(can_frame_processors_.back()->identifier));

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
    // if(!start_can_thread()){
    //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to start CAN thread");
    //     return CallbackReturn::ERROR;
    // }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "on_configure finish!");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DJIMotorHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state){

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "RM_DJIMotorHardwareInterface on_cleanup with previous_state id: " << previous_state.id() << " label: " << previous_state.label());

    can_frame_processors_.clear();
    can_frames_to_send_.clear();
    motor_back_frame_flage_cnt_.clear();

    // this->can_ok.store(false);
    // this->can_thread_stop.store(true);

    ros2_heartbeat_thread_stop_.store(true);
    if(ros2_heartbeat_thread_ && ros2_heartbeat_thread_->joinable()){
        ros2_heartbeat_thread_->join();
        ros2_heartbeat_thread_ = nullptr;
    }

    can_driver_ = nullptr;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "on_cleanup finish!");

    return CallbackReturn::SUCCESS;
    
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
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "RM_DJIMotorHardwareInterface on_activate with previous_state id: " << previous_state.id() << " label: " << previous_state.label());
    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DJIMotorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
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

    #ifdef DEBUG

    if(last_read_time_.nanoseconds() == 0){
        last_read_time_ = time;
    }

    auto interval = time - last_read_time_;
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(interval.seconds());
    this->debug_read_time_interval_publishers_->publish(msg);
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Read interval: " << msg.data << " seconds");
    last_read_time_ = time;

    #endif

    // return return_type::OK;

    auto sleep_duration = rclcpp::Duration::from_nanoseconds(period.nanoseconds() / (10 * read_times_));

    try{
        can_frame recived_frame = {};
        for(int i=0; i<read_times_; i++){
            bool get_frame = false;
            get_frame = can_driver_->receiveMessage(recived_frame);
            if(!get_frame){
                if(!can_driver_->isCanOk()){
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Not receive frame in Canport and Canport is OK");
                }
                else{
                    if(can_driver_->reopenCanSocket()){
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Not receive frame in Canport. Canport is not OK, but reopen Canport successfully");
                    }
                    else{
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Not receive frame in Canport. Canport is not OK, and reopen Canport failed");
                    }
                }
            }
            else{

                #ifdef DEBUG
                auto can_msg = rm_interface::msg::RawCan();
                can_msg.id = recived_frame.can_id;
                can_msg.can_dlc = recived_frame.can_dlc;
                for(int i=0; i<8; i++){
                    can_msg.data[i] = recived_frame.data[i];
                }
                this->debug_can_publishers_->publish(can_msg);
                #endif

                for(std::size_t i=0; i<can_frame_processors_.size(); i++){
                    const auto & processor = can_frame_processors_[i];
                    if(processor->processFrame(recived_frame)){
                        // 清除电机未收消息标志
                        motor_back_frame_flage_cnt_[i] = 0;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_duration.nanoseconds()));

        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Error occurred while reading ROS2 state ports: %s", e.what());
        return return_type::ERROR;
    }

    // 处理电机未收到反馈的情况
    for(std::size_t i=0; i<motor_back_frame_flage_cnt_.size(); i++){
        motor_back_frame_flage_cnt_[i]++;
        if(motor_back_frame_flage_cnt_[i] > 30){
            RCLCPP_WARN_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "No valid CAN frame received for joint " << can_frame_processors_[i]->name << " for " << motor_back_frame_flage_cnt_[i] << " cycles");
            motor_back_frame_flage_cnt_[i] = 30;
        }
    }

    #ifdef DEBUG

    for(const auto & motor : motor_attributes_){
        for(std::size_t i=0; i<motor.state_names.size(); i++){
            std_msgs::msg::Float32 msg;
            msg.data = *(motor.state_interface_ptrs[i]);
            std::string topic_name = "debug_" + motor.joint_name + "/state/" + motor.state_names[i];
            if(debug_publishers_.find(topic_name) != debug_publishers_.end()){
                debug_publishers_[topic_name]->publish(msg);
            }
        }
    }

    #endif


    return return_type::OK;

}

return_type RM_DJIMotorHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){

    #ifdef DEBUG

    if(last_write_time_.nanoseconds() == 0){
        last_write_time_ = time;
    }

    auto interval = time - last_write_time_;
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(interval.seconds());
    this->debug_write_time_interval_publishers_->publish(msg);
    last_write_time_ = time;

    #endif


    (void)time;
    (void)period;
    try{

        for(const auto & processor : can_frame_processors_){
            processor->writeIntoCanInterface();
        }
        for(const auto & frame : can_frames_to_send_){
            #ifdef DEBUG
            auto can_msg = rm_interface::msg::RawCan();
            can_msg.id = frame->can_id;
            can_msg.can_dlc = frame->can_dlc;
            for(int i=0; i<8; i++){
                can_msg.data[i] = frame->data[i];
            }
            this->debug_can_publishers_->publish(can_msg);
            #endif
            bool ok = can_driver_->sendMessage(*frame);
            if(!ok){
                if(!can_driver_->reopenCanSocket()){
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to send CAN frame with id " << std::hex << frame->can_id << " and reopen Canport failed");
                }
                else{
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Failed to send CAN frame with id " << std::hex << frame->can_id << " but reopen Canport successfully");
                }
            }
        }

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(rclcpp::get_logger("RM_DJIMotorHardwareInterface"), "Error occurred while writing ROS2 command ports: %s", e.what());
        return return_type::ERROR;
    }

    #ifdef DEBUG

    for(const auto & motor : motor_attributes_){
        if(motor.command_interface_ptr != nullptr){
            std_msgs::msg::Float32 msg;
            msg.data = *(motor.command_interface_ptr);
            std::string topic_name = "debug_" + motor.joint_name + "/command/" + motor.command_name;
            if(debug_publishers_.find(topic_name) != debug_publishers_.end()){
                debug_publishers_[topic_name]->publish(msg);
            }
        }
    }

    #endif

    return return_type::OK;
}

} // namespace RM_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RM_hardware_interface::RM_DJIMotorHardwareInterface, hardware_interface::SystemInterface);