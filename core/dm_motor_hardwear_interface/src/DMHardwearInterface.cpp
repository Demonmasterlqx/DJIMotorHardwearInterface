#include "dm_motor_hardwear_interface/DMCanframeprocessor.hpp"
#include "dm_motor_hardwear_interface/DMHardwearInterface.hpp"

namespace RM_hardware_interface{

ControlModel RM_DMMotorHardwearInterface::_str_to_control_model(const std::string& control_model_str){
    if(control_model_str == "MIT"){
        return ControlModel::MIT;
    }
    else if(control_model_str == "P" || control_model_str == "position"){
        return ControlModel::POSITION;
    }
    else if(control_model_str == "V" || control_model_str == "velocity"){
        return ControlModel::VELOCITY;
    }
    else if(control_model_str == "R" || control_model_str == "readonly"){
        return ControlModel::READONLY;
    }
    else{
        RCLCPP_WARN(nh_->get_logger(), "Unknown control model string, set to READONLY");
        return ControlModel::READONLY;
    }
}

RM_DMMotorHardwearInterface::RM_DMMotorHardwearInterface(){}

CallbackReturn RM_DMMotorHardwearInterface::on_init(const HardwareInfo & hardware_info){

    nh_ = rclcpp::Node::make_shared(hardware_info.name);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(nh_);

    #ifdef DEBUG

    write_time_interval_publishers_ = nh_->create_publisher<std_msgs::msg::Float32>(hardware_info.name + "/write_time_interval", 10);
    read_time_interval_publishers_ = nh_->create_publisher<std_msgs::msg::Float32>(hardware_info.name + "/read_time_interval", 10);
    can_frame_publisher_ = nh_->create_publisher<rm_interface::msg::RawCan>(hardware_info.name + "/can_frame", 10);

    #endif

    zero_position_sub_ = nh_->create_subscription<std_msgs::msg::Bool>(
        hardware_info.name + "/zero_position", 10, std::bind(&RM_DMMotorHardwearInterface::zero_position_callback, this, std::placeholders::_1)
    );
    enable_motor_sub_ = nh_->create_subscription<std_msgs::msg::Bool>(
        hardware_info.name + "/enable_motor", 10, std::bind(&RM_DMMotorHardwearInterface::enable_motor_callback, this, std::placeholders::_1)
    );

    // can port
    try{
        can_port_ = hardware_info.hardware_parameters.at("can_port");
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(nh_->get_logger(), "Can port not specified in hardware info");
        return CallbackReturn::ERROR;
    }

    if(can_port_ == ""){
        RCLCPP_ERROR(nh_->get_logger(), "Can port is miss or empty");
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(nh_->get_logger(), "Can port set to: %s", can_port_.c_str());

    // read_times
    try{
        read_times_ = std::stoi(hardware_info.hardware_parameters.at("read_times"));
    }
    catch(const std::exception & e){
        RCLCPP_WARN(nh_->get_logger(), "read_times not specified in hardware info, set to 5");
        read_times_ = 5;
    }
    
    if(read_times_ <= 0){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "read_times is not valid with value "<< read_times_ << ". set to 5");
        read_times_ = 5;
    }
    RCLCPP_INFO(nh_->get_logger(), "Read times set to: %d", read_times_);

    ControlModel control_model = ControlModel::READONLY;
    std::string control_model_str = "";

    try{
        control_model_str = hardware_info.hardware_parameters.at("control_model");
    }
    catch(const std::exception & e){
        RCLCPP_WARN(nh_->get_logger(), "control_model not specified in hardware info, set to READONLY");
        control_model = ControlModel::READONLY;
    }

    if(control_model_str != ""){
        control_model = _str_to_control_model(control_model_str);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Global Control model set to: " << control_model_str);
    }

    // 开始填充电机参数
    for(const auto & motor : hardware_info.joints){

        // 拿到基本信息
        const std::string motor_name = motor.name;
        int can_id = 0;
        int MST_ID = 0;
        bool reverse = false;
        ControlModel motor_control_model = control_model;
        MotorLimitTypeDef motor_limit = {};

        try{
            can_id = std::stoi(motor.parameters.at("can_id"));
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " can_id not specified or invalid");
            return CallbackReturn::ERROR;
        }

        try{
            MST_ID = std::stoi(motor.parameters.at("MST_ID"), 0, 16);
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " MST_ID not specified or invalid");
            return CallbackReturn::ERROR;
        }

        try{
            reverse = (motor.parameters.at("reverse") == "true" || motor.parameters.at("reverse") == "True");
        }
        catch(const std::exception & e){
            reverse = false;
        }

        {// 设置 control_model
            std::string motor_control_model_str = "";
            try{
                motor_control_model_str = motor.parameters.at("control_model");
            }
            catch(const std::exception & e){
                motor_control_model_str = "";
            }
            if(motor_control_model_str != ""){
                motor_control_model = _str_to_control_model(motor_control_model_str);
            }
        }

        // 设置电机的限制参数
        try{
            motor_limit.MaxPosition = std::stof(motor.parameters.at("max_position"));
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_position not specified or invalid");
            return CallbackReturn::ERROR;
        }
        try{
            motor_limit.MinPosition = std::stof(motor.parameters.at("min_position"));
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " min_position not specified or invalid");
            return CallbackReturn::ERROR;
        }
        try{
            motor_limit.MaxVelocity = std::stof(motor.parameters.at("max_velocity"));
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_velocity not specified or invalid");
            return CallbackReturn::ERROR;
        }
        try{
            motor_limit.MinVelocity = std::stof(motor.parameters.at("min_velocity"));
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " min_velocity not specified or invalid");
            return CallbackReturn::ERROR;
        }
        try{
            motor_limit.MaxTorque = std::stof(motor.parameters.at("max_torque"));
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_torque not specified or invalid");
            return CallbackReturn::ERROR;
        }
        try{
            motor_limit.MinTorque = std::stof(motor.parameters.at("min_torque"));
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " min_torque not specified or invalid");
            return CallbackReturn::ERROR;
        }

        try{
            motor_limit.MaxKp = std::stof(motor.parameters.at("max_kp"));
        }
        catch(const std::exception & e){
            if(control_model == ControlModel::MIT){
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_kp not specified or invalid");
                return CallbackReturn::ERROR;
            }
            else
                motor_limit.MaxKp = 0.0;
        }
        try{
            motor_limit.MinKp = std::stof(motor.parameters.at("min_kp"));
        }
        catch(const std::exception & e){
            if(control_model == ControlModel::MIT){
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " min_kp not specified or invalid");
                return CallbackReturn::ERROR;
            }
            else
                motor_limit.MinKp = 0.0;
        }
        try{
            motor_limit.MaxKd = std::stof(motor.parameters.at("max_kd"));
        }
        catch(const std::exception & e){
            if(control_model == ControlModel::MIT){
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_kd not specified or invalid");
                return CallbackReturn::ERROR;
            }
            else
                motor_limit.MaxKd = 0.0;
        }
        try{
            motor_limit.MinKd = std::stof(motor.parameters.at("min_kd"));
        }
        catch(const std::exception & e){
            if(control_model == ControlModel::MIT){
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " min_kd not specified or invalid");
                return CallbackReturn::ERROR;
            }
            else
                motor_limit.MinKd = 0.0;
        }

        // 检查所有的限制参数是否合理
        if(std::abs(motor_limit.MaxPosition - motor_limit.MinPosition) <= 1e-9){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_position is less than min_position");
            return CallbackReturn::ERROR;
        }
        if(std::abs(motor_limit.MaxVelocity - motor_limit.MinVelocity) <= 1e-9){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_velocity is less than min_velocity");
            return CallbackReturn::ERROR;
        }
        if(std::abs(motor_limit.MaxTorque - motor_limit.MinTorque) <= 1e-9){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_torque is less than min_torque");
            return CallbackReturn::ERROR;
        }
        if(control_model == ControlModel::MIT){
            if(std::abs(motor_limit.MaxKp - motor_limit.MinKp) <= 1e-9){
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_kp is less than min_kp");
                return CallbackReturn::ERROR;
            }
            if(std::abs(motor_limit.MaxKd - motor_limit.MinKd) <= 1e-9){
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Motor " << motor_name << " max_kd is less than min_kd");
                return CallbackReturn::ERROR;
            }
        }


        motor_attributes_.push_back({
            .joint_name = motor_name,
            .can_id = static_cast<u_int8_t>(can_id),
            .MST_ID = static_cast<u_int8_t>(MST_ID),
            .control_model = motor_control_model,
            .reverse = reverse,
            .limits = motor_limit
        });

        // 输出所有信息

        RCLCPP_INFO_STREAM(nh_->get_logger(), "Motor " << motor_name << " initialized successfully with parameters:");
        RCLCPP_INFO_STREAM(nh_->get_logger(), "  can_id: " << can_id);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "  MST_ID: " << std::hex << MST_ID << std::dec);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "  reverse: " << (reverse ? "true" : "false"));
        RCLCPP_INFO_STREAM(nh_->get_logger(), "  control_model: " << (motor_control_model == ControlModel::MIT ? "MIT" : (motor_control_model == ControlModel::POSITION ? "POSITION" : (motor_control_model == ControlModel::VELOCITY ? "VELOCITY" : "READONLY"))));
        RCLCPP_INFO_STREAM(nh_->get_logger(), "  limits:");
        RCLCPP_INFO_STREAM(nh_->get_logger(), "    MaxPosition: " << motor_limit.MaxPosition);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "    MinPosition: " << motor_limit.MinPosition);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "    MaxVelocity: " << motor_limit.MaxVelocity);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "    MinVelocity: " << motor_limit.MinVelocity);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "    MaxTorque: " << motor_limit.MaxTorque);
        RCLCPP_INFO_STREAM(nh_->get_logger(), "    MinTorque: " << motor_limit.MinTorque);
        if(motor_control_model == ControlModel::MIT){
            RCLCPP_INFO_STREAM(nh_->get_logger(), "    MaxKp: " << motor_limit.MaxKp);
            RCLCPP_INFO_STREAM(nh_->get_logger(), "    MinKp: " << motor_limit.MinKp);
            RCLCPP_INFO_STREAM(nh_->get_logger(), "    MaxKd: " << motor_limit.MaxKd);
            RCLCPP_INFO_STREAM(nh_->get_logger(), "    MinKd: " << motor_limit.MinKd);
        }

        #ifdef DEBUG
        // 创建电机state调试信息的publisher
        try{
            motor_state_debug_info_publishers_.push_back(nh_->create_publisher<rm_interface::msg::MotorState>(hardware_info.name + "/" + motor_name + "/state_debug_info", 10));
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Failed to create motor state debug info publisher for motor " << motor_name << ": " << e.what());
            return CallbackReturn::ERROR;
        }

        // 创建error信息的publisher
        try{
            error_info_publishers_.push_back(nh_->create_publisher<std_msgs::msg::Int32>(hardware_info.name + "/" + motor_name + "/error_info", 10));
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Failed to create motor error info publisher for motor " << motor_name << ": " << e.what());
            return CallbackReturn::ERROR;
        }
        #endif

    }

    {// 检查所有的ID是否重复，包括 MAST_ID 和 can_id
        
        std::set<u_int8_t> can_id_set;
        std::set<u_int8_t> MST_ID_set;
        for(const auto & motor_attr : motor_attributes_){
            if(can_id_set.find(motor_attr.can_id) != can_id_set.end()){
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Duplicate can_id found: " << static_cast<int>(motor_attr.can_id) << " motor name: " << motor_attr.joint_name);
                return CallbackReturn::ERROR;
            }
            can_id_set.insert(motor_attr.can_id);
            if(MST_ID_set.find(motor_attr.MST_ID) != MST_ID_set.end()){
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Duplicate MST_ID found: " << std::hex << static_cast<int>(motor_attr.MST_ID) << std::dec << " motor name: " << motor_attr.joint_name);
                return CallbackReturn::ERROR;
            }
            MST_ID_set.insert(motor_attr.MST_ID);
        }

    }

    // 初始化电机反馈丢失计数器
    try{
        motor_back_frame_flage_cnt_.resize(motor_attributes_.size(), 0);
    }
    catch (const std::exception & e){
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Failed to initialize motor_back_frame_flage_cnt_: " << e.what());
        return CallbackReturn::ERROR;
    }

    // ros2 心跳线程
    try{
        ros2_heartbeat_thread_stop_.store(false);
        ros2_heartbeat_thread_ = std::make_shared<std::thread>([this](){
            while(rclcpp::ok()){
                executor_->spin_once(std::chrono::seconds(1));
                if(ros2_heartbeat_thread_stop_.load()) break;
            }
            if(can_driver_){
                for(int i=0; i<10; i++){
                    for(auto & motor : can_frame_processors_){
                        can_frame can_frame_zero = motor->getDisableFrame();
                        can_driver_->sendMessage(can_frame_zero);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            RCLCPP_WARN(nh_->get_logger(), "ROS2 is shutting down, try send 0.0 to CAN");
        });
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Failed to start ros2 heartbeat thread: " << e.what());
        return CallbackReturn::ERROR;
    }

    try{
        for(const auto & motor_attr : motor_attributes_){
            can_frame_processors_.push_back(std::make_shared<DMCanframeprocessor>(motor_attr));
        }
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Failed to create DMCanframeprocessor: " << e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(nh_->get_logger(), "SUCCESSFULLY initialized DM motor hardware interface");

    return CallbackReturn::SUCCESS;

}

CallbackReturn RM_DMMotorHardwearInterface::on_configure(const rclcpp_lifecycle::State & previous_state){

    RCLCPP_INFO_STREAM(nh_->get_logger(), "RM_DMMotorHardwearInterface on_configure with previous_state id: " << previous_state.id() << " label: " << previous_state.label());

    can_driver_ = std::make_shared<Candriver>(can_port_);

    RCLCPP_INFO(nh_->get_logger(), "RM_DMMotorHardwearInterface configured successfully");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DMMotorHardwearInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state){

    RCLCPP_INFO_STREAM(nh_->get_logger(), "RM_DMMotorHardwearInterface on_cleanup with previous_state id: " << previous_state.id() << " label: " << previous_state.label());

    ros2_heartbeat_thread_stop_.store(true);
    if(ros2_heartbeat_thread_ && ros2_heartbeat_thread_->joinable()){
        ros2_heartbeat_thread_->join();
        ros2_heartbeat_thread_ = nullptr;
    }

    can_frame_processors_.clear();
    can_driver_ = nullptr;
    motor_back_frame_flage_cnt_.clear();

    RCLCPP_INFO(nh_->get_logger(), "RM_DMMotorHardwearInterface cleaned up successfully");
    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RM_DMMotorHardwearInterface::export_state_interfaces(){
    std::vector<StateInterface> state_interfaces = {};
    for(const auto & motor : can_frame_processors_){
        motor->getStateInterfaces(state_interfaces);
    }
    // 输出所有的state interface
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Exporting state interfaces "<< state_interfaces.size() << " interfaces");

    return state_interfaces;
}

std::vector<CommandInterface> RM_DMMotorHardwearInterface::export_command_interfaces(){
    std::vector<CommandInterface> command_interfaces = {};
    for(const auto & motor : can_frame_processors_){
         motor->getCommandInterfaces(command_interfaces);
    }
    // 输出所有的command interface
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Exporting command interfaces "<< command_interfaces.size() << " interfaces");

    return command_interfaces;
}

CallbackReturn RM_DMMotorHardwearInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO_STREAM(nh_->get_logger(), "RM_DMMotorHardwearInterface on_activate with previous_state id: " << previous_state.id() << " label: " << previous_state.label());
    
    // 先给所有的电机发送一个失能帧
    try{
        if(can_driver_){
            if(!can_driver_->isCanOk()){
                if(!can_driver_->reopenCanSocket()){
                    RCLCPP_ERROR(nh_->get_logger(), "Failed to open CAN socket File to Send frame in on_activate");
                    return CallbackReturn::SUCCESS;
                }
            }
            for(auto & motor : can_frame_processors_){
                can_frame can_frame_zero = motor->getDisableFrame();
                can_driver_->sendMessage(can_frame_zero);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        // 再发一个使能帧
            // for(auto & motor : can_frame_processors_){
            //     can_frame can_frame_enable = motor->getEnableFrame();
            //     can_driver_->sendMessage(can_frame_enable);
            //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
            // }
        }
        else{
            RCLCPP_ERROR(nh_->get_logger(), "CAN driver is not initialized in on_activate");
            return CallbackReturn::ERROR;
        }
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Failed to send frame in on_activate: " << e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DMMotorHardwearInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO_STREAM(nh_->get_logger(), "RM_DMMotorHardwearInterface deactivated with previous_state id: " << previous_state.id() << " label: " << previous_state.label());
    
    try{
        // 给所有的电机发送一个失能帧
        if(can_driver_){
            if(!can_driver_->isCanOk()){
                if(!can_driver_->reopenCanSocket()){
                    RCLCPP_ERROR(nh_->get_logger(), "Failed to open CAN socket File to Send disable frame in on_deactivate");
                    return CallbackReturn::SUCCESS;
                }
            }
            for(auto & motor : can_frame_processors_){
                can_frame can_frame_zero = motor->getDisableFrame();
                can_driver_->sendMessage(can_frame_zero);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Failed to send disable frame in on_deactivate: " << e.what());
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DMMotorHardwearInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO_STREAM(nh_->get_logger(), "RM_DMMotorHardwearInterface shutting down called with previous_state id: " << previous_state.id() << " label: " << previous_state.label() << " No thing happen.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RM_DMMotorHardwearInterface::on_error(const rclcpp_lifecycle::State & previous_state){

    RCLCPP_ERROR_STREAM(nh_->get_logger(), "RM_DMMotorHardwearInterface on_error called with previous_state id: " << previous_state.id() << "label: " << previous_state.label() << "try to restart all hardware interface.");

    return CallbackReturn::SUCCESS;
}

return_type RM_DMMotorHardwearInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){
    (void)time;
    (void)period;

    #ifdef DEBUG

    if(last_read_time_.nanoseconds() == 0){
        last_read_time_ = time;
    }

    auto interval = time - last_read_time_;
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(interval.seconds());
    this->read_time_interval_publishers_->publish(msg);
    last_read_time_ = time;

    #endif


    auto sleep_duration = rclcpp::Duration::from_nanoseconds(period.nanoseconds() / (10 * read_times_));

    try{
        can_frame recived_frame = {};
        for(int i=0; i<read_times_; i++){
            bool get_frame = false;
            get_frame = can_driver_->receiveMessage(recived_frame);
            if(!get_frame){
                if(!can_driver_->isCanOk()){
                    RCLCPP_WARN_STREAM(nh_->get_logger(), "Not receive frame in Canport and Canport is OK");
                }
                else{
                    if(can_driver_->reopenCanSocket()){
                        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Not receive frame in Canport. Canport is not OK, but reopen Canport successfully");
                    }
                    else{
                        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Not receive frame in Canport. Canport is not OK, and reopen Canport failed");
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
                this->can_frame_publisher_->publish(can_msg);
                #endif

                for(std::size_t i=0; i<can_frame_processors_.size(); i++){
                    const auto & processor = can_frame_processors_[i];
                    CANFrameProcessorError error = processor->processReceivedFrame(recived_frame);
                    if(error != CANFrameProcessorError::ERR_CANID){
                        motor_back_frame_flage_cnt_[i] = 0;
                        can_frame frame_to_send = {};
                        switch (error){
                        case CANFrameProcessorError::OK_ENABLE:
                            if(!motor_enable_flag_.load()){
                                frame_to_send = processor->getDisableFrame();
                                RCLCPP_INFO_STREAM(nh_->get_logger(), "motor_enable_flag_ set false Motor " << processor->getJointName() << " is enabled, try to enable all motors");
                            }
                            break;
                        case CANFrameProcessorError::ERR_DISABLE:
                            if(motor_enable_flag_.load()){
                                RCLCPP_WARN_STREAM(nh_->get_logger(), "motor_enable_flag_ set true Motor " << processor->getJointName() << " is disabled, try to enable it");
                                frame_to_send = processor->getEnableFrame();
                            }
                            break;
                        default:
                            RCLCPP_ERROR_STREAM(nh_->get_logger(), "Error occurred while processing frame for motor " << processor->getJointName() << ": " << static_cast<int>(error) << ", send clear error frame");
                            frame_to_send = processor->getClearErrorFrame();
                            break;
                        }
                        if(frame_to_send.can_id != 0){
                            if(can_driver_->isCanOk()){
                                can_driver_->sendMessage(frame_to_send);
                            }
                            else{
                                if(can_driver_->reopenCanSocket()){
                                    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Canport is not OK, but reopen Canport successfully");
                                    can_driver_->sendMessage(frame_to_send);
                                }
                                else{
                                    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Canport is not OK, and reopen Canport failed");
                                }
                            }
                        }

                        #ifdef DEBUG
                        rm_interface::msg::MotorState motor_state_msg;
                        motor_state_msg.reserved.resize(1);
                        processor->getMotorState(motor_state_msg.position, motor_state_msg.velocity, motor_state_msg.torque, motor_state_msg.temperature, motor_state_msg.reserved[0]);
                        motor_state_debug_info_publishers_[i]->publish(motor_state_msg);
                        std_msgs::msg::Int32 error_msg;
                        error_msg.data = static_cast<int>(error);
                        error_info_publishers_[i]->publish(error_msg);
                        #endif
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_duration.nanoseconds()));
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(nh_->get_logger(), "Error occurred while reading ROS2 state ports: %s", e.what());
        return return_type::ERROR;
    }

    // 处理电机未收到反馈的情况
    for(std::size_t i=0; i<motor_back_frame_flage_cnt_.size(); i++){
        motor_back_frame_flage_cnt_[i]++;
        if(motor_back_frame_flage_cnt_[i] > 30){
            RCLCPP_WARN_STREAM(nh_->get_logger(), "No valid CAN frame received for joint " << can_frame_processors_[i]->getJointName() << " for " << motor_back_frame_flage_cnt_[i] << " cycles");
            motor_back_frame_flage_cnt_[i] = 30;
        }
    }

    return return_type::OK;

}

return_type RM_DMMotorHardwearInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){

    #ifdef DEBUG

    if(last_write_time_.nanoseconds() == 0){
        last_write_time_ = time;
    }

    auto interval = time - last_write_time_;
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(interval.seconds());
    this->write_time_interval_publishers_->publish(msg);
    last_write_time_ = time;

    #endif


    (void)time;
    (void)period;

    auto sleep_duration = rclcpp::Duration::from_nanoseconds(period.nanoseconds() / (10 * read_times_));

    try{

        std::vector<can_frame> can_frames_to_send_ = {};

        for(const auto & processor : can_frame_processors_){
            can_frames_to_send_.push_back(processor->getCommandFrame());
        }
        for(const auto & frame : can_frames_to_send_){
            bool ok = can_driver_->sendMessage(frame);
            if(!ok){
                if(!can_driver_->reopenCanSocket()){
                    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Failed to send CAN frame with id " << std::hex << frame.can_id << " and reopen Canport failed");
                }
                else{
                    RCLCPP_WARN_STREAM(nh_->get_logger(), "Failed to send CAN frame with id " << std::hex << frame.can_id << " but reopen Canport successfully");
                }
            }
        }

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(nh_->get_logger(), "Error occurred while writing ROS2 command ports: %s", e.what());
        return return_type::ERROR;
    }

    return return_type::OK;
}

void RM_DMMotorHardwearInterface::zero_position_callback(const std_msgs::msg::Bool::SharedPtr msg){
    if(msg->data){
        RCLCPP_WARN(nh_->get_logger(), "Zero position command received, set current position to zero");
        for(const auto & processor : can_frame_processors_){
            processor->setPositionZero();
        }
    }
}

void RM_DMMotorHardwearInterface::enable_motor_callback(const std_msgs::msg::Bool::SharedPtr msg){
    motor_enable_flag_.store(msg->data);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Enable motor command received, set motor_enable_flag_ to " << (msg->data ? "true" : "false"));
}

} // namespace RM_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RM_hardware_interface::RM_DMMotorHardwearInterface, hardware_interface::SystemInterface);