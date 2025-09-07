#include<linux/can.h>
#include<linux/can/raw.h>
#include<string>
#include<stdexcept>
#include<vector>
#include<memory>
#include <realtime_tools/realtime_buffer.hpp>


#ifndef RM_CAN_FRAME_PROCESSOR_HPP
#define RM_CAN_FRAME_PROCESSOR_HPP


namespace RM_hardware_interface {
    
int16_t combine_bytes_to_int16(uint8_t high_byte, uint8_t low_byte);

/**
 * @brief 用于标识每个电机的控制在CAN中对应的位置
 * 
 */
struct CanFramePosition{
    /**
     * @brief 标识符
     * 
     */
    u_int16_t identifier;
    /**
     * @brief 在CAN帧中的位置，两个字节一位，从0开始
     * 
     */
    u_int8_t position;
};

/**
 * @brief 一个线程安全的基类，用于读取和处理CAN帧，并且提供线程安全的读取数据与写入数据的接口
 * 
 */
class CanFrameProcessor {
public:
    CanFrameProcessor(const std::string& name,const u_int8_t canid, const u_int16_t preidentifier);
    virtual ~CanFrameProcessor();

    /**
     * @brief canid
     * 
     */

    const u_int8_t canid;

    /**
     * @brief 标识符
     * 
     */
    const u_int16_t identifier;

    /**
     * @brief joint name
     * 
     */
    const std::string name;

    /**
     * @brief 处理接收到的CAN帧，将can帧中的数据写入ROS2给的state接口中
     * 
     * @param frame 
     */
    virtual bool processFrame(const can_frame& frame) = 0;

    /**
     * @brief 设置 ROS2 Command 的输入地址
     * 
     * @param command_name ROS2 command 接口的名字
     * @param command_interface ROS2 command 接口的地址
     * 
     */
    virtual bool setCommandInterface(std::string command_name, std::shared_ptr<double> command_interface) = 0;

    /**
     * @brief 设置 ROS2 state 输出的地址
     * 
     * @param state_interfaces 
     * @param state_names 
     */
    virtual bool setStateInterfaces(std::vector<std::shared_ptr<double>> state_interfaces, std::vector<std::string> state_names) = 0;

    /**
     * @brief 将ROS2指令接口中的数据写到Can设定的帧中
     * 
     */
    virtual bool writeIntoCanInterface();

    /**
     * @brief Set the Can Command Interface object 其中first_command_interface 是高8位 first_command_interface+1 是低8位
     * 
     * @param first_command_interface 
     * @return true 
     * @return false 
     */
    virtual bool setCanCommandInterface(u_int8_t* first_command_interface);

    /**
     * @brief 获取该电机在CAN控制帧中的位置
     * 
     */
    virtual CanFramePosition getCanFramePosition() = 0;

protected:

    /**
     * @brief ROS2 state 接口
     * 
     */
    std::vector<std::shared_ptr<double>> state_interfaces_;

    /**
     * @brief ROS command 要写入的指令接口
     * 
     */
    std::shared_ptr<double> command_interface_;

    u_int8_t* first_command_interface = nullptr;

    /**
     * @brief 是否反转命令和反馈
     * 
     */
    bool reverse_ = false;

    /**
     * @brief 将电流转换成力矩
     * 
     */
    virtual double _current_to_torque(double current);

    /**
     * @brief 将力矩转换成电流
     * 
     */
    virtual double _torque_to_current(double torque);

    /**
     * @brief 从can帧中获取电流
     * 
     */
    virtual double _get_current(can_frame frame) = 0;

    /**
     * @brief 将电流反向映射到电机中的电流编码器的格式
     * 
     */
    virtual int16_t _get_current_reverse(double current) = 0;

};

class GM6020 : public CanFrameProcessor {
public:
    GM6020(const int canid, const std::string& name, bool reverse);
    ~GM6020();

    bool processFrame(const can_frame& frame) override;
    bool setCommandInterface(std::string command_name, std::shared_ptr<double> command_interface) override;
    bool setStateInterfaces(std::vector<std::shared_ptr<double>> state_interfaces, std::vector<std::string> state_names) override;

    CanFramePosition getCanFramePosition() override;

protected:
    double _get_current(can_frame frame) override;
    int16_t _get_current_reverse(double current) override;

private:

    static constexpr int POSITION_INDEX = 0;
    static constexpr int VELOCITY_INDEX = 1;
    static constexpr int TORQUE_INDEX = 2;
    static constexpr int TEMPERATURE_INDEX = 3;

};

class C620 : public CanFrameProcessor {
public:
    C620(const int canid, const std::string& name, bool reverse);
    ~C620();

    bool processFrame(const can_frame& frame) override;
    bool setCommandInterface(std::string command_name, std::shared_ptr<double> command_interface) override;
    bool setStateInterfaces(std::vector<std::shared_ptr<double>> state_interfaces, std::vector<std::string> state_names) override;

    CanFramePosition getCanFramePosition() override;

protected:
    double _get_current(can_frame frame) override;
    int16_t _get_current_reverse(double current) override;

private:

    static constexpr int POSITION_INDEX = 0;
    static constexpr int VELOCITY_INDEX = 1;
    static constexpr int TORQUE_INDEX = 2;
    static constexpr int TEMPERATURE_INDEX = 3;

};

class C610 : public CanFrameProcessor {
public:
    C610(const int canid, const std::string& name, bool reverse);
    ~C610();

    bool processFrame(const can_frame& frame) override;
    bool setCommandInterface(std::string command_name, std::shared_ptr<double> command_interface) override;
    bool setStateInterfaces(std::vector<std::shared_ptr<double>> state_interfaces, std::vector<std::string> state_names) override;

    CanFramePosition getCanFramePosition() override;

protected:
    double _get_current(can_frame frame) override;
    int16_t _get_current_reverse(double current) override;
    // 他的反馈就是力矩
    double _current_to_torque(double current) override;

private:

    static constexpr int POSITION_INDEX = 0;
    static constexpr int VELOCITY_INDEX = 1;
    static constexpr int TORQUE_INDEX = 2;

};
    

} // namespace RM_hardware_interface

#endif // RM_CAN_FRAME_PROCESSOR_HPP