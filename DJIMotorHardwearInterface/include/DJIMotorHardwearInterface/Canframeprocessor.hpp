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

/**
 * @brief 用于标识每个电机的控制在CAN中对应的位置
 * 
 */
struct CanFramePosition{
    /**
     * @brief 标识符
     * 
     */
    u_int8_t identifier;
    /**
     * @brief 在CAN帧中的位置，两个字节一位，从0开始
     * 
     */
    size_t position;
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

    const u_int16_t identifier;

    const std::string name;

    /**
     * @brief 处理接收到的CAN帧，将can帧中的数据写入缓存中
     * 
     * @param frame 
     */
    virtual bool processFrame(const can_frame& frame) = 0;

    /**
     * @brief 设置力矩的输入地址
     * 
     */
    virtual bool setCommandInterface(std::string command_name, std::shared_ptr<double> command_interface) = 0;

    /**
     * @brief 设置state输出的地址
     * 
     * @param state_interfaces 
     * @param state_names 
     */
    virtual bool setStateInterfaces(std::vector<std::shared_ptr<double>> state_interfaces, std::vector<std::string> state_names) = 0;

    /**
     * @brief 将缓存中的state数据读出来，存到设置的ros2的state接口中
     * 
     */
    virtual bool read() = 0;

    /**
     * @brief 将ROS2指令接口中的数据写到缓存中
     * 
     */
    virtual bool write() = 0;

    /**
     * @brief Set the Can Command Interface object 其中first_command_interface 是高8位 first_command_interface+1 是低8位
     * 
     * @param first_command_interface 
     * @return true 
     * @return false 
     */
    virtual bool setCanCommandInterface(u_int8_t* first_command_interface);

    /**
     * @brief 将命令缓存中的数据写入设定的CAN接口中
     * 
     */
    virtual bool writeIntoCanInterface();

    virtual CanFramePosition getCanFramePosition() = 0;

protected:

    /**
     * @brief 根据各自电机的设定自行排序，没有设置的就使用null
     * 
     */
    std::vector<std::shared_ptr<double>> state_interfaces_;

    /**
     * @brief 根据各自电机的设定自行排序，state_interfaces_对应的buffer
     * 
     */
    std::vector<realtime_tools::RealtimeBuffer<double>> state_buffers_;

    /**
     * @brief 力矩要写入的指令接口
     * 
     */
    std::shared_ptr<double> command_interface_;

    /**
     * @brief command_interface_对应的buffer
     * 
     */
    realtime_tools::RealtimeBuffer<double> command_buffer_;


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

    u_int8_t* first_command_interface = nullptr;

};

class GM6020 : public CanFrameProcessor {
public:
    GM6020(const int canid, const std::string& name);
    ~GM6020();

    bool processFrame(const can_frame& frame) override;
    bool setCommandInterface(std::string command_name, std::shared_ptr<double> command_interface) override;
    bool setStateInterfaces(std::vector<std::shared_ptr<double>> state_interfaces, std::vector<std::string> state_names) override;
    bool read() override;
    bool write() override;

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

} // namespace RM_hardware_interface

#endif // RM_CAN_FRAME_PROCESSOR_HPP;