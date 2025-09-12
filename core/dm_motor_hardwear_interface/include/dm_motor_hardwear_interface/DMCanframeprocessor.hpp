#ifndef RM_DM_CAN_FRAME_PROCESSOR_HPP
#define RM_DM_CAN_FRAME_PROCESSOR_HPP

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <linux/can.h>
#include <atomic>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace RM_hardware_interface{

typedef std::pair<std::string, int> InterfaceNamePos;
using StateInterface = hardware_interface::StateInterface;
using HardwareInfo = hardware_interface::HardwareInfo;
using CallbackReturn = hardware_interface::CallbackReturn;
using CommandInterface = hardware_interface::CommandInterface;
using return_type = hardware_interface::return_type;

enum class ControlModel {
    MIT = 0, // MIT模式
    POSITION = 1, // 位置模式
    VELOCITY = 2, // 速度模式
    READONLY = 3, // 只读模式
};

enum class CANFrameProcessorError : std::uint8_t{
    ERR_DISABLE = 0,         ///< 电机失能
    OK_ENABLE = 1,          ///< 电机使能
    ERR_OVERVOLTAGE = 8,     ///< 电机过压
    ERR_UNDERVOLTAGE = 9,    ///< 电机欠压
    ERR_OVERCURRENT = 0xA,   ///< 电机过电流
    ERR_MOS_OVERTEMP = 0xB,  ///< M驱动上 MOS 过温
    ERR_COIL_OVERTEMP = 0xC, ///< 电机线圈过温
    ERR_COMM_LOST = 0xD,     ///< 通讯丢失
    ERR_OVERLOAD = 0xE,      ///< 过载
    ERR_CANID = 0xF,          ///< CAN ID 错误，也就是说处理的CAN帧不是发给这个电机的
};

// 用于MIT控制模式的控制帧数据结构体
struct MITControlFrameData{
    uint16_t position_des : 16;  ///< 期望位置
    uint16_t velocity_des : 12; ///< 期望速度
    uint16_t kp : 12;
    uint16_t kd : 12;
    uint16_t torque_ff : 12;
}__attribute__((packed));

/**
 * @brief 电机反馈数据结构体
 * 
 */
struct MotorFeedBackTypeDef{
    uint8_t ID : 4;            ///< 电机反馈ID
    CANFrameProcessorError ERR : 4; ///< 电机状态码, 0：失能，1：使能，8：超压，9欠压，A：过电流，B：MOS过温，C：电机线圈过温，D：通讯丢失，E：过载
    uint16_t PositionFdb : 16;     ///< 电机当前位置反馈
    uint16_t SpeedFdb : 12;        ///< 电机当前速度反馈
    uint16_t TorqueFdb : 12;       ///< 电机当前扭矩反馈
    uint16_t TemMOS : 8;          ///< M驱动上 MOS 的平均温度，单位℃
    uint16_t TemRotor : 8;        ///< 表示电机内部线圈的平均温度，单位℃
};

struct MotorLimitTypeDef{
    float MaxPosition;     ///< 最大位置，单位弧度
    float MinPosition;     ///< 最小位置，单位弧度
    float MaxVelocity;     ///< 最大速度，单位弧度每秒
    float MinVelocity;     ///< 最小速度，单位弧度每秒
    float MaxTorque;       ///< 最大扭矩，单位牛米
    float MinTorque;       ///< 最小扭矩，单位牛米
    float MaxKp;           ///< 最大位置环比例增益
    float MinKp;           ///< 最小位置环比例增益
    float MaxKd;           ///< 最大速度环比例增益
    float MinKd;           ///< 最小速度环比例增益
};

/**
 * @brief 电机启动配置
 * 
 */
struct PortAttribute{
    const std::string joint_name;
    const u_int8_t can_id;
    const u_int8_t MST_ID;
    ControlModel control_model;
    bool reverse;
    MotorLimitTypeDef limits;
};

class DMCanframeprocessor{

public:
    /**
     * @brief Construct a new DMCanframeprocessor object
     * 
     * @param attribute 关节属性
     */
    DMCanframeprocessor(const PortAttribute& attribute);


    /**
     * @brief 得到电机对应的指令CAN 帧，这个函数将会从 command_interface_ptrs 中读取数据，并且将其转换为对应的CAN帧，当canid为 0 时，表示不发送控制帧
     * 
     * @return can_frame 
     */
    can_frame getCommandFrame();

    /**
     * @brief 得到电机对应的失能CAN帧
     * 
     * @return can_frame 
     */
    can_frame getDisableFrame();

    /**
     * @brief 得到电机对应的使能CAN帧
     * 
     * @return can_frame 
     */
    can_frame getEnableFrame();

    /**
     * @brief 得到错误处理的CAN帧
     * 
     * @return can_frame 
     */
    can_frame getClearErrorFrame();

    /**
     * @brief 置零模式控制函数，也就是置零指令
     * 
     */
    can_frame getPositionZeroFrame();

    /**
     * @brief 处理接收到的CAN帧
     * 
     * @param frame 接收到的CAN帧
     * @return PortAttribute CAN帧对应的错误状态
     */
    CANFrameProcessorError processReceivedFrame(const can_frame& frame);

    /**
     * @brief 根据得到的关节属性，得到对应的状态接口
     * 
     * @return state_interfaces std::vector<StateInterface> 
     */
    void getStateInterfaces(std::vector<StateInterface>& state_interfaces);

    /**
     * @brief 根据得到的关节属性，得到对应的指令接口
     * 
     * @param command_interfaces std::vector<CommandInterface> 
     */
    void getCommandInterfaces(std::vector<CommandInterface>& command_interfaces);

    /**
     * @brief 获取关节的名字
     * 
     * @return std::string 
     */
    std::string getJointName() const;

    /**
     * @brief 设置位置为零点模式，在一次发送之后将会自动取消该模式
     * 
     */
    void setPositionZero(){
        is_set_position_zero_.store(true);
    }

    /**
     * @brief Get the Motor State object 这是一个获取电机状态的函数，主要用于调试，请保证在电机初始化完成后调用，一般在read函数中调用
     * 
     * @param position 
     * @param velocity 
     * @param torque 
     * @param tempmos 
     * @param temprotor 
     */
    void getMotorState(float & position, float & velocity, float & torque, float & tempmos, float & temprotor);

private:

    /**
     * @brief 状态接口的指针
     * 
     */
    std::vector<std::shared_ptr<double>> state_interface_ptrs_;
    /**
     * @brief 指令接口的指针
     * 
     */
    std::vector<std::shared_ptr<double>> command_interface_ptrs_;

    /**
     * @brief 电机的CANID 注意不是标识符
     * 
     */
    u_int8_t can_id_;

    /**
     * @brief joint的名字
     * 
     */
    std::string joint_name_;

    /**
     * @brief 电机的master_id 反馈时使用
     * 
     */
    u_int8_t MST_ID_;

    /**
     * @brief 电机控制帧对应的CAN帧ID
     * 
     */
    u_int8_t COMMAND_CAN_ID;

    /**
     * @brief 电机的控制模式
     * 
     */
    ControlModel control_model_;

    /**
     * @brief 是否反转电机
     * 
     */
    bool reverse_;

    /**
     * @brief 是否是设置过位置零点状态
     * 
     */
    std::atomic<bool> is_set_position_zero_{false};

    /**
     * @brief 电机参数限制，主要用于值得映射
     * 
     */
    MotorLimitTypeDef limits_;

    /**
     * @brief state interface 的 名字 与 位置
     * 
     */
    const static std::vector<InterfaceNamePos> state_name_pos_vec;

    /**
     * @brief command interface 的 名字 与 位置
     * 
     */
    std::vector<InterfaceNamePos> command_name_pos_ref;

    /**
     * @brief MIT 控制命令的 名字 与 位置
     * 
     */
    const static std::vector<InterfaceNamePos> MIT_comand_name_pos_vec;

    /**
     * @brief POSITION 控制命令的 名字 与 位置
     * 
     */
    const static std::vector<InterfaceNamePos> POSITION_comand_name_pos_vec;

    /**
     * @brief VELOCITY 控制命令的 名字 与 位置
     * 
     */
    const static std::vector<InterfaceNamePos> VELOCITY_comand_name_pos_vec;

    /**
     * @brief  READONLY 控制命令的 名字 与 位置
     * 
     */
    const static std::vector<InterfaceNamePos> READ_comand_name_pos_vec;

    /**
     * @brief ZERO 控制命令的 名字 与 位置
     * 
     */
    const static std::vector<InterfaceNamePos> ZERO_comand_name_pos_vec;

    const int STATE_POSITION = 0;
    const int STATE_VELOCITY = 1;
    const int STATE_EFFORT = 2;
    const int STATE_TEMPMOS = 3;
    const int STATE_TEMPROTOR = 4;
    const int STATE_ERRORLABEL = 5;
    const int COMMAND_VELOCITYDES = 0;
    const int COMMAND_POSITIONDES = 1;
    const int COMMAND_KP = 2;
    const int COMMAND_KD = 3;
    const int COMMAND_TORQUEFF = 4;

    const uint8_t Enable_Frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};           // 使能帧，DM电机需要初始化时发送该帧才能控制
    const uint8_t Disable_Frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};          // 失能帧
    const uint8_t SaveZeroPosition_Frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 保存零点帧
    const uint8_t ClearError_Frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};       // 清除错误帧

    /**
     * @brief 控制函数，根据控制模式的不同，指向不同的函数，在初始化的时候确定
     * 
     */
    std::function<can_frame()> command_function_;

    /**
     * @brief MIT 控制函数
     * 
     */
    can_frame MITCommandFunction();

    /**
     * @brief POSITION 控制函数
     * 
     */
    can_frame POSITIONCommandFunction();

    /**
     * @brief VELOCITY 控制函数
     * 
     */
    can_frame VELOCITYCommandFunction();

    /**
     * @brief 只读模式控制函数
     * 
     */
    can_frame READCommandFunction();

};

} // namespace RM_hardware_interface

#endif