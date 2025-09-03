#include<linux/can.h>
#include<sys/socket.h>
#include<linux/can.h>
#include<linux/can/raw.h>
#include<sys/ioctl.h>
#include<net/if.h>
#include<unistd.h>
#include<fcntl.h>
#include<errno.h>
#include<string.h>
#include<stdlib.h>
#include<string>
#include<stdexcept>

namespace RM_communication{

class CanDriver{
protected:
    // can socket 创建的文件描述符 -1 表示创建失败
    int socket_fd = -1;
    // can socket 地址
    struct sockaddr_can addr;
    struct ifreq ifr;
    // can interface name
    std::string interface_name;

    // can口是否初始化正常
    bool can_is_ok = false;


protected:
    /**
     * @brief 关闭当前的can socket，允许重复调用
     * 
     * @return true 成功
     * @return false 失败
     */
    void closeCanSocket();

    /**
     * @brief 绑定CAN socket ，必须对应closeCanSocket()之后才可再次调用，必须在
     * 
     * @return true 成功
     * @return false 失败
     */
    bool openCanSocket();

    /**
     * @brief Set the Can State object
     * 
     * @param state 
     */
    void setCanState(bool state) {
        can_is_ok = state;
    }

    /**
     * @brief Set the Blocking Mode object
     * 
     * @param blocking 
     * @return true 设置成功
     * @return false 设置失败
     */
    bool setBlockingMode(bool blocking);


public:

    /**
     * @brief 重新打开CAN socket
     * 
     * @return true 
     * @return false 
     */
    bool reopenCanSocket();

    /**
     * @brief Construct a new Can Driver object
     * 
     * @param ifname 指定的Can网口名称
     */
    CanDriver(const std::string ifname);

    /**
     * @brief 发送CAN消息
     * 
     * @param frame 要发送的CAN帧
     * @return true 成功
     * @return false 失败
     */
    bool sendMessage(const can_frame& frame);

    /**
     * @brief 接收CAN消息
     * 
     * @param frame 要接收的CAN帧
     * @return true 成功
     * @return false 失败
     */
    bool receiveMessage(can_frame& frame);

    /**
     * @brief 检查CAN状态，其只会检查到socket和网口状态,并不会检查指定can设备之后的操作
     * 
     * @return true CAN状态正常
     * @return false CAN状态异常
     */
    bool isCanOk();

    /**
     * @brief Destroy the Can Driver object
     * 
     */
    ~CanDriver();

};

} // namespace RM_communication