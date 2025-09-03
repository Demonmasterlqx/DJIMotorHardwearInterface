#include "Candriver/candriver.hpp"
#include <iostream>
#include <cstring>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <errno.h>
#include <thread>


namespace RM_communication{

CanDriver::CanDriver(const std::string ifname):interface_name(ifname) {
    // 检查 ifname 长度
    if (interface_name.size() >= IFNAMSIZ) {
        throw std::runtime_error("CAN interface name is too long");
    }

    if (!openCanSocket()) {
        std::cerr << "Error opening CAN socket." << std::endl;
    }
}

bool CanDriver::openCanSocket() {

    if (isCanOk()) {
        throw std::runtime_error("Socket already opened");
    }

    setCanState(false);
    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ);

    // 创建套接字
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        std::cerr << "Error: Could not create CAN socket. " << strerror(errno) << std::endl;
        setCanState(false);
        closeCanSocket();
        return false;
    }


    // 检查can网口标志
    if (ioctl(socket_fd, SIOCGIFFLAGS, &ifr) < 0) {
        std::cerr << "Error: Could not get interface flags for " << interface_name << ". " << strerror(errno) << std::endl;
        setCanState(false);
        closeCanSocket();
        return false;
    }

    bool up_status = (ifr.ifr_flags & IFF_UP) != 0;

    if (!up_status) {
        // 尝试设置up can网口

        std::cerr << "Error: CAN interface " << interface_name << " is down." << std::endl;

        ifr.ifr_flags |= IFF_UP;
        if (ioctl(socket_fd, SIOCSIFFLAGS, &ifr) < 0) {
            std::cerr << "Error: Could not set interface " << interface_name << " up. " << strerror(errno) << std::endl;
            setCanState(false);
            closeCanSocket();
            return false;
        }
        else{
            std::cout << "Interface " << interface_name << " set to UP." << std::endl;
        }

    }

    // 指定can设备
    if(ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error: Could not get interface index for " << interface_name << ". " << strerror(errno) << std::endl;
        setCanState(false);
        closeCanSocket();
        return false;
    }

    if(ifr.ifr_ifindex <= 0) {
        std::cerr << "Error: Invalid interface index for " << interface_name << "." << std::endl;
        setCanState(false);
        closeCanSocket();
        return false;
    }

    // 绑定CAN socket
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error: Could not bind CAN socket. " << strerror(errno) << std::endl;
        closeCanSocket();
        setCanState(false);
        return false;
    }

    setCanState(true);
    return true;
}


void CanDriver::closeCanSocket() {
    if (socket_fd >= 0) {
        close(socket_fd);
        socket_fd = -1;
    }
    setCanState(false);
    return;
}

bool CanDriver::reopenCanSocket() {
    closeCanSocket();
    return openCanSocket();
}

bool CanDriver::sendMessage(const can_frame& frame) {
    if (!isCanOk()) {
        std::cerr << "Error: CAN socket is not open or interface is down. Cannot send message." << std::endl;
        return false;
    }

    // 在发送的时候设置为阻塞
    setBlockingMode(true);

    int nbytes = write(socket_fd, &frame, sizeof(frame));
    if (nbytes == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            std::cerr << "Warning: CAN socket send buffer full." << std::endl;
            return false;
        } else {
            std::cerr << "Error: Could not send CAN message due to an unrecoverable error. " << strerror(errno) << std::endl;
            setCanState(false);
            return false;
        }
    } else if (nbytes != sizeof(frame)) {
        std::cerr << "Error: Partial CAN message sent. Sent " << nbytes << " of " << sizeof(frame) << " bytes." << std::endl;
        setCanState(false);
        return false;
    }
    setCanState(true);
    return true;
}

bool CanDriver::receiveMessage(can_frame& frame) {
    if (!isCanOk()) {
        std::cerr << "Error: CAN socket is not open or interface is down. Cannot receive message." << std::endl;
        return false;
    }

    // 在读取的时候设置为不阻塞
    setBlockingMode(false);

    int nbytes = read(socket_fd, &frame, sizeof(frame));
    if (nbytes == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false; // 表示当前没有接收到消息
        } else {
            std::cerr << "Error: Could not read CAN message due to an unrecoverable error. " << strerror(errno) << std::endl;
            setCanState(false);
            return false;
        }
    } else if (nbytes != sizeof(frame)) {
        std::cerr << "Error: Partial CAN message read. Read " << nbytes << " of " << sizeof(frame) << " bytes." << std::endl;
        setCanState(false);
        return false;
    }
    setCanState(true);
    return true;
}

CanDriver::~CanDriver() {
    closeCanSocket();
}

bool CanDriver::isCanOk() {

    if(can_is_ok == false) {
        return false;
    }

    // 检查 socket 是否正常
    if(!((socket_fd >= 0) && (fcntl(socket_fd, F_GETFD) != -1 || errno != EBADF))){
        std::cerr << "Error: CAN socket is not valid." << std::endl;
        setCanState(false);
        return false;
    }

    // 检查 can 网口 状态
    if (ioctl(socket_fd, SIOCGIFFLAGS, &ifr) < 0) {
        std::cerr << "Error: Could not get interface flags for " << interface_name << ". " << strerror(errno) << std::endl;
        setCanState(false);
        return false;
    }

    bool up_status = (ifr.ifr_flags & IFF_UP) != 0;

    if (!up_status) {
        std::cerr << "Error: CAN interface " << interface_name << " is down." << std::endl;
        setCanState(false);
        return false;
    }

    return true;
}

bool CanDriver::setBlockingMode(bool blocking) {

    int flags = fcntl(socket_fd, F_GETFL, 0);
    if (flags == -1) {
        std::cerr << "Error getting flags for fd " << socket_fd << ": " << strerror(errno) << std::endl;
        return false;
    }

    if(blocking){
        flags &= ~O_NONBLOCK;
    }
    else{
        flags |= O_NONBLOCK;
    }

    if (fcntl(socket_fd, F_SETFL, flags) == -1) {
        std::cerr << "Error setting flags for fd " << socket_fd << ": " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

} // namespace RM_communication