#include "Candriver/candriver.hpp"

#include <memory>
#include <iostream>
#include <thread>
#include <unordered_map>
#include <chrono>

using RM_communication::CanDriver;

int main(){
    std::shared_ptr<CanDriver> canport = nullptr;
    std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> last_recv_time;

    try{
        canport = std::make_shared<CanDriver>("can0");
    } catch (const std::exception& e) {
        std::cerr << "Error initializing CanDriver: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "CanDriver initialized successfully." << std::endl;

    u_int16_t canid = 0x03; 

    can_frame send_frame = {};
    send_frame.can_id = 0x200 + canid;
    send_frame.can_dlc = 8;
    float v = 2.5;
    memcpy(send_frame.data, &v, sizeof(v));

    while(1){

        canport->reopenCanSocket();

        if (canport->sendMessage(send_frame)) {
            std::cout << "Message sent successfully." << std::endl;
        } else {
            std::cerr << "Error sending message." << std::endl;
            bool reopened = canport->reopenCanSocket();
            if(!reopened) {
                std::cerr << "Error reopening CAN socket." << std::endl;
            }
        }

        // std::this_thread::sleep_for(std::chrono::nanoseconds(1));

        can_frame frame;
        bool rec = canport->receiveMessage(frame);
        if(!rec){
            std::cerr << "Error receiving CAN message. try to reopen" << std::endl;
            bool reopened = canport->reopenCanSocket();
            if(!reopened) {
                std::cerr << "Error reopening CAN socket." << std::endl;
            }
            continue;
        }

        // 检测每个id的接收can帧的时间间隔（微秒）
        auto now = std::chrono::steady_clock::now();
        uint32_t id = frame.can_id;
        if (last_recv_time.count(id)) {
            auto interval = std::chrono::duration_cast<std::chrono::microseconds>(now - last_recv_time[id]).count();
            std::cout << "Interval since last frame for id " << std::hex << id << ": " << interval << " us" << std::endl;
        }
        last_recv_time[id] = now;

        // 输出can帧
        std::cout << "Can id " << frame.can_id;
        std::cout << " dlc " << static_cast<int>(frame.can_dlc);
        std::cout << " data ";
        for(int i=0; i< frame.can_dlc; i++){
            // 输出16进制
            std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::endl;

    }
    
    
}