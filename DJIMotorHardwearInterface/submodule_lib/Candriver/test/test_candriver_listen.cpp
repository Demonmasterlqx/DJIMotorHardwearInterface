#include "Candriver/candriver.hpp"

#include <memory>
#include <iostream>

using RM_communication::CanDriver;

#include <unordered_map>
#include <chrono>

int main(){
    std::shared_ptr<CanDriver> canport = nullptr;

    try{
        canport = std::make_shared<CanDriver>("can0");
    } catch (const std::exception& e) {
        std::cerr << "Error initializing CanDriver: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "CanDriver initialized successfully." << std::endl;

    // 用于记录每个can_id上次接收的时间
    std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> last_receive_time;

    while(1){
        can_frame frame;
        bool rec = canport->receiveMessage(frame);
        if(!rec){
            if(!canport->isCanOk()){
                std::cerr << "Error receiving CAN message. try to reopen" << std::endl;
                bool reopened = canport->reopenCanSocket();
                if(!reopened) {
                    std::cerr << "Error reopening CAN socket." << std::endl;
                }
            }
            else{
                std::cout<< "No message received." << std::endl;
            }
            continue;
        }

        // 检测时间间隔
        auto now = std::chrono::steady_clock::now();
        uint32_t can_id = frame.can_id;
        if (last_receive_time.count(can_id)) {
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_receive_time[can_id]).count();
            std::cout << "[CAN ID " << can_id << "] Interval since last message: " << duration << " ms" << std::endl;
        }
        last_receive_time[can_id] = now;

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