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

    can_frame send_frame1 = {};
    can_frame send_frame2 = {};

    send_frame1.can_id = 0x1FF;
    send_frame2.can_id = 0x2FF;
    send_frame1.can_dlc = 8;
    send_frame2.can_dlc = 8;

    for(int i=0;i<4;i++){
        send_frame1.data[i*2] = 0x17;
        send_frame1.data[i*2+1] = 0x77;
        send_frame2.data[i*2] = 0x17;
        send_frame2.data[i*2+1] = 0x77;
    }

    while(1){

        if (canport->sendMessage(send_frame1)) {
            std::cout << "Message sent successfully." << std::endl;
        } else {
            if(!canport->isCanOk()){
                std::cerr << "Error sending message." << std::endl;
                bool reopened = canport->reopenCanSocket();
                if(!reopened) {
                    std::cerr << "Error reopening CAN socket." << std::endl;
                }
            }
            else{
                std::cout << "Send buffer full, message not sent." << std::endl;
            }
        }

        if (canport->sendMessage(send_frame2)) {
            std::cout << "Message sent successfully." << std::endl;
        } else {
            if(!canport->isCanOk()){
                std::cerr << "Error sending message." << std::endl;
                bool reopened = canport->reopenCanSocket();
                if(!reopened) {
                    std::cerr << "Error reopening CAN socket." << std::endl;
                }
            }
            else{
                std::cout << "Send buffer full, message not sent." << std::endl;
            }
        }

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