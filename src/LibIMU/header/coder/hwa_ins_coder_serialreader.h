#ifndef hwa_ins_coder_serialreader_h
#define hwa_ins_coder_serialreader_h

#include "hwa_ins_coder_udpclient.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX 
#include <windows.h>
#include <string>
#include <thread>
#include <sstream>
#include <chrono>
#include <atomic>
#include <direct.h>     
#include <ctime>        
#include <iomanip>   
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "hwa_base_eigendef.h"
#include "hwa_base_glob.h"
#include "hwa_ins_data.h"

namespace hwa_ins {
    class ins_reader {
    public:
        ins_reader() {};
        ins_reader(int ID, const std::string& port, int baudrate, double _ts, ins_udp_client* udpClient, const std::string& savepath, bool Out);
        ~ins_reader();

        bool start();
        void stop();
        void readThread();
        std::vector<imu_unit> InterpolateIMU(const imu_unit& imu1, const imu_unit& imu2);
        imu_unit Interpolation(double targettime, const imu_unit& imu1, const imu_unit& imu2);
        double floor2w(double time);
        void Ipush(double ImgT, Triple gyo, Triple acc);
        bool Iget(double targetTime, Triple& gyo, Triple& acc);
        void Ipop();
        bool Iempty();
        void Iclear();
        double GetTime() {
            return _imuStartTime;
        }
        bool GetImuTime(double& time) {
            std::lock_guard<std::mutex> lock(Imumutex_);
            if (imucoder.empty()) return false;
            time = imucoder.front().first;
            return true;
        }
        bool isOpen() {
            return _opened;
        }

    private:
        int deviceID;
        HANDLE hSerial;
        std::string log_file;
        imu_unit PreImu;
        imu_unit CurrImu;
        double base_timestamp_ = -1;
        std::string port_;
        int frequency = 100;
        double ts = 0.01;
        bool TerminalOut;
        int baudrate_;
        ins_udp_client* udpClient_;
        std::atomic<bool> running_;
        std::string savepath_;
        std::queue<std::pair<double, std::pair<Triple, Triple>>> imucoder;
        double _imuStartTime = -1000;
        std::mutex Imumutex_;
        Triple prev_gyo;
        Triple prev_acc;
        bool _opened;
    };
}

#endif