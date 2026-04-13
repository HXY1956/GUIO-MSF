#include "hwa_ins_coder_serialreader.h"

using namespace hwa_ins;

namespace {
    constexpr BYTE FRAME_HEAD = 0xfc;
    constexpr BYTE TYPE_IMU = 0x40;
    constexpr int IMU_LEN = 0x38;
}

ins_reader::ins_reader(int ID, const std::string& port, int baudrate, double _ts, ins_udp_client* udpClient, const std::string& savepath, bool Out)
    : deviceID(ID), port_(port), baudrate_(baudrate), ts(_ts), udpClient_(udpClient), running_(false), savepath_(savepath), TerminalOut(Out) {
}

ins_reader::~ins_reader() {
    stop();
}

bool ins_reader::start() {
    std::string full_port = R"(\\.\)" + port_;
    hSerial = CreateFileA(full_port.c_str(), GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "[ERROR] Failed to open " << port_ << std::endl;
        _opened = false;
        return false;
    }
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = baudrate_;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadTotalTimeoutConstant = 50;
    SetCommTimeouts(hSerial, &timeouts);
    log_file = savepath_ + "\\imu" + std::to_string(deviceID) + ".txt";

    _opened = true;
    running_ = true;
    return true;
}

void ins_reader::stop() {
    running_ = false;
}

void ins_reader::readThread() {
    _imuStartTime = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    _imuStartTime -= ProjectStartTime;
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    BYTE header, type, len, sn, crc8, crc16h, crc16l;
    BYTE buf[128];
    std::ofstream fout(log_file);
    if (!fout.is_open()) {
        std::cerr << "[ERROR] Failed to open log file: " << log_file << std::endl;
    }
    fout << "timestamp(s), gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z\n";
    while (running_) {
        DWORD n;
        if (!ReadFile(hSerial, &header, 1, &n, NULL) || n != 1 || header != FRAME_HEAD) continue;
        if (!ReadFile(hSerial, &type, 1, &n, NULL) || n != 1) continue;
        if (!ReadFile(hSerial, &len, 1, &n, NULL) || n != 1) continue;
        if (!ReadFile(hSerial, &sn, 1, &n, NULL)) continue;
        if (!ReadFile(hSerial, &crc8, 1, &n, NULL)) continue;
        if (!ReadFile(hSerial, &crc16h, 1, &n, NULL)) continue;
        if (!ReadFile(hSerial, &crc16l, 1, &n, NULL)) continue;

        int data_len = len;
        if (!ReadFile(hSerial, buf, data_len, &n, NULL) || n != data_len) continue;

        if (type == TYPE_IMU && data_len == IMU_LEN) {
            imu_raw_package raw;
            memcpy(&raw, buf, sizeof(imu_raw_package));

            imu_package d;
            d.timestamp = raw.timestamp_us / 1e6f;
            memcpy(d.gyro, raw.gyro, sizeof(float) * 3);
            memcpy(d.accel, raw.accel, sizeof(float) * 3);

            if (base_timestamp_ == -1)
                base_timestamp_ = d.timestamp - _imuStartTime;
            double rel_time = d.timestamp - base_timestamp_;
            if (PreImu.Status && (rel_time < PreImu.time || rel_time > PreImu.time + 20 * ts)) {
                rel_time = PreImu.time + ts;
                double _base_timestamp_ = d.timestamp - rel_time;
                std::cout << "Time Base Changed for IMU" << deviceID<<" at: "<<rel_time << "; Base Time Changed from "<< base_timestamp_<<" to "<< _base_timestamp_ << std::endl;
                std::cout << "Status For Debug: 1. Initial Stamp:" << d.timestamp << " 2. PreIMU.time " << PreImu.time << std::endl;
                base_timestamp_ = _base_timestamp_;
            }
            //udpClient_->send(reinterpret_cast<const char*>(&d), imu_package::PacketSize);
            Triple gyo{d.gyro[1] ,d.gyro[0],-d.gyro[2]};
            Triple acc{d.accel[1] ,d.accel[0],-d.accel[2]};
            CurrImu = imu_unit(rel_time, gyo, acc);
            if (!PreImu.Status) {
                PreImu = CurrImu;
                continue;
            }
            std::vector<imu_unit> imu2w = InterpolateIMU(PreImu, CurrImu);
            for (int i = 0; i < imu2w.size();i++) {
                imu_unit iter = imu2w[i];
                fout << std::fixed << std::setprecision(3) << iter.time << ","
                    << std::setprecision(6)
                    << iter.gyo[0] << "," << iter.gyo[1] << "," << iter.gyo[2] << ","
                    << iter.acc[0] << "," << iter.acc[1] << "," << iter.acc[2] << "\n";
                if (TerminalOut && iter.time == int(iter.time)) std::cerr << "IMU" << deviceID << " Read Successful at: " << std::setiosflags(std::ios::fixed)<< std::setprecision(2) << iter.time << std::endl;
                iter.gyo *= ts;
                iter.acc *= ts;
                Ipush(iter.time, iter.gyo, iter.acc);
            }
            fout.flush();
            PreImu = CurrImu;
        }
    }

    fout.close();
    CloseHandle(hSerial);
}

double ins_reader::floor2w(double time) {
    double proc = 0.01 / ts;
    return floor(time * 100 * proc) / (proc * 100.0);
}

imu_unit ins_reader::Interpolation(double targettime, const imu_unit& imu1, const imu_unit& imu2) {
    imu_unit res = imu_unit();
    double timeproc = imu2.time - imu1.time;
    res.gyo = imu1.gyo * (imu2.time - targettime) + imu2.gyo * (targettime - imu1.time);
    res.gyo /= timeproc;
    res.acc = imu1.acc * (imu2.time - targettime) + imu2.acc * (targettime - imu1.time);
    res.acc /= timeproc;
    res.time = targettime;
    return res;
};

std::vector<imu_unit> ins_reader::InterpolateIMU(const imu_unit& imu1, const imu_unit& imu2) {
    std::vector<imu_unit> ans;
    double t1 = imu1.time;
    double t2 = imu2.time;
    for (double i = floor2w(t1) + ts; i <= (floor2w(t2) + 1e-4); i += ts) {
        ans.push_back(Interpolation(i, imu1, imu2));
    }
    return ans;
}

bool Iequal(double& t1, double t2) {
    return abs(t1 - t2) < 1e-4;
}

bool ins_reader::Iget(double targetTime, Triple& gyo, Triple& acc) {
    std::lock_guard<std::mutex> lock(Imumutex_);
    while (true) {
        if (imucoder.empty()) return false;
        double ImuT = imucoder.front().first;
        if (Iequal(ImuT, targetTime)) {
            gyo = imucoder.front().second.first;
            acc = imucoder.front().second.second;
            break;
        }
        if (ImuT < targetTime) imucoder.pop();
        else return false;
    }
    return true;
}

void ins_reader::Ipush(double ImuT, Triple gyo, Triple acc) {
    std::lock_guard<std::mutex> lock(Imumutex_);
    imucoder.push(std::make_pair(ImuT, std::make_pair(gyo, acc)));
    //std::cerr << "Imu" << deviceID << " After Push: " << imucoder.size() << std::endl;
}

void ins_reader::Ipop() {
    std::lock_guard<std::mutex> lock(Imumutex_);
    if (imucoder.empty()) return;
    imucoder.pop();
    //std::cerr << "Imu" << deviceID << " After Pop: " << imucoder.size() << std::endl;
}

bool ins_reader::Iempty() {
    std::lock_guard<std::mutex> lock(Imumutex_);
    return imucoder.empty();
}

void ins_reader::Iclear() {
    std::lock_guard<std::mutex> lock(Imumutex_);
    while (!imucoder.empty()) imucoder.pop();
}