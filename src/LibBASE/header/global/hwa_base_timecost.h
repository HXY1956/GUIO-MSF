#ifndef hwa_base_timecost_h
#define hwa_base_timecost_h
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

namespace hwa_base {
    class base_scopedtimer {
    public:
        base_scopedtimer(const std::string& name, std::ofstream& file, bool YorN) : Status(YorN), name_(name), OutFile(file), start_(std::chrono::high_resolution_clock::now()) {}
        ~base_scopedtimer() {
            if (Status) {
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count();
                OutFile << "[Timer] " << name_ << " took " << duration << " ms\n";
            }
        }
    private:
        bool Status;
        std::string name_;
        std::chrono::high_resolution_clock::time_point start_;
        std::ofstream& OutFile;
    };
}

#endif
