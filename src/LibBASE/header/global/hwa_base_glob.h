#ifndef hwa_base_glob_h
#define hwa_base_glob_h
#include <condition_variable> 
#include <mutex>              
#include <atomic>        
#include <vector>
#include <chrono>      
#include <iostream>
#include <fstream>
#include <thread>

namespace hwa_detail {
    template<class T>
    constexpr bool is_allowed =
        std::is_same<T, int>::value ||
        std::is_same<T, float>::value ||
        std::is_same<T, double>::value;
}

extern double ProjectStartTime;
extern std::condition_variable cv_daheng, cv_hicon, cv_usb;
extern std::mutex mtx_daheng, mtx_hicon, mtx_usb;
extern bool ready_daheng, ready_hicon, ready_usb;
extern bool stop;

struct global_variable {
    int trigger = 0;  //1 for Point-and-click type; 0 for Continous type
    std::atomic<bool> request_capture{ false };
    std::atomic<bool> request_stop{ false };
};

#endif