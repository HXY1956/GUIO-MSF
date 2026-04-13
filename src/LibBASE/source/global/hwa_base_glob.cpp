#include "hwa_base_glob.h"

double ProjectStartTime;
std::condition_variable cv_daheng, cv_hicon, cv_usb;
std::mutex mtx_daheng, mtx_hicon, mtx_usb;
bool ready_daheng = false, ready_hicon = false, ready_usb = false;
bool stop = false;
