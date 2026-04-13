#ifndef hwa_base_sharedresource_h
#define hwa_base_sharedresource_h
#include <condition_variable> 
#include <mutex>

namespace hwa_base {
    class base_cond_var {
    public:
        std::mutex mtx;
        std::condition_variable cv_button;
        bool new_status = false;
    };
}

#endif
