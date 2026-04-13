#ifndef hwa_set_proc_h
#define hwa_set_proc_h
#include "hwa_set_base.h"
#include <sstream>
#include "Eigen/Eigen"
#define XMLKEY_PROCESS "rt-process"

namespace hwa_set
{
    class set_proc : public virtual set_base
    {
    public:
        set_proc() {};
        set_proc(set_base* gset);
        virtual ~set_proc() {};
        void help() {};
        void checkdefault();
        void check() {
            checkdefault();
        };
        double insinterval();
        double visinterval();
        bool TimeCostDebug();

    protected:
        bool TimeCostDebugStatus;
    };
}
#endif