#ifndef hwa_msf_base_processer_h
#define hwa_msf_base_processer_h

#include "hwa_ins_proc.h"
#include "hwa_base_eigendef.h"
#include "hwa_base_allpar.h"
#include "hwa_base_mutex.h" 
#include "hwa_base_time.h"
#include "hwa_base_data.h"
#include "hwa_base_posdata.h"
#include "hwa_base_filter.h"
#include "hwa_set_base.h"
#include "hwa_set_ign.h"

#define DELAY 0.001

using namespace hwa_base;
using namespace hwa_set;
using namespace hwa_ins;

namespace hwa_msf {
    struct MEAS_INFO {
        double tmeas, MeasYaw, MeasVf, MeasHgt;        /// meas time,meas yaw,meas velocity foward,meas height
        Triple MeasVel, MeasPos, MeasAtt;  /// meas velosity and position  & attitude addwh
        Triple _Cov_MeasVn, _Cov_MeasPos, _Cov_MeasAtt;  /// meas velosity and position convariance
        Triple _Cov_MeasNHC, _Cov_MeasZUPT;
        double _Cov_MeasZIHR, _Cov_MeasOdo, _Cov_MeasYaw;
    };

    enum SENSOR_TYPE {
        UWB,
        GNSS,
        VISION,
        HGT,
        WIFI,
        LIDAR
    };

    class baseprocesser {
    public:
        baseprocesser() {};
        explicit baseprocesser(const baseprocesser& B): 
        _gset(B._gset), _sins(B._sins), param_of_sins(B.param_of_sins),
        _spdlog(B._spdlog), _name(B._name), TimeStamp(B.TimeStamp), beg(B.beg),
        end(B.end), _shm(B._shm), _Estimator(B._Estimator), _Updater(B._Updater) 
        {
        };
        explicit baseprocesser(std::shared_ptr<set_base> gset, base_log spdlog, std::string name, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME) :
            _gset(gset), _spdlog(spdlog), _name(name), beg(_beg), end(_end), TimeStamp(_beg),
            _sins(std::make_shared<hwa_ins::ins_obj>(gset.get())),
            _shm(std::make_shared<hwa_ins::ins_scheme>(gset.get())),
            param_of_sins(std::make_shared<base_allpar>())
        {
            _Estimator = dynamic_cast<set_ign*>(_gset.get())->fuse_type();
        };
        ~baseprocesser() {};
        base_time& Time() { return TimeStamp;}
        base_time& _beg() { return beg; }
        base_time& _end() { return end; }
        virtual int ProcessOneEpoch() { return 1; };
        virtual void AddData(base_data* data) {};
        virtual void timesynchronization(base_time t) {};
        virtual bool _time_valid(base_time time) { return true; };
        virtual MEAS_TYPE _getPOS(base_time inst, base_posdata::data_pos& pos, MEAS_INFO& m) { return NO_MEAS; };
        virtual bool _init() { return true; };
        virtual bool load_data() { return true; };
        virtual void _feed_back();
        template <class T1, class T2>
        void m_out(T1 const& name, T2 const& matrix)
        {
            std::cout << name << std::endl;
            std::cout << std::fixed << std::setprecision(6) << std::setw(15) << matrix << std::endl;
            return;
        }
        bool timecheck() {
            return TimeStamp <= end && TimeStamp >= beg;
        };

    protected:
        std::shared_ptr<set_base> _gset;
        std::shared_ptr<ins_obj> _sins;
        std::shared_ptr<ins_scheme> _shm;
        std::shared_ptr<base_allpar> param_of_sins;
        std::string _name;
        base_time TimeStamp = FIRST_TIME;
        base_time beg = FIRST_TIME;
        base_time end = LAST_TIME;
        base_log _spdlog;
        Estimator _Estimator;
        base_updater _Updater;

    };
}


#endif