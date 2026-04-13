#ifndef hwa_uwb_data_h
#define hwa_uwb_data_h
#include "hwa_base_data.h"
#include "hwa_base_time.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_uwb
{
    struct UWB_NODE
    {
        double range = -1.0;
        double SNR = -100.0;
        double noise = -1.0;;
        double successrate = 0.0;
        double fpRSSI = -200, rxRSSI = -200;
    };

    typedef std::map<std::string, UWB_NODE> hwa_map_id_uwbnode;
    typedef std::map<double, hwa_map_id_uwbnode> hwa_map_ti_uwbnode;
    typedef std::map<double, UWB_NODE> hwa_map_time_uwbnode;
    typedef std::map<std::string, hwa_map_time_uwbnode> hwa_map_it_uwbnode;

    static double prediction(std::vector<std::pair<double, double>> points, int degree, double x, double& mean_cost);

    class uwb_data : public hwa_base::base_data
    {
    public:
        /** @brief default constructor. */
        explicit uwb_data();

        /** @brief default destructor. */
        virtual ~uwb_data() {}

        int add_uwb(const double& t, const std::string& id, const UWB_NODE& node);

        /**
         * @brief
         *
         * @param nodes
         */
        void add_nodes(const std::vector<std::string>& nodes);

        bool interpolation(hwa_map_time_uwbnode& df, double t);
        /**
         * @brief
         *
         * @param crt
         * @param dt
         * @param val
         * @return true
         * @return false
         */
        bool load(const hwa_base::base_time& crt, double dt, hwa_map_id_uwbnode& val);

        /**
         * @brief
         *
         * @param crt
         * @param dt
         * @param val
         * @param intv
         * @return true
         * @return false
         */
        bool load(const hwa_base::base_time& crt, double dt, hwa_map_id_uwbnode& val, double& intv);

        /**
         * @brief
         *
         * @param t
         * @param intv
         * @param val
         * @return true
         * @return false
         */
        bool loadUWB(const double& t, const double& intv, hwa_map_id_uwbnode& val);

        //virtual bool available(const base_time& t);
        void interpolation_flag(bool flag, double n) {
            iflag = flag;
            noise = n;
        }

        void velocity(Triple v) {
            vel = v;
        }


    private:
        int _node_num;
        hwa_map_ti_uwbnode _alldata;
        hwa_map_it_uwbnode id_alldata;
        bool iflag = false;
        double noise;
        std::vector<std::string> _nodes;
        Triple vel;
    };

}

#endif