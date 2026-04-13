#ifndef hwa_gnss_data_interp_H
#define hwa_gnss_data_interp_H

#include "hwa_base_eigendef.h"
#include "hwa_base_time.h"
#include "hwa_base_pair.h"
#include "hwa_base_data.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_data_interp based on base_data. */
    class gnss_data_interp : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_interp();

        gnss_data_interp(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_interp();

        /** @brief 1D type. */
        enum INTERP_1D
        {
            LINEAR,
            SPLINE
        };

        /** @brief 2D type. */
        enum INTERP_2D
        {
            BILINEAR,
            IDW,
            TPS
        };

        /** @brief 3D type. */
        enum INTERP_3D
        {
            VER2HOR,
            HOR2VER
        };

        /** @brief HT type. */
        enum INTERP_HT
        {
            INTERPOLATE,
            SCALE
        };

        /** @brief convert str to interp 1d/2d/3d/ht. */
        INTERP_1D str_to_interp_1d(const std::string &str);
        INTERP_2D str_to_interp_2d(const std::string &str);
        INTERP_3D str_to_interp_3d(const std::string &str);
        INTERP_HT str_to_interp_ht(const std::string &str);

        /** @brief convert interp to str 1d/2d/3d/ht. */
        std::string interp_1d_to_str(const INTERP_1D &typ);
        std::string interp_2d_to_str(const INTERP_2D &typ);
        std::string interp_3d_to_str(const INTERP_3D &typ);
        std::string interp_ht_to_str(const INTERP_HT &typ);

        /** @brief Linear Interpolation (double). */
        int linear(std::map<double, double> &data, double val, double &fval);

        /** @brief Spline Interpolation (double). */
        int spline(std::map<double, double> &data, double val, double &fval);

        /** @brief Linear Interpolation (time). */
        int linear(const std::map<base_time, double> &data, const base_time &epo, double &fval);

        /** @brief Spline Interpolation (time). */
        int spline(const std::map<base_time, double> &data, const base_time &epo, double &fval);

        /** 
        *@brief bilinear interpolation
        * 
        * 0 .. 11 (bottom-left)         21 *---------* 22
        * 1 .. 12 (bottom-right)           |         |
        * 2 .. 21 (top-left)               |         |
        * 3 .. 22 (top-right)           11 *---------* 12
        */
        int bilinear(const std::map<base_pair, double> &data, const base_pair &req_pos, double &fval);

        /** @brief idw. */
        int idw(const std::map<base_pair, double> &data, const base_pair &req_pos, double &fval);

        /** @brief tps. */
        int tps(std::map<base_pair, double> &data, const base_pair &req_pos, double &alpha, double &fval);

    protected:
        INTERP_1D _interp_1d; ///< 1d Interpolation
        INTERP_2D _interp_2d; ///< 2d Interpolation
        INTERP_3D _interp_3d; ///< 3d Interpolation
        INTERP_HT _interp_ht; ///< ht Interpolation

        /** @brief idw. */
        double _processSumOfDistances(const std::map<base_pair, double> &data, const base_pair &req);
        double _iDistance(const base_pair &pos, const base_pair &req);
    };

}

#endif
