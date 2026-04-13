#ifndef hwa_gnss_data_opl_H
#define hwa_gnss_data_opl_H

#include "hwa_base_globaltrans.h"
#include "hwa_base_data.h"
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief model of OPL    */
    class gnss_data_opl : public base_data
    {
    public:
        /** @brief constructor    */
        gnss_data_opl() { id_type(base_data::OPL); };
        gnss_data_opl(base_log spdlog);

        /** @brief destructor.*/
        ~gnss_data_opl();

        /** @brief get parameter of ocean pole load    */
        void setdata(double a, double b, double c, double d, double e, double f, double g, double h);

    private:
        double lon;      ///< longitude
        double lat;      ///< latitude
        Triple rne_r; ///< TODO
        Triple rne_i; ///<TODO
    };
}
#endif // !GOPL_H
