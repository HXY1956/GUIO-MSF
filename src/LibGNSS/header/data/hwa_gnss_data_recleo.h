#ifndef hwa_gnss_data_recLEO_H
#define hwa_gnss_data_recLEO_H

#include "hwa_gnss_data_rec.h"

namespace hwa_gnss
{

    /** @brief class for grace leo satellite. */
    class gnss_data_rec_leo : public gnss_data_rec
    {

    public:
        /** @brief default constructor. */
        gnss_data_rec_leo();
        //   gnss_data_rec(const gnss_data_rec& obj);
        gnss_data_rec_leo(hwa_base::base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_rec_leo();

    protected:
    private:
    };

} // namespace

#endif
