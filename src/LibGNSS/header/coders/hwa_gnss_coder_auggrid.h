#ifndef hwa_gnss_augGRID_H
#define hwa_gnss_augGRID_H

#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_coder_Rtcm3products.h"
#include "hwa_gnss_data_augGRID.h"
#include "hwa_gnss_data_aug.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for aug data storaging, derive from t_gcoder
    */
    class gnss_coder_auggrid : public gnss_base_coder
    {
    public:
        /**
        * @brief constructor.
        *
        * @param[in]  s              gsetbase
        * @param[in]  version      version
        * @param[in]  sz          size
        *
        */
        gnss_coder_auggrid(set_base* s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_auggrid();

        /**
        * @brief decode the head of the aug data file.
        *
        * @param[in]  buff        data flow
        * @param[in]  sz          size
        * @param[in]  errmsg      error message
        * @return      int          decode mode
        *
        */
        int decode_head(char* buff, int sz, std::vector<std::string>& errmsg) override;

        /**
        * @brief decode the data body of the aug data file.
        * @param[in]  buff        data flow
        * @param[in]  sz          size
        * @param[in]  cnt          todo
        * @param[in]  errmsg      error message
        * @return      int          decode mode
        *
        */
        int decode_data(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg) override;

    protected:
        bool _realtime = false;                ///< real time
        unsigned char* _ubuffer = nullptr;     ///< u buffer
        std::string _Marker;
        std::string _ID;
        double _reflat;
        double _reflon;
        double _spacelat;
        double _spacelon;
        int _countlat;
        int _countlon;
        std::map<GSYS, GOBSBAND> _sys_band;
        base_time _epoch;
        int _nsat;
        double _res[MAXGRIDNUM];
    };
}

#endif
