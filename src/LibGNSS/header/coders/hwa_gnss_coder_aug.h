#ifndef hwa_gnss_coder_aug_h
#define hwa_gnss_coder_aug_h

#include "hwa_gnss_coder_base.h"
#include "hwa_gnss_data_aug.h"
#include "hwa_gnss_coder_rtcm3products.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for aug data storaging, derive from t_gcoder
    */
    class gnss_coder_aug : public gnss_base_coder
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
        gnss_coder_aug(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_aug();

        /** 
        * @brief decode the head of the aug data file.
        * 
        * @param[in]  buff        data flow
        * @param[in]  sz          size
        * @param[in]  errmsg      error message
        * @return      int          decode mode
        * 
        */
        int decode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /** 
        * @brief decode the data body of the aug data file. 
        * @param[in]  buff        data flow
        * @param[in]  sz          size
        * @param[in]  cnt          todo
        * @param[in]  errmsg      error message
        * @return      int          decode mode
        * 
        */
        int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

        /**
        * @brief get message of VS2015.
        * @param[in]  buff        data flow
        * @return      int          get message mode
        *
        */
        int GetMessage_VS2015(unsigned char *buffer);

        /**
        * @brief get aug.
        * @param[in]  augs        aug string
        * @param[in]  buffer      data flow
        * @param[in]  size          size
        * @return      AUG_RETURN  getaug mode
        *
        */
        AUG_RETURN GetAUG(std::string &augs, const char *buffer, size_t size);

        /**
        * @brief decode GREAT Aug correction.
        * @param[in]  buff        data flow
        * @param[in]  size          size
        * @return      bool          decode mode 
        *
        */
        bool DecodeGREATAUGCorrection(unsigned char *buffer, int size);

    protected:
        bool _realtime = false;                ///< real time
        std::map<GSYS, vector<hwa_pair_augtype>> _augtype; ///< aug type
        std::string _site;                          ///< site name
        base_time _epoch;                        ///< current epoch
        size_t _BlockSize = 0;                 ///< block size
        size_t _SkipBytes = 0;                 ///< skip bytes
        std::map<std::string, std::string> caster;            ///< todo
        std::set<std::string> sysall;                    ///< all systems
        unsigned char *_ubuffer = nullptr;     ///< u buffer
        int _last_size = 0;                    ///< last size
        int _tail_size = 0;                    ///< tail size
        int _beg_pos = 0;                      ///< begin position
        bool _this_crc = false;                ///< this crc
        bool _first_crc = false;               ///< first crc
    };
}

#endif
