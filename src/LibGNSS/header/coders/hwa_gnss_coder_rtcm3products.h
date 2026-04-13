#ifndef hwa_gnss_coder_RTCM3PRODUCTS_H
#define hwa_gnss_coder_RTCM3PRODUCTS_H

#include "hwa_gnss_sys.h"
#include "hwa_base_time.h"
#include "hwa_gnss_data_upd.h"
using namespace std;
using namespace hwa_base;
using namespace hwa_gnss;

#define GREATAUGRTCM_ID 4025
#define GREATUPDRTCM_ID 4012


namespace hwa_gnss {
    uint32_t CRC24(long size, const unsigned char* buf);
    /* UPD return */
    enum UPD_RETURN
    {
        /* all well */
        UPDR_OK = 0,
        /* unknown data, a warning */
        UPDR_UNKNOWNTYPE = -1,
        UPDR_UNKNOWNDATA = -2,
        UPDR_CRCMISMATCH = -3,
        UPDR_SHORTMESSAGE = -4,
        /* not enough data - can't decode the block completely */
        UPDR_SHORTBUFFER = -30,
        UPDR_MESSAGEEXCEEDSBUFFER = -31
    };

    enum AUG_RETURN
    {
        /* all well */
        AUGR_OK = 0,
        /* unknown data, a warning */
        AUGR_UNKNOWNTYPE = -1,
        AUGR_UNKNOWNDATA = -2,
        AUGR_CRCMISMATCH = -3,
        AUGR_SHORTMESSAGE = -4,
        /* not enough data - can't decode the block completely */
        AUGR_SHORTBUFFER = -30,
        AUGR_MESSAGEEXCEEDSBUFFER = -31
    };

    /* buffer should point to a RTCM3 block */
    /**
     * @brief
     *
     * @param augstring
     * @param buffer
     * @param size
     * @return size_t
     */
    size_t MakeAUG_RTCM(const string& augstring, char* buffer, size_t size);

    /**
     * @brief
     *
     * @param augs
     * @param buffer
     * @param size
     * @return enum AUG_RETURN
     */
    enum AUG_RETURN GetAUG(std::string& augs, const char* buffer, size_t size);

    /**
     * @brief
     *
     * @param t
     * @param updWL
     * @param updNL
     * @param buffer
     * @param size
     * @return size_t
     */
    size_t MakeUPD_RTCM(const base_time t, const one_epoch_upd& updWL, const one_epoch_upd& updNL, char* buffer, size_t size);

}

#endif
