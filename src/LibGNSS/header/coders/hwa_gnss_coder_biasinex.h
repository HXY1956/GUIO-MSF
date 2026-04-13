/**
*
* @file     biasinex.h
* @brief    The base class used to decode bias SINEX file information.
*
*/
#ifndef hwa_gnss_coder_biasinex_H
#define hwa_gnss_coder_biasinex_H

#include "hwa_set_base.h"
#include "hwa_base_time.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_coder_sinex.h"
#include "hwa_gnss_all_Bias.h"

#define SNX_WINDOW_EXTENTION 6 * 3600 // [s] window for SNX temporal approximation

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief       Class for decoding the ??? data
    */
    class gnss_coder_biasinex : public gnss_coder_sinex
    {

    public:
        /**
        * @brief default constructor.
        *
        * @param[in]  s            setbase control
        * @param[in]  version    version of the gcoder
        * @param[in]  sz        size of the buffer
        * @param[in]  id        string for reporting
        */
        explicit gnss_coder_biasinex(set_base *s, std::string version, int sz = DEFAULT_BUFFER_SIZE, std::string id = "biasinex");

        /** @brief default destructor. */
        virtual ~gnss_coder_biasinex(){};

    protected:
        /**
        * @brief decode ??? data file.
        *
        * The function is used for decoding ??? file.
        * pay attention to the buff and the size of buff which may cause some trouble when
        using the wrong value in decoding.
        *
        * @return
        @retval >=0 consume size of header decoding
        @retval <0  finish reading
        */
        virtual int _decode_comm();

        /**
        * @brief decode ??? data file.
        *
        * The function is used for decoding ??? file.
        * pay attention to the buff and the size of buff which may cause some trouble when
        using the wrong value in decoding.
        *
        * @return
        @retval >=0 consume size of header decoding
        @retval <0  finish reading
        */
        virtual int _decode_block();

        /**
        * @brief add ALLBIAS data to _allbias
        *
        * @param[in]  id        data type
        * @param[in]  pt_data    ALLBIAS data
        * @return void
        */
        virtual void _add_data(std::string id, base_data *pt_data);

        gnss_all_bias *_allbias; ///< ???
    };

} // namespace

#endif
