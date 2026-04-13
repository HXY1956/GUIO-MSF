/**
*
* @file     bncobs.h
* @brief    The base class used to decode synchronized real-time observation in BNC structure.
*/
#ifndef hwa_gnss_coder_bncobs_H
#define hwa_gnss_coder_bncobs_H

#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_obsmanager.h"
#include "hwa_gnss_all_obs.h"

using namespace hwa_set;

namespace hwa_gnss
{

    //const string begEpoch = "BEGEPOCH";
    //const string endEpoch = "ENDEPOCH";

    /**
    *@brief       Class for decoding the synchronized real-time observation in BNC structure
    */
    class gnss_coder_bncobs : public gnss_base_coder
    {

    public:
        /**
        * @brief default constructor.
        *
        * @param[in]  s            setbase control
        * @param[in]  version    version of the gcoder
        * @param[in]  sz        size of the buffer
        */
        explicit gnss_coder_bncobs(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        ~gnss_coder_bncobs(){};

        /**
        * @brief decode the header of the synchronized real-time observation in BNC structure.
        *
        * The function is used for decoding the synchronized real-time observation in BNC structure.
        * pay attention to the buff and the size of buff which may cause some trouble when
        using the wrong value in decoding.
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  bufLen      buffer size of the data
        * @param[out] errmsg      error message of the data decoding
        * @return
        @retval >=0 consume size of header decoding
        @retval <0  finish reading
        */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
        * @brief decode the data body of the synchronized real-time observation in BNC structure.
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  bufLen      buffer size of the data
        * @param[out] cnt          number of line
        * @param[out] errmsg      error message of the data decoding
        * @return
        @retval >=0 consume size of body decoding
        @retval <0  finish reading
        */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

        /**
        * @brief encode the header of the synchronized real-time observation in BNC structure.
        *
        * The function is used for encoding the synchronized real-time observation in BNC structure.
        * pay attention to the buff and the size of buff which may cause some trouble when
        using the wrong value in encoding.
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[out] errmsg      error message of the data encoding
        * @return
        @retval >=0 consume size of header encoding
        @retval <0  finish reading
        */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
        * @brief encode the data body of the synchronized real-time observation in BNC structure.
        *
        * The function is used for encoding the data body of synchronized real-time observation in BNC structure.
        * pay attention to the buff and the size of buff which may cause some trouble when
        using the wrong value in encoding.
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[out] cnt         number of line
        * @param[out] errmsg      error message of the data encoding
        * @return
        @retval >=0 consume size of body encoding
        @retval <0  finish reading
        */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

        /**
        * @brief ???
        *
        * @param[in]  now        the current epoch
        * @return
        @retval ???
        @retval ???
        */
        virtual bool available(const base_time &now);

        /**
        * @brief ???
        *
        * @return
        @retval void
        */
        virtual void available_false();

        /**
        * @brief ???
        *
        * @return void
        */
        virtual void available_true();

    protected:
    private:
        /**
        * @brief checking if a receiver object exists
        *
        * @param[in]  site        site
        * @return void
        */
        void _check_recobj(std::string &site);

        /**
        * @brief ???
        *
        * @param[in]  t        ???
        * @return
        @retval ???
        @retval ???
        */
        bool _validepo(const base_time &t);

        bool _begepoch; ///< ???
        // bool    _endepoch;
        base_time _tt; ///< ???

        bool _available; ///< ???
    };

} // namespace

#endif
