/**
 * The poleut1 file  provides the pole and ut1-TAI information.\n
 * Following is an example for the header of poleut1 file.
 *
 * @verbatim
     %% conveop             zhangwei            07-Mar-19                        ! the author of this file and the time of creation
     %% Bulletin A file : finals2000A.all
     %% Input EOP  file : poleut1
     %% 
     +pole&ut1
     % UT1 type = UT1R
     % Start&End%Interval =    51544   58616   1.00                              ! the start time and end time(mjd) of the file,the interval(unit: day) of the record
     % Num. of Vars&Units =     5  0.1D+01  0.1D+01  0.1D+01  0.1D+01  0.1D+01   ! the number of the parameters in a line and the unit of them
     % Format = (f9.2,1x,2f10.6,f15.7,2f10.3,6(1x,a1))                           ! the format of the record(one line)
     %% MJD        XPOLE     YPOLE      UT1-TAI        DPSI     DEPSI    PRED_ID ! the information of the record(one line)
   @endverbatim
 *
 * An example for the record of the attitude file
 * @verbatim
   =============================================================================================
   !MJD        XPOLE     YPOLE       UT1-TAI         DPSI      DEPSI PRED_ID
   =============================================================================================
    51544.00   0.043240  0.377910    -31.6444900     0.000     0.000 I I I - - -
    51545.00   0.043510  0.377750    -31.6453930     0.000     0.000 I I I - - -
    51546.00   0.043620  0.377450    -31.6461550     0.000     0.000 I I I - - -
    .
    .
    .
    58614.00   0.095760  0.418733    -37.1681963     0.000     0.000 P P P - - -
    58615.00   0.097222  0.419217    -37.1690213     0.000     0.000 P P P - - -
    58616.00   0.098694  0.419680    -37.1698288     0.000     0.000 P P P - - -
   -pole&ut1                   ! end of the file
   @endverbatim
 *
 * @file            poleut1.h
 * @brief            The base class used to decode poleut1 file information.
 */

#ifndef hwa_gnss_coder_poleut_H
#define hwa_gnss_coder_poleut_H

#include "hwa_base_time.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_poleut.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for decoding the poleut1 data
    */
    class gnss_coder_poleut : public gnss_base_coder
    {
    public:
        /**
         * @brief default constructor.
         *
         * @param[in]  s        setbase control
         * @param[in]  version  version of the gcoder
         * @param[in]  sz       size of the buffer
         */
        explicit gnss_coder_poleut(hwa_set::set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_poleut(){};

        /**
         * @brief decode the header of the poleut1 data file.
         *
         * The function is used for decoding the head of poleut1 file.\n
         * pay attention to the buff and the size of buff which may cause some trouble when
            using the wrong value in decoding.
         *
         * @param[in]  buff        buffer of the data
         * @param[in]  sz          buffer size of the data
         * @param[in]  errmsg      error message of the data decoding
         * @return
            @retval >=0 consume size of header decoding
            @retval <0  finish reading
         */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
         * @brief decode the data body of the poleut1 data file.
         *
         * decode data body of poleut1 file, all the data read will store in the gpoleut1
         *
         * @param[in]  buff        buffer of the data
         * @param[in]  sz          buffer size of the data
         * @param[in]  errmsg      error message of the data decoding
         * @return
            @retval >=0 consume size of body decoding
            @retval <0  finish reading
         */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

    protected:
        std::string _timetype;        ///< type of time(UT1R)
        double _begtime;         ///< beg time(modified julian day)   jdhuang change int to double
        double _endtime;         ///< end time(modified julian day)   jdhuang change int to double
        double _interval;        ///< time interval(unit: day)
        int _parnum;             ///< number of parameters
        std::vector<double> _parunit; ///< unit of each parameter
        std::vector<std::string> _parname; ///< name of each parameter
    };
} // namespace

#endif