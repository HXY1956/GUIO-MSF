#ifndef hwa_gnss_coder_sp3_H
#define hwa_gnss_coder_sp3_H

#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_base_time.h"
#include "hwa_gnss_all_prec.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief Class for gnss_coder_sp3derive from base_data
    */
    class gnss_coder_sp3: public gnss_base_coder
    {

    public:
        /** @brief constructor set + version + sz. */
        explicit gnss_coder_sp3(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_sp3(){};

        /** @brief decode head. */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
         * @brief 
         * 
         * @param buff 
         * @param sz 
         * @param cnt 
         * @param errmsg 
         * @return int 
         */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

        /** @brief encode head. */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
         * @brief 
         * 
         * @param buff 
         * @param sz 
         * @param cnt 
         * @param errmsg 
         * @return int 
         */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

    protected:
    private:
        base_time _start;           ///< start time
        base_time _lastepo;         ///< last time
        long _orbintv;            ///< orb interval [sec]
        int _nepochs;             ///< number of epoch
        int _nrecmax;             ///< max number of reciver
        int _nrecord;             ///< number of reciver
        std::string _orbrefs;          ///< orb reference
        std::string _orbtype;          ///< orb type
        std::string _agency;           ///< agency
        int _num_leo = 0;         ///< number of leo
        std::map<std::string, std::string> _leo; ///< leo
        std::vector<std::string> _prn;      ///< satellite prn
        std::vector<std::string> _sagnss_coder_sp3;  ///< satellite name
        std::vector<int> _acc;         ///< acc
        std::vector<std::string> _timesys;  ///< time of system
        std::vector<int> _accbase;     ///< acc base
        int _maxsats;             ///< max number of satellite
        std::string _sattype;          ///< satellite type
        std::string _data_type;        ///< data type
        hwa_map_ITIV _mapsp3;        ///< map sp3
        hwa_map_iv _sp3data;       ///< sp3 data
    };

} // namespace

#endif
