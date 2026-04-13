/**
* @file        gambinp.h
* @brief    Storage the ambinp files' data include par_amb and neq
*/

#ifndef hwa_gnss_data_ambinp_H
#define hwa_gnss_data_ambinp_H

#include "hwa_base_data.h"
#include "hwa_base_time.h"
#include "hwa_base_par.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief     Class for storaging ambinp file data
    */
    class gnss_data_ambinp_head
    {
    public:
        /** @brief default constructor. */
        gnss_data_ambinp_head();
        /** @brief default constructor. */
        ~gnss_data_ambinp_head();

    public:
        base_time beg;      ///< begin time
        base_time end;      ///< end time
        int num_amb;      ///< number of ambiguity
        double interval;  ///< time interval
        std::string fix_mode;  ///< mode of ambiguity fix
        std::set<std::string> sat;  ///< satellite list
        std::set<std::string> site; ///< site list
    };

    /**
    *@brief     Class for storaging ambinp file data
    */
    class gnss_data_ambinp : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_ambinp();

        /** @brief default constructor. */
        gnss_data_ambinp(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_ambinp();

        /**
        * @brief add head info
        * @param[in]  hd    head info
        */
        void add_hdinfo(gnss_data_ambinp_head *hd);
        /**
        * @brief add ambiguity parameters
        * @param[in]  ambpar    ambiguity parameters
        */
        void add_ambpar(base_par ambpar);
        /**
        * @brief add normal equation
        * @param[in]  parname    name of parameters
        * @param[in]  q            q matrix
        */
        void add_neq(std::string parname, std::vector<double> &q);
        /**
        * @brief get head info
        * @param[in]  hd        head info
        */
        void get_hdinfo(gnss_data_ambinp_head *hd);
        /**
        * @brief get all ambiguity parameters
        * @param[in]  ambpar    ambiguity parameters
        */
        void get_allambpar(std::vector<base_par> &ambpar);
        /**
        * @brief get normal equation
        * @param[in]  parlist    list of parameters
        * @param[in]  neq        normal equation
        */
        void get_neq(std::vector<std::string> &parlist, std::vector<std::vector<double>> &neq);

    protected:
        std::vector<base_par> _amb;                  ///< ambiguity
        std::vector<std::vector<double>> _neq;          ///< normal equation
        std::vector<std::string> _par_list;             ///< ipt for neq(parname)
        gnss_data_ambinp_head *_ambinp_hd = nullptr; ///< head info of ambiguity

    private:
    };

}
#endif // !GAMBINP_H