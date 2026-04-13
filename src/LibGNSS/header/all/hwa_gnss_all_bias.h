/**
  * @file     gallbias.h
  * @brief    container for all biases
  */

#ifndef hwa_gnss_all_bias_H
#define hwa_gnss_all_bias_H

#include "hwa_base_data.h"
#include "hwa_gnss_data_Bias.h"
#include "hwa_base_time.h"

namespace hwa_gnss
{
    /**
     * @brief 
     * 
     */
    typedef std::shared_ptr<gnss_data_bias> hwa_spt_bias;

    /**
     *@brief Class for bias setting derive from base_data
     */
    class gnss_all_bias : public base_data
    {
        /// @relates gnss_all_bias
        ///< first : GNSS Observations, second : bias
        typedef std::map<GOBS, hwa_spt_bias> hwa_map_gobs;

        /// @relates gnss_all_bias
        ///< first : sat name, second : observations
        typedef std::map<std::string, hwa_map_gobs> hwa_map_sat;

        /// @relates gnss_all_bias
        ///< first : time, second : sat data
        typedef std::map<base_time, hwa_map_sat> hwa_map_tiv;

        /// @relates gnss_all_bias
        ///< first : ac name, second : epoch
        typedef std::map<std::string, hwa_map_tiv> hwa_map_ac;

    public:
        /** @brief default constructor. */
        explicit gnss_all_bias();

        /** @brief default constructor. */
        explicit gnss_all_bias(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_all_bias();

        /**
        * @brief std::set single bias element value.
        *
        * @param[in]  ac        ac name of the data
        * @param[in]  epo       epoch of the data
        * @param[in]  obj        object of the data
        * @param[in]  pt_bias    pt bias of the data
        * @return void
        */
        void add(const std::string &ac, const base_time &epo, const std::string &obj, hwa_spt_bias pt_bias);

        /**
        * @brief get DCB.
        *
        * @param[in]  epo       epoch of the data
        * @param[in]  obj        object of the data
        * @param[in]  gobs1        first observation of the data
        * @param[in]  gobs2        second observation bias of the data
        * @param[in]  ac            ac of the data
        * @return DCB
        */
        double get(const base_time &epo, const std::string &obj, const GOBS &gobs1, const GOBS &gobs2, const std::string &ac = "");

        /**
        * @brief get single bias element.
        *
        * @param[in]  prd       
        * @param[in]  epo       epoch of the data
        * @param[in]  obj        object of the data
        * @param[in]  gobs1        observation of the data
        * @param[in]  meter        unit
        * @return single bias
        */
        double get(const std::string &prd, const base_time &epo, const std::string &obj, const GOBS &gobs1, const bool &meter = true);

        /**
        * @brief get ac list.
        * @return ac list
        */
        std::vector<std::string> get_ac();

        /**
        * @brief std::set ac priority.
        * @return priority of ac
        */
        std::string get_ac_priority();

        /**
        * @brief std::set used ac.
        * @param[in]  ac        name of ac
        * @return    void
        */
        void set_used_ac(const std::string &ac);

        /**
        * @brief get used av.
        * @return used ac
        */
        std::string get_used_ac();

        /**
        * @brief if observation specific bias.
        * @return 
            @used_ac.find("_A") != used_ac.npos    true 
            @else                                false
        */
        bool is_osb();

        /**
        * @brief std::set/get overwrite mode.
        * @param[in]  b    
        * @return    void
        */
        void overwrite(const bool &b) { _isOverWrite = b; }

        /**
        * @brief if overwrite.
        * @return _overwrite
        */
        const bool &overwrite() const { return _isOverWrite; }

        /**
        * @brief clean rtcm & gnav.
        * @param[in]  beg    begin time
        * @param[in]  end    end time
        * @return    void
        */
        void clean_outer(const base_time &beg, const base_time &end);

        /**
        * @brief add bia intv.
        * @param[in]  intv
        * @return    void
        */
        void add_bia_intv(const int &intv);

        /**
        * @brief if bias available.
        * @param[in]  now    time of now
        * @return
        *    @EPO>now        true
        *    @else        false
        */
        bool bias_avail(const base_time &now);

    protected:
        /**
        * @brief get single bias element pointer.
        *
        * @param[in]  ac            ac of the data
        * @param[in]  epo       epoch of the data
        * @param[in]  obj        object of the data
        * @param[in]  gobs        observation of the data
        * @return    pt bias
        */
        hwa_spt_bias _find(const std::string &ac, const base_time &epo, const std::string &obj, const GOBS &gobs);

        /**
        * @brief get single bias element pointer.
        *
        * @param[in]  ac            ac of the data
        * @param[in]  epo       epoch of the data
        * @param[in]  obj        object of the data
        * @param[in]  ref        
        * @return    vec bias
        */
        std::vector<hwa_spt_bias> _find_ref(const std::string &ac, const base_time &epo, const std::string &obj, const GOBS &ref);

        /**
        * @brief convert type of observations.
        *
        * @param[in]  ac            ac of the data
        * @param[in]  obj        object of the data
        * @param[in]  obstype    observation type
        * @return    void
        */
        void _convert_obstype(const std::string &ac, const std::string &obj, GOBS &obstype);

        /**
        * @brief connect DCB pt_cb2 with first GOBS.
        *
        * @param[in]  pt_cb1    
        * @param[in]  pt_cb2
        * @return    void
        */
        void _connect_first(const hwa_spt_bias &pt_cb1, const hwa_spt_bias &pt_cb2);

        /**
        * @brief connect DCB pt_cb2 with second GOBS.
        *
        * @param[in]  pt_cb1
        * @param[in]  pt_cb2
        * @return    void
        */
        void _connect_second(const hwa_spt_bias &pt_cb1, const hwa_spt_bias &pt_cb2);

        /**
        * @brief consolidate all biases with reference signal of pt_cb2.
        *
        * @param[in]  ac            ac of the data
        * @param[in]  obj        object of the data
        * @param[in]  pt_cb1
        * @param[in]  pt_cb2
        * @return    void
        */
        void _consolidate(const std::string &ac, const std::string &obj, const hwa_spt_bias &pt_cb1, const hwa_spt_bias &pt_cb2);

        /** ================================================ ******  =========================================================== */

    protected:
        std::string _acUsed;            ///<
        std::string _acPri = "DLR_R";   ///< primary AC
        std::map<std::string, int> _acOrder; ///< std::map of all ACs

        int _udbiaInt = 99999; ///< flag of undifferentiated bia

        bool _isOverWrite = false; ///< flag of overwrite
        bool _isOrdered = false;   ///< if AC is ordered

        hwa_map_ac _mapBias; ///< std::map of all satellite biases (all ACs & all period & all objects)
    };

} // namespace

#endif
