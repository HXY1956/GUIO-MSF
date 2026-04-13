#ifndef hwa_gnss_data_obj_H
#define hwa_gnss_data_obj_H

#include "hwa_base_time.h"
#include "hwa_base_data.h"
#include "hwa_gnss_data_rxnhdr.h"
#include "hwa_gnss_all_pcv.h"
#include "hwa_gnss_data_pcv.h"
#include "Eigen/Eigen"

namespace hwa_gnss
{
    /** @brief class for gnss_data_obj. */
    class gnss_data_obj : public hwa_base::base_data
    {

    public:
        /** @brief default constructor. */
        explicit gnss_data_obj();

        /** @brief default constructor. */
        explicit gnss_data_obj(hwa_base::base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_obj();

        typedef std::map<hwa_base::base_time, std::shared_ptr<gnss_data_pcv>> hwa_map_pcv;
        typedef std::map<hwa_base::base_time, Triple> hwa_map_eccxyz;
        typedef std::map<hwa_base::base_time, Triple> hwa_map_eccneu;
        typedef std::map<hwa_base::base_time, std::pair<Triple, Triple>> hwa_map_crd;
        typedef std::map<hwa_base::base_time, std::string> hwa_map_ant;

        /** @brief std::set/get id (uniq internal id). */
        virtual void id(std::string str);
        /**
         * @brief 
         * 
         * @return std::string 
         */
        virtual std::string id() const;

        /** @brief std::set/get name (name). */
        virtual void name(std::string str);
        /**
         * @brief 
         * 
         * @return std::string 
         */
        virtual std::string name() const;

        /** @brief std::set/get name (domes). */
        virtual void domes(std::string str);

        /**
         * @brief 
         * 
         * @return std::string 
         */
        virtual std::string domes() const;

        /** @brief std::set/get description (full name). */
        virtual void desc(std::string str);

        /**
         * @brief 
         * 
         * @return std::string 
         */
        virtual std::string desc() const;

        /**
         * @brief 
         * 
         * @param ecc 
         * @param beg 
         * @param end 
         */
        virtual void eccxyz(const Triple &ecc, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME);

        /** @brief std::set/get ecc offsets (>=t) w.r.t. center of mass/reference point. */
        virtual Triple eccxyz(const hwa_base::base_time &t) const;

        /**
         * @brief 
         * 
         * @param t 
         * @param beg 
         * @param end 
         */
        virtual void eccxyz_validity(const hwa_base::base_time &t, hwa_base::base_time &beg, hwa_base::base_time &end) const;

        /**
         * @brief 
         * 
         * @param ecc 
         * @param beg 
         * @param end 
         */
        virtual void eccneu(const Triple &ecc, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME);

        /** @brief std::set/get ecc offsets (>=t) w.r.t. center of mass/reference point. */
        virtual Triple eccneu(const hwa_base::base_time &t) const;

        /**
         * @brief 
         * 
         * @param t 
         * @param beg 
         * @param end 
         */
        virtual void eccneu_validity(const hwa_base::base_time &t, hwa_base::base_time &beg, hwa_base::base_time &end) const;

        /**
         * @brief 
         * 
         * @param pcv 
         * @param beg 
         * @param end 
         */
        virtual void pcv(std::shared_ptr<gnss_data_pcv> pcv, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME);

        /** @brief std::set/get external pointer to pcv element (>=t). */
        virtual std::shared_ptr<gnss_data_pcv> pcv(const hwa_base::base_time &t) const;

        /**
         * @brief 
         * 
         * @param ant 
         * @param beg 
         * @param end 
         */
        virtual void ant(std::string ant, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME);

        /** @brief std::set/get antenna. */
        virtual std::string ant(const hwa_base::base_time &t) const;

        /**
         * @brief 
         * 
         * @param t 
         * @param beg 
         * @param end 
         */
        virtual void ant_validity(const hwa_base::base_time &t, hwa_base::base_time &beg, hwa_base::base_time &end) const;

        /**
         * @brief 
         * 
         * @param chk 
         */
        virtual void channel(int chk){};

        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int channel() const { return 255; };

        /**
         * @brief 
         * 
         * @param crd 
         * @param std 
         * @param beg 
         * @param end 
         * @param overwrite 
         */
        virtual void crd(const Triple &crd, const Triple &std, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME, bool overwrite = false);

        /** @brief add by glfeng std::set crd & std. */
        virtual void crd(const Triple &crd, const Triple &std);
        virtual void clear_crd();

        /** @brief get crd of Marker point. */
        virtual Triple crd(const hwa_base::base_time &t) const;

        /** @brief get std of crd. */
        virtual Triple std(const hwa_base::base_time &t) const;

        /** @brief get crd of ARP (ARP = MARKER + ECC). */
        virtual Triple crd_arp(const hwa_base::base_time &t) const;

        /**
         * @brief 
         * 
         * @param t 
         * @param beg 
         * @param end 
         */
        virtual void crd_validity(const hwa_base::base_time &t, hwa_base::base_time &beg, hwa_base::base_time &end) const;

        /**
         * @brief Get the recent crd object
         * 
         * @param t 
         * @param ref_std 
         * @param crd 
         * @param std 
         * @return true 
         * @return false 
         */
        virtual bool get_recent_crd(const hwa_base::base_time &t, const double &ref_std, Triple &crd, Triple &std);

        /** @brief add by xiongyun (get adjacent snx crd). */
        virtual bool get_adjacent_crd(const hwa_base::base_time &t, const double &ref_std, Triple &crd, Triple &std);

        //   virtual std::vector<hwa_base::base_time> ecc_id() const;
        virtual std::vector<hwa_base::base_time> pcv_id() const;

        /**
         * @brief 
         * 
         * @return std::vector<hwa_base::base_time> 
         */
        virtual std::vector<hwa_base::base_time> ant_id() const;

        /**
         * @brief 
         * 
         * @return std::vector<hwa_base::base_time> 
         */
        virtual std::vector<hwa_base::base_time> crd_id() const;

        /**
         * @brief 
         * 
         * @param gobj 
         * @param tt 
         * @param source 
         */
        virtual void compare(std::shared_ptr<gnss_data_obj> gobj, const hwa_base::base_time &tt, std::string source);

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        virtual bool isrec() = 0;

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        virtual bool istrn() = 0;

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool overwrite();

        /**
         * @brief 
         * 
         * @param overwrite 
         */
        void overwrite(bool overwrite);

        /**
         * @brief 
         * 
         * @param pcvs 
         */
        virtual void sync_pcv(gnss_all_pcv *pcvs);

        /**
         * @brief 
         * 
         * @param t 
         * @return true 
         * @return false 
         */
        virtual bool operator<(const gnss_data_obj &t) const;

        /**
         * @brief 
         * 
         * @param t 
         * @return true 
         * @return false 
         */
        virtual bool operator==(const gnss_data_obj &t) const;

    protected:
        std::string _id;              ///< object id (internal)
        std::string _name;            ///< object name
        std::string _domes;           ///< object domes
        std::string _desc;            ///< object description (full name)
        hwa_map_crd _mapcrd;        ///< object position
        hwa_map_pcv _mappcv;        ///< std::map of pco+pcv
        hwa_map_eccxyz _mapeccxyz; ///< std::map of xyz eccentricities (to center of mass or reference point)
        hwa_map_eccneu _mapeccneu; ///< std::map of neu eccentricities (to center of mass or reference point)
        hwa_map_ant _mapant;        ///< std::map of antennas + dome

        bool _overwrite;

        // source for public (mutexed) interfaces
        std::shared_ptr<gnss_data_pcv> _pcv(const hwa_base::base_time &t) const;
        void _pcv(std::shared_ptr<gnss_data_pcv> pcv, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME);
        void _ant(std::string ant, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME);
        std::string _ant(const hwa_base::base_time &t) const;
        std::vector<hwa_base::base_time> _ant_id() const;
        void _crd(const Triple &crd, const Triple &std, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME, bool overwrite = false); ///< overwrite for solution must over write; by ZHJ
        Triple _crd(const hwa_base::base_time &t) const;
        Triple _std(const hwa_base::base_time &t) const;
        void _eccxyz(const Triple &ecc, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME);
        Triple _eccxyz(const hwa_base::base_time &t) const;
        void _eccneu(const Triple &ecc, const hwa_base::base_time &beg, const hwa_base::base_time &end = LAST_TIME);
        Triple _eccneu(const hwa_base::base_time &t) const;
        std::shared_ptr<gnss_data_pcv> _pcvnull;

    private:
    };

} // namespace

#endif
