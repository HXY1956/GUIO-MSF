#ifndef hwa_gnss_data_rec_h
#define hwa_gnss_data_rec_h

#include <stdio.h>
#include <string>

#include "hwa_gnss_data_obj.h"
#include "hwa_base_data.h"
#include "hwa_gnss_data_rnxhdr.h"

namespace hwa_gnss
{
    /** @brief class for grec. */
    class gnss_data_rec : public gnss_data_obj
    {

    public:
        /** @brief default constructor. */
        gnss_data_rec();
        //   gnss_data_rec(const gnss_data_rec& obj);
        gnss_data_rec(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_rec();

        /** @brief map rec. */
        typedef std::map<base_time, std::string> hwa_map_rec;

        /** @brief map header. */
        typedef std::map<base_time, gnss_data_rnxhdr> hwa_map_hdr;

        /** @brief add rinex header. */
        virtual void addhdr(const gnss_data_rnxhdr &hdr, const base_time &epo, std::string path);

        /** @brief change rinex header. */
        void changehdr(const gnss_data_rnxhdr &hdr, const base_time &epo, std::string path);

        /** @brief get all rinex headr. */
        hwa_map_hdr gethdr();

        /** @brief get one rinex headr. */
        gnss_data_rnxhdr gethdr(const base_time &epo);

        /** @brief get maprec. */
        hwa_map_rec get_map_rec() { return _maprec; }

        //   virtual void sync_orb(gnss_all_nav* orb) override;

        /** @brief set receiver name. */
        void rec(std::string rec, const base_time &beg, const base_time &end = LAST_TIME);

        /** @brief get receiver name. */
        std::string rec(const base_time &t) const; // set/get receiver

        /** @brief return validity for receiver at epoch t. */
        void rec_validity(const base_time &t, base_time &beg, base_time &end) const;

        /** @brief return isrec. */
        virtual bool isrec() override { return true; }

        /** @brief return istrn. */
        virtual bool istrn() override { return false; }

        /** @brief check consistency. */
        virtual void compare(std::shared_ptr<gnss_data_rec> grec, const base_time &tt, std::string source);

        /** @brief get time tags. */
        virtual std::vector<base_time> rec_id() const;

        /** @brief get time tags. */
        void fill_rnxhdr(const gnss_data_rnxhdr &rnxhdr);

    protected:
        /** @brief fill data members form rinex header. */
        void _fill_rnxhdr(const gnss_data_rnxhdr &rnxhdr);

        /** @brief get one rinex headr. */
        gnss_data_rnxhdr _gethdr(const base_time &epo);

        /** @brief set receiver name. */
        void _rec(std::string rec, const base_time &beg, const base_time &end = LAST_TIME);

        /** @brief get receiver name (>=t). */
        std::string _rec(const base_time &t) const;

        hwa_map_rec _maprec; ///< map of receviers
        hwa_map_hdr _maphdr; ///< map of rinex header information

    private:
    };

} // namespace

#endif
