#ifndef hwa_gnss_data_pcv_H
#define hwa_gnss_data_pcv_H

#include "hwa_base_data.h"
#include "hwa_base_const.h"
#include "hwa_base_time.h"
#include "hwa_base_log.h"
#include "hwa_gnss_sys.h"
#include "hwa_gnss_model_ephplan.h"
#include "hwa_gnss_data_SATDATA.h"

namespace hwa_gnss
{
    /** @brief class for gnss_data_pcv based base_data. */
    class gnss_data_pcv : public hwa_base::base_data
    {

    public:
        /** @brief default constructor. */
        gnss_data_pcv();

        gnss_data_pcv(hwa_base::base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_pcv();

        typedef std::map<double, double> hwa_map_Z;  ///< std::map of ZEN-dependence
        typedef std::map<double, hwa_map_Z> hwa_map_A; ///< std::map of AZI-dependence & ZEN-dependence

        typedef std::map<GFRQ, Triple> hwa_map_PCO; ///< North/East/Up phase centre offsets     (N-freqs)
        typedef std::map<GFRQ, hwa_map_Z> hwa_map_ZEN;   ///< std::map of AZI-dependence                  (N-freqs)
        typedef std::map<GFRQ, hwa_map_A> hwa_map_azi;   ///< std::map of AZI-dependence & ZEN-dependence (N-freqs)

        /** @brief model is for transmitter (satellite). */
        bool is_transmitter() const { return _trans; }

        /** @brief model is for receiver (station or LEO satellite). */
        bool is_receiver() const { return !_trans; }

        /** @brief std::set/get antenna type definition (ATX field 0-19). */
        void anten(const std::string &s) { _anten = s; }
        const std::string &anten() const { return _anten; }

        /** @brief std::set/get antenna identification (ATX field 20-39). */
        void ident(const std::string &s) { _ident = s; }
        const std::string &ident() const { return _ident; }

        /** @brief std::set/get satellite (svn) code (ATX field 40-59). */
        void svcod(const std::string &s)
        {
            _svcod = s;
            if (s.empty())
                _trans = false;
            else
                _trans = true;
        } // _trans=false/true (receiver/transmitter)
        const std::string &svcod() const { return _svcod; }

        /** @brief pcvall first std::map key (antenna/satellite prn). */
        std::string pcvkey() const
        {
            if (is_transmitter())
                return ident();
            else
                return anten();
        }

        /** @brief pcvall second std::map key (individual/type callibration). */
        std::string pcvtyp() const
        {
            if (is_transmitter())
                return "";
            else
                return ident();
        }

        /** @brief std::set/get method of callibration. */
        void method(const std::string &s) { _method = s; }
        const std::string &method() const { return _method; }

        /** @brief std::set/get source of callibration. */
        void source(const std::string &s) { _source = s; }
        const std::string &source() const { return _source; }

        /** @brief std::set/get sinex code. */
        void snxcod(const std::string &s) { _snxcod = s; }
        const std::string &snxcod() const { return _snxcod; }

        /** @brief std::set/get valid from. */
        void beg(const hwa_base::base_time &t) { _beg = t; }
        const hwa_base::base_time &beg() const { return _beg; }

        /** @brief std::set/get valid until. */
        void end(const hwa_base::base_time &t) { _end = t; }
        const hwa_base::base_time &end() const { return _end; }

        /** @brief std::set/get azimuth sampling. */
        void dazi(const double &d) { _dazi = d; }
        const double &dazi() const { return _dazi; }

        /** @brief std::set/get zenith sampling. */
        void dzen(const double &d) { _dzen = d; }
        const double &dzen() const { return _dzen; }

        /** @brief std::set/get zenith1. */
        void zen1(const double &d) { _zen1 = d; }
        const double &zen1() const { return _zen1; }

        /** @brief std::set/get zenith2. */
        void zen2(const double &d) { _zen2 = d; }
        const double &zen2() const { return _zen2; }

        /** @brief correct satellite coord. */
        double pco(const double &zen, const double &azi, const GFRQ &f); // pco correction - !!!! TEMP !!!!

        // jdhuang add for gnss muti-freqs
        // ===========================================================================
        int pcoS(gnss_data_sats &satdata, Triple &pco, OBSCOMBIN lc, GOBSBAND &b1, GOBSBAND &b2);
        int pcoR(gnss_data_sats &satdata, Triple &pco, OBSCOMBIN lc, GOBSBAND &b1, GOBSBAND &b2);

        /** @brief pcb correction - satellite (nadir). */
        int pcvS(double &corr, gnss_data_sats &satdata, OBSCOMBIN lc, GOBSBAND &b1, GOBSBAND &b2, Triple &site);

        /** @brief pcv correction - site (zenith). */
        int pcvR(double &corr, gnss_data_sats &satdata, OBSCOMBIN lc, GOBSBAND &b1, GOBSBAND &b2);
        int pcoS_cmb(gnss_data_sats &satdata, Triple &pco, GOBSBAND &b1, GOBSBAND &b2);
        int pcoR_cmb(gnss_data_sats &satdata, Triple &pco, GOBSBAND &b1, GOBSBAND &b2);

        /** @brief pcb correction - satellite (nadir). */
        int pcvS_cmb(double &corr, gnss_data_sats &sat, GOBSBAND &b1, GOBSBAND &b2, Triple &site);

        /** @brief pcv correction - site (zenith). */
        int pcvR_cmb(double &corr, gnss_data_sats &sat, GOBSBAND &b1, GOBSBAND &b2);

        int pcoS_raw(gnss_data_sats &satdata, Triple &pco, GOBSBAND &b1);
        int pcoR_raw(gnss_data_sats &satdata, Triple &pco, GOBSBAND &b1);

        /** @brief pcb correction - satellite (nadir). */
        int pcvS_raw(double &corr, gnss_data_sats &sat, GOBSBAND &b1, Triple &site);
        int pcvR_raw(double &corr, gnss_data_sats &sat, GOBSBAND &b1);

        // ===========================================================================

        /** @brief pco correction - satellite. */
        int pcoS(gnss_data_sats &satdata, Triple &pco, GOBS_LC lc, GOBSBAND k1, GOBSBAND k2);
        int pcoR(gnss_data_sats &satdata, Triple &pco, GOBS_LC lc, GOBSBAND k1, GOBSBAND k2);

        /** @brief pco correction - receiver. */
        int pcoR(gnss_data_sats &satdata, Triple &dx, Triple &site, GOBS_LC lc);

        /** @brief pco projection into LoS. */
        int pco_proj(double &corr, gnss_data_sats &satdata, Triple &site, Triple &dx);

        /** @brief pcb correction - satellite (nadir). */
        int pcvS(double &corr, gnss_data_sats &sat, Triple &site, GOBS_LC lc, GOBSBAND k1, GOBSBAND k2);

        /** @brief pcv correction - site (zenith). */
        int pcvR(double &corr, gnss_data_sats &sat, GOBS_LC lc, GOBSBAND k1, GOBSBAND k2);

        /** @brief std::set/get PCO. */
        void pco(GFRQ f, const Triple &t) { _mappco[f] = t; }
        Triple pco(GFRQ f) { return _mappco[f]; } // CANNOT BE CONST ?
        hwa_map_PCO pco() const { return _mappco; }

        /** @brief std::set/get PCO zen. */
        void pcvzen(GFRQ f, const hwa_map_Z &t) { _mapzen[f] = t; }
        hwa_map_Z pcvzen(GFRQ f) { return _mapzen[f]; } // CANNOT BE CONST ?
        hwa_map_ZEN pcvzen() const { return _mapzen; }

        /** @brief std::set/get PCO aiz. */
        void pcvazi(GFRQ f, const hwa_map_A &t) { _mapazi[f] = t; }
        hwa_map_A pcvazi(GFRQ f) { return _mapazi[f]; } // CANNOT BE CONST ?
        hwa_map_azi pcvazi() const { return _mapazi; }
        void is_noazi(bool b) { _pcv_noazi = b; }

    private:
        /** @brief Does the calibration contain azi-depenedant data?. */
        bool _azi_dependent(GFRQ f);

        gnss_model_ephplan _ephplan;

        bool _trans;   ///< transmitter[true], receiver[false]
        std::string _anten; ///< antenna type
        std::string _ident; ///< antenna identification
        std::string _svcod; ///< svn code

        std::string _method; ///< method of calibration
        std::string _source; ///< source of calibration
        std::string _snxcod; ///< sinex code
        hwa_base::base_time _beg;   ///< valid from
        hwa_base::base_time _end;   ///< valid until
        double _dazi;   ///< azimuth sampling
        double _dzen;   ///< zenith sampling
        double _zen1;   ///< zenith1
        double _zen2;   ///< zenith2

        hwa_map_PCO _mappco; ///< std::map of PCOs (all frequencies)
        hwa_map_ZEN _mapzen; ///< std::map of NOAZI values (all frequencies)
        hwa_map_azi _mapazi; ///< std::map of AZI-dep values (all frequencies)

        bool _pcv_noazi;
    };

} // namespace

#endif
