#ifndef hwa_set_gtype_h
#define hwa_set_gtype_h
#include "hwa_base_eigendef.h"
#include "hwa_set_amb.h"
#include "hwa_set_proc.h"
#include "hwa_set_gproc_type.h"

namespace hwa_gnss
{
    /** @brief TYPEDEF. */
    typedef std::vector<GOBSATTR> hwa_vec_attr;          ///< attr std::vector
    typedef std::vector<GOBSBAND> hwa_vec_band;          ///< band std::vector
    typedef std::vector<GFRQ> hwa_vec_freq;              ///< frequency std::vector
    typedef std::map<GOBSTYPE, hwa_vec_attr> hwa_map_attr; ///< attr std::map
    typedef std::map<GOBSBAND, hwa_map_attr> hwa_map_type; ///< type std::map
    typedef std::map<GSYS, std::set<std::string>> hwa_map_sats;    ///< sats std::map
    typedef std::map<GSYS, std::set<std::string>> hwa_map_gnav;    ///< nav std::map
    typedef std::map<GSYS, hwa_map_type> hwa_map_gnss;     ///< gnss std::map
    typedef std::map<GSYS, hwa_vec_band> hwa_map_band;     ///< band std::map
    typedef std::map<GSYS, hwa_vec_freq> hwa_map_freq;     ///< frequency mao

    typedef std::map<GOBSBAND, Triple> hwa_map_band_atx; ///< Triple: ATX  NORTH / EAST / UP
    typedef std::map<GSYS, hwa_map_band_atx> hwa_map_gsys_pco;

    /** @brief GLOBAL FUNCTIONS. */
    GOBSATTR str2gobsattr(std::string s); ///< get GOBSATTR enum from gobs std::string
    GOBSBAND str2gobsband(std::string s); ///< get GOBSBAND enum from gobs std::string
    GOBSTYPE str2gobstype(std::string s); ///< get GOBSTYPE enum from gobs std::string
    GNAVTYPE str2gnavtype(std::string s); ///< get GNAVTYPE enum from gobs std::string
    FREQ_SEQ str2sysfreq(std::string s);  ///< get FREQ_SEQ enum from std::string (add by xiongyun)
    FREQ_SEQ str2gnssfreq(std::string s); ///< get FREQ_SEQ enum from gobs std::string

    GOBSATTR char2gobsattr(char c); ///< get GOBSATTR enum from char
    GOBSBAND char2gobsband(char c); ///< get GOBSBAND enum from char
    GOBSBAND int2gobsband(int c);   ///< get GOBSBAND enum from char
    GOBSTYPE char2gobstype(char c); ///< get GOBSTYPE enum from char
    FREQ_SEQ char2gnssfreq(char c); ///< get FREQ_SEQ enum from char
    GOBS_LC int2gobsfreq(int c);    ///< get GOBS_LC enum from char

    std::string gobsattr2str(GOBSATTR e); ///< get std::string enum from GOBSATTR
    std::string gobsband2str(GOBSBAND e); ///< get std::string enum from GOBSBAND
    std::string gobstype2str(GOBSTYPE e); ///< get std::string enum from GOBSTYPE
    std::string gnavtype2str(GNAVTYPE e); ///< get std::string enum from GNAVTYPE

    std::string gobs2str(GOBS);                             ///< get std::string from GOBS enum
    GOBS str2gobs(std::string s);                           ///< get GOBS enum from std::string
    GOBS tba2gobs(GOBSTYPE t, GOBSBAND b, GOBSATTR a); ///< get GOBS from type, band, and attribute
    std::string gfreqseq2str(FREQ_SEQ f);                   ///< convert FREQ_SEQ to std::string

    int gobs2band(GOBS o); ///< get band from GOBS enum

    GOBS pha2snr(GOBS o);        ///  get GOBS enum (pha->snr)
    GOBS pl2snr(GOBS o);         ///  get GOBS enum (pha or code->snr) add wh
    bool gobs_code(GOBS o);      ///< get true for code obs
    bool gobs_phase(GOBS o);     ///< get true for phase obs
    bool gobs_doppler(GOBS o);   ///< get true for doppler obs
    bool gobs_snr(GOBS o);       ///< get true for snr obs

    hwa_map_sats gnss_sats();      ///< static std::map of default GNSS satellites
    hwa_map_gnav gnss_gnav();      ///< static std::map of default GNSS navigation types

    hwa_map_gnss gnss_data_priority(); ///< static std::map of default GNSS data types/bands/attrs priorities
    hwa_map_band gnss_band_sorted();                             ///< static std::map of sorted GNSS band w.r.t. wavelength

    std::vector<GOBSBAND> sort_band(GSYS gs, std::set<GOBSBAND> &bands); // sort std::set of bands w.r.t. wavelength

    hwa_map_gsys_pco gnss_pco_offsets(); ///< static std::map of default GNSS PCO offsets
    std::set<GSYS> gnss_supported(); ///< supported GNSS

    /** @brief GNSS frequency. */
    const hwa_map_freq GNSS_FREQ_PRIORITY = 
    {
        {GPS, {LAST_GFRQ, G01, G02, G05}},
        {GLO, {LAST_GFRQ, R01, R02, R03_CDMA, R05_CDMA}},
        {GAL, {LAST_GFRQ, E01, E05, E07, E08, E06}},
        {BDS, {LAST_GFRQ, C02, C07, C06, C05, C09, C08, C01}},
        {QZS, {LAST_GFRQ, J01, J02, J05, J06}},
        {SBS, {LAST_GFRQ, S01, S05}},
        {GNS, {}},
    }; // static std::map of default GNSS freq priorities

    /** @brief GNSS band. */
    const hwa_map_band GNSS_BAND_PRIORITY = 
    {
        {GPS, {BAND, BAND_1, BAND_2, BAND_5}},
        {GLO, {BAND, BAND_1, BAND_2, BAND_3, BAND_5}},
        {GAL, {BAND, BAND_1, BAND_5, BAND_7, BAND_8, BAND_6}},
        {BDS, {BAND, BAND_2, BAND_7, BAND_6, BAND_5, BAND_9, BAND_8, BAND_1}},
        {QZS, {BAND, BAND_1, BAND_2, BAND_5, BAND_6}},
        {SBS, {BAND, BAND_1, BAND_5}},
        {GNS, {}},
    }; // static std::map of default GNSS band priorities

} // namespace

#endif // GOBS_H
