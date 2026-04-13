#ifndef hwa_gnss_data_obs_H
#define hwa_gnss_data_obs_H

#include "hwa_set_gtype.h"
#include "hwa_set_gproc.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /** @brief class for gnss_data_attr. */
    class gnss_data_attr
    {

    public:
        /** @brief constructor 1. */
        gnss_data_attr() { _gattr = ATTR; };

        /** @brief constructor 2. */
        explicit gnss_data_attr(const GOBSATTR& a) { _gattr = a; };

        /** @brief default destructor. */
        ~gnss_data_attr() {};

        /** @brief std::set attr. */
        virtual void attr(const GOBSATTR& a);

        /** @brief get attr. */
        virtual GOBSATTR attr() const;

        /** @brief override operator ==. */
        virtual bool operator==(const gnss_data_attr& g) const;

        /** @brief get valid. */
        virtual bool valid() const;

    protected:
        GOBSATTR _gattr; ///< gnss attr
    };

    /** @brief class for gnss_data_band derive from gnss_data_attr. */
    class gnss_data_band : public gnss_data_attr
    {

    public:
        /** @brief constructor 1. */
        gnss_data_band() : gnss_data_attr() { _gband = BAND; };

        /** @brief constructor 2. */
        gnss_data_band(GOBSBAND b, GOBSATTR a) : gnss_data_attr(a) { _gband = b; };

        /** @brief default destructor. */
        virtual ~gnss_data_band() {};

        /** @brief std::set band. */
        virtual void band(const GOBSBAND& g);

        /** @brief get band. */
        virtual GOBSBAND band() const;

        /** @brief std::set gnss_data_attr. */
        virtual void gattr(const gnss_data_attr& g);

        /** @brief get gnss_data_attr. */
        virtual gnss_data_attr gattr() const;

        /** @brief override operator ==. */
        virtual bool operator==(const gnss_data_band& g) const;

        /** @brief get valid. */
        virtual bool valid() const override;

    protected:
        GOBSBAND _gband; ///< gnss band
    };

    /** @brief class for gnss_data_obs derive from gnss_data_attr. */
    class gnss_data_obs : public gnss_data_band
    {

    public:
        /** @brief constructor 1. */
        gnss_data_obs() : gnss_data_band() { _gtype = TYPE; };

        /** @brief constructor 2. */
        gnss_data_obs(GOBSTYPE t, GOBSBAND b, GOBSATTR a) : gnss_data_band(b, a) { _gtype = t; };

        /** @brief constructor 3. */
        explicit gnss_data_obs(const GOBS& g) { gobs(g); };

        /** @brief default destructor. */
        virtual ~gnss_data_obs() {};

        /** @brief std::set type (only, inherit band&attr!). */
        virtual void type(const GOBSTYPE& t);

        /** @brief get type. */
        virtual GOBSTYPE type() const;

        /** @brief std::set attr. */
        virtual void gband(const gnss_data_band& g);

        /** @brief get attr. */
        virtual gnss_data_band gband() const;

        /** @brief std::set type (only! inherit). */
        int gobs(const GOBS& g);
        int gobs(const std::string& s);

        /** @brief get gobs enum. */
        GOBS gobs() const;

        /** @brief get 2char gobs (only C1/C2 and P1/P2 and L1/L2). */
        GOBS gobs2CH(GSYS gs) const;

        /** @brief get 3char gobs ( ). */
        GOBS gobs3CH() const;

        /** @brief change obs from 2 to 3. */
        void gobs2to3(GSYS gs);

        /** @brief override operator ==. */
        bool operator==(const gnss_data_obs& g) const;

        /** @brief valid. */
        bool valid() const override;

        /** @brief get is_code. */
        bool is_code() const;

        /** @brief get is_phase. */
        bool is_phase() const;

        /** @brief get is_doppler. */
        bool is_doppler() const; // added by zhshen

    protected:
        GOBSTYPE _gtype; ///< gtype
    };

    // The Define for LC PC
    class gnss_data_obscombtype
    {
    public:
        gnss_data_obscombtype();
        gnss_data_obscombtype(const gnss_data_obscombtype& other);
        explicit gnss_data_obscombtype(const std::string& obscombtype);
        gnss_data_obscombtype(const gnss_data_obs& obstype, OBSCOMBIN combtype);
        gnss_data_obscombtype(const gnss_data_obs& obstype, GOBSBAND b1, FREQ_SEQ freq_1, OBSCOMBIN combtype);
        gnss_data_obscombtype(const gnss_data_obs& obstype, GOBSBAND b1, GOBSBAND b2, FREQ_SEQ freq_1, FREQ_SEQ freq_2, OBSCOMBIN combtype);
        gnss_data_obscombtype(GOBSTYPE t, GOBSBAND b, OBSCOMBIN obscomb);

        std::string convert2str() const;

        bool operator==(const gnss_data_obscombtype& g) const;
        bool operator<(const gnss_data_obscombtype& g) const;
        bool is_freq(const FREQ_SEQ& freq_1, const FREQ_SEQ& freq_2) const
        {
            return (_obs_freq_1 == freq_1 && _obs_freq_2 == freq_2);
        }
        bool is_freq12() const { return (_obs_freq_1 == FREQ_1 && _obs_freq_2 == FREQ_2); }
        bool is_freq13() const { return (_obs_freq_1 == FREQ_1 && _obs_freq_2 == FREQ_3); }
        bool is_freq14() const { return (_obs_freq_1 == FREQ_1 && _obs_freq_2 == FREQ_4); }
        bool is_freq15() const { return (_obs_freq_1 == FREQ_1 && _obs_freq_2 == FREQ_5); }
        bool is_freq_raw1() const { return (_obs_freq_1 == FREQ_1 && _obs_freq_2 == FREQ_X); }

        GOBSBAND getBand_1() const { return _obs_band_1; }; // Added by lvhb 20200428
        GOBSBAND getBand_2() const { return _obs_band_2; }; // Added by jdhuang
        FREQ_SEQ getFreq_1() const { return _obs_freq_1; };
        FREQ_SEQ getFreq_2() const { return _obs_freq_2; };
        std::pair<GOBSBAND, GOBSBAND> getBand_pair() const { return std::make_pair(_obs_band_1, _obs_band_2); }; // jdhuang : added
        std::pair<FREQ_SEQ, FREQ_SEQ> getFreq_pair() const { return std::make_pair(_obs_freq_1, _obs_freq_2); }; // jdhuang : added
        bool is_phase() const;
        bool is_pseudorange() const;
        bool is_code() const;
        bool is_SLR() const;
        bool is_KBR() const;
        bool is_LRI() const;

        GOBSBAND getBand() { return _obs_band_1; }; // Added by lvhb 20200428

    protected:
        GOBSTYPE _obs_type = GOBSTYPE::TYPE;
        GOBSBAND _obs_band = BAND;
        GOBSBAND _obs_band_1 = BAND;   //jdhuang: change to band 1 for IF, jdhuang
        GOBSBAND _obs_band_2 = BAND;   //jdhuang:
        FREQ_SEQ _obs_freq_1 = FREQ_X; //jdhuang:
        FREQ_SEQ _obs_freq_2 = FREQ_X; //jdhuang:
        OBSCOMBIN _obs_combine = OBSCOMBIN::DEF_OBSCOMBIN;
    };
} // namespace

#endif // GOBS_H
