#include <iostream>
#include <iomanip>
#include "hwa_gnss_data_obs.h"

using namespace hwa_set;

namespace hwa_gnss
{
    bool gnss_data_attr::valid() const
    {
        return (_gattr != ATTR);
    };
    // std::set attr
    // ----------
    void gnss_data_attr::attr(const GOBSATTR &a)
    {
        _gattr = a;
    }

    // get attr
    // -----------
    GOBSATTR gnss_data_attr::attr() const
    {
        return _gattr;
    }

    // operator
    // ----------
    bool gnss_data_attr::operator==(const gnss_data_attr &g) const
    {
        return (_gattr == g.attr());
    }

    // -------------------------------------------------------------------------------------------
    // class T_GBAND
    // -------------------------------------------------------------------------------------------
    bool gnss_data_band::valid() const
    {
        return (gnss_data_attr::valid() && _gband != BAND);
    }

    // std::set band
    // ----------
    void gnss_data_band::band(const GOBSBAND &b)
    {
        _gband = b;
    }

    // get band
    // -----------
    GOBSBAND gnss_data_band::band() const
    {
        return _gband;
    }

    // std::set attr
    // ----------
    void gnss_data_band::gattr(const gnss_data_attr &g)
    {
        _gattr = g.attr();
        _gband = BAND;
    }

    // get attr
    // -----------
    gnss_data_attr gnss_data_band::gattr() const
    {
        gnss_data_attr g(_gattr);
        return g;
    }

    // operators
    // ----------
    bool gnss_data_band::operator==(const gnss_data_band &g) const
    {
        return (_gband == g.band() &&
                _gattr == g.attr());
    }

    // -------------------------------------------------------------------------------------------
    // class gnss_data_obs
    // -------------------------------------------------------------------------------------------

    // valid ?
    // -----------
    bool gnss_data_obs::valid() const
    {
        return (gnss_data_band::valid() && _gtype != TYPE);
    }

    // std::set type
    // ----------
    void gnss_data_obs::type(const GOBSTYPE &t)
    {
        _gtype = t;
    }

    // get type
    // -----------
    GOBSTYPE gnss_data_obs::type() const
    {
        return _gtype;
    }

    // std::set gband
    // ----------
    void gnss_data_obs::gband(const gnss_data_band &g)
    {
        _gattr = g.attr();
        _gband = g.band();
        _gtype = TYPE;
    }

    // get gband
    // -----------
    gnss_data_band gnss_data_obs::gband() const
    {
        gnss_data_band g(_gband, _gattr);
        return g;
    }

    // operator
    // ----------
    bool gnss_data_obs::operator==(const gnss_data_obs &g) const
    {
        return (_gtype == g.type() &&
                _gband == g.band() &&
                _gattr == g.attr());
    }

    // std::set from GOBS
    // -----------
    int gnss_data_obs::gobs(const GOBS &g)
    {
        std::string s = gobs2str(g);
        _gtype = str2gobstype(s);
        _gband = str2gobsband(s);
        _gattr = str2gobsattr(s);

        return 1;
    }

    // std::set from std::string
    // -----------
    int gnss_data_obs::gobs(const std::string &s)
    {
        _gtype = str2gobstype(s);
        _gband = str2gobsband(s);
        _gattr = str2gobsattr(s);

        return 1;
    }

    // get GOBS enum
    // -----------
    GOBS gnss_data_obs::gobs() const
    {
        std::string s = gobstype2str(_gtype) +
                   gobsband2str(_gband) +
                   gobsattr2str(_gattr);
        return str2gobs(s);
    }

    // get 2char gobs
    // ----------
    GOBS gnss_data_obs::gobs2CH(GSYS gs) const
    {
        GOBS g = X;

        if (_gattr == ATTR_NULL)
        { // already 2char signal
            g = this->gobs();
        }
        else
        {
            if (_gtype == TYPE_C)
            {
                if (_gattr != ATTR_C && (gs == GPS || gs == GLO))
                    g = tba2gobs(TYPE_P, _gband, ATTR_NULL);
                else
                    g = tba2gobs(_gtype, _gband, ATTR_NULL);
            }
            if (_gtype == TYPE_L)
            {
                g = tba2gobs(_gtype, _gband, ATTR_NULL);
            }
        }

        return g;
    }

    // get 2char gobs
    // ----------
    GOBS gnss_data_obs::gobs3CH() const
    {
        GOBS g = X;

        if (_gattr != ATTR_NULL && _gattr != ATTR)
        { // already 3char signal
            g = this->gobs();
        }
        else
        {
            if (_gtype == TYPE_C)
            {
                g = tba2gobs(_gtype, _gband, ATTR_C);
            }
            else if (_gtype == TYPE_P)
            {
                g = tba2gobs(TYPE_C, _gband, ATTR_W);
            }
        }

        return g;
    }

    // change obs 2.xx to 3.xx
    // -----------------
    void gnss_data_obs::gobs2to3(GSYS sys)
    {
        if (_gattr != ATTR_NULL)
        {
            return;
        }
        GOBS obs2 = this->gobs();
        switch (sys)
        {
        case GPS:
            if (obs2 == L1 || obs2 == L2 || obs2 == C1 || obs2 == C2 || obs2 == D1 || obs2 == D2 || obs2 == S1 || obs2 == S2)
            {
                _gattr = ATTR_C;
            }
            else if (obs2 == L5 || obs2 == C5 || obs2 == D5 || obs2 == S5)
            {
                _gattr = ATTR_Q;
            }
            else if (obs2 == P1 || obs2 == P2 || obs2 == P5)
            {
                _gtype = TYPE_C;
                _gattr = ATTR_W;
            }
            break;
        case GAL: // modified by glfeng
            if (obs2 == L1 || obs2 == L6 || obs2 == C1 || obs2 == C6 || obs2 == S1 || obs2 == S6 || obs2 == D1 || obs2 == D6)
            {
                _gattr = ATTR_X;
            }
            else if (obs2 == L5 || obs2 == L7 || obs2 == L8 || obs2 == C5 || obs2 == C7 || obs2 == C8 || obs2 == D5 || obs2 == D7 || obs2 == D8 || obs2 == S5 || obs2 == S7 || obs2 == S8)
            {
                _gattr = ATTR_I;
            }
            else if (obs2 == P5)
            {
                _gtype = TYPE_C;
                _gattr = ATTR_Q;
            }
            break;
        case GLO: // modified by glfeng
            if (obs2 == L1 || obs2 == L2 || obs2 == C1 || obs2 == C2 || obs2 == D1 || obs2 == D2 || obs2 == S1 || obs2 == S2)
            {
                _gattr = ATTR_C;
            }
            else if (obs2 == P1 || obs2 == P2)
            {
                _gtype = TYPE_C;
                _gattr = ATTR_P;
            }
            break;
        case BDS: // added by lijie
            if (obs2 == L1 || obs2 == L7 || obs2 == C1 || obs2 == C7 || obs2 == S1 || obs2 == S7 || obs2 == D1 || obs2 == D7)
            {
                _gattr = ATTR_I;
            }
            else if (obs2 == L2 || obs2 == C2 || obs2 == D2 || obs2 == S2)
            {
                _gband = BAND_7;
                _gattr = ATTR_I;
            }
            else if (obs2 == P1)
            {
                _gtype = TYPE_C;
                _gattr = ATTR_Q;
            }
            else if (obs2 == P2)
            {
                _gtype = TYPE_C;
                _gband = BAND_7;
                _gattr = ATTR_Q;
            }
            else if (obs2 == C5 || obs2 == L5)
            {
                _gband = BAND_3;
                _gattr = ATTR_I;
            }
            break;
        case QZS: // modified by glfeng
            if (obs2 == L1 || obs2 == L5 || obs2 == C1 || obs2 == C5)
            {
                _gattr = ATTR_C;
            }
            else if (obs2 == D1 || obs2 == D5 || obs2 == S1 || obs2 == S5)
            {
                _gattr = ATTR_C;
            }
            break;
        case SBS:
            break;
        case IRN:
            break;
        case GNS:
            break;
        default:
            break;
        }
        return;
    }

    // get true if code observation
    // -----------
    bool gnss_data_obs::is_code() const
    {
        return (_gtype == TYPE_C || _gtype == TYPE_P);
    }

    // get true if phase observation
    // -----------
    bool gnss_data_obs::is_phase() const
    {
        return (_gtype == TYPE_L);
    }
    bool gnss_data_obs::is_doppler() const
    {
        return (_gtype == TYPE_D);
    }

    bool gnss_data_obscombtype::is_pseudorange() const
    {
        return _obs_type == TYPE_C;
    }

    gnss_data_obscombtype::gnss_data_obscombtype() : _obs_type(GOBSTYPE::TYPE),
        _obs_band_1(GOBSBAND::BAND),
        _obs_combine(OBSCOMBIN::DEF_OBSCOMBIN)
    {
    }

    gnss_data_obscombtype::gnss_data_obscombtype(const gnss_data_obscombtype& other) : _obs_type(other._obs_type),
        _obs_band_1(other._obs_band_1),
        _obs_band_2(other._obs_band_2),
        _obs_freq_1(other._obs_freq_1),
        _obs_freq_2(other._obs_freq_2),
        _obs_combine(other._obs_combine)
    {
    }

    gnss_data_obscombtype::gnss_data_obscombtype(const std::string& obscombtype) : _obs_type(GOBSTYPE::TYPE),
        _obs_band_1(GOBSBAND::BAND),
        _obs_combine(OBSCOMBIN::DEF_OBSCOMBIN)
    {
        if (obscombtype.length() < 2)
        {
            // TODO: need throwing error in constructor
            // throw ...
            return;
        }
        std::string str_type = obscombtype.substr(0, 1);
        if (str_type == "P")
            str_type = "C";
        _obs_type = str2gobstype(str_type);

        if (obscombtype == "SLR")
        {
            _obs_type = str2gobstype(obscombtype.substr(2, 1));
        }
        if (obscombtype == "KRANGE")
        {
            _obs_type = GOBSTYPE::TYPE_KBRRANGE;
            _obs_band = GOBSBAND::BAND_KBR;
        }
        if (obscombtype == "LRANGE")
        {
            _obs_type = GOBSTYPE::TYPE_LRIRANGE;
            _obs_band = GOBSBAND::BAND_LRI;
        }
        if (obscombtype == "KRATE")
        {
            _obs_type = GOBSTYPE::TYPE_KBRRATE;
            _obs_band = GOBSBAND::BAND_KBR;
        }
        if (obscombtype == "LRATE")
        {
            _obs_type = GOBSTYPE::TYPE_LRIRATE;
            _obs_band = GOBSBAND::BAND_LRI;
        }

        std::string str_band = obscombtype.substr(1);
        if (str_band.substr(0, 1) == "C")
        {
            _obs_combine = OBSCOMBIN::IONO_FREE;
            // jdhuang : Ϊ�˱�ʶ1X
            //_obs_band_1 = str2gobsband(obscombtype.substr(2,1));
            //_obs_band_2 = str2gobsband(obscombtype.substr(3,1));
            _obs_freq_1 = str2gnssfreq(obscombtype.substr(2, 1));
            _obs_freq_2 = str2gnssfreq(obscombtype.substr(3, 1));
        }
        else
        {
            //_obs_band_1 = str2gobsband(str_band);
            _obs_freq_1 = str2gnssfreq(obscombtype.substr(1, 1));
            //_obs_freq_2 = str2gnssfreq(obscombtype.substr(3, 1));
        }
    }

    gnss_data_obscombtype::gnss_data_obscombtype(const gnss_data_obs& obstype, OBSCOMBIN combtype) : _obs_type(obstype.type()),
        _obs_band_1(obstype.band()),
        _obs_combine(combtype)
    {
    }

    gnss_data_obscombtype::gnss_data_obscombtype(const gnss_data_obs& obstype, GOBSBAND b1, FREQ_SEQ freq_1, OBSCOMBIN combtype) : _obs_type(obstype.type()),
        _obs_band_1(b1),
        _obs_freq_1(freq_1),
        _obs_combine(combtype)
    {
    }

    gnss_data_obscombtype::gnss_data_obscombtype(const gnss_data_obs& obstype,
        GOBSBAND b1,
        GOBSBAND b2,
        FREQ_SEQ freq_1,
        FREQ_SEQ freq_2,
        OBSCOMBIN combtype) : _obs_type(obstype.type()),
        _obs_band_1(b1),
        _obs_band_2(b2),
        _obs_freq_1(freq_1),
        _obs_freq_2(freq_2),
        _obs_combine(combtype)
    {
    }

    gnss_data_obscombtype::gnss_data_obscombtype(GOBSTYPE t, GOBSBAND b, OBSCOMBIN obscomb) : _obs_type(t),
        _obs_band_1(b),
        _obs_combine(obscomb)
    {
    }

    std::string gnss_data_obscombtype::convert2str() const
    {
        std::string ans(gobstype2str(_obs_type == TYPE_C ? TYPE_P : _obs_type));
        if (_obs_combine == OBSCOMBIN::IONO_FREE)
        {
            // jdhuang : L12 1X
            ans += "C";
            //if (_obs_band_1 != BAND) ans += gobsband2str(_obs_band_1);
            //if (_obs_band_2 != BAND) ans += gobsband2str(_obs_band_2);
            if (_obs_freq_1 != FREQ_X)
                ans += gfreqseq2str(_obs_freq_1);
            if (_obs_freq_2 != FREQ_X)
                ans += gfreqseq2str(_obs_freq_2);
        }
        else
        {
            //if (_obs_band_1 != BAND) ans += gobsband2str(_obs_band_1);
            if (_obs_freq_1 != FREQ_X)
                ans += gfreqseq2str(_obs_freq_1);
        }
        return ans;
    }

    bool gnss_data_obscombtype::operator==(const gnss_data_obscombtype& g) const
    {
        return this->convert2str() == g.convert2str();
    }

    bool gnss_data_obscombtype::operator<(const gnss_data_obscombtype& g) const
    {
        return this->convert2str() < g.convert2str();
    }

    bool gnss_data_obscombtype::is_phase() const
    {
        return _obs_type == TYPE_L;
    }

    bool gnss_data_obscombtype::is_SLR() const
    {
        return _obs_type == TYPE_SLR;
    }
    bool gnss_data_obscombtype::is_KBR() const
    {
        if (_obs_type == TYPE_KBRRANGE || _obs_type == TYPE_KBRRATE)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool gnss_data_obscombtype::is_LRI() const
    {
        if (_obs_type == TYPE_LRIRANGE || _obs_type == TYPE_LRIRATE)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool gnss_data_obscombtype::is_code() const
    {
        return (_obs_type == TYPE_C || _obs_type == TYPE_P);
    }

}; // namespace
