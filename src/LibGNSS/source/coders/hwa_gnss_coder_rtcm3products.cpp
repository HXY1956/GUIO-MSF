#include <math.h>
#include <stdio.h>
#include <string.h>
#ifndef sparc
#include <stdint.h>
#else
#include <sys/types.h>
#endif
#include "hwa_gnss_coder_rtcm3products.h"
#include "hwa_gnss_coder_aug.h"
#include "hwa_gnss_coder_auggrid.h"

using namespace std;

namespace hwa_gnss {

    uint32_t CRC24(long size, const unsigned char* buf)
    {
        uint32_t crc = 0;
        int i;

        while (size--)
        {
            crc ^= (*buf++) << (16);
            for (i = 0; i < 8; i++)
            {
                crc <<= 1;
                if (crc & 0x1000000)
                    crc ^= 0x01864cfb;
            }
        }
        return crc;
    }

    static int sys2index(char sys)
    {
        int isys;
        switch (sys)
        {
        case 'G':
            isys = 0x01;
            break;
        case 'R':
            isys = 0x04;
            break;
        case 'E':
            isys = 0x08;
            break;
        case 'C':
            isys = 0x20;
            break;
        case 'J':
            isys = 0x10;
            break;
        case 'S':
            isys = 0x02;
            break;
        case 'I':
            isys = 0x40;
            break;
        default:
            isys = -1;
            break;
        }
        return isys;
    }

    static char index2sys(int index)
    {
        switch (index)
        {
        case 0x01:
            return 'G';
        case 0x04:
            return 'R';
        case 0x08:
            return 'E';
        case 0x20:
            return 'C';
        case 0x10:
            return 'J';
        case 0x02:
            return 'S';
        case 0x40:
            return 'I';
        default:
            return 'X';
        }
    }

#define STOREBITS                                 \
    while (numbits >= 8)                          \
    {                                             \
        if (!size)                                \
            return 0;                             \
        *(buffer++) = bitbuffer >> (numbits - 8); \
        numbits -= 8;                             \
        ++ressize;                                \
        --size;                                   \
    }

#define ADDBITS(a, b)                                            \
    {                                                            \
        bitbuffer = (bitbuffer << (a)) | ((b) & ((1 << a) - 1)); \
        numbits += (a);                                          \
        STOREBITS                                                \
    }

#define ADDBITS32(a, b)         \
    {                           \
        int x = a - 30;         \
        ADDBITS(x, ((b) >> 30)) \
        ADDBITS(30, (b))        \
    }

#define STARTDATA       \
    size_t ressize = 0; \
    char *blockstart;   \
    int numbits;        \
    uint64_t bitbuffer = 0;

#define INITBLOCK        \
    numbits = 0;         \
    blockstart = buffer; \
    ADDBITS(8, 0xD3)     \
    ADDBITS(6, 0)        \
    ADDBITS(10, 0)

#define ENDBLOCK                                                 \
    if (numbits)                                                 \
    {                                                            \
        ADDBITS((8 - numbits), 0)                                \
    }                                                            \
    {                                                            \
        int len = buffer - blockstart - 3;                       \
        blockstart[1] |= len >> 8;                               \
        blockstart[2] = len;                                     \
        if (len > 1023)                                          \
            return 0;                                            \
        len = CRC24(len + 3, (const unsigned char *)blockstart); \
        ADDBITS(24, len)                                         \
    }

#define SCALEADDBITS(a, b, c) ADDBITS(a, nearestint(b *c))

#define T_MESSAGE_NUMBER(a) ADDBITS(12, a) /* DF002 */
#define T_INDEX(a) ADDBITS(8, a)
#define T_WEEK(a) ADDBITS(12, a)
#define T_TOW(a) ADDBITS(20, a) /* DF224 */
#define T_NSAT(a) ADDBITS(8, a)
#define T_SYS(a) ADDBITS(8, a)
#define T_PRN(a) ADDBITS(8, a)
#define gnss_coder_augDATA(a) SCALEADDBITS(30, 10000.0, a)
#define T_LAT(a) SCALEADDBITS(30, 10000.0, a)
#define T_LON(a) SCALEADDBITS(30, 10000.0, a)
#define T_HEIGHT(a) SCALEADDBITS(30, 10000.0, a)
#define nearestint(fl) ((int)(fl + (fl < 0.0 ? -0.5 : 0.5)))

    size_t MakeAUG_RTCM(const string& augstring, char* buffer, size_t size)
    {
        int gpsw;
        double gpssec;
        char site_1char, site_2char, site_3char, site_4char;
        string str, site, prn;
        int isys, iprn, nband, band1, band2, band3;
        double p1 = 0.0, p2 = 0.0, l1 = 0.0, l2 = 0.0, p3 = 0.0, l3 = 0.0;

        stringstream os(augstring.substr(1));
        os >> str >> gpsw >> gpssec >> site >> prn >> nband;
        for (int i = 0; i < nband; i++)
        {
            if (i == 0)
                os >> band1 >> p1 >> l1;
            if (i == 1)
                os >> band2 >> p2 >> l2;
            if (i == 2)
                os >> band3 >> p3 >> l3;
        }
        //os >> str >> gpsw >> gpssec >> site >> prn >> p1 >> l1 >> p2 >> l2;
        site_1char = site[0];
        site_2char = site[1];
        site_3char = site[2];
        site_4char = site[3];

        isys = sys2index(prn[0]);

        istringstream is(prn.substr(1));
        is >> iprn;

        STARTDATA
            INITBLOCK
            T_MESSAGE_NUMBER(GREATAUGRTCM_ID)
            T_WEEK(gpsw)
            T_TOW((int)gpssec)
            T_INDEX((int)site_1char)
            T_INDEX((int)site_2char)
            T_INDEX((int)site_3char)
            T_INDEX((int)site_4char)
            T_SYS(isys)
            T_PRN(iprn)
            T_INDEX(nband)
            if (nband > 0)
            {
                T_INDEX(band1)
                    gnss_coder_augDATA(p1)
                    gnss_coder_augDATA(l1)
            }
        if (nband > 1)
        {
            T_INDEX(band2)
                gnss_coder_augDATA(p2)
                gnss_coder_augDATA(l2)
        }
        if (nband > 2)
        {
            T_INDEX(band3)
                gnss_coder_augDATA(p3)
                gnss_coder_augDATA(l3)
        }
        ENDBLOCK
            return ressize;
    }

#define DECODESTART  \
    int numbits = 0; \
    uint64_t bitbuffer = 0;

#define LOADBITS(a)                                                      \
    {                                                                    \
        while ((a) > numbits)                                            \
        {                                                                \
            if (!size--)                                                 \
                return AUGR_SHORTMESSAGE;                                \
            bitbuffer = (bitbuffer << 8) | ((unsigned char)*(buffer++)); \
            numbits += 8;                                                \
        }                                                                \
    }

    /* extract bits from data stream
       b = variable to store result, a = number of bits */
#define GETBITS(b, a)                                    \
    {                                                    \
        LOADBITS(a)                                      \
        b = (bitbuffer << (64 - numbits)) >> (64 - (a)); \
        numbits -= (a);                                  \
    }

       /* extract signed floating value from data stream
             b = variable to store result, a = number of bits */
#define GETFLOATSIGN(b, a, c)                                                         \
    {                                                                                 \
        LOADBITS(a)                                                                   \
        b = ((double)(((int64_t)(bitbuffer << (64 - numbits))) >> (64 - (a)))) * (c); \
        numbits -= (a);                                                               \
    }

#define SKIPBITS(b)     \
    {                   \
        LOADBITS(b)     \
        numbits -= (b); \
    }

#define G_HEADER(a) GETBITS(a, 8)
#define G_RESERVEDH(a) GETBITS(a, 6)
#define G_SIZE(a) GETBITS(a, 10)
#define G_INDEX(a) GETBITS(a, 8)
#define G_NSAT(a) GETBITS(a, 8)
#define G_SYS(a) GETBITS(a, 8)
#define G_PRN(a) GETBITS(a, 8)
#define G_MESSAGE_NUMBER(a) GETBITS(a, 12) /* DF002 */
#define G_WEEK(a) GETBITS(a, 12)
#define G_TOW(a) GETBITS(a, 20) /* DF224 */
#define G_augDATA(a) GETFLOATSIGN(a, 30, 1 / 10000.0)
#define G_LAT(a) GETFLOATSIGN(a, 30, 1 / 10000.0)
#define G_LON(a) GETFLOATSIGN(a, 30, 1 / 10000.0)
#define G_HEIGHT(a) GETFLOATSIGN(a, 30, 1 / 10000.0)

    enum AUG_RETURN gnss_coder_aug::GetAUG(string& augs, const char* buffer, size_t size)
    {
        int rs = 0, h = 0, type = 0;
        size_t blocksize;
        const char* blockstart = buffer;

        char csys = 'X';

        int gpsw = 0, gpssec = 0, site_1 = 0, site_2 = 0, site_3 = 0, site_4 = 0, isys = 0, iprn = 0;
        double p1 = 0.0, l1 = 0.0, p2 = 0.0, l2 = 0.0, p3 = 0.0, l3 = 0.0;
        int nband = 0, band1 = 0, band2 = 0, band3 = 0;
        char a1, a2, a3, a4;
        string site = "", prn = "";
        stringstream os;

        DECODESTART
            if (size < 7)
                return AUGR_SHORTBUFFER;

        G_HEADER(h)
            G_RESERVEDH(rs)
            G_SIZE(blocksize);

        if ((unsigned char)h != 0xD3 || rs)
            return AUGR_UNKNOWNDATA;
        /* 3 header bytes already removed */
        if (size < blocksize + 3)
            return AUGR_MESSAGEEXCEEDSBUFFER;
        if (CRC24(blocksize + 3, (const unsigned char*)blockstart) !=
            (uint32_t)((((unsigned char)buffer[blocksize]) << 16) | (((unsigned char)buffer[blocksize + 1]) << 8) |
                (((unsigned char)buffer[blocksize + 2]))))
            return AUGR_CRCMISMATCH;

        size = blocksize; /* reduce size, so overflows are detected */

        G_MESSAGE_NUMBER(type)
            if (type != GREATAUGRTCM_ID)
                return AUGR_UNKNOWNTYPE;

        G_WEEK(gpsw)
            G_TOW(gpssec)
            G_INDEX(site_1)
            G_INDEX(site_2)
            G_INDEX(site_3)
            G_INDEX(site_4)
            G_SYS(isys)
            G_PRN(iprn)
            G_INDEX(nband)
            if (nband > 0)
            {
                G_INDEX(band1)
                    G_augDATA(p1)
                    G_augDATA(l1)
            }
        if (nband > 1)
        {
            G_INDEX(band2)
                G_augDATA(p2)
                G_augDATA(l2)
        }
        if (nband > 2)
        {
            G_INDEX(band3)
                G_augDATA(p3)
                G_augDATA(l3)
        }

        a1 = (char)(site_1);
        a2 = (char)(site_2);
        a3 = (char)(site_3);
        a4 = (char)(site_4);

        site += a1;
        site += a2;
        site += a3;
        site += a4;

        csys = index2sys(isys);
        stringstream oos;
        oos << csys << setw(2) << setfill('0') << iprn;
        prn = oos.str();
        GSYS gsys = gnss_sys::char2gsys(csys);

        if (sysall.find(gnss_sys::gsys2str(gsys)) == sysall.end())
            return AUGR_UNKNOWNDATA;

        if (iprn > 0 && csys != 'X')
        {
            hwa_pair_augtype augtype;
            map<string, base_data*>::iterator it = _data.begin();

            while (it != _data.end())
            {
                if (it->second->id_type() == base_data::AUG)
                {
                    base_time t(gpsw, gpssec * 1.0);
                    if (nband > 0)
                    {
                        augtype = make_pair(AUGTYPE::TYPE_P, int2gobsband(band1));
                        ((gnss_data_aug*)it->second)->add_data(site, t, prn, augtype, p1);

                        augtype = make_pair(AUGTYPE::TYPE_L, int2gobsband(band1));
                        ((gnss_data_aug*)it->second)->add_data(site, t, prn, augtype, l1);
                    }

                    if (nband > 1)
                    {
                        augtype = make_pair(AUGTYPE::TYPE_P, int2gobsband(band2));
                        ((gnss_data_aug*)it->second)->add_data(site, t, prn, augtype, p2);

                        augtype = make_pair(AUGTYPE::TYPE_L, int2gobsband(band2));
                        ((gnss_data_aug*)it->second)->add_data(site, t, prn, augtype, l2);
                    }

                    if (nband > 2)
                    {
                        augtype = make_pair(AUGTYPE::TYPE_P, int2gobsband(band3));
                        ((gnss_data_aug*)it->second)->add_data(site, t, prn, augtype, p3);

                        augtype = make_pair(AUGTYPE::TYPE_L, int2gobsband(band3));
                        ((gnss_data_aug*)it->second)->add_data(site, t, prn, augtype, l3);
                    }

                    break;
                }
                it++;
            }

            os << "> AUG " << fixed << setw(4) << gpsw << setw(18) << setprecision(5) << gpssec * 1.0
                << setw(7) << site << setw(9) << prn << setw(6) << nband;
            if (nband > 0)
            {
                os << setw(5) << band1 << setw(12) << setprecision(4) << p1
                    << setw(12) << setprecision(4) << l1;
            }
            if (nband > 1)
            {
                os << setw(5) << band2 << setw(12) << setprecision(4) << p2
                    << setw(12) << setprecision(4) << l2 << endl;
            }
            if (nband > 2)
            {
                os << setw(5) << band3 << setw(12) << setprecision(4) << p3
                    << setw(12) << setprecision(4) << l3 << endl;
            }
        }

        augs = os.str();
        return AUGR_OK;
    }

    ////////////////////////////////////////////////////////////////////////////
    bool gnss_coder_aug::DecodeGREATAUGCorrection(unsigned char* buffer, int size)
    {

        // int bytesused = 0;
        string augss = "";
        GetAUG(augss, reinterpret_cast<char*>(buffer), size);
        //printf(augss.c_str());
        return true;
    }
#define T_MESSAGE_NUMBER(a) ADDBITS(12, a) /* DF002 */
#define T_WEEK(a) ADDBITS(12, a)
#define T_TOW(a) ADDBITS(20, a) /* DF224 */
#define T_NSAT(a) ADDBITS(8, a) // jxy 20180116
#define T_SYS(a) ADDBITS(8, a)
#define T_PRN(a) ADDBITS(8, a)
#define T_WL(a) SCALEADDBITS(30, 10000.0, a)
#define T_NL(a) SCALEADDBITS(30, 10000.0, a)
#define T_WL_SIG(a) SCALEADDBITS(30, 10000.0, a)
#define T_NL_SIG(a) SCALEADDBITS(30, 10000.0, a)
#define T_STAWL(a) ADDBITS(8, a)
#define T_STANL(a) ADDBITS(8, a)
#define T_STAWL_all(a) ADDBITS(8, a)
#define T_STANL_all(a) ADDBITS(8, a)

    size_t MakeUPD_RTCM(const base_time t, const one_epoch_upd& updWL, const one_epoch_upd& updNL, char* buffer, size_t size)
    {
        // int i;
        STARTDATA

            INITBLOCK
            T_MESSAGE_NUMBER(GREATUPDRTCM_ID)
            T_WEEK(t.gwk());
        T_TOW(t.sow());

        int nvalid = 0;
        for (const auto& itsat : updWL)
        {
            if (itsat.second->npoint < 6 || itsat.second->ratio < 0.7)
                continue;
            nvalid++;
        }

        T_NSAT(nvalid);
        for (const auto& itsat : updWL)
        {
            if (itsat.second->npoint < 6 || itsat.second->ratio < 0.7)
                continue;

            T_SYS((int)(itsat.first.c_str()[0]))
                T_PRN(std::atoi(itsat.first.substr(1, 2).c_str()))
                T_WL(itsat.second->value)
                //T_NL((*updNL[itsat.first])->value)
                T_WL_SIG(itsat.second->sigma)
                //T_NL_SIG(upds[i].nl.sig)
                T_STAWL(itsat.second->npoint)
                //T_STANL(upds[i].nl.nsta)
                T_STAWL_all((int)(itsat.second->npoint / itsat.second->ratio))
                //T_STANL_all(upds[i].nl.nsta_all)
        }
        ENDBLOCK

            return ressize;
    }

    //size_t MakeUPD_RTCM(const base_time t, const vector<string>& sat, const vector<string>& obj, const vector<int>& npoint,
    //    const vector<double>& ratio, const vector<double>& value, const vector<double>& sigma, char* buffer, size_t size)
    //{
    //    int i;
    //    STARTDATA
    //
    //    INITBLOCK
    //    T_MESSAGE_NUMBER(GREATUPDRTCM_ID)
    //    T_WEEK(t.gwk());
    //    T_TOW(t.sow());
    //
    //    int nvalid = 0;
    //    for (i = 0; i < obj.size(); i++)
    //    {
    //        //if (itsat.second->npoint < 6 || itsat.second->ratio < 0.7)continue;
    //        if (npoint[i] < 6 || ratio[i] < 0.7)continue;
    //        nvalid++;
    //    }
    //
    //    T_NSAT(nvalid);
    //    for (i = 0; i < obj.size(); i++)
    //    {
    //        if (npoint[i] < 6 || ratio[i] < 0.7)continue;
    //
    //        /*T_SYS((int)(itsat.first.c_str()[0]))
    //        T_PRN(std::atoi(itsat.first.substr(1, 2).c_str()))
    //        T_WL(itsat.second->value)
    //        T_WL_SIG(itsat.second->sigma)
    //        T_STAWL(itsat.second->npoint)
    //        T_STAWL_all((int)(itsat.second->npoint / itsat.second->ratio))*/
    //
    //        T_SYS((int)(sat[i].c_str()[0]))
    //        T_PRN(std::atoi(sat[i].substr(1, 2).c_str()))
    //        T_WL(value[i])
    //        T_WL_SIG(sigma[i])
    //        T_STAWL(npoint[i])
    //        T_STAWL_all((int)(npoint[i] / ratio[i]))
    //    }
    //    ENDBLOCK
    //
    //        return ressize;
    //}

    //
    //AUG_RETURN BASE_LIBRARY_EXPORT GetUPD(one_epoch_upd& upd, const char* buffer, size_t size)
    //{
    //    return AUG_RETURN BASE_LIBRARY_EXPORT();
    //}
}
