#ifndef hwa_set_gproc_type_h
#define hwa_set_gproc_type_h

namespace hwa_gnss {
    enum class CONSTRPAR
    {
        EST,
        FIX,
        KIN,
        SIMU_KIN,
        CONSTRPAR_UNDEF
    };
    enum HMTCONSTR
    {
        NNT,
        NNR,
        NNTNNR,
        HMTCONSTR_UNDEF
    };
    enum class GRDMPFUNC
    {
        DEF_GRDMPFUNC,
        TILTING,
        CHEN_HERRING,
        BAR_SEVER
    };
    enum class ZTDMPFUNC
    {
        DEF_ZTDMPFUNC,
        COSZ,
        GMF,
        NO_MF
    }; //lvhb added "NO_MF" for npp, 20200917
    enum class IONMPFUNC
    {
        DEF_IONMPFUNC,
        ICOSZ,
        QFAC,
        NONE
    };
    enum class OBSWEIGHT
    {
        DEF_OBS_WEIGHT,
        EQUAL,
        SINEL,
        SINEL2,
        SINEL4,
        CODPHA,
        MLTPTH,
        PARTELE,
        SNR,
        SNRELE          // added by hlgou
    };
    enum class TROPMODEL
    {
        DEF_TROPMODEL,
        SAASTAMOINEN,
        DAVIS,
        HOPFIELD,
        MOPS,
        GPTW,
        GPT2W,
        GAL27,
        GALTROPO27,
        EXTERN
    };
    enum class ZTDMODEL
    {
        DEF_ZTDMODEL,
        PWC,
        STO
    };
    enum class RESIDTYPE
    {
        DEF_RESIDTYPE,
        RES_ORIG,
        RES_NORM,
        RES_ALL
    };
    enum class OBSCOMBIN
    {
        DEF_OBSCOMBIN,
        IONO_FREE,
        RAW_SINGLE,
        RAW_DOUBLE,
        RAW_ALL,
        MW_COMBIN,
        EWL_COMBIN,
        WL_COMBIN,
        RAW_MIX,
        IF_P1
    };
    enum class PHASEBIAS
    {
        DEF_POSTPRD,
        RTCM_PRD,
        CNES_PRD,
        ESTI_PRD
    };
    enum class ATTITUDES
    {
        DEF_YAWMODEL,
        YAW_NOMI,
        YAW_RTCM
    };
    enum class CBIASCHAR
    {
        DEF_CBIASCHAR,
        ORIG,
        CHAR2,
        CHAR3
    };
    enum class SYSBIASMODEL
    {
        AUTO_CON,
        AUTO_RWK,
        ISB_CON,
        ISB_RWK,
        AUTO_WHIT,
        ISB_WHIT
    }; //glfeng
    enum class GRDMODEL
    {
        DEF_GRDMODEL,
        GRD_PWC,
        GRD_STO
    }; //glfeng
    enum class base_pos
    {
        RINEXH,
        SPP,
        CFILE,
        KIN2KIN,
        EXTERNAL
    };

    /** @brief  enum for lsq model */
    enum class LSQ_MODE
    {
        EPO, ///< EPO model:solve every epoch
        lsq, ///< lsq model:Overall solution
        LSQ_MODE_UNDEF
    };
    enum RECEIVERTYPE
    {
        DEF,
        F9P,
        And
    };

    //
    enum class SLIPMODEL
    {
        DEF_DETECT_MODEL, ///<
        TURBO_EDIT,
        SLIPMODEL_UNDEF
    };

    //KBR or LRI
    enum KBRLRIMODE
    {
        RANGE,
        RATE,
        RANGE_RATE
    };

    enum IONMODEL
    {
        VION,
        SION,
        DEF_ION,
        IONMODEL_UNDEF
    };

    enum class IFCB_MODEL
    {
        EST,
        COR,
        DEF,
        IFCB_MODEL_UNDEF
    };

    enum class IFB_model
    {
        EST_REC_IFB,
        EST_SAT_IFB,
        EST_IFB,
        COR_REC_IFB,
        COR_SAT_IFB,
        COR_IFB,
        NONE,
        IFB_model_UNDEF
    };

    enum class DCB_model
    {
        CODE,
        CAS,
        NONE,
        DCB_model_UNDEF
    };

    enum class ISFIRSTSOL
    {
        YES,
        NO
    };

    enum class ISCONSPW
    {
        YES,
        NO
    };

    // Hwang Shih added
    enum class OFSTREAM_MODE
    {
        REAL_TIME, // Processing while reading O files
        READ_all,  // Processing after reading all of the O files which is defaulted
        READ_OFILE_MODE_UNDEF
    };
    enum class PCV_MODE
    {
        ZENITH,  // zenith/elevation dependent
        AZIMUTH, // azimuth dependent
        ALL,     // elevation and azimuth dependent
        NONE     // not estimate
    };

    enum class NPP_MODEL
    {
        NO,
        VRS,
        URTK,
        UPD,
        PPP_RTK
    };

    /** @brief GNSS systems and augmentations. */
    enum GSYS
    { // GXX = -1,
        GPS = 1,
        GAL = 2,
        GLO = 3,
        BDS = 4,
        QZS = 5,
        SBS = 6,
        IRN = 7,
        LEO = 8,
        GNS = 999
    };

    /** @brief GNSS freq Sequence ID. */
    enum FREQ_SEQ
    {
        FREQ_1 = 1,
        FREQ_2 = 2,
        FREQ_3 = 3,
        FREQ_4 = 4,
        FREQ_5 = 5,
        FREQ_6 = 6,
        FREQ_7 = 7,
        FREQ_X = 999
    };

    /** @brief GNSS frequencies. */
    enum GFRQ
    { // FXX = -1,
        G01 = 10,
        G02 = 11,
        G05 = 12, // GPS
        R01 = 20,
        R02 = 21, // GLONASS FDMA
        R01_CDMA = 30,
        R02_CDMA = 31,
        R03_CDMA = 32,
        R05_CDMA = 33, // GLONASS CDMA
        E01 = 50,
        E05 = 51,
        E07 = 52,
        E08 = 53,
        E06 = 54, // Galileo
        // BeiDou C02(IQX) C07(7-IQX) C06(IQX) C05(5-DPX)  C09(7-DPZ)  C08(8-DPX)  C01(1-DPX)
        //        B1I      B2I(BDS-2) B3I      B2a(BDS-3)  B2b(BDS-3)  B2(BDS-3)   B1C(BDS-3)
        C02 = 60,
        C07 = 61,
        C06 = 62,
        C05 = 63,
        C09 = 64,
        C08 = 65,
        C01 = 66,
        J01 = 70,
        J02 = 71,
        J05 = 72,
        J06 = 73, // QZSS
        S01 = 80,
        S05 = 81, // SBAS
        I05 = 90, // I09,                // IRNSS
        LAST_GFRQ = 999
    };

    /** @brief GNSS receiver types. */
    enum RECTYPE
    {
        P1P2, // receiver providing C1, P1, P2
        C1X2, // cross-correlation
        C1P2  // modern receivers providing C1, P2
    };

    /** @brief Broadcast messages types. */
    enum GNAVTYPE
    {
        FNAV,
        INAV,
        INAV_E01,
        INAV_E07,
        CNAV,
        NAV
    };

    // enum GNAVTYPE {  NAV_G01, CNAV_G02, CNAV_G05,
    //                  NAV_R01,
    //                  NAV_C02,
    //                  FNAV_E05, INAV_E01, INAV_E07, CNAV_E06, GNAV_E01, GNAV_E06,
    //                  NAV_DEF
    //              };

    /** @brief GNSS type/band/attr definitions. */
    enum GOBSTYPE
    {
        TYPE_C = 1,
        TYPE_L = 2,
        TYPE_D = 3,
        TYPE_S = 4,
        TYPE_P = 101, // only for P-code!
        TYPE_SLR,
        TYPE_KBRRANGE,
        TYPE_LRIRANGE,
        TYPE_KBRRATE,
        TYPE_LRIRATE,
        TYPE = 999 // ""  UNKNOWN
    };
    enum GOBSBAND
    {
        BAND_1 = 1,
        BAND_2 = 2,
        BAND_3 = 3,
        BAND_5 = 5,
        BAND_6 = 6,
        BAND_7 = 7,
        BAND_8 = 8,
        BAND_9 = 9, // Band_9 is std::set for BDS-3 B2b
        BAND_A = 101,
        BAND_B = 102,
        BAND_C = 103,
        BAND_D = 104,
        BAND_SLR,
        BAND_KBR,
        BAND_LRI,
        BAND = 999 // ""  UNKNOWN
    };
    enum GOBSATTR
    {
        ATTR_A = 1,
        ATTR_B = 2,
        ATTR_C = 3,
        ATTR_D = 4,
        ATTR_I = 5,
        ATTR_L = 6,
        ATTR_M = 7,
        ATTR_N = 8,
        ATTR_P = 9,
        ATTR_Q = 10,
        ATTR_S = 11,
        ATTR_W = 12,
        ATTR_X = 13,
        ATTR_Y = 14,
        ATTR_Z = 15,
        ATTR_NULL = 16, // " " 2CHAR code
        ATTR = 999 // ""  UNKNOWN
    };

    /** @brief GNSS observations. */
    enum GOBS
    {

        // psedorange [in meters] (RINEX 3.x)
        C1A = 0,
        C1B = 1,
        C1C = 2,
        C1D = 3,
        C1I = 4,
        C1L = 5,
        C1M = 6,
        C1P = 7,
        C1S = 8,
        C1Q = 9,
        C1W = 10,
        C1X = 11,
        C1Y = 12,
        C1Z = 13,
        C2C = 14,
        C2D = 15,
        C2I = 16,
        C2L = 17,
        C2M = 18,
        C2P = 19,
        C2S = 20,
        C2Q = 21,
        C2W = 22,
        C2X = 23,
        C2Y = 24,
        C3I = 25,
        C3Q = 26,
        C3X = 27,
        C5A = 28,
        C5B = 29,
        C5C = 30,
        C5D = 31,
        C5I = 32,
        C5P = 33,
        C5Q = 34,
        C5X = 35,
        C6A = 36,
        C6B = 37,
        C6C = 38,
        C6I = 39,
        C6L = 40,
        C6S = 41,
        C6Q = 42,
        C6X = 43,
        C6Z = 44,
        C7I = 45,
        C7Q = 46,
        C7X = 47,
        C8D = 48,
        C8I = 49,
        C8P = 50,
        C8Q = 51,
        C8X = 52,
        C9D = 53,
        C9P = 54,
        C9Z = 55, // BDS-3 B2b

        // carrier phase [in whole cycles] (RINEX 3.x)
        L1A = 100,
        L1B = 101,
        L1C = 102,
        L1D = 103,
        L1I = 104,
        L1L = 105,
        L1M = 106,
        L1N = 107,
        L1P = 108,
        L1S = 109,
        L1Q = 110,
        L1W = 111,
        L1X = 112,
        L1Y = 113,
        L1Z = 114,
        L2C = 115,
        L2D = 116,
        L2I = 117,
        L2L = 118,
        L2M = 119,
        L2N = 120,
        L2P = 121,
        L2S = 122,
        L2Q = 123,
        L2W = 124,
        L2X = 125,
        L2Y = 126,
        L3I = 127,
        L3Q = 128,
        L3X = 129,
        L5A = 130,
        L5B = 131,
        L5C = 132,
        L5D = 133,
        L5I = 134,
        L5P = 135,
        L5Q = 136,
        L5X = 137,
        L6A = 138,
        L6B = 139,
        L6C = 140,
        L6I = 141,
        L6L = 142,
        L6S = 143,
        L6Q = 144,
        L6X = 145,
        L6Z = 146,
        L7I = 147,
        L7Q = 148,
        L7X = 150,
        L8D = 151,
        L8I = 152,
        L8P = 153,
        L8Q = 154,
        L8X = 155,
        L9D = 156,
        L9P = 157,
        L9Z = 158, // BDS-3 B2b

        // doppler [cycles/sec] (RINEX 3.x)
        D1A = 200,
        D1B = 201,
        D1C = 202,
        D1D = 203,
        D1I = 204,
        D1L = 205,
        D1M = 206,
        D1N = 207,
        D1P = 208,
        D1S = 209,
        D1Q = 210,
        D1W = 211,
        D1X = 212,
        D1Y = 213,
        D1Z = 214,
        D2C = 215,
        D2D = 216,
        D2I = 217,
        D2L = 218,
        D2M = 219,
        D2N = 220,
        D2P = 221,
        D2S = 222,
        D2Q = 223,
        D2W = 224,
        D2X = 225,
        D2Y = 226,
        D3I = 227,
        D3Q = 228,
        D3X = 229,
        D5A = 230,
        D5B = 231,
        D5C = 232,
        D5D = 233,
        D5I = 234,
        D5P = 235,
        D5Q = 236,
        D5X = 237,
        D6A = 238,
        D6B = 239,
        D6C = 240,
        D6I = 241,
        D6L = 242,
        D6S = 243,
        D6Q = 244,
        D6X = 245,
        D6Z = 246,
        D7I = 247,
        D7Q = 248,
        D7X = 249,
        D8D = 250,
        D8I = 251,
        D8P = 252,
        D8Q = 253,
        D8X = 254,
        D9D = 255,
        D9P = 256,
        D9Z = 257, // BDS-3 B2b

        // signal strength [DBHZ] (RINEX 3.x)
        S1A = 300,
        S1B = 301,
        S1C = 302,
        S1D = 303,
        S1I = 304,
        S1L = 305,
        S1M = 306,
        S1N = 307,
        S1P = 308,
        S1S = 309,
        S1Q = 310,
        S1W = 311,
        S1X = 312,
        S1Y = 313,
        S1Z = 314,
        S2C = 315,
        S2D = 316,
        S2I = 317,
        S2L = 318,
        S2M = 319,
        S2N = 320,
        S2P = 321,
        S2S = 322,
        S2Q = 323,
        S2W = 324,
        S2X = 325,
        S2Y = 326,
        S3I = 327,
        S3Q = 328,
        S3X = 329,
        S5A = 330,
        S5B = 331,
        S5C = 332,
        S5D = 333,
        S5I = 334,
        S5P = 335,
        S5Q = 336,
        S5X = 337,
        S6A = 338,
        S6B = 339,
        S6C = 340,
        S6I = 341,
        S6L = 342,
        S6S = 343,
        S6Q = 344,
        S6X = 345,
        S6Z = 346,
        S7I = 347,
        S7Q = 348,
        S7X = 349,
        S8D = 350,
        S8I = 351,
        S8P = 352,
        S8Q = 353,
        S8X = 354,
        S9D = 355,
        S9P = 356,
        S9Z = 357, // BDS-3 B2b

        // special cases: v2.x or unknown tracking modes
        P1 = 1000,
        P2 = 1001,
        P5 = 1002,
        C1 = 1003,
        C2 = 1004,
        C5 = 1005,
        C6 = 1006,
        C7 = 1007,
        C8 = 1008,
        CA = 1009,
        CB = 1010,
        CC = 1011,
        CD = 1012,

        L1 = 1100,
        L2,
        L5,
        L6,
        L7,
        L8,
        LA,
        LB,
        LC,
        LD,

        D1 = 1200,
        D2,
        D5,
        D6,
        D7,
        D8,
        DA,
        DB,
        DC,
        DD,

        S1 = 1300,
        S2,
        S5,
        S6,
        S7,
        S8,
        SA,
        SB,
        SC,
        SD,

        X = 9999 // LASgnss_data_obs
    };

    /** @brief GOBS LC. */
    enum GOBS_LC
    {
        LC_L1 = 1,
        LC_L2 = 2,
        LC_L3 = 3,
        LC_L4 = 4,
        LC_L5 = 5,
        LC_IF = 6,
        LC_MW = 7,
        LC_NL = 8,
        LC_WL = 9,
        LC_GF = 10,
        LC_UNDEF = 999
    };
};


#endif
