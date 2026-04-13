#ifndef hwa_base_data_h
#define hwa_base_data_h

#include "hwa_base_log.h"
#include "hwa_base_mutex.h"
#include "hwa_base_note.h"
#include "hwa_base_common.h"

namespace hwa_base
{
    class base_data
    {

    public:
        explicit base_data();
        /** @brief copy constructor. */
        explicit base_data(const base_data& data);
        explicit base_data(base_log spdlog);

        /** @brief default destructor. */
        virtual ~base_data();

        /** @brief override operator =. */
        base_data& operator=(const base_data& data);

        /** @brief data type */
        enum ID_TYPE
        {
            NONE,    ///< = 0,  none
            OBJ,     ///< = 1,  object
            TRN,     ///< = 2,  transmitter
            REC,     ///< = 3,  receiver
            REC_LEO, ///        receiver in LEO
            FIL,     ///< = 4,  file

            OBS,     ///< = 10, obseravation base
            OBSGNSS, ///< = 11, gnss observations
            SATDATA, ///< = 12, gnss observations + satellite data

            QCDATA, ///< = xx, data quality control
            QCPROD, ///< = xx, prod quality control

            EPH,     ///< = 20, navigation base
            EPHGPS,  ///< = 21, navigation
            EPHGLO,  ///< = 22, navigation
            EPHGAL,  ///< = 23, navigation
            EPHQZS,  ///< = 24, navigation
            EPHBDS,  ///< = 24, navigation
            EPHSBS,  ///< = 25, navigation
            EPHIRN,  ///< = 26, navigation
            EPHPREC, ///< = 27, sp3/clocks
            EPHRTCM, ///< = 28, navigation + RTCM

            GRID,    ///< = xx, regular data grid
            GEOBASE, ///< = xx,   geoid data grid
            NWMBASE, ///< = xx, surface data grid
            NWMSURF, ///< = xx, surface data grid
            NWMPROF, ///< = xx, profile data grid

            RESOBS, ///< = xx, observation residuals
            RESPAR, ///< = xx, parameter residuals

            ALLGIO,  ///< = xx  , all files
            ALLNAV,  ///< = 28, all navigation all
            ALLPREC, ///< = 29, all sp3 + rinexc
            ALLORB,
            ALLRTCM,    ///< = 30, all rtcm + nav
            ALLOBS,     ///< = 31, all observations
            ALLOBJ,     ///< = 32, all objects
            ALLPCV,     ///< = 33, all PCV
            ALLOTL,     ///< = 34, all OTL
            ALLVLBIOTL, ///< = xx, all OTL for VLBI stations
            ALLOPL,     ///< = xx, all ocean pole looad
            ALLATL,     ///< = xx, all ATL
            ALLVLBIATL,
            ALLANTL,         ///< = xx, all ANTL
            ALLSURF,         ///< = xx, all NWM SURF
            ALLPROF,         ///< = xx, all NWM PROF
            ALLPROD,         ///< = xx, all PROD
            ALLBIAS,         ///< = xx, all PROD
            ALLPANNEL,       ///< = xx, pannel
            ALLSOLAR,        ///<
            ALLPOLEUT1,      ///< = xx, poleut
            ALLPOLEUT1_VLBI, ///< = xx, poleut VLBI
            ALLVLIBIATL,     ///<
            AllATTITUDE,     ///< = xx, attitude
            ALLRECOVER,      ///< = xx, recover
            ALLALBEDO,       ///< = xx, albedo
            ALLLRA,          ///< = xx, ratio
            ALLCOM,          ///< = xx, combination
            ALLKBR,          ///< = xx, kbr
            ALLLRI,          ///< = xx, llri
            ALLEOP,          ///< = xx, eop
            ALLRB,           ///< = xx, lrb
            ALLSTA,          ///< = xx, sta
            ALLNPT,          ///< = xx, npt
            ALLNGS,          ///< = xx, ngs
            ALLPREOBS,       ///< = xx, pre observation
            ALLPREproc,      ///< = xx, pre process
            ALLIMUDATA,
            ALLOBX,          ///< = xx, obx attitude
            STRBUFF, ///< = xx, generic product (ASCII string) for strbuff encoder
            POS,     ///< = 35, XYZT position/time
            POST,    ///< = 36, SP3 + CLOCKS (satellite position)
            MET,     ///< = xx, meteorological parameters
            TRP,     ///< = 37, tropospheric parameters (ztd+gradients)
            TRPSLT,  ///< = 38, tropospheric parameters (slant delays)
            CLK,     ///< = 37, clocks
            ION,     ///< = xx, ionospheric parameters
            IONEX,   ///< = xx, ionospheric delay from tec grid products (GIM)

            PCV,  ///< = 40, PCV model
            OTL,  ///< = 41, ocean loading model
            ATL,  ///< = 42, atmosphere tidal load
            ANTL, ///< = 42, atmosphere non tidal load
            OPL,
            BIAS, ///< = 42, code & phase biases
            ERP,  ///< = 43, Earth orientation model

            SOL,         ///< = 50  solution
            IMUDATA,     ///< = xx, imu data
            ODODATA,     ///< = xx, odo data
            LCI_POS,     ///< = xx, lci position
            CAMDATA,     ///< = xx, cam data
            LIDARDATA,   ///< = xx, lidar data
            FEATUREDATA, ///< = xx, feature data
            UPD,         ///< add for upd
            AMBFLAG,     ///< = xx, ambiguity flag of 12_frequency
            AMBFLAG13,   ///< = xx, ambiguity flag of 13_frequency
            AMBFLAG14,   ///< = xx, ambiguity flag of 14_frequency
            AMBFLAG15,   ///< = xx, ambiguity flag of 15_frequency
            AMBUPD,      ///< = xx, ambiguity upd
            //UPD_EPOCH,
            IFCB,   ///< = xx, Inter-Frequency Clock Bias
            AMBINP, ///< = xx, ambinp file for ambfix
            AMBCON, ///< = xx, amb constraint

            ALLDE,      ///< = xx, all planeteph
            ALLICS,     ///< = xx, ics
            ALLICSLEO,  ///< = xx, ics of leo
            ALLSUM,     ///< = xx, sum
            ALLRESULT,  ///< = xx, result
            ALLVLBISTA, ///< = xx, vlbi station all
            ALLVLBISRC, ///< = xx, vlbi src        all
            ALLVLBICLK, ///< = xx, vlbi clk        all
            ALLMORBCLK, ///< = xx, orbit and clk monit

            AUG, ///< = xx, aug file
            AUGGRID,

            EGM,              ///< = xx, EGM20008 file
            LEAPSECOND,       ///< = xx, leap Second
            OCEANTIDE,        ///< = xx, ocean tide
            DESAISCOPOLECOEF, ///< = xx, ocean tide
            ORB,              ///< = xx, orb file
            SATPARS,          ///< = xx, sat_parameters_new file
            NPT,              ///< = xx, npt
            SLRF,             ///< = xx, slrf
            NGS,              ///< = xx, ngs
            KBROMC,           ///< = xx, kbr omc
            VLBISTA,          ///< = xx, vlbi sta
            VLBISTACOR,       ///< = xx, vlbi stacor
            VLBISOURCE,       ///< = xx, vlbi source
            VLBICLK,          ///< = xx, vlbi clk
            ALLACC,           ///< = xx, acc
            ALLPSD,           ///< = xx, PSD model
            ALLMANEUVER,      ///< = xx, orbit maneuver information
            PCVNEQ,           ///< = xx, PCV NEQ information
            ALLPCVNEQ,        ///< = xx, all PCV NEG
            UWBDATA,          ///< = xx, UWB data
            TAGINFO,          ///< = xx, visual tag
            RSSI,             ///< = xx, rssi
            RSSIMAP,          ///< = xx, rssi std::map
            ALLIPP,           ///< = xx, all Ionosphere puncture point information
            ATTDATA,          ///< = xx, body attitude information
            LAST              ///< = xx, last
        };

        /** @brief group type */
        enum ID_GROUP
        {
            GRP_NONE,    ///< = 0,  // none
            GRP_obsERV,  ///< = 10, // observations
            GRP_EPHEM,   ///< = 20, // ephemerides
            GRP_prodUCT, ///< = 30, // positions/products
            GRP_model,   ///< = 40, // models
            GRP_SOLUT,   ///< = 50, // solutions
            GRP_objECT,  ///< = 60, // objects
            GRP_GRID,    ///< = 70, // grid data
            GRP_GIO,     ///< = 80, // gio
            GRP_QC,      ///< = 90, // quality control
            GRP_IMU,     ///< = xx, imu
            GRP_MONIT,   ///< = 100 // monit for gnss
            GRP_LAST     ///< = xx, last

        };

        /** @brief show the information */
        virtual std::string show(int verb);

        /** @brief get data type */
        const ID_TYPE& id_type() const { return _type; }

        /** @brief get group type */
        const ID_GROUP& id_group() const { return _group; }

        /** @brief get data type in std::string format */
        std::string str_type() const;

        /** @brief get group type in std::string format */
        std::string str_group() const;

        /** @brief convert data type to std::string */
        static std::string type2str(ID_TYPE type);

        /** @brief lock mutex */
        void lock() const { this->_gmutex.lock(); }

        /** @brief unlock mutex */
        void unlock() const { this->_gmutex.unlock(); }

        /** @brief std::set glog pointer */
        void spdlog(base_log spdlog);

        /** @brief get glog pointer */
        base_log spdlog() const { return _spdlog; }

        /** @brief std::set gnote pointer */
        void gnote(base_note_all* n) { _gnote = n; }

        /** @brief get gnote pointer */
        base_note_all* gnote() const { return _gnote; }

    protected:
        /**
         * @brief
         *
         * @param t
         * @return int
         */
        int id_type(const ID_TYPE& t); ///< data type

        /**
         * @brief
         *
         * @param g
         * @return int
         */
        int id_group(const ID_GROUP& g); ///< group type

        mutable base_mutex _gmutex; ///< mutex
        ID_TYPE _type;            ///< type_ID
        ID_GROUP _group;          ///< group_ID
        base_note_all* _gnote;       ///< gnote
        base_log _spdlog;         ///< spdlog pointer
    private:
    };
}

#endif