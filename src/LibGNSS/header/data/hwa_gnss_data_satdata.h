#ifndef hwa_gnss_data_satdata_h
#define hwa_gnss_data_satdata_h

#include "hwa_set_gproc.h"
#include "hwa_base_data.h"
#include "hwa_base_const.h"
#include "hwa_base_time.h"
#include "hwa_gnss_data_obsmanager.h"
#include "hwa_gnss_all_Nav.h"
#include "hwa_gnss_all_prec.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss 
{
    /** @brief class for satellite data. */
    class gnss_data_sats : public gnss_data_obs_manager
    {

    public:
        /** @brief default constructor. */
        gnss_data_sats();

        gnss_data_sats(base_log spdlog);

        gnss_data_sats(base_log spdlog,
                   const std::string &site,
                   const std::string &sat,
                   const base_time &t);

        gnss_data_sats(base_log spdlog,
                   const std::string &site,
                   const std::string &sat,
                   const base_time &t,
                   const bool &isleo);

        /** @brief copy constructor 3. */
        explicit gnss_data_sats(const gnss_data_obs_manager &obs);

        /** @brief default destructor. */
        virtual ~gnss_data_sats();

        /** @brief add satellite position. */
        void addcrd(const Triple &crd);

        /** @brief add satellite position in crs. */
        void addcrdcrs(const Triple &crd);

        /** @brief add satellite pco. */
        void addpco(const Triple &pco);

        /** @brief add satellite velocity. */
        void addvel(const Triple &vel);

        void addvel_crs(const Triple &vel);

        /** @brief add satellite clocks at the transmision time. */
        void addclk(const double &d);

        /** @brief add satellite relative delay. */
        void addreldelay(const double &d);

        /** @brief add receiver postion. */
        void addreccrd(const Triple &crd);

        /** @briefadd reciver postion in crs. */
        void addreccrdcrs(const Triple &crd);

        /** @brief add add the TRS2CRS Matrix. */
        void addTRS2CRS(const Matrix &rotmat, 
            const Matrix &drdxpole, 
            const Matrix &drdypole, 
            const Matrix &drdut1);

        /** @brief add add the TRS2CRS Matrix. */
        void addSCF2CRS(const Matrix &scf2crs, const Matrix &scf2trs);

        /** @brief add add the orb funct. */
        void addorbfunct(const Matrix &orbfunct);

        /** @brief add add the rate. */
        void adddrate(const double &drate);

        /** @brief add add the loudx. */
        void adddloudx(const Triple &unit);

        /** @brief add add the sat index. */
        void addsatindex(const int &idx);

        /** @brief add add the rec Time. */
        void addrecTime(const base_time &recTime);

        /** @brief add add the sat Time. */
        void addsatTime(const base_time &satTime);

        /** @brief determine wheather the satellite is eclipsed (postshadow is considered). */
        void addecl(std::map<std::string, base_time> &lastEcl);

        /** @brief reset the eclipsing time. */
        void setecl(const bool &ecl);

        /** @brief add reciver postion in crs. */
        void addsat2reccrs(const Triple &crd);

        /** @brief true  to support PPP/SPP (std::set by INP:chk_health). */
        int addprd(gnss_all_nav *gnav, const bool &corrTOT = true, const bool &msk_health = true);

        /**
         * @brief 
         * 
         * @param gnav 
         * @param corrTOT 
         * @param msk_health 
         * @return int 
         */
        int addprd_realtime(gnss_all_nav *gnav, const bool &corrTOT = true, const bool &msk_health = true); ///< added by zhshen

        /**
         * @brief 
         * 
         * @param T_sat 
         * @param gnav 
         * @param corrTOT 
         * @param msk_health 
         * @return int 
         */
        int addprd(const base_time &T_sat, 
            gnss_all_nav *gnav, 
            const bool &corrTOT = true, 
            const bool &msk_health = true); ///< add by glfeng

        /** @brief false to support QC (combines INP:chk_health+QC:use_health for Anubis). */
        int addprd_nav(gnss_all_nav *gnav, const bool &corrTOT = true, const bool &msk_health = false);

        /**
         * @brief 
         * 
         * @param xyz 
         * @return int 
         */
        int cmpVal(const Triple &xyz);

        /** @brief add satellite elevation. */
        void addele(const double &d);

        /** @brief add satellite elevation. */
        void addele_leo(const double &d);

        /** @brief add receiver azimuth. */
        void addazi_rec(const double &d);

        /** @brief add receiver zenith. */
        void addzen_rec(const double &d);

        /** @brief add satellite-side azimuth ang, added by yqyuan. */
        void addazi_sat(const double &d);

        /** @brief add satellite-side zenith ang*/
        void addzen_sat(const double &d);

        /** @brief add satellite-side nadir angle, added by yqyuan. */
        void addnadir(const double &d);

        /** @brief add satellite rho-std::vector. */
        void addrho(const double &d);

        /** @brief add res. */
        void addres(const RESIDTYPE &restype, const GOBSTYPE &type, const double &res);

        /** @brief add mfH. */
        void addmfH(const double &d);

        /** @brief add mfW. */
        void addmfW(const double &d);

        /** @brief add mfG. */
        void addmfG(const double &d);

        /** @brief add wind. */
        void addwind(const double &d);

        /** @brief get satellite position. */
        const Triple &satcrd() const;

        /** @brief get satellite position crs. */
        const Triple &satcrdcrs() const;

        /** @brief get satellite velocity. */
        const Triple &satpco() const;

        /**
         * @brief 
         * 
         * @return const Triple& 
         */
        const Triple &satvel() const;

        /**
         * @brief 
         * 
         * @return const Triple& 
         */
        const Triple &satvel_crs() const; ///< add by glfeng

        /** @brief get satellite clocks at the transmision time. */
        const double &clk() const;

        /** @brief get satellite clocks drift at the transmision time. */
        const double &dclk() const;

        /**
         * @brief 
         * 
         * @return int 
         */
        int satindex(); ///< get sat index

        /** @brief get satellite relative delay. */
        const double &reldelay() const;

        /**
         * @brief 
         * 
         * @return const double& 
         */
        const double &drate() const;

        /**
         * @brief 
         * 
         * @return const Triple& 
         */
        const Triple &dloudx() const; ///< add by glfeng

        /**
         * @brief 
         * 
         * @return const Triple& 
         */
        const Triple &reccrd() const; ///< get reciver coord

        /**
         * @brief 
         * 
         * @return const Triple& 
         */
        const Triple &reccrdcrs() const; ///< get reciver coord crs

        /**
         * @brief 
         * 
         * @return const Triple& 
         */
        const Triple &sat2reccrs() const; ///< get reciver coord

        /**
         * @brief 
         * 
         * @return const Matrix& 
         */
        const Matrix &orbfunct() const; ///< get satellite funct

        /**
         * @brief 
         * 
         * @return const Matrix& 
         */
        const Matrix &rotmat() const;

        /**
         * @brief 
         * 
         * @return const Matrix& 
         */
        const Matrix &drdxpole() const;

        /**
         * @brief 
         * 
         * @return const Matrix& 
         */
        const Matrix &drdypole() const;

        /**
         * @brief 
         * 
         * @return const Matrix& 
         */
        const Matrix &drdut1() const;

        /**
         * @brief 
         * 
         * @return const Matrix& 
         */
        const Matrix &scf2crs() const;

        /**
         * @brief 
         * 
         * @return const Matrix& 
         */
        const Matrix &scf2trs() const;

        /**
         * @brief 
         * 
         * @return const base_time& 
         */
        const base_time &recTime() const; ///< get the site receiver time

        /**
         * @brief 
         * 
         * @return const base_time& 
         */
        const base_time &satTime() const; ///< get the satellite send time

        /**
         * @brief 
         * 
         * @return const double& 
         */
        const double &ele() const; ///< get satellite elevation

        /**
         * @brief 
         * 
         * @return const double& 
         */
        const double &ele_leo() const; ///< get satellite elevation

        /**
         * @brief 
         * 
         * @return double 
         */
        double ele_deg() const; ///< get satellite elevation [deg]

        /**
         * @brief 
         * 
         * @return double 
         */
        double ele_leo_deg() const; ///< get satellite elevation [deg]

        /**
         * @brief 
         * 
         * @return const double& 
         */
        const double &azi() const; ///< get satellite azimuth

        /**
         * @brief 
         * 
         * @return const double& 
         */
        const double &azi_sat() const; ///< get satellite-side azimuth ang, added by yqyuan

        /**
         * @brief 
         * 
         * @return const double& 
         */
        const double &nadir() const; ///< get satellite-side nadir angle, added by yqyuan

        /**
         * @brief 
         * 
         * @return const double& 
         */
        const double &rho() const; ///< get satellite rho-std::vector

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        const bool &ecl() const; ///< get eclipsing

        /**
         * @brief 
         * 
         * @return double 
         */
        const double &mfH() const;

        /**
         * @brief 
         * 
         * @return double 
         */
        const double &mfW() const;

        /**
         * @brief 
         * 
         * @return double 
         */
        const double &mfG() const;

        /**
         * @brief 
         * 
         * @return double 
         */
        const double &wind() const;

        /**
         * @brief 
         * 
         * @param flag 
         */
        void addslip(const bool &flag);

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        const bool &islip() const;

        /**
         * @brief 
         * 
         * @return double 
         */
        double beta(); ///< Sun elevation relative to orbital plane

        /**
         * @brief 
         * 
         * @return double 
         */
        double orb_angle(); ///< Orbit angle

        /**
         * @brief 
         * 
         * @return double 
         */
        const double &yaw() const { return _yaw; } ///< get yaw angle

        /**
         * @brief 
         * 
         * @param yaw 
         */
        void yaw(const double &yaw) { _yaw = yaw; } ///< std::set yaw angle

        /**
         * @brief 
         * 
         * @param restype 
         * @param type 
         * @return std::vector<double> 
         */
        std::vector<double> residuals(const RESIDTYPE &restype, const GOBSTYPE &type);

        /**
         * @brief 
         * 
         * @param restype 
         */
        void clear_res(const RESIDTYPE &restype);

        /**
         * @brief 
         * 
         */
        void clear();

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool valid();

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        const bool &tb12() const { return _isTb12Valid; };

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        const bool &tb13() const { return _isTb13Valid; };

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        const bool &tb14() const { return _isTb14Valid; };

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        const bool &tb15() const { return _isTb15Valid; };

        /**
         * @brief 
         * 
         * @param band 
         * @return true 
         * @return false 
         */
        bool is_carrier_range(const GOBSBAND& band) const;


        /**
         * @brief 
         * 
         * @param tb12 
         */
        void tb12(const bool &tb12) { _isTb12Valid = tb12; };

        /**
         * @brief 
         * 
         * @param tb13 
         */
        void tb13(const bool &tb13) { _isTb13Valid = tb13; };

        /**
         * @brief 
         * 
         * @param tb14 
         */
        void tb14(const bool &tb14) { _isTb14Valid = tb14; };

        /**
         * @brief 
         * 
         * @param tb15 
         */
        void tb15(const bool &tb15) { _isTb15Valid = tb15; };

        /**
         * @brief 
         * 
         * @param band 
         * @param b 
         */
        void is_carrier_range(const GOBSBAND &band, const bool &b);

        Triple _conf_crd; ///< velocity direction std::vector
        Triple _e;        ///< direction std::vector
        bool is_process = false;  ///< cmb_equ process flag, hyChang add

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        const bool &exisgnss_coder_aug() const { return _isExistAug; };

        /**
         * @brief 
         * 
         * @param aug 
         */
        void exisgnss_coder_aug(const bool &aug) { _isExistAug = aug; };

    private:
        /** @brief add satellite pos, clk and ecl (corrTOT is correction of transmition time). */
        int _addprd(gnss_all_nav *gnav, const bool &corrTOT = true, const bool &msk_health = true);

        /**
         * @brief 
         * 
         * @param gnav 
         * @param corrTOT 
         * @param msk_health 
         * @return int 
         */
        int _addprd_realtime(gnss_all_nav *gnav, const bool &corrTOT = true, const bool &msk_health = true); // added by zhshen

        int  _addprd_realtime_new(gnss_all_nav* gnav, bool corrTOT = true, bool msk_health = true);          // added by hjj from tyx

        /**
         * @brief 
         * 
         * @param T_sat 
         * @param gnav 
         * @param corrTOT 
         * @param msk_health 
         * @return int 
         */
        int _addprd(const base_time &T_sat, gnss_all_nav *gnav, const bool &corrTOT = true, const bool &msk_health = true); //add by glfeng

        /**
         * @brief 
         * 
         * @param xyz 
         * @param vxyz 
         * @param clk 
         * @param dclk 
         * @param xyz_corr 
         * @param vxyz_corr 
         * @param clk_corr 
         * @param dclk_corr 
         * @return int 
         */
        int _correction(double *xyz, double *vxyz,
                        double &clk, double &dclk,
                        double *xyz_corr, double *vxyz_corr,
                        double &clk_corr, double &dclk_corr);

        /**
         * @brief 
         * 
         * @return double 
         */
        double _b();

        /**
         * @brief 
         * 
         * @return double 
         */
        double _orb_angle();

        /**
         * @brief 
         * 
         */
        virtual void _clear() override;

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        virtual bool _valid() const override;

        Triple _satcrd;    ///< satellite position (X,Y,Z)
        Triple _satcrdcrs; ///< satellite position in crs(X,Y,Z)
        Triple _satpco;    ///< satellite pco
        Triple _satvel;    ///< satellite velocity
        Triple _satvel_crs;
        Matrix _orbfunct;      ///< satellite funct
        int _satindex;         ///< satellite index in lsq
        Triple _reccrd;     ///< reciver position(TRS)
        Triple _reccrdcrs;  ///< reciver position(TRS)
        Triple _sat2reccrs; ///< reciver position(TRS)
        Matrix _rotmat;
        Matrix _drdxpole;
        Matrix _drdypole;
        Matrix _drdut1;
        Matrix _scf2crs;
        Matrix _scf2trs;
        double _drate;
        Triple _dloudx;
        base_time _TR;            ///< site recieve time
        base_time _TS;            ///< satellite sends
        double _clk = 0.0;      ///< satellite clocks (precise, time of transmision)
        double _dclk = 0.0;     ///< satellite clocks drift (precise, time of transmision)
        double _reldelay = 0.0; ///< satellite releative delay
        double _ele = 0.0;      ///< satellite elevation
        double _ele_leo = 0.0;  ///< satellite elevation
        double _azi_rec = 0.0;  ///< satellite azimuth
        double _azi_sat = 0.0;  ///< azimuth at satellite-side, added by yqyuan
        double _zen_rec = 0.0;  ///< satellite zenith
        double _zen_sat = 0.0;  ///< zenith at satellite-side
        double _nadir = 0.0;    ///< nadir angle at satellite-side, added by yqyuan
        double _rho = 0.0;      ///< satellite-station geometrical distance
        bool _eclipse = 0.0;    ///< eclipsed satellite
        double _mfH = 0.0;
        double _mfW = 0.0;
        double _mfG = 0.0;
        double _wind = 0.0;
        bool _low_prec = 0.0;                  ///< low precision of sat pos
        bool _slipf = 0.0;                     ///< cycle slip flag
        bool _isTb12Valid = false;             ///< jdhuang : add to mark is avial in tb log 12;
        bool _isTb13Valid = false;             ///< jdhuang : add to mark is avial in tb log 13;
        bool _isTb14Valid = false;             ///< jdhuang : add to mark is avial in tb log 14;
        bool _isTb15Valid = false;             ///< jdhuang : add to mark is avial in tb log 15;
        std::map<GOBSBAND, bool> _is_carrier_range; ///< jqwu : add to mark carrier-range
        bool _isExistAug = false;

        // normalized residuals
        std::vector<double> _code_res_norm;
        std::vector<double> _phase_res_norm;

        // original residuals
        std::vector<double> _code_res_orig;
        std::vector<double> _phase_res_orig;
        int               _pv_iod = -1;         // tyx : add to mark IOD of nav which is used to calculate satpos
        int               _clk_iod = -1;        // tyx : add to mark IOD of nav which is used to calculate satclk

        double _beta_val = 0.0;
        double _orb_angle_val = 0.0;
        double _yaw = 0.0;
    };

} // namespace

#endif
