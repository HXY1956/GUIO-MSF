#ifndef hwa_ins_base_utility_h
#define hwa_ins_base_utility_h

#include "hwa_set_base.h"
#include "hwa_set_ins.h"
#include "hwa_set_ign.h"
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include "windows.h"
#include <string>
#include "hwa_base_globaltrans.h"

namespace hwa_ins
{
#define UNICORE_ACC_SCALE 400 / pow(2, 31)     //m/s/LSB
#define UNICORE_GYRO_SCALE 2160 / pow(2, 31) //deg/LSB
#define hwa_ACC_SCALE          400 / pow(2,31)    //m/s/LSB
#define hwa_GYRO_SCALE         2160 / pow(2,31)   //deg/LSB
#define STARNETO_ACC_SCALE 0.05 / pow(2, 15)
#define STARNETO_GYRO_SCALE 0.1 / (3600 * 256.0)
#define STARNETO_G 9.80144145
#define IMR_FSAS_GYRO_SCALE 0.1 / pow(2, 8)      // arcsec/LSB
#define IMR_FSAS_ACCE_SCALE 0.05 / pow(2, 15) // m/s/LSB

    typedef long long int ImuStateIDType;

    struct ImuState
    {
        ImuStateIDType id;                // id of imu state,not use
        double time;                    // timstamp of imu state,not use
        Eigen::Quaterniond orientation; // attitude of imu state
        Triple position;        // position of imu state

        ImuState() : id(0), time(0),
            orientation(Eigen::Quaterniond::Identity()),
            position(Triple::Zero()) {}
    };

    struct ins_scheme
    {
        double t = 0.0; /// add:now time for Unequal intervals
        double ts = 0.0;
        double T = 0.0;
        int freq = 0; /// IMU Freq
        double start = 0.0;
        double end = 0.0;        /// start time and end time.
        double delay = 0.0;        /// the GNSS misalignment time diff with imu.
        double delay_odo = 0.0; /// the ODO misalignment time diff with imu.
        double max_pdop = 0.0;    /// the max PDOP threshold for LCI.
        int min_sat = 0;        /// the minimum satellite number threshold.
        int align_time = 0;        /// coarse align time.
        int nSamples = 0;
        int Cps = 0;
        bool align = false;
        int GM = 0;
        bool _imu_scale = false;
        bool _imu_inst_rot = false;
        bool _imu_inst_trans = false;
        bool Status = true;

        ins_scheme();
        ins_scheme(hwa_set::set_base* set);
    };

    double Lagrange(double t[], double x[], int n, double t1);
    int sign(double d);
    void removeRow(Matrix& matrix, unsigned int rowToRemove);
    void removeRow(Vector& matrix, unsigned int rowToRemove);
    void removeColumn(Matrix& matrix, unsigned int colToRemove);
}

#endif
