#ifndef hwa_base_posdata_h
#define hwa_base_posdata_h

#include "hwa_base_data.h"
#include "hwa_base_eigendef.h"

namespace hwa_base
{
    class base_posdata : public base_data
    {
    public:
        /** @brief default constructor. */
        base_posdata();

        base_posdata(base_log spdlog);
        /** @brief default destructor. */
        ~base_posdata();

        /**
        * @struct data_pos
        * @brief describe pos data information
        */
        struct data_pos
        {
            double t;             ///< time
            Triple pos;  ///< position
            Triple vn;   ///<
            Triple Rpos; ///<
            Triple Rvn;  ///<
            double PDOP;          ///< the value of PDOP
            int nSat;             ///< the number of satellite
            bool amb_state;       ///< the state of ambiguity
            double sigma0_2;      ///< the value of sigma0^2 (added by tyx for cps)
            bool operator<(const data_pos &dp) const
            { ///< override operator <
                return (t < dp.t);
            }
            bool operator<(const double &d) const
            { ///< override operator <
                return (t < d);
            }
            bool operator>(const double &d) const
            { ///  override operator >
                return (t > d);
            }
            void operator=(const double &d)
            { ///< override operator =
                t = d;
                pos = Triple::Zero();
                vn = Triple::Zero();
            }
        };

        struct data_dd
        {
            Matrix A;
            Matrix P;
            Vector l;
            std::map<std::vector<std::string>, std::vector<Triple>> sat;
            std::map<std::vector<std::string>, std::vector<Triple>> ant;
        };

        struct rtk_pos
        {
            double t;                  ///< time
            Triple baseline;  ///< baseline
            Triple Rbaseline; ///< baseline rms
            Triple xbase;     ///< base station coordinate
            double baseline_len;       ///< baseline length
            double PDOP;               ///< the value of PDOP
            int nSat;                  ///< the number of satellite
            bool amb_state;            ///< the state of ambiguity
            double sigma0_2;           ///< the value of sigma0^2
            bool operator<(const rtk_pos &dp) const
            { ///< override operator <
                return (t < dp.t);
            }
            bool operator<(const double &d) const
            { ///< override operator <
                return (t < d);
            }
            bool operator>(const rtk_pos &dp) const
            { ///  override operator >
                return (t > dp.t);
            }
            bool operator>(const double &d) const
            { ///  override operator >
                return (t > d);
            }
            bool operator==(const rtk_pos &dp)
            { ///< override operator =
                return t == dp.t;
            }
            bool operator==(const double &d)
            { ///< override operator =
                return t == d;
            }
        };

        /**
        * @brief add one data into std::vector
        * @note data includes time and position and velocity
        * @param[in] t            time
        * @param[in] pos        position
        * @param[in] v            velocity
        * @return int            value reprents success or failure of add
        */
        int add_pos(const double &t, const Triple &pos, const Triple &v);

        /**
        * @brief add one data into std::vector
        * @note data includes time and position and velocity
        * @param[in] t
        * @param[in] pos
        * @param[in] v
        * @return int
        */
        int add_pos(const double &t, const Triple &pos, const Triple &v,
                    const Triple &R_pos, const Triple &R_v, const double &pdop, const int &n);

        /**
        * @brief add one data into std::vector
        * @note data includes time and position and velocity
        * @param[in] t
        * @param[in] pos
        * @param[in] v
        * @return int
        */
        int add_pos(const double &t, const Triple &pos, const Triple &v,
                    const Triple &R_pos, const Triple &R_v, const double &pdop, const int &n, const bool &state);

        /**
        * @brief load data into pos and v and t accroding to direction
        * @note direction stands for loading direction
        * @param[in] direction    direction = forward(true)/backward(false)
        * @param[out] pos
        * @param[out] v
        * @param[out] t
        */
        void load(double &t, Triple &pos, Triple &v, bool direction);

        /**
        * @brief load data into pos and v and t accroding to direction
        * @param[in] t1    find the data nearest to t1 
        * @param[out] pos
        * @param[out] v
        * @param[out] t
        */
        void load(double &t, Triple &pos, Triple &v, bool direction, double t1);

        /**
        * @brief load data into pos and v and t accroding to direction
        * @param[in] t1    find the data nearest to t1
        * @param[out] pos        position
        * @param[out] v            velocity
        * @param[out] t
        * @param[out] Qpos        position variance
        * @param[out] Qv        velocity variance
        * @param[out] PDOP        PDOP
        * @param[out] nSat        number of satellite
        */
        void load(double &t, Triple &pos, Triple &v, Triple &Qpos, Triple &Qv,
                  double &PDOP, int &nSat, bool direction, double t1);

        /**
        * @brief load data into pos and v and t accroding to direction
        * @param[in] t1    find the data nearest to t1
        * @param[out] pos        position
        * @param[out] v            velocity
        * @param[out] t
        * @param[out] Qpos        position variance
        * @param[out] Qv        velocity variance
        * @param[out] PDOP        PDOP
        * @param[out] nSat        number of satellite
        */
        void load(data_pos &data, double t1);

        /**
        * @brief judge the info valid
        * @note data includes time and position and velocity
        * @param[in] t
        * @param[in] pos
        * @param[in] tot
        * @param[in] threshold        check the data is valid or not by threshold
        * @return true or false is valid or invalid
        */
        static bool IsValid(const double &t, const Triple &pos, const double &tot, const double &threshold);

        virtual double beg_obs(); // get first epoch
        virtual double end_obs(); // get end epoch

    private:
        std::vector<data_pos> _vecpos; /// latitude and longitude are saved by radian
        std::set<data_pos> _setpos;    /// latitude and longitude are saved by radian
        int _ptr;
    };

}

#endif