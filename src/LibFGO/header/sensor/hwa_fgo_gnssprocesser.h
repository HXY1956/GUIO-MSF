#ifndef hwa_fgo_gnss_processer_h
#define hwa_fgo_gnss_processer_h

#include "hwa_set_ign.h"
#include "hwa_base_allproc.h"
#include "hwa_fgo_baseprocesser.h"
#include "hwa_gnss_amb_manager.h"
#include "hwa_gnss_proc_lsq.h"
#include "hwa_gnss_proc_pppflt.h"
#include "hwa_gnss_proc_pvtflt.h"
#include "hwa_fgo_factor_carrierphase_dd.h"
#include "hwa_fgo_factor_carrierphase_dd_integration.h"
#include "hwa_fgo_factor_initial_gnss_amb.h"
#include "hwa_fgo_factor_pseudorange_dd.h"
#include "hwa_fgo_factor_pseudorange_dd_integration.h"
#include "hwa_fgo_factor_gnss_info.h"

using namespace hwa_gnss;

namespace hwa_fgo {
    class gnssprocesser : public baseprocesser, public gnss_proc_pvtflt {
    public:
        explicit gnssprocesser(std::string site, std::string site_base, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* allproc, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME);
        explicit gnssprocesser(const baseprocesser& B, std::string site, std::string site_base, std::shared_ptr<set_base> gset, base_log spdlog, base_all_proc* allproc);
		~gnssprocesser();
        void timesynchronization(base_time t) override;
        int ProcessOneEpoch() override;
        bool _time_valid(base_time instime) override;
        MEAS_TYPE _getPOS(base_time inst, base_posdata::data_pos& pos, MEAS_INFO& m) override;
        bool load_data() override;
		void _addResidualBlocks(ceres::Problem& problem) override;
		void _addMarginInfo() override;
		void slide_window() override;
		void _fgo_vector_to_double() override {
			for (int i = 0; i < _amb_manager->ambiguity_ids.size(); i++)
			{
				int amb_id = _amb_manager->ambiguity_ids[i];
				_fgo_info->_para_amb[amb_id][0] = _amb_manager->getAmb(amb_id);
			}
			_amb_manager->generateAmbSearchIndex();
		}
		void _fgo_double_to_vector() override {
			baseprocesser::_fgo_double_to_vector();
			for (int i = 0; i < _amb_manager->ambiguity_ids.size(); i++)
			{
				int amb_id = _amb_manager->ambiguity_ids[i];
				_amb_manager->updateAmb(amb_id, _fgo_info->_para_amb[amb_id][0]);
			}
		}
		MEAS_TYPE _get_gnss_measurements();
		void _gnss_amb_resolution();
		void clearWindow();

        const Triple get_lever() { 
			return lever; 
		}
        void _prt_port(base_time instime);
        int _prt_ins_kml(base_time instime);
        double get_pdop() { 
			return _dop.pdop();
		}
        Triple get_site_pos() {
            Triple gnss_pos;
            _param->getCrdParam(_site, gnss_pos);
            return gnss_pos; 
        }
        std::set<std::string> ambs_name(){ 
			return _param->amb_prns(); 
		}
        Matrix _getQx() {
            return _Qx.matrixR();
        }
		bool is_last_node() {
			return _node_index_copy().back() == _fgo_info->rover_count - 1;
		};
		base_posdata::data_pos get_posdata() {
			base_posdata::data_pos pos;
			_get_result(TimeStamp, pos);
			return pos;
		}
		Triple _getRobustFixedPosition();

		/**
		 * @brief Remove outlier satellite from current processing
		 * Eliminates outlier satellite from DD observations and ambiguity management
		 * @param outlier Pair of satellite name and global ID to be removed
		 * @return true if removal successful, false if insufficient satellites remain
		 */
		bool _remove_outlier_sat(const std::pair<std::string, int>& outlier);
		/**
		 * @brief Detect GNSS observation outliers using normalized residuals
		 * Identifies and flags satellites with excessive residuals for removal
		 * @param outlier Output pair of satellite name and ID to be removed
		 * @return Index of detected outlier, -1 if none found
		 */
		int  _gobs_outlier_detection(std::pair<std::string, int>& outlier);
		/**
		 * @brief Perform posteriori validation after optimization
		 * Computes covariance matrices and validates solution quality
		 * Updates GNSS information with residuals and statistical metrics
		 */
		void _posteriori_test(ceres::Problem& problem);

	protected:
		class DDEquMsg
		{
		public:
			base_time time;
			gnss_data_sats  rover_ref_sat;
			gnss_data_sats  rover_nonref_sat;
			gnss_data_sats  base_ref_sat;
			gnss_data_sats  base_nonref_sat;
			GOBSTYPE  obs_type;
			FREQ_SEQ  freq;
			GOBSBAND band;
			std::string base_site;
			std::string rover_site;
			std::string ref_sat;
			std::string nonref_sat;
			int ref_sat_global_id = -1;
			int nonref_sat_global_id = -1;

		public:
			explicit DDEquMsg(const gnss_data_sats& _ref_sat, const gnss_data_sats& _nonref_sat, const GOBSTYPE& _obs_type, const FREQ_SEQ& _freq);

		};
		/**
		 * @brief Get processed GNSS observation data for rover and base stations
		 * Retrieves satellite observation data, applies DCB corrections, and analyzes
		 * frequency availability for both rover and base stations.
		 */
		bool _get_gdata(const base_time& now, std::vector<gnss_data_sats>* data_rover = NULL, std::vector<gnss_data_sats>* data_base = NULL);
		/**
		 * @brief Initialize state parameters for GNSS positioning
		 * Performs ambiguity prediction, coordinate estimation, and state vector initialization
		 */
		virtual void _get_initial_value();
		/**
		 * @brief Initialize state for current epoch with slip detection
		 * Manages sliding window, detects cycle slips, and updates ambiguity parameters
		 */
		virtual void _set_initial_value();
		/**
		 * @brief Create temporary parameter set with base station coords and clock offsets
		 * Adds base station coordinates and receiver clock parameters to parameter set
		 */
		bool _gtemp_params(base_allpar& params, base_allpar& params_temp);
		/**
		 * @brief Set receiver coordinates and clock offsets
		 * Stores base station position and receiver clock parameters for processing
		 */
		void _set_rec_info(const Triple& xyz_base, double clk_rover, double clk_base);
		/**
		 * @brief Generate double-difference observations
		 * Creates double-difference combinations between reference and non-reference satellites
		 */
		int _combine_DD();
		/**
		 * @brief Select reference satellite and form DD combinations
		 * Chooses highest elevation satellite as reference and creates double-difference pairs with other satellites for all GNSS systems and frequencies
		 */
		void _select_ref_sat();
		/**
		 * @brief Complete DD message with base station data and frequency information
		 * Populates the double-difference message with corresponding base station observations and frequency band details. Verifies common satellite visibility between rover and base.
		 *
		 * @param dd_msg Double-difference message to be completed (input/output)
		 * @param base_sat_data Base station satellite observation data
		 * @return true if DD data successfully prepared, false if common visibility check fails
		 */
		bool _get_DD_data(DDEquMsg& dd_msg, std::vector<gnss_data_sats> base_sat_data);
		/**
		 * @brief Prepare data for ambiguity resolution
		 * Transfers linearized GNSS observation equations to filter for parameter estimation
		 * Sets up design matrix, weight matrix, residuals and covariance matrices
		 */
		bool _pre_amb_resolution();
		/**
		 * @brief Slide processing window forward by one epoch
		 * Shifts window data when full, removes oldest epoch, updates ambiguity states and _para_CRD
		 */
		virtual void _slide_window();
		/**
		 * @brief Check if satellite is marked as outlier
		 */
		bool _check_outlier(const std::string& sat);

    private:
        Triple lever;
        bool time_lock = false;

		std::vector<double*> _parameter_blocks;
		double posterior_std_X, posterior_std_Y, posterior_std_Z;

		std::map<base_time, std::vector<gnss_data_sats>> _map_basedata;
		std::map<base_time,  base_allpar> _map_param;

		std::vector<DDEquMsg> _DD_msg;
		std::vector<std::vector<DDEquMsg>> _vDD_msg;
		std::vector<std::pair<std::pair<std::string, int>, std::pair<FREQ_SEQ, GOBSTYPE>>> _gnss_obs_index;
		int              _global_sat_id = -1;
		int              _global_amb_id = -1;
		double           _loss_func_value = 2.0;
		double           _gtime_interval = 1.0;
		std::shared_ptr<gnss_amb_manager>  _amb_manager;               
		base_allpar _all_para_win;
		std::map<std::string, int> cur_sat_prn;
		std::vector<std::string> outlier_sats;
		std::vector<std::pair<std::string, int>> _removed_sats;
		bool _batch_remove = false;
		bool _initial_prior = true;
		std::unordered_map<long, int> amb_idx;
		double _gclk_rover, _gclk_base;
		int _obs_level = 3;
		Triple _gcrd_base;
		gnss_model_bias* _gbias_model = nullptr;					                    ///< baise model
		GNSSInfo* _last_gnss_info = nullptr;                             ///save gnss equ
		std::unordered_map<long, int> all_parameter_block; /// block to store info of last marginalized parameters

		bool is_first = true;
    };
}

#endif
