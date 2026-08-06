#ifndef hwa_gnss_amb_manager_h
#define hwa_gnss_amb_manager_h

#include "hwa_gnss_proc_pvtflt.h"

namespace hwa_gnss
{
	class gnss_sat_map
	{
	public:
		gnss_sat_map(GSYS sys, std::string sat_name, int id) :
			_gnss_system(sys), _sat_name(sat_name), _global_id(id)

		{
			DD_used = false;
		}
		gnss_sat_map(GSYS sys, std::string sat_name, double obs_time, int id) :
			_gnss_system(sys), _sat_name(sat_name), _start_obs_time(obs_time), _global_id(id)

		{
			DD_used = false;
			time_span.push_back(_start_obs_time);
		}
		GSYS _gnss_system;
		std::string _sat_name;
		double _start_obs_time;
		const int _global_id;
		std::vector<int> _amb_ids;   //stored the amb id;
		bool DD_used;
		std::vector<double> time_span;

	};
	class gnss_rover_msg
	{
	public:
		gnss_rover_msg()
		{

		}
		gnss_rover_msg(base_time _cur_time, Eigen::Vector3d _crd, Eigen::Vector3d _vel = Eigen::Vector3d::Identity()) :
			cur_time(_cur_time), crd(_crd), vel(_vel)
		{
		}
	public:
		base_time cur_time;         //current epoch
		Eigen::Vector3d crd;      //position
		Eigen::Vector3d vel;      //velocity
	protected:
		std::vector<gnss_data_sats> _sat_data; //sat data
	};
	class gnss_amb_per_id
	{
	public:
		gnss_amb_per_id(std::pair<FREQ_SEQ, GOBSBAND> freq_band, GSYS sys, std::string prn, int sat_id, int amb_id, int start_rover, double initial_amb) :
			_freq_band(freq_band), _gnss_system(sys), _sat_prn(prn), _sat_global_id(sat_id), _amb_id(amb_id), _start_rover_count(start_rover),
			_initial_amb(initial_amb), _estimated_amb(initial_amb), _fixed_amb(-1.0), _is_fix(false), _is_slip(false)
		{
			_rover_list.push_back(start_rover);
		}
		int endRover();
		int startRover();
		void setBeg(base_time beg);
		void setEnd(base_time end);
		void setSlip();
		void addRover(int rover_count);
		double get_initial_value();
		double get_est_value();
		void set_est_value(double est_value);
		void set_fixed_value(double fixed_value);
		void setFix();
		std::string getPRN();
		std::pair<FREQ_SEQ, GOBSBAND> getFB();
	public:
		GSYS _gnss_system;
		std::string _sat_prn;
		int _sat_global_id = -1;
		std::pair<FREQ_SEQ, GOBSBAND> _freq_band;
		const  int _amb_id;
		int _start_rover_count = 0;
		std::vector<int> _rover_list;  //stored the rover index in sliding window
		bool _is_slip;
		double _initial_amb = 0.0;
		double _estimated_amb = -1.0;
		double _fixed_amb = -1.0;
		bool _is_fix = false;
		base_time _beg;	  ///< begin time
		base_time _end;	  ///< end time	(end time corresponds to a new amb arc start)
	};
	class gnss_amb_manager
	{
	public:
		gnss_amb_manager();
		gnss_amb_manager(std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> band_index);
		~gnss_amb_manager();
		/**
		 * @brief Clear all ambiguity management states
		 * Resets satellite maps, ambiguity parameters and search indices
		 * Prepares for new processing session
		 */
		void clearState();
		/**
		 * @brief Remove satellite and update ambiguity states
		 * Handles satellite loss by clearing or updating affected ambiguities
		 * Maintains consistency in multi-epoch tracking
		 */
		void removeSat(const int& sat_global_id, const int& rover_index);
		/**
		 * @brief Slide window and update ambiguity tracking
		 * Removes oldest epoch data and shifts rover indices
		 * Maintains satellite-ambiguity consistency in window
		 */
		void slidingWindow();
		/**
		 * @brief Get total ambiguity parameter count
		 */
		int getAmbCount();
		/**
		 * @brief Get start rover ID for ambiguity
		 * Returns first epoch index where ambiguity appears
		 */
		int getAmbStartRoverID(const int& amb_search_id);
		/**
		 * @brief Get end rover ID for ambiguity
		 * Returns last epoch index where ambiguity appears
		 */
		int getAmbEndRoverID(const int& amb_search_id);
		/**
		 * @brief Get ambiguity parameter vector
		 * Returns all current ambiguity values as vector
		 */
		Eigen::VectorXd getAmbVector();
		/**
		 * @brief Get estimated ambiguity value
		 */
		double getAmb(int amb_id);
		/**
		* @brief Get initial ambiguity value
		*/
		double getInitialAmb(const int& amb_id);
		/**
		 * @brief Get ambiguities for marginalization
		 * Returns ambiguities ending at oldest epoch
		 */
		std::vector<int> getMarginAmb();
		/**
		 * @brief Get current window ambiguities
		 * Returns all active ambiguities in window
		 */
		std::vector<int> getCurWinAmb();
		/**
		 * @brief Update ambiguity parameter value
		 * Sets new estimated float ambiguity
		 */
		void updateAmb(int amb_id, double value);
		/**
		 * @brief Generate ambiguity search index
		 * Creates mapping from satellite-frequency to ambiguity ID
		 */
		void generateAmbSearchIndex();
		/**
		 * @brief Get current epoch satellites
		 * Updates satellite list for latest epoch
		 */
		int get_last_epoch_sats(double time);
		/**
		 * @brief Get satellite global ID
		 * Returns satellite ID from PRN name
		 */
		int get_sat_id(std::string prn);
		/**
		 * @brief Set estimated ambiguities from vector
		 * Updates all ambiguity values from solution vector
		 */
		void setEstAmb(const Eigen::VectorXd& x);
		/**
		 * @brief Get ambiguity index by satellite and frequency
		 */
		int getAmbSearchIndex(const std::pair<int, FREQ_SEQ>& sat_freq);
		/**
		 * @brief Add new satellite with ambiguities
		 * Initializes satellite tracking and creates ambiguity parameters
		 */
		void addNewSat(const base_time& cur_time, const int& rover_index, const int& sat_index, int& amb_index, const gnss_data_sats& sat_data, base_allpar params);
		/**
		 * @brief Extend satellite tracking to new epoch
		 * Updates all ambiguities for given satellite to include new rover epoch
		 */
		void addRover(std::string sat_name, const int& rover_index);
		/**
		 * @brief Add rover epoch with timestamp
		 * Extends satellite tracking time span and ambiguity rover lists
		 */
		void addRover(double time, std::string sat_name, const int& rover_index);
		/**
		 * @brief Create ambiguity parameters for new satellite
		 * Initializes single-difference ambiguities for all frequencies
		 * Updates satellite-ambiguity mapping and global ID tracking
		 */
		bool addAmb(const base_time& cur_time, std::vector<base_par> amb_para, const  GSYS& gnss_system, int rover_index, const int& sat_index, int& amb_index);
		/**
		 * @brief Add ambiguity parameter to parameter set
		 */
		void addGpara(base_allpar& params, const int& idx, double value);
		/**
		 * @brief Mark satellite as used in DD
		 */
		void checkDDSat(const int& sat_id);
		/**
		 * @brief Remove unused satellites and ambiguities
		 * Cleans up satellites not used in double-difference processing
		 * Maintains only actively tracked ambiguities
		 */
		void updateAmb();
		/**
		 * @brief Check if satellite is currently tracked
		 */
		bool is_sat_tracking(std::string prn);

		const std::map<int, std::shared_ptr<gnss_sat_map>>& getSatMap() { return _sat_map; }

	public:
		std::vector<int> ambiguity_ids;
		std::vector<std::pair<std::string, int>> cur_sats;
		std::map<std::string, int> removed_sat;
		std::vector<int> removed_ambs;
	protected:
		std::map<int, std::shared_ptr<gnss_amb_per_id>> _ambiguity;		// All ambiguity information within the current window (corresponding satellite and index, frequency, ambiguity index, whether cycle slip, whether fixed)
		std::map<int, std::shared_ptr<gnss_sat_map>> _sat_map;			// All satellite information within the current window (satellite index and corresponding ambiguity index)
		std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index;
		std::map<par_type, FREQ_SEQ> ambtype_list = {
				{par_type::AMB_L1,FREQ_1},
				{par_type::AMB_L2,FREQ_2},
				{par_type::AMB_L3,FREQ_3},
				{par_type::AMB_L4,FREQ_4},
				{par_type::AMB_L5,FREQ_5} };
		std::map<FREQ_SEQ, par_type> freq_ambtype_list = {
				{FREQ_1, par_type::AMB_L1},
				{FREQ_2, par_type::AMB_L2},
				{FREQ_3, par_type::AMB_L3},
				{FREQ_4, par_type::AMB_L4},
				{FREQ_5, par_type::AMB_L5} };
		std::map<std::pair<int, FREQ_SEQ>, int> search_index;
	};
}

#endif