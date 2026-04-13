#ifndef hwa_gnss_proc_lsqmatrix_H
#define hwa_gnss_proc_lsqmatrix_H

#include <Eigen/Core>
#include "hwa_set_gproc.h"
#include "hwa_base_eigendef.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_data_turboedit.h"
#include "hwa_gnss_model_base.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /**
    * @brief  Matrix for storage observe equations of B,P,L
    */
    class gnss_proc_lsq_equationmatrix : public gnss_model_base_equation
    {
    public:
        /** @brief default constructor */
        gnss_proc_lsq_equationmatrix();

        /** @brief default destructor  */
        ~gnss_proc_lsq_equationmatrix();

        /**
        * @brief add equations
        * @param[in] B_value coeff of observ equation
        * @param[in] P_value weight of observ equation
        * @param[in] l_value res of observ equaion
        */
        void add_equ(const std::vector<std::pair<int, double>> &B_value, const double &P_value, const double &l_value, const std::string &stie_name, const std::string &sat_name, const gnss_data_obscombtype &obscombtype, const bool &is_newamb);

        // jdhuang : add for save raw P for IF combination
        //void add_equ(const vector<pair<int, double> >& B_value, const vector<pair<int, double> >& P_raw, const double& P_value, const double& l_value, const string& stie_name, const string& sat_name, const gnss_data_obscombtype& obscombtype, const bool& is_newamb);

        void add_equ(const gnss_proc_lsq_equationmatrix &Other);
        /**
        * @brief add equations by newmat format
        * @param[in] B_value coeff of observ equation newmat format
        * @param[in] P_value weight of observ equation newmat format
        * @param[in] l_value res of observ equaion newmat format
        */
        void add_equ(const Matrix &B_value, const Diag &P_value, const Vector &l_value, const std::vector<std::string> &site_name, const std::vector<std::string> &sat_name, const std::vector<gnss_data_obscombtype> &obstype);

        void set_newamb(const int &idx, const bool &is_newamb);
        void set_newamb(gnss_data_turboedit *tb_slip);
        void set_newamb(const FREQ_SEQ &freq_1, const FREQ_SEQ &freq_2, gnss_data_turboedit *tb_slip);
        /**
        * @brief remove specified equations
        * @note idx from 1
        * param[in] idx specified loc
        */
        void remove(const int &idx);

        void remove_last_equation();

        /**
        * @brief change equations to newmat format
        * @param[out] B_value coeff of observ equation newmat format
        * @param[out] P_value weight of observ equation newmat format
        * @param[out] l_value res of observ equaion newmat format
        */
        void chageNewMat(Matrix &B_value, Diag &P_value, Vector &l_value, const int &par_num);
        void chageNewMat(Matrix &B_value, Symmetric &P_value, Vector &l_value, const int &par_num);

        /** @brief print B P L Matrix*/
        void print();

        /**
        * @brief get numbers of equations
        * @return size of equations
        */
        int num_equ() const;

        /**
        * @brief get resiuals of equations
        * @return LTPL of equations
        */
        double res_equ() const;

        /**
        * @brief get resiuals of equations
        * @param[in] phase phase only or both
        * @return LTPL of equations
        */
        double res_equ(bool phase) const; //add by xiongyun
        /**
         * @brief Get the sitename
         * @param[in]  equ_idx   index of equ
         * @return string station name
         */
        std::string get_sitename(int equ_idx) const;
        /**
         * @brief Get the satlist
         * @param[in]  rec       station name
         * @return set<string> satellite name list
         */
        std::set<std::string> get_satlist(std::string rec) const;
        /**
         * @brief Get the satname
         * @param[in]  equ_idx   index of equ
         * @return string satellite name
         */
        std::string get_satname(int equ_idx) const;
        /**
         * @brief Get the site sat pair
         * @return vector<pair<string, string>> station name & satellite name
         */
        std::vector<std::pair<std::string, std::string>> get_site_sat_pair() const;
        /**
         * @brief Get the string of obscombtype
         * @param[in]  equ_idx   index of equ
         * @return string string type of observation
         */
        std::string get_obscombtype2str(int equ_idx) const;
        /**
         * @brief Get the obscombtype
         * @param[in]  equ_idx   index of equ
         * @return gnss_data_obscombtype type of observation
         */
        gnss_data_obscombtype get_obscombtype(int equ_idx) const;
        /**
         * @brief Get the code omc
         * @return vector<double> code omc 
         */
        std::vector<double> get_codeomc() const;
        /**
         * @brief Get the phase omc
         * @return vector<double> phase omc 
         */
        std::vector<double> get_phaseomc() const;
        /**
         * @brief whether a new arc
         * @param[in]  equ_idx   index of equ
         * @return
         *         @retval true is new arc
         *         @retval false not new arc
         */
        bool is_newamb(int equ_idx) const;
        /**
         * @brief print equ infomation
         * @param[in]  equ       equation
         */
        friend void print_equ_debinfo(const gnss_proc_lsq_equationmatrix &equ);
        /**
         * @brief find certain station equation
         * @param[in]  site      station name
         * @return vector<int> index of certain station equation
         */
        std::vector<int> find_equ(const std::string &site);
        /**
         * @brief find certain station and satellite equation
         * @param[in]  site      station name
         * @param[in]  sat       satellite name
         * @return vector<int> index of certain station and satellite equation
         */
        std::vector<int> find_equ(const std::string &site, const std::string &sat);
        /**
         * @brief find certain station, satellite and observation type equation
         * @param[in]  site      station name
         * @param[in]  sat       satellite name
         * @param[in]  obscomtype observation type
         * @return int index of certain station, satellite and observation type equation
         */
        int find_equ(const std::string &site, const std::string &sat, const gnss_data_obscombtype &obscomtype);
        /**
         * @brief clear all equation
         */
        void clear_allequ();
        /**
         * @brief swap all equation
         */
        void swap_allequ();

    protected:
        std::vector<std::pair<std::string, std::string>> _site_sat_pairlist; ///< station & satellite name
        std::vector<gnss_data_obscombtype> _obstypelist;             ///< observation type
        std::vector<bool> _newamb_list;                       ///< whether new arc
    };

    /**
    * @brief  Matrix for storage Matrix NEQ
    */
    class gnss_proc_lsqbaseSymmetric
    {
    public:
        /**
        * @brief default destructor
        * @return size of NEQ matrix
        */
        virtual int num() const = 0;

        virtual double &num(int a, int b) = 0;
        //double  num(int a, int b) const ;

        /**
        * @brief resize the NEQ matrix according to specified size
        * @note clean the data before
        * @param[in] size specified size
        */
        virtual void resize(int size) = 0;

        /**
        * @brief add observ equations
        * @parma[in] equ observ equations
        */
        virtual void add(const gnss_proc_lsq_equationmatrix &equ) = 0;

        /** @brief add one zero dimension at last*/
        virtual void addBackZero() = 0;

        /**
        * @brief removed specified dimension in the NEQ matrix
        * @note idx from 1
        * @param[in] idx specified dimension
        */
        virtual void remove(int idx) = 0;

        /**
        * @brief print NEQ matrix in std out
        */
        virtual void print() = 0;

        /**
        * @brief get diagonal value in NEQ matrix by specified loc
        * @note idx from 1
        * @param[in] idx specified loc
        */
        virtual double center_value(int idx) const = 0;

        /**
        * @brief change NEQ matrix to NewMat format
        * @return NEQ matrix in newmat format
        */
        virtual Symmetric changeNewMat() const = 0;
    };

    /**
    * @brief class for store BTPL(W) matrix
    */
    class gnss_proc_lsq_COLUMNVECTOR
    {
    public:
        /**
        * @brief get size of column vector
        * @return size
        */
        virtual int num() const = 0;

        virtual double &num(int a) = 0;
        //double num(int a) const;

        /**
        * @brief add observ equation
        * @param[in] equ observ equation
        */
        virtual void add(const gnss_proc_lsq_equationmatrix &equ) = 0;

        /**
        * @brief add zero dimension in the last loc
        */
        virtual void addBackZero() = 0;

        /**
        * @brief resize the column vector
        * @note resize will clean data before
        * @param[in] size specified size of column vector
        */
        virtual void resize(int size) = 0;

        /**
        * @brief remove specified dimension of W matrix
        * @param[in] idx specified loc
        */
        virtual void remove(int idx) = 0;

        /** @brief print W matrix in stdout */
        virtual void print() = 0;

        virtual Vector changeNewMat() = 0;
    };

    class L_Symmetric : public gnss_proc_lsqbaseSymmetric
    {
    public:
        /** @brief default constructor */
        L_Symmetric();

        /** @brief default destructor */
        ~L_Symmetric();

        /**
        * @brief default destructor
        * @return size of NEQ matrix
        */
        int num() const;

        double &num(int a, int b);
        //double  num(int a, int b) const ;

        /**
        * @brief resize the NEQ matrix according to specified size
        * @note clean the data before
        * @param[in] size specified size
        */
        void resize(int size);

        /**
        * @brief add observ equations
        * @parma[in] equ observ equations
        */
        void add(const gnss_proc_lsq_equationmatrix &equ);

        /** @brief add one zero dimension at last*/
        void addBackZero();

        /**
        * @brief removed specified dimension in the NEQ matrix
        * @note idx from 1
        * @param[in] idx specified dimension
        */
        void remove(int idx);

        /**
        * @brief print NEQ matrix in std out
        */
        void print();

        /**
        * @brief get diagonal value in NEQ matrix by specified loc
        * @note idx from 1
        * @param[in] idx specified loc
        */
        double center_value(int idx) const;

        /**
        * @brief change NEQ matrix to NewMat format
        * @return NEQ matrix in newmat format
        */
        Symmetric changeNewMat() const;

    private:
        std::list<std::list<double>> _element; ///< element in NEQ Matrix,store element in lower triangle
        std::list<std::list<double>>::iterator row_it;
        std::list<double>::iterator col_it;
        int row_record, col_record;
    };

    class L_Vector : public gnss_proc_lsq_COLUMNVECTOR
    {
    public:
        /** @brief default constructor */
        L_Vector();
        /** @brief default destructor */
        ~L_Vector();

        /**
        * @brief get size of column vector
        * @return size
        */
        int num() const;

        double &num(int a);
        //double num(int a) const;

        /**
        * @brief add observ equation
        * @param[in] equ observ equation
        */
        void add(const gnss_proc_lsq_equationmatrix &equ);

        /**
        * @brief add zero dimension in the last loc
        */
        void addBackZero();

        /**
        * @brief resize the column vector
        * @note resize will clean data before
        * @param[in] size specified size of column vector
        */
        void resize(int size);

        /**
        * @brief remove specified dimension of W matrix
        * @param[in] idx specified loc
        */
        void remove(int idx);

        /** @brief print W matrix in stdout */
        void print();

        Vector changeNewMat();

    private:
        std::list<double> _element; ///< element in W matrix
        std::list<double>::iterator row_it;
        int row_record;
    };

    class V_Symmetric;

    class V_Vector : public gnss_proc_lsq_COLUMNVECTOR
    {
    public:
        /**
         * @brief get number of elements
         * @return int number of elements
         */
        int num() const;
        /**
         * @brief get value of certain elements
         * @param[in]  a         index of elements
         * @return double& value of certain elements
         */
        double &num(int a);
        /**
         * @brief     reset elements size
         * @param[in]  num       size of elements
         */
        void resize(int num);
        /**
         * @brief add a new equation
         * @param[in]  equ       new equation
         */
        void add(const gnss_proc_lsq_equationmatrix &equ);
        //void add_related(const gnss_proc_lsq_equationmatrix& equ);
        /**
         * @brief delete a equation
         * @param[in]  equ       equtaion
         */
        void del(const gnss_proc_lsq_equationmatrix &equ);
        /**
         * @brief add a new equation
         * @param[in]  equ       equation
         * @param[in]  phase     whether phase
         */
        void add(const gnss_proc_lsq_equationmatrix &equ, bool phase); //add by xiongyun
        /**
         * @brief add 0 element
         */
        void addBackZero();
        /**
         * @brief move certain element
         * @param[in]  idx       index of element
         */
        void remove(int idx);
        /**
         * @brief print element
         */
        void print();
        /**
         * @brief element 2 Vector
         * @return Vector after transform element
         */
        Vector changeNewMat();
        /**
         * @brief swap element
         * @param[in]  a         index of element
         * @param[in]  b         index of element
         */
        void swap(int a, int b);
        /**
         * @brief remove certain element matrix
         * @param[in]  idx       index of element
         * @param[in]  NEQ       neq
         * @param[in]  W         w
         * @return 
         *         @retval true remove correct
         *         @retval false remove false
         */
        friend bool remove_lsqmatrix(int idx, V_Symmetric &NEQ, V_Vector &W);
        friend bool rearrange_lsqmatrix(const std::vector<int> &remove_idx, base_allpar &allpar, V_Symmetric &NEQ, V_Vector &W, Matrix &N11, Matrix &N21, Matrix &N22, Vector &W1, Vector &W2, bool idx_from_zero);
        /**
         * @brief rearange certain element matrix
         * @param[in]  remove_idx index of element
         * @param[in]  NEQ       neq
         * @param[in]  W         w
         * @param[in]  allpar    parameter
         */
        friend int rearrange_lsqmatrix(std::vector<int> &remove_idx, V_Symmetric &NEQ, V_Vector &W, base_allpar &allpar);
        friend class gnss_proc_lsqbase;

    private:
        std::vector<double> _element; ///< element
    };

    class V_Symmetric : public gnss_proc_lsqbaseSymmetric
    {
    public:
        int num() const;
        inline double &num(int a, int b) { return (a < b) ? _element[b - 1][a - 1] : _element[a - 1][b - 1]; }
        void resize(int num);
        void add(const gnss_proc_lsq_equationmatrix &equ);
        void add_related(const gnss_proc_lsq_equationmatrix &equ);
        void del(const gnss_proc_lsq_equationmatrix &equ);             // add by wangbo
        void add(const gnss_proc_lsq_equationmatrix &equ, bool phase); //add by xiongyun
        void addBackZero();
        void remove(int idx);
        void print();
        double center_value(int idx) const;
        Symmetric changeNewMat() const;
        void swap(int a, int b);

        friend bool remove_lsqmatrix(int idx, V_Symmetric &NEQ, V_Vector &W);
        friend bool rearrange_lsqmatrix(const std::vector<int> &remove_idx, base_allpar &allpar, V_Symmetric &NEQ, V_Vector &W, Matrix &N11, Matrix &N21, Matrix &N22, Vector &W1, Vector &W2, bool idx_from_zero);
        friend int rearrange_lsqmatrix(std::vector<int> &remove_idx, V_Symmetric &NEQ, V_Vector &W, base_allpar &allpar);
        friend class gnss_proc_lsqbase;

    private:
        std::vector<std::vector<double>> _element;
    };

    bool remove_lsqmatrix(int idx, V_Symmetric &NEQ, V_Vector &W);

    // by matrix part
    bool rearrange_lsqmatrix(const std::vector<int> &remove_idx, base_allpar &allpar, V_Symmetric &NEQ, V_Vector &W, Matrix &N11, Matrix &N21, Matrix &N22, Vector &W1, Vector &W2, bool idx_from_zero = true);
    int rearrange_lsqmatrix(std::vector<int> &remove_idx, V_Symmetric &NEQ, V_Vector &W, base_allpar &allpar);
}

#endif /*  GlsqMAT_H  */