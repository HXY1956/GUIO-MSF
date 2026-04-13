#ifndef HWA_BASE_MATRIX_H
#define HWA_BASE_MATRIX_H

#include <Eigen/Eigen>
#include "HWA_BASE_Vector.h"
#include "HWA_BASE_GLOB.h"
#include <stdexcept>
#include <vector>

namespace HWA_BASE {
    class RowProxy {
        Eigen::Ref<Eigen::MatrixXd> mat_;
        const int row_;
    public:
        RowProxy(Eigen::Ref<Eigen::MatrixXd> m, int r) : mat_(m), row_(r) {}
        double  operator[](int j) const { return mat_(row_, j); }
        double& operator[](int j) { return mat_(row_, j); }
    };

    class ConstRowProxy {
        Eigen::Ref<const Eigen::MatrixXd> mat_;
        int row_;
    public:
        ConstRowProxy(Eigen::Ref<const Eigen::MatrixXd> m, int r) : mat_(m), row_(r) {}
        double operator[](int j) const {
            return mat_(row_, j);
        }
    };

    class Matrix {

    public:
        template<class Derived,
            std::enable_if_t<!std::is_same_v<
            std::decay_t<Derived>,
            Eigen::Ref<Eigen::MatrixXd>>, int> = 0>
        Matrix(const Eigen::MatrixBase<Derived>& eigenMat)   // ·Ç explicit
            : A_(eigenMat), ref_(A_){}
        explicit Matrix():A_(3, 3), ref_(A_) {};
        explicit Matrix(int v, int m, int n) : A_(m, n), ref_(A_) { setConstant(v); }
        explicit Matrix(int m, int n) : A_(m, n), ref_(A_) { A_.setZero(); }
        explicit Matrix(int n) : A_(n, n), ref_(A_) { A_.setZero(); }
        explicit Matrix(Eigen::MatrixXd A): A_(A), ref_(A_) {}
        explicit Matrix(Eigen::Ref<Eigen::MatrixXd> r)
            : ref_(r) {A_ = ref_;}
        virtual ~Matrix() = default;

        operator Eigen::MatrixXd()& { return ref_; }
        operator Eigen::MatrixXd() const& { return ref_; }
        
        class CommaInitializer;
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        CommaInitializer operator<<(T first)
        {
            ref_.setZero();
            ref_(0,0) = static_cast<double>(first);
            return CommaInitializer(*this, 1);
        }
        bool sizeequal(const Matrix& A, const Matrix& B) { bool equal = A.rows() == B.rows() && A.cols() == B.cols(); return equal; }
        bool operator==(const Matrix& D) const { return ref_ == D.ref(); }
        Matrix operator=(const Matrix& D) { if (!sizeequal(*this, D)) { resize(D.rows(), D.cols());}; ref_ = D.ref_; return *this; };
        template <class T, class = std::enable_if_t<detail::is_allowed<T>>>
        operator T() const { assert(ref_.size() == 1); return static_cast<T>(A_(0)); }
        Matrix operator*(const Matrix& D) const {return Matrix(ref_ * D.ref());}
        Matrix operator/(const Matrix& D) const { return *this * D.inverse(); }
        Matrix operator+(const Matrix& D) const {return Matrix(ref_ + D.ref());}
        Matrix operator+=(const Matrix& D) { ref_ = ref_ + D.ref(); return *this; }
        Matrix operator-(const Matrix& D) const {return Matrix(ref_ - D.ref());}
        Matrix operator-=(const Matrix& D) { ref_ = ref_ - D.ref(); return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator*(T b) const { return Matrix(ref_ * static_cast<double>(b)); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator*=(T b) { ref_ = ref_ * static_cast<double>(b); return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator/(T b) const { return Matrix(ref_ / static_cast<double>(b)); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator/=(T b) { ref_ = ref_ / static_cast<double>(b); return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Matrix operator*(T b, const Matrix& a) { return Matrix(a.ref() * static_cast<double>(b)); }

        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator+(T b) const {
            Matrix tmp(rows(), cols());
            for (int i = 0; i < rows(); i++) {
                for (int j = 0; j < cols(); j++) {
                    tmp(i, j) = ref_(i, j) + static_cast<double>(b);
                }
            }
            return tmp;
        }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator-(T b) const {
            Matrix tmp(rows(), cols());
            for (int i = 0; i < rows(); i++) {
                for (int j = 0; j < cols(); j++) {
                    tmp(i, j) = ref_(i, j) - static_cast<double>(b);
                }
            }
            return tmp;
        }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Matrix operator+(T b, const Matrix& a) { return (a + b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Matrix operator-(T b, const Matrix& a) { return (a * (-1.0) + b); }

        inline Matrix operator-() const { return Matrix(ref_ * (-1.0)); }
        inline Matrix operator+() const { return Matrix(ref_ * (1.0)); }
        double operator()(int i, int j) const { return ref_(i, j); }
        double& operator()(int i, int j) { return ref_(i, j); }
        void operator<<(const Matrix& A) { ref_ = A.ref(); }
        RowProxy operator[](int i) { return RowProxy(ref_, i); }
        ConstRowProxy operator[](int i) const { return ConstRowProxy(ref_, i); }

        Eigen::MatrixXd matrix() const { return ref_; }
        Eigen::Ref<Eigen::MatrixXd> ref() { return ref_; }
        Eigen::Ref<const Eigen::MatrixXd> ref() const { return ref_; }
        int rows() const { return static_cast<int>(ref_.rows()); }
        int cols() const { return static_cast<int>(ref_.cols()); }
        int size()  const { return static_cast<int>(ref_.size()); }
        void setZero() { ref_.setZero(); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        void setConstant(T a) { ref_.setConstant(a); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        static Matrix Constant(T v, int m, int n) {Matrix tmp(v, m, n);return tmp;}
        static Matrix Ones(int m, int n) {Matrix tmp(1, m, n);return tmp;}
        auto rowwise() { return ref_.rowwise(); }
        double val(int idx_) const {
            assert(idx_ < size());
            const int r = idx_ % rows();
            const int c = idx_ / rows();
            return ref_(r, c);
        }
        double val() const { assert(size()==1); return ref_(0,0); }
        double sum() const { return ref_.sum(); };
        double norm() const { return ref_.norm(); }
        Matrix transpose() { return Matrix(ref_.transpose()); }
        void setIdentity() { A_.setIdentity(); new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);}
        void setmatrix(Eigen::MatrixXd A) { A_ = A; new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);}
        void resize(int m, int n) { A_.resize(m, n); new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);}
        void resize(int n) { A_.resize(n, n); new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);}
        Matrix trace() const { return Matrix(ref_.trace()); }
        Matrix llt() const { return Matrix(ref_.llt()); }
        Matrix matrixL() const{ return Matrix(ref_.llt().matrixL()); }
        Matrix matrixU() const { return Matrix(ref_.llt().matrixU()); }
        Matrix matrixLLT() const { return Matrix(ref_.llt().matrixLLT()); }
        Matrix ldlt_solve(Matrix b) const { return Matrix(ref_.ldlt().solve(b.ref_)); }
        Matrix inverse() const {
            assert(rows() == cols());
            return Matrix(ref_.inverse());
        }
        Matrix eigenvalues() const {Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(ref_); return Matrix(es.eigenvalues());}
        Matrix eigenvectors() const {
            Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(ref_);
            return Matrix(eigenSolver.eigenvectors().real());
        }
        static Matrix Identity(int n) {return Matrix(Eigen::MatrixXd::Identity(n, n));}
        static Matrix Identity(int m,int n) {return Matrix(Eigen::MatrixXd::Identity(m, n));}
        static Matrix Zero(int n) {return Matrix(Eigen::MatrixXd::Zero(n, n));}
        static Matrix Zero(int m, int n) {return Matrix(Eigen::MatrixXd::Zero(m, n));}
        auto col(int n) { return ref_.col(n); }
        auto row(int n) { return ref_.row(n); }
        Matrix block(int start, int end, int row, int col) { return Matrix(Eigen::Ref<Eigen::MatrixXd>(A_.block(start, end, row, col))); }
        const Matrix block(int start, int end, int row, int col) const { return Matrix(A_.block(start, end, row, col)); }
        Matrix leftCols(int col) { return Matrix(Eigen::Ref<Eigen::MatrixXd>(A_.leftCols(col))); }
        const Matrix leftCols(int col) const { return Matrix(A_.leftCols(col)); }
        Matrix rightCols(int col) { return Matrix(Eigen::Ref<Eigen::MatrixXd>(A_.leftCols(col))); }
        const Matrix rightCols(int col) const { return Matrix(A_.leftCols(col)); }
        void Matrix_remRC(int row, int col);
        void Matrix_rem(std::vector<int>& ind);
        void Matrix_addRC(int row, int col);
        void Matrix_remR(int row);
        void Matrix_remR(std::vector<int>& ind);
        static Matrix rotX(double Angle);
        static Matrix rotY(double Angle);
        static Matrix rotZ(double Angle);

        class CommaInitializer
        {
            Matrix& mat_;
            int     idx_;
        public:
            CommaInitializer(Matrix& m, int i) : mat_(m), idx_(i) {}
            template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
            CommaInitializer&& operator,(T x)&&
            {
                if (idx_ >= mat_.size())
                    throw std::out_of_range("Matrix::operator<< too many values");
                const int r = idx_ % mat_.rows();
                const int c = idx_ / mat_.rows();
                mat_.ref_(r,c) = static_cast<double>(x);
                ++idx_;
                return std::move(*this);
            }
            template<class Derived>
            operator Derived& ()&&
            {
                static_assert(std::is_base_of_v<Matrix, Derived>,
                    "Can only convert to derived class of Matrix");
                return static_cast<Derived&>(mat_);
            }
        };

    protected:
        Eigen::MatrixXd A_;
        Eigen::Ref<Eigen::MatrixXd> ref_;
    };

    class SO3 : public Matrix {
    public:
        SO3(const Matrix& R){};
        SO3() : Matrix(3, 3) {};
        explicit SO3(Eigen::Ref<Eigen::MatrixXd> r)
            : Matrix(r) {}
        SO3 operator=(const Matrix& D) { if (!sizeequal(*this, D)) resize(D.rows(),D.cols()); ref_ = D.ref(); return *this; };
        static SO3 Identity() { SO3 tmp; tmp.setmatrix(Eigen::MatrixXd::Identity(3, 3)); return tmp; }
        static SO3 Zero() { return SO3(); }

    };

    class SE3 : public Matrix {
    public:
        SE3() : Matrix(4, 4) {};
        SE3(const Matrix& R) {};
        explicit SE3(Eigen::Ref<Eigen::MatrixXd> r)
            : Matrix(r) {}
        SE3 operator=(const Matrix& D) { if (!sizeequal(*this, D)) resize(D.rows(), D.cols()); ref_ = D.ref(); return *this; };
        static SE3 Identity() { SE3 tmp; tmp.block(0, 0, 3, 3) = Matrix::Identity(3, 3); return tmp;}
        static SE3 Zero() { return SE3(); }
        SO3 linear() {return SO3(Eigen::Ref<Eigen::MatrixXd>(A_.block(0, 0, 3, 3)));}
        Triple translation() { return Triple(Eigen::Ref<Eigen::MatrixXd>(A_.block(0, 3, 3, 1))); }
        SO3 linear() const { return SO3(Eigen::Ref<Eigen::MatrixXd>(A_.block(0, 0, 3, 3))); }
        Triple translation() const { return Triple(Eigen::Ref<Eigen::MatrixXd>(A_.block(0, 3, 3, 1))); }
    };

    class Symmetric : public Matrix {
    public:
        Symmetric() : Matrix() {};
        Symmetric(const Matrix& R) {};
        explicit Symmetric(int n) : Matrix(n, n) {};

        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Symmetric operator*=(T b) { ref_ *= b; return *this; };
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Symmetric operator/=(T b) { ref_ /= b; return *this; };
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Symmetric operator/(T b) const {
            Symmetric tmp;
            tmp.setmatrix(ref_ / b);
            return tmp;
        };
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Symmetric operator*(T b) const {
            Symmetric tmp;
            tmp.setmatrix(ref_ * b);
            return tmp;
        };
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Symmetric operator*(T b, const Symmetric& a) { return Symmetric(a.matrix() * b); }
        Symmetric operator=(const Matrix& rhs) { if (!sizeequal(*this, rhs)) resize(rhs.rows(), rhs.cols()); ref_ = rhs.ref();}
        Symmetric operator=(const Symmetric& rhs) { if (!sizeequal(*this, rhs)) resize(rhs.rows(), rhs.cols()); ref_ = rhs.ref(); return *this;}
        Symmetric operator+=(double alpha) { ref_.diagonal().array() += alpha; return *this; }
        Symmetric rank1Update(const Eigen::VectorXd& v, double beta = 1.0) { ref_.selfadjointView<Eigen::Lower>().rankUpdate(v, beta);return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        void set(T v, int i, int j) { ref_(i, j) = static_cast<double>(v); if (i != j) ref_(j, i) = static_cast<double>(v); }
        template<class Derived>
        void setLower(const Eigen::MatrixBase<Derived>& L) {
            const int n = static_cast<int>(L.rows());
            resize(n, n);
            A_.template triangularView<Eigen::Lower>() = L;
            A_.template triangularView<Eigen::StrictlyUpper>() = A_.template triangularView<Eigen::StrictlyLower>().transpose();
            new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);
        }
        void Matrix_remRC(int row, int col);
        void Matrix_rem(std::vector<int>& ind);
        void Matrix_addRC(int row, int col);
        void Matrix_swap(int a, int b);
        void Matrix_cpRC(const Symmetric& Q2, int r, int c);
        Symmetric cov2corr()
        {
            Symmetric C(rows());
            for (int i = 0; i < rows(); i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    if (i == j)
                        C(i, j) = 1;
                    assert(ref_(i, i) > 0 && ref_(j, j) > 0);
                    C(i, j) = ref_(i, j) / sqrt(ref_(i, i) * ref_(j, j));
                }
            }
            return C;
        }
    };

    class Diagonal : public Matrix {
    public:
        Diagonal() : Matrix() {};
        Diagonal(const Matrix& R) {};
        explicit Diagonal(int n): Matrix(n,n){}
        explicit Diagonal(Vector V) : Matrix(V.size()){ ref_.diagonal() = V.matrix();}
        const Vector diagonal() const {return Vector(ref_.diagonal());}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        void set(T v, int m) { ref_(m, m) = static_cast<double>(v);}
        void set(Vector V) { resize(V.size(), V.size()); ref_.setZero(); ref_.diagonal() = V.matrix();}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Diagonal operator*=(T b) { ref_.diagonal() *= b; return *this;}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Diagonal operator/=(T b) { ref_.diagonal() /= b; return *this;}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Diagonal operator/(T b) const { return Diagonal(diagonal() / b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Diagonal operator*(T b) const { return Diagonal(diagonal() * b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Diagonal operator*(T b, const Diagonal& a) { return Diagonal(a.diagonal() * b); }
        Diagonal operator=(const Matrix& _D) { set(_D.matrix().diagonal()); return *this; };
        Diagonal operator=(const Diagonal& _D) { if (!sizeequal(*this, _D)) resize(_D.rows(), _D.cols()); ref_ = _D.ref(); return *this; };
        Diagonal SR() const{
            Diagonal SR(rows());  
            for (int i = 0; i < rows(); i++) {
                double tmp = ref_(i, i);
                assert(tmp >= 0);
                SR(i, i) = sqrt(tmp);
            } 
            return SR;
        }
        void Matrix_remRC(int row);
        void Matrix_addRC(int row);
    };
}

#endif
