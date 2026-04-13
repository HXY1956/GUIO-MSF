#ifndef hwa_base_eigendef_h
#define hwa_base_eigendef_h

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "hwa_base_glob.h"

namespace hwa_base {
    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    using Matrix_T = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    using SE3_T = Eigen::Transform<T, 3, Eigen::Isometry>;
    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    using SO3_T = Eigen::Matrix<T, 3, 3>;
    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    using RowVector_T = Eigen::Matrix<T, 1, Eigen::Dynamic>;
    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    using Vector_T = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    using Quad_T = Eigen::Matrix<T, 4, 1>;
    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    using Triple_T = Eigen::Matrix<T, 3, 1>;
    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    using Dual_T = Eigen::Matrix<T, 2, 1>;

    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    class Symmetric_T {
    public:
        Symmetric_T() {};
        explicit Symmetric_T(int n) : A_(n, n) { A_.setZero(); };
        void set(T v, int i, int j) { A_(i, j) = A_(j, i) = v; }
        const T& get(int i, int j) const { return A_(i, j); }
        const T& operator()(int i, int j) const { return A_(i, j); }
        const int rows() const { return A_.rows(); }
        const int cols() const { return A_.cols(); }
        const int size() const { return A_.size(); }
        void resize(int n) { A_.resize(n, n); }
        void setZero() { A_.setZero(); }
        void setIdentity() { A_.setIdentity(); }
        Matrix_T<T>& matrixW() { return A_; }
        const Matrix_T<T>& matrixR() const { return A_; }
        void setLower(const Matrix_T<T>& L) {
            const int n = static_cast<int>(L.rows());
            resize(n);
            A_.template triangularView<Eigen::Lower>() = L;
            A_.template triangularView<Eigen::StrictlyUpper>() = A_.template triangularView<Eigen::StrictlyLower>().transpose();
        }
        Symmetric_T cov2corr()
        {
            Symmetric_T C(A_.rows());
            for (int i = 0; i < A_.rows(); i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    T denom = sqrt(A_(i, i) * A_(j, j));
                    T tmp = denom == 0 ? 0 : A_(i, j) / denom;
                    C.set(tmp, i, j);
                }
            }
            return C;
        }
        Symmetric_T SymSubMatrix(int m, int n) {
            int length = n - m + 1;
            Symmetric_T ans(length);
            ans.A_ = A_.block(m, m, length, length);
            return ans;
        }
        void ResizeKeep(int m)
        {
            Matrix_T<T> Q(m, m);
            int row = std::min(m, rows());
            Q.setZero();
            Q.block(0, 0, row, row) = A_.block(0, 0, row, row);
            A_ = Q;
        }

        void Matrix_remRC(int row, int col)
        {
            int n = this->rows();
            assert(row == col);
            assert(row < n);
            if (row < 0) return;
            Symmetric_T<T> Qt = *this;
            this->resize(n - 1);
            this->setZero();
            int rr = 0;
            for (int r = 0; r < Qt.rows(); r++)
            {
                if (r == row)
                    continue;
                int cc = 0;

                for (int c = 0; c < Qt.cols(); c++)
                {
                    if (c == col)
                        continue;
                    this->set(Qt(r, c), rr, cc);
                    cc++;
                }
                rr++;
            }
        }

        void Matrix_rem(std::vector<int>& ind)
        {
            std::vector<int>::iterator it;
            std::vector<int>::iterator it2;
            for (it = ind.begin(); it != ind.end(); it++)
            {
                Matrix_remRC(*it, *it);
                for (it2 = it; it2 != ind.end(); it2++)
                    (*it2)--;
            }
        }

        void Matrix_addRC(int row, int col)
        {
            assert(row == col);
            if (row < 0) return;
            Symmetric_T<T> Qt = *this;
            this->resize(this->rows() + 1);
            this->setZero();
            for (int i = 0, rr = 0; i < this->rows(); ++i) {
                if (i == row) continue;
                for (int j = 0, cc = 0; j <= i; ++j) {
                    if (j == col) continue;
                    this->set(Qt(rr, cc), i, j);
                    ++cc;
                }
                ++rr;
            }
        }

        void Matrix_swap(int a, int b)
        {
            A_.row(a).swap(A_.row(b));
            A_.col(a).swap(A_.col(b));
        }

        void Matrix_cpRC(const Symmetric_T<T>& Q, int r, int c)
        {
            assert(Q.rows() >= rows() && Q.cols() >= cols());
            if (r < rows()) {
                for (int i = 0; i < cols(); i++) this->set(Q(r, i), r, i);
            }
            if (c < cols()) {
                for (int i = 0; i < rows(); i++) this->set(Q(i, c), i, c);
            }
        }

    protected:
        Matrix_T<T> A_;
    };

    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    class Diag_T {
    public:
        Diag_T() {};
        explicit Diag_T(int n) : A_(n, n) { A_.setZero(); }
        explicit Diag_T(Vector_T<T> V) : A_(V.size(), V.size()) { A_.setZero();  A_.diagonal() = V; }
        const Vector_T<T> diagonal() const { return A_.diagonal(); }
        void set(T v, int m) { A_(m, m) = v; }
        void set(Vector_T<T> V) { resize(V.size()); A_.setZero(); A_.diagonal() = V; }
        Diag_T SR() const {
            Diag_T SR(rows());
            for (int i = 0; i < rows(); i++) {
                double tmp = A_(i, i);
                assert(tmp >= 0);
                SR.set(sqrt(tmp), i);
            }
            return SR;
        }
        const T get(int i, int j) const { return A_(i, j); }
        const T operator()(int i, int j) const { return A_(i, j); }
        const T operator()(int i) const { return A_(i, i); }
        T& operator()(int i) { return A_(i, i); }
        const int rows() const { return A_.rows(); }
        const int cols() const { return A_.cols(); }
        const int size() const { return A_.size(); }
        void resize(int n) { A_.resize(n, n); }
        void setZero() { A_.setZero(); }
        void setIdentity() { A_.setZero(); for (int i = 0; i < A_.rows(); i++) A_(i, i) = 1; }
        Matrix_T<T>& matrixW() { return A_; }
        const Matrix_T<T> matrixR() const { return A_; }

        void Matrix_remRC(int row)
        {
            const int n = this->rows();
            if (row < 0 || row >= n) return;
            const Vector_T<T> d = this->diagonal();
            Vector_T<T> newD(n - 1);
            newD.head(row) = d.head(row);
            newD.tail(n - 1 - row) = d.tail(n - row - 1);
            this->resize(n - 1);
            this->set(newD);
        }

        void Matrix_addRC(int row)
        {
            const int n = rows();
            if (row < 0 || row >= n) return;
            const Vector_T<T> d = this->diagonal();
            Vector_T<T> newD(n + 1);
            newD.setZero();
            newD.head(row) = d.head(row);
            newD.tail(n - row) = d.tail(n - row);
            this->resize(n + 1);
            this->set(newD);
        }

    protected:
        Matrix_T<T> A_;
    };

    using Matrix = Matrix_T<double>;
    using SE3 = SE3_T<double>;
    using SO3 = SO3_T<double>;
    using Vector = Vector_T<double>;
    using Quad = Quad_T<double>;
    using Triple = Triple_T<double>;
    using Dual = Dual_T<double>;
    using Symmetric = Symmetric_T<double>;
    using Diag = Diag_T<double>;
    using RowVector = RowVector_T<double>;

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    Matrix_T<T> rotX(T Angle)
    {
        const double C = std::cos(static_cast<double>(Angle));
        const double S = std::sin(static_cast<double>(Angle));
        Matrix_T<T> UU(3, 3);
        UU(0, 0) = 1.0;
        UU(0, 1) = 0.0;
        UU(0, 2) = 0.0;
        UU(1, 0) = 0.0;
        UU(1, 1) = +C;
        UU(1, 2) = +S;
        UU(2, 0) = 0.0;
        UU(2, 1) = -S;
        UU(2, 2) = +C;
        return UU;
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    Matrix_T<T> rotY(T Angle)
    {
        const double C = std::cos(static_cast<double>(Angle));
        const double S = std::sin(static_cast<double>(Angle));
        Matrix_T<T> UU(3, 3);
        UU(0, 0) = +C;
        UU(0, 1) = 0.0;
        UU(0, 2) = -S;
        UU(1, 0) = 0.0;
        UU(1, 1) = 1.0;
        UU(1, 2) = 0.0;
        UU(2, 0) = +S;
        UU(2, 1) = 0.0;
        UU(2, 2) = +C;
        return UU;
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    Matrix_T<T> rotZ(T Angle)
    {
        const double C = std::cos(static_cast<double>(Angle));
        const double S = std::sin(static_cast<double>(Angle));
        Matrix_T<T> UU(3, 3);
        UU(0, 0) = +C;
        UU(0, 1) = +S;
        UU(0, 2) = 0.0;
        UU(1, 0) = -S;
        UU(1, 1) = +C;
        UU(1, 2) = 0.0;
        UU(2, 0) = 0.0;
        UU(2, 1) = 0.0;
        UU(2, 2) = 1.0;
        return UU;
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void addR(Vector_T<T>& V, int row)
    {
        if (row < 0 || row > V.rows()) return;
        const Vector_T<T> Vt = V;
        V.resize(V.rows() + 1);
        V.setZero();
        int rr = 0;
        for (int r = 0; r < V.rows(); r++)
        {
            if (r == row)
                continue;

            V.operator()(r) = Vt(rr);
            rr++;
        }
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void remR(Vector_T<T>& V, int row)
    {
        if (row < 0 || row >= V.rows()) return;
        const Vector_T<T> Vt = V;
        V.resize(V.rows() - 1);
        V.setZero();
        int rr = 0;
        for (int r = 0; r < Vt.rows(); r++)
        {
            if (r == row)
                continue;

            V.operator()(rr) = Vt(r);
            rr++;
        }
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void remR(Vector_T<T>& V, std::vector<int> ind) {
        std::vector<int>::iterator it;
        std::vector<int>::iterator it2;
        for (it = ind.begin(); it != ind.end(); it++)
        {
            remR(V, *it);
            for (it2 = it; it2 != ind.end(); it2++)
                (*it2)--;
        }
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void Matrix_remRC(Matrix_T<T>& Q, int row, int col)
    {
        Matrix_T<T> Qt = Q;
        Q.resize(Q.rows() - (row >= 0), Q.cols() - (col >= 0));
        int rr = 0;
        for (int r = 0; r < Qt.rows(); r++)
        {
            if (r == row)
                continue;
            int cc = 0;

            for (int c = 0; c < Qt.cols(); c++)
            {
                if (c == col)
                    continue;
                Q.operator()(rr, cc) = Qt(r, c);
                cc++;
            }
            rr++;
        }
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void Matrix_rem(Matrix_T<T>& Q, std::vector<int>& ind)
    {
        std::vector<int>::iterator it;
        std::vector<int>::iterator it2;
        for (it = ind.begin(); it != ind.end(); it++)
        {
            Matrix_remRC(Q, *it, *it);
            for (it2 = it; it2 != ind.end(); it2++)
                (*it2)--;
        }
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void Matrix_addRC(Matrix_T<T>& Q, int row, int col)
    {
        Matrix_T<T> Qt = Q;
        Q.resize(Q.rows() + (row >= 0), Q.cols() + (col >= 0));
        int rr = 0;
        for (int r = 0; r < Q.rows(); r++)
        {
            if (r == row)
                continue;
            int cc = 0;

            for (int c = 0; c < Q.cols(); c++)
            {
                if (c == col)
                    continue;
                Q.operator()(r, c) = Qt(rr, cc);
                cc++;
            }
            rr++;
        }
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void Matrix_remR(Matrix_T<T>& Q, int row)
    {
        int nrow = Q.rows(), ncols = Q.cols();
        if (row < 0) return;
        const Matrix_T<T> At = Q;
        Q.resize(nrow - 1, ncols);
        Q.setZero();
        Q.block(0, 0, row, ncols) = At.block(0, 0, row, ncols);
        if (row < nrow - 1) {
            Q.block(row, 0, nrow - row - 1, ncols) = At.block(row + 1, 0, nrow - row - 1, ncols);
        }
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void Matrix_remR(Matrix_T<T>& Q, std::vector<int>& ind)
    {
        std::vector<int>::iterator it;
        std::vector<int>::iterator it2;
        for (it = ind.begin(); it != ind.end(); it++)
        {
            Matrix_remR(Q, *it);
            for (it2 = it; it2 != ind.end(); it2++)
                (*it2)--;
        }
    }

    template<class Derived1, class Derived2>
    typename Eigen::Matrix<typename Derived1::Scalar,
        Derived1::RowsAtCompileTime + Derived2::RowsAtCompileTime,
        Derived1::ColsAtCompileTime>
        Vstack(const Eigen::MatrixBase<Derived1>& Q1, const Eigen::MatrixBase<Derived2>& Q2) {
        int col1 = Q1.cols(); int row1 = Q1.rows();
        int col2 = Q2.cols(); int row2 = Q2.rows();
        assert(col1 == col2);
        Matrix Q3(row1 + row2, col1);
        Q3.block(0, 0, row1, col1) = Q1;
        Q3.block(row1, 0, row2, col1) = Q2;
        return Q3;
    }

    template<class Derived1, class Derived2>
    typename Eigen::Matrix<typename Derived1::Scalar,
        Derived1::RowsAtCompileTime,
        Derived1::ColsAtCompileTime + Derived2::ColsAtCompileTime>
        Hstack(const Eigen::MatrixBase<Derived1>& Q1, const Eigen::MatrixBase<Derived2>& Q2) {
        int col1 = Q1.cols(); int row1 = Q1.rows();
        int col2 = Q2.cols(); int row2 = Q2.rows();
        assert(row1 == row2);
        Matrix Q3(row1, col1 + col2);
        Q3.block(0, 0, row1, col1) = Q1;
        Q3.block(0, col1, row1, col2) = Q2;
        return Q3;
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void ResizeKeep(Matrix_T<T>& Q, int m, int n) {
        Eigen::Index row = std::min(static_cast<Eigen::Index>(m), Q.rows());
        Eigen::Index col = std::min(static_cast<Eigen::Index>(n), Q.cols());
        Matrix_T<T> Q1(m, n);
        Q1.setZero();
        Q1.block(0, 0, row, col) = Q.block(0, 0, row, col);
        Q = Q1;
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    void ResizeKeep(Vector_T<T>& Q, int m) {
        Eigen::Index row = std::min(static_cast<Eigen::Index>(m), Q.size());
        Vector_T<T> Q1(m);
        Q1.setZero();
        Q1.block(0, 0, row, 1) = Q.block(0, 0, row, 1);
        Q = Q1;
    }

    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    Triple_T<T> fromarray(T* Q) {
        return Triple_T<T>(Q[0], Q[1], Q[2]);
    }

    template<class T, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    Matrix_T<T> MatrixFromArray(T* Q, int m, int n) {
        Matrix_T<T> ans(m, n);
        for (int i = 0; i < m; ++i)
            for (int j = 0; j < n; ++j)
                ans(i, j) = Q[i * n + j];
        return ans;
    }

    template<class T = double, class = std::enable_if_t<hwa_detail::is_allowed<T>>>
    bool compareL(Triple_T<T> Q1, Triple_T<T> Q2) {
        return Q1[0] < Q2[0] ||
            (Q1[0] == Q2[0] && Q1[1] < Q2[1]) ||
            (Q1[0] == Q2[0] && Q1[1] == Q2[1] && Q1[2] < Q2[2]);
    }

    template<class Derived1, class Derived2>
    bool SVD(const Eigen::MatrixBase<Derived1>& Q1, Eigen::MatrixBase<Derived2>& Q2) {
        using Scalar = typename Derived1::Scalar;
        Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> svd(
            Q1, Eigen::ComputeThinU | Eigen::ComputeThinV
        );

        if (svd.info() != Eigen::Success) {
            std::cerr << "SVD decomposition failed!" << std::endl;
            return false;
        }

        Eigen::VectorXd singularValues = svd.singularValues();
        int k = singularValues.rows();
        Q2.derived().resize(k, k);
        Q2.setZero();
        Q2.diagonal() = singularValues;

        return true;
    }
}

#endif

