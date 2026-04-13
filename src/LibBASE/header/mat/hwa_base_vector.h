#ifndef HWA_BASE_VECTOR_H
#define HWA_BASE_VECTOR_H

#include <Eigen/Eigen>
#include "HWA_BASE_Matrix.h"
#include "HWA_BASE_GLOB.h"
#include <stdexcept>
#include <vector>

namespace HWA_BASE {
    class Vector {
    public:
        template<typename Derived>
        Vector(const Eigen::MatrixBase<Derived>& v)
            : Matrix(v.rows(), 1) {
            if (v.cols() != 1) throw std::invalid_argument("Need column vector");
            this->A_ = v;
        }
        Vector() : Matrix() {};
        Vector(const Matrix& R) {};
        explicit Vector(int n) : Matrix(n, 1) {}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        explicit Vector(T v, int n) : Matrix(v, n, 1) {}
        explicit Vector(const double* arr, const int n) : Matrix(n, 1) { for (int i = 0; i < n; i++) ref_(i, 0) = arr[i]; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        Vector(std::initializer_list<T> lst)
            : Matrix(lst.size(), 1)
        {
            auto it = lst.begin();
            int i = 0;
            while (it != lst.end()) (*this)(i++) = static_cast<double>(*it++);
        }
        explicit Vector(const Eigen::VectorXd& v) : Matrix(v) {}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        static Vector Constant(T v, int m) { return Vector(v, m); }
        static Vector Ones(int m) { return Vector(1, m); }
        static Vector Zero(int m) { return Vector(0, m); }

        operator Eigen::VectorXd() const { return ref_; }
        double& operator[](int i) { return ref_(i, 0); }
        double operator[](int i) const { return ref_(i, 0); }
        double& operator()(int i) { return ref_(i, 0); }
        double operator()(int i) const { return ref_(i, 0); }

        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Vector operator*=(T b) { ref_ *= b; return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Vector operator/=(T b) { ref_ /= b; return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Vector operator/(T b) const { return Vector(ref_ / b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Vector operator*(T b) const { return Vector(ref_ * b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Vector operator*(T b, const Vector& a) { return Vector(a.ref() * b); }
        inline Vector operator-() const { return Vector(ref_ * (-1)); }
        Vector operator=(const Eigen::VectorXd& D) { ref_ = D; return *this; };
        Vector operator=(const Matrix& D) { if (!sizeequal(*this, D)) resize(D.rows()); assert(D.cols() == 1); ref_ = D.ref(); return *this; }
        Vector operator+(const Matrix& D) const { assert(D.size() == size()); return Vector(ref_ + D.ref()); }
        Vector operator-(const Matrix& D) const { assert(D.size() == size()); return Vector(ref_ - D.ref()); }
        Matrix operator*(const Matrix& D) const { assert(D.rows() == 1); return Matrix(ref_ * D.ref()); }
        friend inline Matrix operator*(const Matrix& b, const Vector& a) { assert(b.cols() == a.size()); return Matrix(b.ref() * a.ref()); }

        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        void fill(T v) { ref_.setConstant(v); }
        auto head(int m) { return Eigen::Ref<Eigen::VectorXd>(ref_).head(m); }
        auto tail(int m) { return Eigen::Ref<Eigen::VectorXd>(ref_).tail(m); }
        auto head(int m) const { return Eigen::Ref<Eigen::VectorXd>(ref_).head(m); }
        auto tail(int m) const { return Eigen::Ref<Eigen::VectorXd>(ref_).tail(m); }
        void resize(int n) { A_.resize(n, 1); new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_); }
        void insert(int pos, double value) {
            Eigen::VectorXd tmp = A_;
            tmp.conservativeResize(tmp.size() + 1, Eigen::NoChange);
            tmp.tail(tmp.size() - pos - 1) = tmp.segment(pos, tmp.size() - pos - 1);
            tmp(pos) = value;
            A_ = std::move(tmp);
            new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);
        }
        void erase(int pos) {
            Eigen::VectorXd tmp = A_;
            tmp.segment(pos, tmp.size() - pos - 1) = tmp.segment(pos + 1, tmp.size() - pos - 1);
            tmp.conservativeResize(tmp.size() - 1, Eigen::NoChange);
            A_ = std::move(tmp);
            new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);
        }
        void addR(int row);

    protected:
        Eigen::VectorXd A_;
        Eigen::Ref<Eigen::VectorXd> ref_;

    };

    class Triple : public Vector {
    public:
        template<typename Derived>
        Triple(const Eigen::MatrixBase<Derived>& eigenVec)
            : Vector(eigenVec) {
            if (eigenVec.size() != 3) throw std::invalid_argument("Need 3¡Á1 vector");
        }

        Triple() : Vector(3) {};
        Triple(const Matrix& R) {};
        Triple(const Vector& R) {};
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        explicit Triple(T v) : Vector(v, 3) {};
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        Triple(std::initializer_list<T> lst)
            : Vector(3)
        {
            if (lst.size() != 3)
                throw std::invalid_argument("Triple needs exactly 3 elements");
            auto it = lst.begin();
            (*this)(0) = static_cast<double>(*it++);
            (*this)(1) = static_cast<double>(*it++);
            (*this)(2) = static_cast<double>(*it);
        }
        template<class T0, class T1, class T2,
            std::enable_if_t<detail::is_allowed<std::common_type_t<T0, T1, T2>>, int> = 0>
        Triple(const T0 a0, const T1 a1, const T2 a2) : Vector(3) {
            (*this)(0) = static_cast<double>(a0);
            (*this)(1) = static_cast<double>(a1);
            (*this)(2) = static_cast<double>(a2);
        }
        explicit Triple(const double* arr) : Vector(3) { for (int i = 0; i < 3; i++) ref_(i, 0) = arr[i]; }
        explicit Triple(const Eigen::Vector3d& v) : Vector(v) {}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        static Triple Constant(T v) { return Triple(v); }
        static Triple Ones() { return Triple(1); }
        static Triple Zero() { return Triple(0); }

        operator Eigen::Vector3d() const { return ref_; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Triple operator*=(T b) { ref_ *= b; return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Triple operator/=(T b) { ref_ /= b; return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Triple operator/(T b) const { return Triple(ref_ / b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Triple operator*(T b) const { return Triple(ref_ * b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Triple operator*(T b, const Triple& a) { return Triple(a.ref() * b); }
        Triple operator=(const Matrix& D) { ref_ = D.ref(); return *this; }
        Triple operator+(const Triple& D) const { return Triple(ref_ + D.ref()); }
        Triple operator-(const Triple& D) const { return Triple(ref_ - D.ref()); }
        Matrix operator*(const Matrix& D) const { assert(D.rows() == 1); return Matrix(ref_ * D.ref()); }
        inline Triple operator-() const { return Triple(ref_ * (-1)); }
        Triple cross(const Triple& b) const {
            Triple c;
            c[0] = (*this)[1] * b[2] - (*this)[2] * b[1];
            c[1] = (*this)[2] * b[0] - (*this)[0] * b[2];
            c[2] = (*this)[0] * b[1] - (*this)[1] * b[0];
            return c;
        }
    };
}

#endif