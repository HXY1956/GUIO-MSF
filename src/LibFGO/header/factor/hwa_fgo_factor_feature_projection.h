#ifndef HWA_FGO_FEATURE_PROJECTION_FACTOR
#define HWA_FGO_FEATURE_PROJECTION_FACTOR

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include"hwa_base_posetrans.h"

using namespace hwa_base;

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
public:
    ProjectionFactor(const Eigen::Vector3d& _pts_i, const Eigen::Vector3d& _pts_j);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
    void check(double** parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

#endif