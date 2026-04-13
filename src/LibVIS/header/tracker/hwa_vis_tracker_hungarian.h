#ifndef hwa_vis_tracker_hungarian_h
#define hwa_vis_tracker_hungarian_h
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis {
    using Point = cv::Point3f;
    using Perm = std::array<int, 4>;

    class Hungarian {
    public:
        static double calculate_distance(const Point& p1, const Point& p2);
        static double hungarian(const std::vector<std::vector<double>>& cost, std::vector<int>& assignment);
        static std::vector<std::vector<double>> DistMatrix(const std::vector<Point>& original_points);
        static void match_points_hungarian(
            const std::vector<Point>& original_points,
            std::vector<Point>& target_points);
    };

    class Point4Matcher
    {
        Perm lastMap = { 2,3,1,0 };
    public:
        std::array<Perm, 24> PERM4 = {
        0,1,2,3, 0,1,3,2, 0,2,1,3, 0,2,3,1, 0,3,1,2, 0,3,2,1,
        1,0,2,3, 1,0,3,2, 1,2,0,3, 1,2,3,0, 1,3,0,2, 1,3,2,0,
        2,0,1,3, 2,0,3,1, 2,1,0,3, 2,1,3,0, 2,3,0,1, 2,3,1,0,
        3,0,1,2, 3,0,2,1, 3,1,0,2, 3,1,2,0, 3,2,0,1, 3,2,1,0
        };

        void update(const std::vector<Point>& src,
            std::vector<Point>& tgt);
        Perm match4Points(const std::vector<Point>& src,
            const std::vector<Point>& tgt,
            const Perm& lastMap = { 0,1,2,3 },
            double wEdge = 1.0,
            double wPrior = 2.0);
        double priorCost(const Perm& p, const Perm& last);
        double edgeCost(const std::vector<Point>& a,
            const std::vector<Point>& b,
            const Perm& p);

    private:
        bool isFirst = true;
    };
}

#endif

