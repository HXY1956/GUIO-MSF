#include "hwa_vis_tracker_hungarian.h"

double hwa_vis::Hungarian::calculate_distance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double hwa_vis::Hungarian::hungarian(const std::vector<std::vector<double>>& cost,
    std::vector<int>& assignment)
{
    int n = cost.size();
    int m = cost[0].size();
    assert(n == m);           
    assignment.assign(n, -1);
    std::vector<double> u(n + 1), v(n + 1);
    std::vector<int> p(n + 1), way(n + 1);
    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
        std::vector<bool> used(n + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = 0;
            double delta = std::numeric_limits<double>::infinity();
            for (int j = 1; j <= n; ++j) {
                if (!used[j]) {
                    double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= n; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    for (int j = 1; j <= n; ++j) assignment[p[j] - 1] = j - 1;
    return -v[0];
}

std::vector<std::vector<double>> hwa_vis::Hungarian::DistMatrix(const std::vector<Point>& original_points) {
    int n = original_points.size();
    std::vector<std::vector<double>> source_dist_matrix;
    source_dist_matrix.resize(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            source_dist_matrix[i][j] = calculate_distance(original_points[i], original_points[j]);
            source_dist_matrix[j][i] = source_dist_matrix[i][j];
        }
    }
    return source_dist_matrix;
}

void hwa_vis::Hungarian::match_points_hungarian(
    const std::vector<Point>& original_points,
    std::vector<Point>& target_points)
{
    std::vector<std::vector<double>> source_dist_matrix = DistMatrix(original_points);
    const int n = source_dist_matrix.size();
    assert(n > 0);
    assert(n == target_points.size());

    std::vector<std::vector<double>> target_dist_matrix(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double dist = calculate_distance(target_points[i], target_points[j]);
            target_dist_matrix[i][j] = dist;
            target_dist_matrix[j][i] = dist;
        }
    }

    std::vector<int> currenhwa_map_ping(n);
    std::iota(currenhwa_map_ping.begin(), currenhwa_map_ping.end(), 0);

    const int max_iterations = 100;
    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(n, 0.0));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                double total_cost = 0.0;
                for (int k = 0; k < n; ++k) {
                    if (k != i) {
                        double dist_s = source_dist_matrix[i][k];
                        double dist_t = target_dist_matrix[j][currenhwa_map_ping[k]];
                        total_cost += std::abs(dist_s - dist_t);
                    }
                }
                cost_matrix[i][j] = total_cost;
            }
        }

        std::vector<int> new_mapping(n);
        hungarian(cost_matrix, new_mapping);
        bool converged = true;
        for (int i = 0; i < n; ++i) {
            if (new_mapping[i] != currenhwa_map_ping[i]) {
                converged = false;
                break;
            }
        }
        currenhwa_map_ping = new_mapping;
        if (converged) {
            break;
        }
    }

    std::vector<cv::Point3f> reordered(n);
    for (int i = 0; i < n; ++i) reordered[i] = target_points[currenhwa_map_ping[i]];
    target_points.swap(reordered);
}


double hwa_vis::Point4Matcher::edgeCost(const std::vector<Point>& a,
    const std::vector<Point>& b,
    const Perm& p)
{
    double cost = 0;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
            cost += std::abs(cv::norm(a[i] - a[j]) -
                cv::norm(b[p[i]] - b[p[j]]));
    return cost;
}

double hwa_vis::Point4Matcher::priorCost(const Perm& p, const Perm& last)
{
    int diff = 0;
    for (int i = 0; i < 4; ++i) diff += (p[i] != last[i]);
    return diff;
}

hwa_vis::Perm hwa_vis::Point4Matcher::match4Points(const std::vector<Point>& src,
    const std::vector<Point>& tgt,
    const Perm& lastMap,
    double wEdge,
    double wPrior) 
{
    Perm bestPerm = lastMap;
    double bestScore = 1e9;
    for (const auto& p : PERM4)
    {
        double ed = edgeCost(src, tgt, p);
        double pr = priorCost(p, lastMap);
        double score = wEdge * ed
            + wPrior * pr;
        if (score < bestScore)
        {
            bestScore = score;
            bestPerm = p;
        }
    }
    return bestPerm;
}

void hwa_vis::Point4Matcher::update(const std::vector<Point>& src,
    std::vector<Point>& tgt)
{
    double wPrior = 1;
    if (isFirst) {
        wPrior = 0.001;
        isFirst = false;
    } 
    Perm best = match4Points(src, tgt, lastMap, 1, wPrior);
    lastMap = best;         
    std::vector<Point> tmp(4);
    for (int i = 0; i < 4; ++i) tmp[i] = tgt[best[i]];
    tgt.swap(tmp);
}