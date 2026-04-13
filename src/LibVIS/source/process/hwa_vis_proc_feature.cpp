#include "hwa_vis_proc_feature.h"
#include <numeric>

hwa_vis::vis_feature::vis_feature() :id(0), position(Triple::Zero()),
is_initialized(false)
{
}

hwa_vis::vis_feature::vis_feature(const FeatureIDType& new_id) : id(new_id),
position(Triple::Zero()),
is_initialized(false), is_KeyFrame(false), is_initialized_NonKey(false)
{
}

//hwa_vis::vis_feature::vis_feature(set_base* gset, const FeatureIDType& new_id) : id(new_id),
//position(Triple::Zero()),
//is_initialized(false), is_KeyFrame(false), is_initialized_NonKey(false)
//{
//    translation_threshold = dynamic_cast<set_vis*>(gset)->translation_threshold(id);
//    huber_epsilon = dynamic_cast<set_vis*>(gset)->huber_epsilon(id);
//    estimation_precision = dynamic_cast<set_vis*>(gset)->estimation_precision(id);
//    initial_damping = dynamic_cast<set_vis*>(gset)->initial_damping(id);
//    outler_loop_max_iteration = dynamic_cast<set_vis*>(gset)->outler_loop_max_iteration(id);
//    inner_loop_max_iteration = dynamic_cast<set_vis*>(gset)->inner_loop_max_iteration(id);
//}

bool hwa_vis::vis_feature::triangulatePoint(const CamStateServer& cam_states) const
{
    const FeatureIDType& first_cam_id = observations.begin()->first;
    const FeatureIDType& last_cam_id = (--observations.end())->first;


    SE3 first_cam_pose; /// R_c_w, camera to world
    first_cam_pose.linear() =
        //cam_states.find(first_cam_id)->second.orientation.toRotationMatrix().transpose();
        cam_states.find(first_cam_id)->second.orientation.toRotationMatrix();
    first_cam_pose.translation() =
        cam_states.find(first_cam_id)->second.position;

    SE3 last_cam_pose; /// R_c_w, camera to world
    last_cam_pose.linear() =
        //cam_states.find(last_cam_id)->second.orientation.toRotationMatrix().transpose();
        cam_states.find(last_cam_id)->second.orientation.toRotationMatrix();
    last_cam_pose.translation() =
        cam_states.find(last_cam_id)->second.position;


    Triple feature_direction(
        observations.begin()->second(0),
        observations.begin()->second(1), 1.0);
    feature_direction = feature_direction / feature_direction.norm();
    feature_direction = first_cam_pose.linear()*feature_direction;



    Triple translation = last_cam_pose.translation() -
        first_cam_pose.translation();
    double parallel_translation =
        translation.transpose()*feature_direction;
    Triple orthogonal_translation = translation -
        parallel_translation * feature_direction;

    if (orthogonal_translation.norm() >
        translation_threshold)
        return true;
    else
        return false;
}

bool hwa_vis::vis_feature::checkMotion(const CamStateServer& cam_states) const
{
    std::vector<double> x_vector, y_vector;
    for (auto iter = observations.begin(); iter != (observations.end()); iter++)
    {
        auto iter_2 = iter;
        iter_2++;
        if (iter_2 == observations.end())
            break;
        x_vector.emplace_back(iter->second(0) - iter_2->second(1));
        y_vector.emplace_back(iter->second(0) - iter_2->second(1));
    }
    int size_vector = x_vector.size();
    if (size_vector < 3)
    {
        return false;
    }
    double x_average = std::accumulate(x_vector.begin(), x_vector.end(), 0) / size_vector;
    double y_average = std::accumulate(y_vector.begin(), y_vector.end(), 0) / size_vector;
    double x_std = 0.0, y_std = 0.0;
    for (size_t i = 0; i < size_vector; i++)
    {
        x_std += pow(x_vector[i] - x_average, 2) / size_vector;
        y_std += pow(y_vector[i] - y_average, 2) / size_vector;
    }
    x_std = sqrt(x_std);
    y_std = sqrt(y_std);

    if (x_std > 10 || y_std > 8)
    {
        return false;
    }
    for (size_t i = 0; i < size_vector; i++)
    {
        if (x_vector[i] > x_average + (x_std * 3) || x_vector[i] < x_average - (x_std * 3))
        {
            return false;
        }
        if (y_vector[i] > y_average + (y_std * 3) || y_vector[i] < y_average - (y_std * 3))
        {
            return false;
        }
    }
    return true;
}

void hwa_vis::vis_feature::generateInitialGuess(
    const SE3& T_c1_c2, const Eigen::Vector2d& z1,
    const Eigen::Vector2d& z2, Triple& p) const
{

    Triple m = T_c1_c2.linear() * Triple(z1(0), z1(1), 1.0);

    Eigen::Vector2d A(0.0, 0.0);
    A(0) = m(0) - z2(0)*m(2);
    A(1) = m(1) - z2(1)*m(2);

    Eigen::Vector2d b(0.0, 0.0);
    b(0) = z2(0)*T_c1_c2.translation()(2) - T_c1_c2.translation()(0);
    b(1) = z2(1)*T_c1_c2.translation()(2) - T_c1_c2.translation()(1);

    double depth = (A.transpose() * A).inverse() * A.transpose() * b;
    p(0) = z1(0) * depth;
    p(1) = z1(1) * depth;
    p(2) = depth;
    return;
}

void hwa_vis::vis_feature::cost(const SE3& T_c0_ci,
    const Triple& x, const Eigen::Vector2d& z,
    double& e) const
{

    const double& alpha = x(0);
    const double& beta = x(1);
    const double& rho = x(2);

    Triple h = T_c0_ci.linear()*
        Triple(alpha, beta, 1.0) + rho * T_c0_ci.translation();
    double& h1 = h(0);
    double& h2 = h(1);
    double& h3 = h(2);


    Eigen::Vector2d z_hat(h1 / h3, h2 / h3);


    e = (z_hat - z).squaredNorm();
    return;
}

void hwa_vis::vis_feature::jacobian(const SE3& T_c0_ci,
    const Triple& x, const Eigen::Vector2d& z,
    Eigen::Matrix<double, 2, 3>& J, Eigen::Vector2d& r,
    double& w) const
{


    const double& alpha = x(0);
    const double& beta = x(1);
    const double& rho = x(2);

    Triple h = T_c0_ci.linear()*
        Triple(alpha, beta, 1.0) + rho * T_c0_ci.translation();
    double& h1 = h(0);
    double& h2 = h(1);
    double& h3 = h(2);


    SO3 W;
    W.leftCols(2) = T_c0_ci.linear().leftCols(2);
    W.rightCols(1) = T_c0_ci.translation();

    J.row(0) = 1 / h3 * W.row(0) - h1 / (h3*h3)*W.row(2);
    J.row(1) = 1 / h3 * W.row(1) - h2 / (h3*h3)*W.row(2);


    Eigen::Vector2d z_hat(h1 / h3, h2 / h3);
    r = z_hat - z;


    double e = r.norm();
    if (e <= huber_epsilon)
        w = 1.0;
    else
        w = sqrt(2 * huber_epsilon / e);
        //w = huber_epsilon / (2 * e);
        //w = huber_epsilon / e;
    return;
}



bool hwa_vis::vis_feature::initializePosition(const CamStateServer& cam_states)
{    
    // for keyframe ,need to keep
    if (observations.size() < 5) return false;

    std::vector<SE3,
        Eigen::aligned_allocator<SE3> > cam_poses(0);
    std::vector<Eigen::Vector2d,
        Eigen::aligned_allocator<Eigen::Vector2d> > measurements(0);

    for (auto& m : observations)
    {
        auto cam_state_iter = cam_states.find(m.first);
        if (cam_state_iter == cam_states.end())
            continue;


        measurements.push_back(m.second.head<2>());
        if(stereo)
        measurements.push_back(m.second.tail<2>());


        SE3 cam0_pose;
        //cam0_pose.linear() = cam_state_iter->second.orientation.toRotationMatrix().transpose();  /// R_c_w, camera to world
        cam0_pose.linear() = cam_state_iter->second.orientation.toRotationMatrix();
        cam0_pose.translation() = cam_state_iter->second.position;

        SE3 cam1_pose;
        if (stereo)
        cam1_pose = cam0_pose * T_cam0_cam1.inverse();


        cam_poses.push_back(cam0_pose);
        if (stereo)
        cam_poses.push_back(cam1_pose);
    }


    SE3 T_c0_w = cam_poses[0];
    for (auto& pose : cam_poses)
        pose = pose.inverse() * T_c0_w;
    // c0->ci


    Triple initial_position(0.0, 0.0, 0.0);
    
    generateInitialGuess(cam_poses[cam_poses.size() - 1], measurements[0],
        measurements[measurements.size() - 1], initial_position);

    Triple solution(
        initial_position(0) / initial_position(2),
        initial_position(1) / initial_position(2),
        1.0 / initial_position(2));



    double lambda = initial_damping;
    int inner_loop_cntr = 0;
    int outer_loop_cntr = 0;
    bool is_cost_reduced = false;
    double delta_norm = 0;


    double total_cost = 0.0;
    for (int i = 0; i < cam_poses.size(); ++i)
    {
        double this_cost = 0.0;
        cost(cam_poses[i], solution, measurements[i], this_cost);
        total_cost += this_cost;
    }
    do
    {
        SO3 A = SO3::Zero();
        Triple b = Triple::Zero();

        for (int i = 0; i < cam_poses.size(); ++i)
        {
            Eigen::Matrix<double, 2, 3> J;
            Eigen::Vector2d r;
            double w;


            jacobian(cam_poses[i], solution, measurements[i], J, r, w);


            if (w == 1)
            {
                A += J.transpose() * J;
                b += J.transpose() * r;
            }
            else
            {
                double w_square = w * w;
                A += w_square * J.transpose() * J;
                b += w_square * J.transpose() * r;
            }
        }
        do
        {
            SO3 damper = lambda * SO3::Identity();
            Triple delta = (A + damper).ldlt().solve(b);
            Triple new_solution = solution - delta;
            delta_norm = delta.norm();

            double new_cost = 0.0;
            for (int i = 0; i < cam_poses.size(); ++i)
            {
                double this_cost = 0.0;
                cost(cam_poses[i], new_solution, measurements[i], this_cost);
                new_cost += this_cost;
            }

            if (new_cost < total_cost)
            {
                is_cost_reduced = true;
                solution = new_solution;
                total_cost = new_cost;
                lambda = lambda / 10 > 1e-10 ? lambda / 10 : 1e-10;
            }
            else
            {
                is_cost_reduced = false;
                lambda = lambda * 10 < 1e12 ? lambda * 10 : 1e12;
            }

        } while (inner_loop_cntr++ <
            inner_loop_max_iteration && !is_cost_reduced);
        inner_loop_cntr = 0;

    } while (outer_loop_cntr++ <
        outler_loop_max_iteration &&
        delta_norm > estimation_precision);

    Triple final_position(solution(0) / solution(2),
        solution(1) / solution(2), 1.0 / solution(2));
    bool is_valid_solution = true;
    for (const auto& pose : cam_poses)
    {
        Triple position =
            pose.linear()*final_position + pose.translation();
        if (position(2) <= 0)
        {
            is_valid_solution = false;
            break;
        }

    }
    position = T_c0_w.linear()*final_position + T_c0_w.translation();

    if (is_valid_solution)
        is_initialized = true;
    return is_valid_solution;
}


