#include "hwa_ins_proc_publish.h"

hwa_ins::ins_publish::ins_publish()
{
}

hwa_ins::ins_publish::~ins_publish()
{
    if (viewer != nullptr) delete viewer;
}

void hwa_ins::ins_publish::Initialize()
{
    
    viewer = new ins_viewer();
    viewer->Show();
}

void hwa_ins::ins_publish::UpdateNewState(const ImuState& imu_state)
{
    if (viewer != nullptr)
    {
        if (firstFlag_imu)
        {
            if (imu_state.position.norm() >= 0.0)
            {
                _init_imupos = imu_state.position;
                Triple BLH = Cart2Geod(_init_imupos, false);
                R_e_n = hwa_base::Cen(BLH).transpose();
                firstFlag_imu = false;
            }
        }
        viewer->SetFrames(std::vector<std::pair<SO3, Triple>>(0));
        std::vector<std::pair<SO3, Triple>> frames;
        Triple Position;
        SO3 atti;

        Position = R_e_n * (imu_state.position - _init_imupos);
        atti = imu_state.orientation.toRotationMatrix();
        frames.push_back(std::make_pair(atti, Position));

        //att
        viewer->SetFrames(frames);
        //trajectory
        viewer->AddNewPos(Position);
    }
}

# if (defined USE_vis) || (defined USE_LIDAR)
void hwa_ins::ins_publish::UpdateNewState(const ImuState &imu_state, const CamStateServer &cam_states, const Triple & init_campos)
{
    if (viewer != nullptr)
    {
        if (firstFlag_imu)
        {
            if (imu_state.position.norm() >= 0.0)
            {
                _init_imupos = imu_state.position;
                Triple BLH = Cart2Geod(_init_imupos, false);
                R_e_n = hwa_base::Cen(BLH).transpose();
                _init_campos = init_campos;
                firstFlag_imu = false;
            }
        }
        if (firstFlag_cam && init_campos.norm() > 0 && cam_states.size() > 0)
        {
            firstFlag_cam = false;
            _init_campos = init_campos;
        }

        viewer->SetFrames(std::vector<std::pair<SO3, Triple>>(0));
        std::vector<std::pair<SO3, Triple>> frames;
        Triple Position;
        SO3 atti;

        Position = R_e_n * (imu_state.position - _init_imupos);
        atti = imu_state.orientation.toRotationMatrix();
        frames.push_back(std::make_pair(atti, Position));
        if (cam_states.size() > 0)
        {
            for (auto & cam_state : cam_states)
            {
                Triple pos_cam;
                SO3 att_cam;
                pos_cam = R_e_n * (cam_state.second.position + _init_campos - _init_imupos);
                att_cam = R_e_n * cam_state.second.orientation.toRotationMatrix();
                frames.push_back(std::make_pair(att_cam, pos_cam));
            }

        }

        //att
        viewer->SetFrames(frames);
        //trajectory
        viewer->AddNewPos(Position);
    }

}


void hwa_ins::ins_publish::UpdateNewState(const ImuState &imu_state, const LidarStateServer &lidar_states, std::vector<Triple> lidarPointcloud)
{
    if (viewer != nullptr)
    {
        if (firstFlag_imu)
        {
            if (imu_state.position.norm() >= 0.0)
            {
                _init_imupos = imu_state.position;
                Triple BLH = Cart2Geod(_init_imupos, false);
                R_e_n = hwa_base::Cen(BLH).transpose();
                firstFlag_imu = false;
            }
        }

        viewer->SetFrames(std::vector<std::pair<SO3, Triple>>(0));
        std::vector<std::pair<SO3, Triple>> frames;
        Triple Position;
        SO3 atti;
        VPointCloud lpc;

        Position = R_e_n * (imu_state.position - _init_imupos);
        atti = imu_state.orientation.toRotationMatrix();
        frames.push_back(std::make_pair(atti, Position));
        if (lidar_states.size() > 0)
        {
            for (auto & lidar_state : lidar_states)
            {
                Triple pos_lidar;
                SO3 att_lidar;
                pos_lidar = R_e_n * (lidar_state.second.position - _init_imupos);
                att_lidar = R_e_n * lidar_state.second.orientation.toRotationMatrix();
                frames.push_back(std::make_pair(att_lidar, pos_lidar));
            }

        }

        for (auto & pt : lidarPointcloud)
        {
            pt = R_e_n * (pt - _init_imupos);
        }

        //att    
        viewer->SetFrames(frames);

        //trajectory
        viewer->AddNewPos(Position);

        //pointcloud
        viewer->SetPointCloud(lidarPointcloud);

        //Triple att = imu_state.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        //outputFile << std::setw(20) << std::setprecision(nq) << imu_state.time << ","
        //    << std::setw(nq) << std::setprecision(10) << imu_state.position(0) << ","
        //    << std::setw(nq) << std::setprecision(10) << imu_state.position(1) << ","
        //    << std::setw(nq) << std::setprecision(10) << imu_state.position(2) << ","
        //    << std::setw(nq) << std::setprecision(6) << imu_state.orientation.x() << " ,"
        //    << std::setw(nq) << std::setprecision(6) << imu_state.orientation.y() << " ,"
        //    << std::setw(nq) << std::setprecision(6) << imu_state.orientation.z() << " ,"
        //    << std::setw(nq) << std::setprecision(6) << imu_state.orientation.w() << " ,"
        //    << std::setw(nq) << std::setprecision(8) << att(0)*180.0 / M_PI << ","
        //    << std::setw(nq) << std::setprecision(8) << att(1)*180.0 / M_PI << " ,"
        //    << std::setw(nq) << std::setprecision(8) << att(2)*180.0 / M_PI << std::endl;
    }

}

void hwa_ins::ins_publish::UpdateMapPoints(const std::map<FeatureIDType, Triple> & map_points)
{
    if (viewer != nullptr)
    {
        std::vector<Triple> map_points_e;
        for (const auto& pt : map_points)
        {
            map_points_e.push_back(R_e_n*pt.second);
        }
        viewer->SetPointCloud(map_points_e);
    }
}

void hwa_ins::ins_publish::UpdateMapPoints(const std::vector<std::map<FeatureIDType, Triple>>& map_points)
{
    if (viewer != nullptr)
    {
        std::vector<std::vector<Triple>> all_map_points_e;

        for (int i = 0; i < map_points.size(); i++)
        {
            std::vector<Triple> map_points_e;
            for (const auto& pt : map_points[i])
            {
                map_points_e.push_back(R_e_n*pt.second);
            }
            all_map_points_e.push_back(map_points_e);
        }
        viewer->SetPointCloud(all_map_points_e);
    }

}
#endif

void hwa_ins::ins_publish::AddMapPoints(const std::vector<Triple> & map_points)
{
    std::vector<Triple> map_points_e;
    for (const auto& pt : map_points)
    {
        map_points_e.push_back(R_e_n*pt);
    }
    viewer->AddNewPoint(map_points_e);
}


void hwa_ins::ins_publish::UpdateVisualPoints(const std::vector<Triple> & map_points)
{
    if (viewer != nullptr)
    {
        std::vector<Triple> visual_points;
        for (auto point : map_points)
        {
            Triple vis_point = R_e_n * (point + _init_campos - _init_imupos);
            visual_points.push_back(vis_point);
        }
        //pos_cam = R_e_n * (cam_state.second.position + _init_campos - _init_imupos);
        viewer->SetPointCloud(visual_points);
    }
}

void hwa_ins::ins_publish::UpdatePlanePoints(const std::vector<Triple> & pcs, const std::vector<std::vector<Triple>> &_near_points, SO3 R_l_e, Triple t_l_e)
{
    if (viewer != nullptr)
    {
        std::vector<std::vector<Triple>> surroundings;
        std::vector<Triple> centers;
        for (int i = 0; i < pcs.size(); i++)
        {
            std::vector<Triple> one;
            centers.push_back(R_e_n*(R_l_e*pcs.at(i) + t_l_e - _init_imupos));
            for (int j = 0; j < _near_points.at(i).size(); j++)
            {
                one.push_back(R_e_n * (R_l_e*_near_points.at(i).at(j) + t_l_e - _init_imupos));
            }
            surroundings.push_back(one);
        }
        
        viewer->SetPlaneCloud(centers, surroundings);
    }

}