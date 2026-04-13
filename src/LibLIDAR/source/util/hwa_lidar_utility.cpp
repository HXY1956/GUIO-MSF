#include "hwa_lidar_utility.h"
#include "hwa_set_lidar.h"

using namespace hwa_base;
using namespace hwa_lidar;
using namespace std;

vector<std::string> hwa_lidar::split(const string &s, const string &seperator) {
    vector<string> result;
    typedef string::size_type string_size;
    string_size i = 0;

    while (i != s.size()) {
        int flag = 0;
        while (i != s.size() && flag == 0) {
            flag = 1;
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[i] == seperator[x]) {
                    ++i;
                    flag = 0;
                    break;
                }
        }

        flag = 0;
        string_size j = i;
        while (j != s.size() && flag == 0) {
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[j] == seperator[x]) {
                    flag = 1;
                    break;
                }
            if (flag == 0)
                ++j;
        }
        if (i != j) {
            result.push_back(s.substr(i, j - i));
            i = j;
        }
    }
    return result;
}


//SO3 hwa_lidar::skew(const Triple& v)
//{
//    SO3 vnx;
//    vnx << 0, -v(2), v(1),
//        v(2), 0, -v(0),
//        -v(1), v(0), 0;
//    return vnx;
//}

//transform from n frame to e frame
SO3 hwa_lidar::R_ENU_ECEF(const Triple &BLH)
{
    SO3 ans;
    double sinB = sin(BLH(0));
    double cosB = cos(BLH(0));
    double sinL = sin(BLH(1));
    double cosL = cos(BLH(1));
    ans(0, 0) = -sinL;            ans(0, 1) = cosL;                ans(0, 2) = 0;//E
    ans(1, 0) = -cosL * sinB;        ans(1, 1) = -sinL * sinB;            ans(1, 2) = cosB;//N
    ans(2, 0) = cosL * cosB;        ans(2, 1) = sinL * cosB;            ans(2, 2) = sinB;//U
    return ans;
}

Triple hwa_lidar::XYZ2BLH(const Triple &X)
{
    double a1 = 6378137;
    double e2 = 0.00669437999013;
    double pi = 4 * atan(1);
    Triple BLH;
    double B1, B2, N;
    BLH(1) = atan(X(1) / X(0));
    BLH(1) = BLH(1);
    if (X(0) < 0 && X(1) > 0) BLH(1) = BLH(1) + pi;
    if (X(0) < 0 && X(1) < 0) BLH(1) = BLH(1) - pi;
    B1 = atan(X(2) / sqrt(X(0)*X(0) + X(1)*X(1)));
    N = a1 / sqrt(1 - e2 * sin(B1)*sin(B1));
    B2 = atan((X(2) + N * e2*sin(B1)) / sqrt(X(0)*X(0) + X(1)*X(1)));
    ///< iterative method to find the latitude of the earth
    while (abs(B1 - B2) > (0.001 / 3600 * pi / 180))
    {
        B1 = B2;
        N = a1 / sqrt(1 - e2 * sin(B1)*sin(B1));
        B2 = atan((X(2) + N * e2*sin(B1)) / sqrt(X(0)*X(0) + X(1)*X(1)));
    }
    BLH(0) = B2;
    BLH(2) = X(2) / sin(B2) - N * (1 - e2);
    return (BLH);
}

SO3 hwa_lidar::skewSymmetric(const Triple& w)
{
    SO3 w_hat;
    w_hat(0, 0) = 0;
    w_hat(0, 1) = -w(2);
    w_hat(0, 2) = w(1);
    w_hat(1, 0) = w(2);
    w_hat(1, 1) = 0;
    w_hat(1, 2) = -w(0);
    w_hat(2, 0) = -w(1);
    w_hat(2, 1) = w(0);
    w_hat(2, 2) = 0;
    return w_hat;
}

vector<Triple> hwa_lidar::PointXYZI2Triple(pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi, SO3 R_l_e, Triple t_l_e)
{
    vector<Triple> xyz;
    for (int i = 0; i < xyzi->points.size(); i++)
    {
        xyz.push_back(R_l_e*Triple(xyzi->points[i].x, xyzi->points[i].y, xyzi->points[i].z) + t_l_e);
    }
    return xyz;
}

vector<Triple> hwa_lidar::PointXYZI2Triple(pcl::PointCloud<pcl::PointXYZI> &xyzi, SO3 R_l_e, Triple t_l_e)
{
    vector<Triple> xyz;
    for (int i = 0; i < xyzi.points.size(); i++)
    {
        xyz.push_back(R_l_e*Triple(xyzi.points[i].x, xyzi.points[i].y, xyzi.points[i].z) + t_l_e);
    }
    return xyz;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr hwa_lidar::Triple2PointXYZI(vector<Triple>& xyz, SO3 R_l_e, Triple t_l_e)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ans(new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < xyz.size(); i++)
    {
        Triple trans = R_l_e * xyz.at(i) + t_l_e;
        pcl::PointXYZI tmp;
        tmp.x = trans.x(); tmp.y = trans.y(); tmp.z = trans.z();
        ans->points.push_back(tmp);
    }
    return ans;
}

//pcl::PointCloud<pcl::PointXYZI> hwa_lidar::Triple2PointXYZI(vector<Triple>& xyz)
//{
//    pcl::PointCloud<pcl::PointXYZI> ans;
//    for (int i = 0; i < xyz.size(); i++)
//    {
//        pcl::PointXYZI tmp;
//        tmp.x = xyz.at(i).x(); tmp.y = xyz.at(i).y(); tmp.z = xyz.at(i).z();
//        ans.points.push_back(tmp);
//    }
//    return ans;
//}

void hwa_lidar::transPointXYZI(pcl::PointXYZI& p, SO3 R, Triple t)
{
    Triple temp_p = R * Triple(p.x, p.y, p.z) + t;
    p.x = temp_p.x();
    p.y = temp_p.y();
    p.z = temp_p.z();
}

void hwa_lidar::transPointXYZI(pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi, SO3 R, Triple t)
{
    for (int i = 0; i < xyzi->points.size(); i++)
    {
        transPointXYZI(xyzi->points[i], R, t);
    }
}


