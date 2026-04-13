#ifndef hwa_vis_tracker_circle_h
#define hwa_vis_tracker_circle_h
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <thread>
#include <string>
#include <iostream>
#include <algorithm>
#include <memory>
#include "Eigen/Eigen"
#include "hwa_base_glob.h"
#include "hwa_set_base.h"
#include "hwa_set_tracker.h"
#include "hwa_vis_coder_Hicon.h"
#include "hwa_vis_coder_Daheng.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis {
    class vis_circle_tracker {
    public:
        vis_circle_tracker() {};
        vis_circle_tracker(std::shared_ptr<hwa_set::set_tracker> _gset) {
            mindist = _gset->mindist();
            canny = _gset->canny();
            acc = _gset->acc();
            minradius = _gset->minradius();
            maxradius = _gset->maxradius();
            type = _gset->type();
        };
        virtual ~vis_circle_tracker() = default;
        void openfile() {
            outfile.open(m_saveDir + "\\TrackingPos.txt", std::ios::out | std::ios::trunc);
        }
        void closefile() {
            outfile.close();
        }
        virtual bool Init() { return false; };
        void computeRectificationMaps(
            cv::Mat& map11, cv::Mat& map12,
            cv::Mat& map21, cv::Mat& map22,
            cv::Mat& Q);
        cv::Point3f reprojectTo3D(float u, float v, float d, const cv::Mat& Q);
        virtual void TwoMonoTrackingLoop() {};
        virtual void AddImage() {};
        virtual bool sequential2cam() { return false; };

        bool GetOpenStatus() { return TrackerOpenStatus; }
        template<typename T>
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cvMatToEigen(const cv::Mat& mat);
        bool GetStatus() {
            return Status;
        }
        void SetStatus(bool s) {
            Status = s;
        }
        std::string GetType() { return type; }
        void Ipush(double TrackerT, SO3 R, Triple t);
        bool Iget(double& TrakcerT, SO3& R, Triple& t);
        bool Ipop();
        bool Iempty();
        void Iclear();

        bool DetectHalconCircle(const cv::Mat& img, std::vector<cv::Point2f>& centers, std::vector<float>& radius);
        bool DetectHalconCircle(const cv::Mat& img, std::vector<cv::Point2f>& centers, std::vector<float>& radius, cv::Rect roi);
        bool TriangulateCircleCenters(const std::vector<cv::Point2f>& left_pt, const std::vector<cv::Point2f>& right_pt, std::vector<cv::Point3f>& position);
        bool SVDRegister(std::vector<cv::Point3f>& observed_pts, cv::Mat& R, cv::Mat& t);

    protected:
        bool TrackerOpenStatus = false;
        std::string m_saveDir;
        bool m_running = false;
        int m_counter = 0;
        std::ofstream outfile;

        cv::Mat K1_, K2_;
        cv::Mat R_, T_;
        cv::Mat D1_;
        cv::Mat D2_;
        cv::Size imgSize;
        std::vector<cv::Point3f> model_pts;
        std::queue<std::pair<double, std::pair<SO3, Triple>>> TrackerPVT;
        bool Status = false;
        bool intialize = false;
        std::mutex trackermtx_;
        std::mutex listmtx_;

        double mindist;
        double canny;
        double acc;
        int minradius;
        int maxradius;
        std::string type;
    };


    class vis_circle_tracker_dh : virtual public vis_circle_tracker {
    public:
        vis_circle_tracker_dh() {}
        explicit vis_circle_tracker_dh(std::shared_ptr<vis_daheng_reader> dahengCam,
            std::shared_ptr<vis_hicon_reader> hiconCam,
            const std::string& save_dir);
        virtual ~vis_circle_tracker_dh() {
            closefile();
            m_running = false;
        };

        bool Init() override;
        void TwoMonoTrackingLoop() override;
        void AddImage() override;
        bool sequential2cam() override;

    private:
        std::queue<std::pair<std::pair<int, double>, cv::Mat>> dahengImgList;
        std::queue<std::pair<std::pair<int, double>, cv::Mat>> hiconImgList;
        std::shared_ptr<vis_daheng_reader> m_dahengCam;
        std::shared_ptr<vis_hicon_reader> m_hiconCam;
    };

    class vis_circle_tracker_hh : virtual public vis_circle_tracker {
    public:
        vis_circle_tracker_hh() {}
        explicit vis_circle_tracker_hh(std::shared_ptr<vis_hicon_reader> hiconCam1,
            std::shared_ptr<vis_hicon_reader> hiconCam2,
            const std::string& save_dir);
        virtual ~vis_circle_tracker_hh() {
            closefile();
            m_running = false;
        };

        bool Init() override;
        void TwoMonoTrackingLoop() override;
        void AddImage() override;
        bool sequential2cam() override;

    private:
        std::queue<std::pair<std::pair<int, double>, cv::Mat>> hiconImgList1;
        std::queue<std::pair<std::pair<int, double>, cv::Mat>> hiconImgList2;
        std::shared_ptr<vis_hicon_reader> m_hiconCam1;
        std::shared_ptr<vis_hicon_reader> m_hiconCam2;
    };

    class vis_circle_tracker_dd : virtual public vis_circle_tracker {
    public:
        vis_circle_tracker_dd() {}
        explicit vis_circle_tracker_dd(std::shared_ptr<vis_daheng_reader> dahengCam1,
            std::shared_ptr<vis_daheng_reader> dahengCam2,
            const std::string& save_dir);
        virtual ~vis_circle_tracker_dd() {
            closefile();
            m_running = false;
        };

        bool Init() override;
        void TwoMonoTrackingLoop() override;
        void AddImage() override;
        bool sequential2cam() override;

    private:
        std::queue<std::pair<std::pair<int, double>, cv::Mat>> dahengImgList1;
        std::queue<std::pair<std::pair<int, double>, cv::Mat>> dahengImgList2;
        std::shared_ptr<vis_daheng_reader> m_dahengCam1;
        std::shared_ptr<vis_daheng_reader> m_dahengCam2;
    };
}

#endif