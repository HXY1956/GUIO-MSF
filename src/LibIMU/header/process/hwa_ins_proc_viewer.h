#ifndef hwa_ins_proc_viewer_h
#define hwa_ins_proc_viewer_h
#include <Eigen/Core>
#include <GLFW/glfw3.h>
#include <vector>
#include <utility>
#include <thread>
#include <mutex>
#include <math.h>
#include <map>

typedef std::vector<std::pair<SO3, Triple>> Frames;
typedef std::vector<Triple> VPointCloud;
typedef std::vector<Triple> Trajectory;

namespace hwa_ins
{
    class ins_viewer
    {
    public:
        ins_viewer();
        ~ins_viewer();
    public:
        void Show();
        void ClearView();
        void SetFrames(const Frames &t);
        void SetFrames(const std::vector<Frames> &vt);
        void AddNewFrame(const std::pair<SO3, Triple> &f);

        void SetPointCloud(const VPointCloud &pc);
        void SetPointCloud(const std::vector<VPointCloud> &vpc);
        void SetPlaneCloud(const std::vector<Triple> &_pcs, const std::vector<std::vector<Triple>> & _near_points);
        void AddNewPoint(const Triple &p);
        void AddNewPoint(const std::vector<Triple> &p);

        void SetTrajectory(const Trajectory &t);
        void SetTrajectory(const std::vector<Trajectory> &vt);
        void AddNewPos(const Triple &p);
        void Hide();
    private:
        void Run();

    private:
        static std::vector<ins_viewer*> vptr;
        GLFWwindow* window;
        std::thread *t;
        std::mutex m_mutex;
        std::vector<Trajectory> mv_trajectory;
        std::vector<VPointCloud> mv_pointCloud;
        std::vector<Triple> pcs;
        std::vector<std::vector<Triple>> near_points;
        std::vector<Frames> mv_frames;
    };
}
#endif