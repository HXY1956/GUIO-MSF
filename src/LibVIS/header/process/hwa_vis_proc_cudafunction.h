#ifndef hwa_vis_proc_cudafunction_h
#define hwa_vis_proc_cudafunction_h
#include <opencv2/core/cuda.hpp>
#include <cuda_runtime.h>
#include <opencv2/core/cuda_types.hpp>
#include <opencv2/core/types.hpp> 
#include <opencv2/core/mat.hpp> 
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/scan.h>
#include <thrust/device_ptr.h>
#include <thrust/reduce.h>

namespace gcuda {
    __device__ __forceinline__
        void setPixelPitch(cv::cuda::PtrStepSz<uchar3> img, int x, int y, uchar3 c);
    __device__ void drawSolidCircle(cv::cuda::PtrStepSz<uchar3> img,
        int cx, int cy, int r, uchar3 col);
    __device__  void drawLineKernel(cv::cuda::PtrStepSz<uchar3> img,
        int x0, int y0, int x1, int y1, uchar3 color);
    __global__ void DrawLine(
        cv::cuda::PtrStepSz<uchar3> img,
        const float2* __restrict__ prev_pts0,
        const float2* __restrict__ curr_pts0,
        int* __restrict__ _lifetime,
        uchar3 color_tracked,
        int N, int x_offset = 0
    );

    __global__ void DrawCircle(
        cv::cuda::PtrStepSz<uchar3> img,
        const float2* __restrict__ pts0,
        int* __restrict__ _lifetime,
        uchar3 color_tracked,
        uchar3 color_new,
        int N, int radius, int x_offset_pts = 0
    );

    __global__ void markValidPointsByStatus(const cv::cuda::PtrStepSz<uchar> status, int* __restrict__ flags, int N);
    __global__ void markValidPointsByRange(const float2* __restrict__ curr_pts, int* __restrict__ flags, int N, int _width, int _height);
    __global__ void markValidPointsByOutlier(const float2* __restrict__ prev_pts,
        const float2* __restrict__ curr_pts,
        int* __restrict__ flags,
        const double* E,
        int N, float threshold);
    __global__ void SeperatePointIntoGrid(
        const float2* __restrict__ pts,
        int* __restrict__ area,
        int grid_row, int grid_col,
        int grid_height, int grid_width,
        int N);
    template <typename T>
    __global__ void compactPoints_vec(const T* preparam,
        const int* valid_flags,
        const int* prefix_sum,
        T* currparam,
        int N);
    __global__ void compactPoints(const cv::cuda::PtrStepSz<float2> srcPts,
        const int* valid_flags,
        const int* prefix_sum,
        cv::cuda::PtrStepSz<float2> dstPts,
        int N);
    template <typename T>
    void _download(cv::cuda::GpuMat pts, std::vector<T>& _pts) {
        cv::Mat ptsMat;
        pts.download(ptsMat);
        _pts = std::vector<T>(ptsMat.total());
        memcpy(_pts.data(), ptsMat.ptr<cv::Point2f>(), ptsMat.total() * sizeof(cv::Point2f));
    };
    template<class T>
    static int cvTypeOf() {
        if constexpr (std::is_same_v<T, cv::Point2f>) return CV_32FC2;
        else if constexpr (std::is_same_v<T, cv::Point2d>) return CV_64FC2;
        else return cv::DataType<T>::type;
    }
    template <typename T>
    void _upload(cv::cuda::GpuMat& pts, const std::vector<T>& _pts) {
        cv::Mat ptsMat(1, (int)_pts.size(), cvTypeOf<T>(), const_cast<T*>(_pts.data()));
        pts.upload(ptsMat);
    }
    cv::cuda::GpuMat concat(cv::cuda::GpuMat pts, cv::cuda::GpuMat pts_new, cv::cuda::Stream& stream);
    template <typename T>
    void appendDeviceVector(thrust::device_vector<T>& dst, const thrust::device_vector<T>& src, const cv::cuda::Stream& stream);
    int sum_flags(int* d_flags, int N);
}

#endif