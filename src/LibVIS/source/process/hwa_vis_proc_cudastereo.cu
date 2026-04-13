#include "hwa_vis_proc_stereolk.h"

__device__ __forceinline__
void gcuda::setPixelPitch(cv::cuda::PtrStepSz<uchar3> img, int x, int y, uchar3 c) {
    if ((unsigned)x < (unsigned)img.cols && (unsigned)y < (unsigned)img.rows)
        img(y, x) = c;
}

__device__ void gcuda::drawSolidCircle(cv::cuda::PtrStepSz<uchar3> img,
    int cx, int cy, int r, uchar3 col) {
    const int r2 = r * r;
    for (int dy = -r; dy <= r; ++dy) {
        int y = cy + dy;
        int dx_max = (int)ceilf(sqrtf((float)(r2 - dy * dy)));;
        int xL = cx - dx_max;
        int xR = cx + dx_max;
        for (int x = xL; x <= xR; ++x) {
            setPixelPitch(img, x, y, col);
        }
    }
}

__device__ void gcuda::drawLineKernel(cv::cuda::PtrStepSz<uchar3> img,
    int x0, int y0, int x1, int y1, uchar3 col){
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        setPixelPitch(img, x0, y0, col);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

__global__ void gcuda::DrawLine(
    cv::cuda::PtrStepSz<uchar3> img,
    const float2* __restrict__ prev_pts0,
    const float2* __restrict__ curr_pts0,
    int* __restrict__ _lifetime,
    uchar3 color_tracked,
    int N, int x_offset
){
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N || _lifetime[i] <= 1) return;
    gcuda::drawLineKernel(img,prev_pts0[i].x + x_offset, prev_pts0[i].y, curr_pts0[i].x + x_offset, curr_pts0[i].y, color_tracked);
}

__global__ void gcuda::DrawCircle(
    cv::cuda::PtrStepSz<uchar3> img,
    const float2* __restrict__ pts0,
    int* __restrict__ _lifetime,
    uchar3 color_tracked,
    uchar3 color_new,
    int N, int radius, int x_offset_pts
){
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    const bool tracked = _lifetime[i] > 1;
    const uchar3 col = tracked ? color_tracked : color_new;

    int x0 = (int)ceilf(pts0[i].x + x_offset_pts);
    int y0 = (int)ceilf(pts0[i].y);

    drawSolidCircle(img, x0, y0, radius, col);
}

int gcuda::sum_flags(int* d_flags, int N) {
    thrust::device_ptr<int> dev_ptr(d_flags);
    return thrust::reduce(dev_ptr, dev_ptr + N, 0, thrust::plus<int>());
}

__global__ void gcuda::markValidPointsByOutlier(
    const float2* __restrict__ prev_pts,
    const float2* __restrict__ curr_pts,
    int* __restrict__ flags,
    const double* E,
    int N, float threshold)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float x0 = prev_pts[i].x, y0 = prev_pts[i].y;
    float x1 = curr_pts[i].x, y1 = curr_pts[i].y;

    float a = E[0] * x0 + E[1] * y0 + E[2];
    float b = E[3] * x0 + E[4] * y0 + E[5];
    float c = E[6] * x0 + E[7] * y0 + E[8];

    float num = fabsf(a * x1 + b * y1 + c);
    float den = sqrtf(a * a + b * b) + 1e-12f;
    flags[i] = flags[i] && (num / den <= threshold);
}


__global__ void gcuda::markValidPointsByStatus(const cv::cuda::PtrStepSz<uchar> status, int* __restrict__ flags, int N) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < N) {
        flags[idx] = flags[idx] && (status(0, idx) != 0);
    }
}

__global__ void gcuda::markValidPointsByRange(const float2* __restrict__ curr_pts, int* __restrict__ flags, int N, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < N) {
        bool flag = curr_pts[idx].y < 0 ||
            curr_pts[idx].y > height - 1 ||
            curr_pts[idx].x < 0 ||
            curr_pts[idx].x > width - 1;
        flags[idx] = flags[idx] && !flag;
    }
}

__global__ void gcuda::compactPoints(
    const cv::cuda::PtrStepSz<float2> srcPts,
    const int* valid_flags,
    const int* prefix_sum,
    cv::cuda::PtrStepSz<float2> dstPts,
    int N)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < N && valid_flags[idx]) {
        int pos = prefix_sum[idx]; 
        dstPts(0, pos) = srcPts(0, idx);
    }
}

template <typename T>
__global__ void gcuda::compactPoints_vec(
    const T* preparam,
    const int* valid_flags,
    const int* prefix_sum,
    T* currparam,
    int N)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < N && valid_flags[idx]) {
        int pos = prefix_sum[idx];
        currparam[pos] = preparam[idx];
    }
}

__global__ void gcuda::SeperatePointIntoGrid(
    const float2* __restrict__ pts,
    int* __restrict__ area,
    int grid_row, int grid_col,
    int grid_height, int grid_width,
    int N)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;
    int row = pts[i].y / grid_height;
    int col = pts[i].x / grid_width;
    int code = row * grid_col + col;
    area[i] = code;
}

template <typename T>
void gcuda::appendDeviceVector(thrust::device_vector<T>& dst,
    const thrust::device_vector<T>& src,
    const cv::cuda::Stream& stream)
{
    if (src.empty()) return;
    size_t old_size = dst.size();
    size_t new_size = old_size + src.size();
    dst.resize(new_size);
    auto s = reinterpret_cast<cudaStream_t>(stream.cudaPtr());
    thrust::copy(thrust::cuda::par.on(s),
        src.begin(), src.end(),
        dst.begin() + old_size);
}

cv::cuda::GpuMat gcuda::concat(cv::cuda::GpuMat pts, cv::cuda::GpuMat pts_new, cv::cuda::Stream& stream) {
    int N = pts.cols;
    int N_new = pts_new.cols;
    if (N_new == 0)
    {
        return pts;
    }
    if (N == 0)
    {
        return pts_new;
    }
    cv::cuda::GpuMat out;
    out.create(1, N + N_new, pts.type());
    pts.copyTo(out.colRange(0, N), stream);
    pts_new.copyTo(out.colRange(N, out.cols), stream);
    return out;
}


void hwa_vis::vis_stereo_lk_gpu::download(std::vector<cv::Point2f>& cam0_inlier, std::vector<cv::Point2f>& cam1_inlier) { 
    switch (Tp)
    {
    case STEREO_MATCH:
        gcuda::_download(d_currPts0, cam0_inlier);
        gcuda::_download(d_currPts1, cam1_inlier);
        break;
    case MONO_TRACK:
        gcuda::_download(d_prevPts0, cam0_inlier);
        gcuda::_download(d_currPts0, cam1_inlier);
        break;
    case STEREO_DETECT:
        gcuda::_download(d_currPts0_new, cam0_inlier);
        gcuda::_download(d_currPts1_new, cam1_inlier);
        break;
    default:
        break;
    }
}

void hwa_vis::vis_stereo_lk_gpu::upload(std::vector<cv::Point2f>& cam0_inlier, std::vector<cv::Point2f>& cam1_inlier) {
    switch (Tp)
    {
    case STEREO_MATCH:
        gcuda::_upload(d_currPts0, cam0_inlier);
        gcuda::_upload(d_currPts1, cam1_inlier);
        break;
    case MONO_TRACK:
        gcuda::_upload(d_prevPts0, cam0_inlier);
        gcuda::_upload(d_currPts0, cam1_inlier);
        break;
    case STEREO_DETECT:
        gcuda::_upload(d_currPts0_new, cam0_inlier);
        gcuda::_upload(d_currPts1_new, cam1_inlier);
        break;
    default:
        break;
    }
}

void hwa_vis::vis_stereo_lk_gpu::setparam(cv::cuda::GpuMat& _cam0, cv::cuda::GpuMat& _cam1, cv::cuda::GpuMat& _d_status) {
    d_status = _d_status;
    int N = d_status.cols;
    valid_flags.resize(N);
    switch (Tp)
    {
    case STEREO_MATCH:
        d_currPts0 = _cam0;
        d_currPts1 = _cam1;
        break;
    case MONO_TRACK:
        d_prevPts0 = _cam0;
        d_currPts0 = _cam1;
        break;
    case STEREO_DETECT:
        d_currPts0_new = _cam0;
        d_currPts1_new = _cam1;
    default:
        break;
    }
}
void hwa_vis::vis_stereo_lk_gpu::setparam_undistorted(cv::cuda::GpuMat& _cam0, cv::cuda::GpuMat& _cam1) {
    switch (Tp)
    {
    case STEREO_MATCH:
        d_currPts0_undistorted = _cam0;
        d_currPts1_undistorted = _cam1;
        break;
    case MONO_TRACK:
        d_prevPts0_undistorted = _cam0;
        d_currPts0_undistorted = _cam1;
        break;
    case STEREO_DETECT:
        d_currPts0_new_undistorted = _cam0;
        d_currPts1_new_undistorted = _cam1;
        break;
    default:
        break;
    }
}

void hwa_vis::vis_stereo_lk_gpu::setnewptsflags() {
    base_scopedtimer timer("setnewptsflags()", TimeCostDebugOutFile, TimeCostOut);
    int cuda_threads = 256;
    int N = d_status_new.cols;
    if (N == 0) return;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;
    valid_flags_newpts.assign(N, 1);
    int* flags_ptr = thrust::raw_pointer_cast(valid_flags_newpts.data());
    std::cout << "New Flag Before: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    gcuda::markValidPointsByStatus<<<cuda_blocks, cuda_threads >>> (cv::cuda::PtrStepSz<uchar>(d_status_new), flags_ptr, N);
    std::cout << "New Flag After SortByStatus: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    gcuda::markValidPointsByRange<<<cuda_blocks, cuda_threads >>> (d_currPts1_new.ptr<float2>(), flags_ptr, N, width, height);
    std::cout << "New Flag After SortByRange: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    if (useEpipolar) {
        const float2* prev_ptr = reinterpret_cast<const float2*>(d_currPts0_new_undistorted.ptr<float>(0));
        const float2* curr_ptr = reinterpret_cast<const float2*>(d_currPts1_new_undistorted.ptr<float>(0));
        double hE[9] = {
            Epipolar(0,0), Epipolar(0,1), Epipolar(0,2),
            Epipolar(1,0), Epipolar(1,1), Epipolar(1,2),
            Epipolar(2,0), Epipolar(2,1), Epipolar(2,2)
        };
        double* dE;
        cudaMalloc(&dE, sizeof(hE));
        cudaMemcpy(dE, hE, sizeof(hE), cudaMemcpyHostToDevice);

        gcuda::markValidPointsByOutlier <<<cuda_blocks, cuda_threads >>> (prev_ptr, curr_ptr, flags_ptr, dE, N, threshold);
        std::cout << "Flag After SortByOutlier: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    }
}

void hwa_vis::vis_stereo_lk_gpu::setflags() {
    cv::cuda::GpuMat d_currPts, _d_status;
    cv::cuda::GpuMat d_prevPts_undistorted, d_currPts_undistorted;
    switch (Tp)
    {
    case STEREO_MATCH:
        d_currPts = d_currPts1;
        _d_status = d_status;
        d_prevPts_undistorted = d_currPts0_undistorted;
        d_currPts_undistorted = d_currPts1_undistorted;
        break;
    case MONO_TRACK:
        d_currPts = d_currPts0;
        _d_status = d_status;
        d_prevPts_undistorted = d_prevPts0_undistorted;
        d_currPts_undistorted = d_currPts0_undistorted;
        break;
    default:
        break;
    }
    int cuda_threads = 256;
    int N = d_currPts.cols;
    if (N == 0) return;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;
    valid_flags.assign(N, 1);
    int* flags_ptr = thrust::raw_pointer_cast(valid_flags.data());
    std::cout << "Flag Before: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    gcuda::markValidPointsByStatus<<<cuda_blocks, cuda_threads>>> (cv::cuda::PtrStepSz<uchar>(_d_status), flags_ptr, N);
    std::cout << "Flag After SortByStatus: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    gcuda::markValidPointsByRange<<<cuda_blocks, cuda_threads>>> (d_currPts.ptr<float2>(), flags_ptr, N, width, height);
    std::cout << "Flag After SortByRange: " << gcuda::sum_flags(flags_ptr, N) << std::endl;

    if (useEpipolar) {
        const float2* prev_ptr = reinterpret_cast<const float2*>(d_prevPts_undistorted.ptr<float>(0));
        const float2* curr_ptr = reinterpret_cast<const float2*>(d_currPts_undistorted.ptr<float>(0));
        double hE[9] = {
            Epipolar(0,0), Epipolar(0,1), Epipolar(0,2),
            Epipolar(1,0), Epipolar(1,1), Epipolar(1,2),
            Epipolar(2,0), Epipolar(2,1), Epipolar(2,2)
        };
        double* dE;
        cudaMalloc(&dE, sizeof(hE));
        cudaMemcpy(dE, hE, sizeof(hE), cudaMemcpyHostToDevice);
        gcuda::markValidPointsByOutlier <<<cuda_blocks, cuda_threads>>> (prev_ptr, curr_ptr, flags_ptr, dE, N, threshold);
        std::cout << "Flag After SortByOutlier: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    } 
}

void hwa_vis::vis_stereo_lk_gpu::filterNewPointsGPU()
{
    base_scopedtimer timer("filterNewPointsGPU()", TimeCostDebugOutFile, TimeCostOut);
    int cuda_threads = 256;
    int N = d_currPts0_new.cols;
    if (N == 0) return;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;
    cudaDeviceSynchronize();
    thrust::device_vector<int> prefix_sum(N);
    thrust::exclusive_scan(valid_flags_newpts.begin(), valid_flags_newpts.end(), prefix_sum.begin());
    int last_flag, last_prefix;
    cudaMemcpy(&last_flag, thrust::raw_pointer_cast(valid_flags_newpts.data() + N - 1), sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&last_prefix, thrust::raw_pointer_cast(prefix_sum.data() + N - 1), sizeof(int), cudaMemcpyDeviceToHost);
    int valid_count = last_prefix + last_flag;

    cv::cuda::GpuMat d_currPts0_final; d_currPts0_final.create(1, valid_count, d_currPts0_new.type());
    cv::cuda::GpuMat d_currPts1_final; d_currPts1_final.create(1, valid_count, d_currPts1_new.type());
    gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
        cv::cuda::PtrStepSz<float2>(d_currPts0_new), thrust::raw_pointer_cast(valid_flags_newpts.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        cv::cuda::PtrStepSz<float2>(d_currPts0_final), N);
    gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
        cv::cuda::PtrStepSz<float2>(d_currPts1_new), thrust::raw_pointer_cast(valid_flags_newpts.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        cv::cuda::PtrStepSz<float2>(d_currPts1_final), N);
    d_currPts0_new = d_currPts0_final;
    d_currPts1_new = d_currPts1_final;

    thrust::device_vector<int> lifetime_final;
    thrust::device_vector<int> ids_final;
    thrust::device_vector<int> area_final;
    lifetime_final.assign(valid_count, 0);
    ids_final.assign(valid_count, 0);
    area_final.assign(valid_count, 0);
    gcuda::compactPoints_vec<int> <<<cuda_blocks, cuda_threads >>> (
        thrust::raw_pointer_cast(lifetime_newpts.data()), thrust::raw_pointer_cast(valid_flags_newpts.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        thrust::raw_pointer_cast(lifetime_final.data()), N);
    gcuda::compactPoints_vec<int> <<<cuda_blocks, cuda_threads >>> (
        thrust::raw_pointer_cast(ids_newpts.data()), thrust::raw_pointer_cast(valid_flags_newpts.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        thrust::raw_pointer_cast(ids_final.data()), N);
    gcuda::compactPoints_vec<int> <<<cuda_blocks, cuda_threads >>> (
        thrust::raw_pointer_cast(area_newpts.data()), thrust::raw_pointer_cast(valid_flags_newpts.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        thrust::raw_pointer_cast(area_final.data()), N);
    lifetime_newpts = lifetime_final;
    ids_newpts = ids_final;
    area_newpts = area_final;
    valid_flags_newpts.assign(valid_count, 0);
    cudaDeviceSynchronize();
}

void hwa_vis::vis_stereo_lk_gpu::filterPointsGPU()
{
    int cuda_threads = 256;
    int N = d_currPts0.cols;
    if (N == 0) return;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;
    cudaDeviceSynchronize();
    thrust::device_vector<int> prefix_sum(N);
    thrust::exclusive_scan(valid_flags.begin(), valid_flags.end(), prefix_sum.begin());
    int last_flag, last_prefix;
    cudaMemcpy(&last_flag, thrust::raw_pointer_cast(valid_flags.data() + N - 1), sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&last_prefix, thrust::raw_pointer_cast(prefix_sum.data() + N - 1), sizeof(int), cudaMemcpyDeviceToHost);
    int valid_count = last_prefix + last_flag;

    cv::cuda::GpuMat d_prevPts0_final; d_prevPts0_final.create(1, valid_count, d_prevPts0.type());
    cv::cuda::GpuMat d_currPts0_final; d_currPts0_final.create(1, valid_count, d_currPts0.type());
    cv::cuda::GpuMat d_prevPts1_final; d_prevPts1_final.create(1, valid_count, d_prevPts1.type());
    cv::cuda::GpuMat d_currPts1_final; d_currPts1_final.create(1, valid_count, d_currPts1.type());

    switch (F)
    {
    case MATCH_FILTER:
        gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
            cv::cuda::PtrStepSz<float2>(d_currPts0), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
            cv::cuda::PtrStepSz<float2>(d_currPts0_final), N);
        gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
            cv::cuda::PtrStepSz<float2>(d_currPts1), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
            cv::cuda::PtrStepSz<float2>(d_currPts1_final), N);
        break;
    case TRACK_FILTER:
        gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
            cv::cuda::PtrStepSz<float2>(d_prevPts0), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
            cv::cuda::PtrStepSz<float2>(d_prevPts0_final), N);
        gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
            cv::cuda::PtrStepSz<float2>(d_currPts0), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
            cv::cuda::PtrStepSz<float2>(d_currPts0_final), N);
        break;
    case TOTAL_FILTER:
        gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
            cv::cuda::PtrStepSz<float2>(d_currPts0), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
            cv::cuda::PtrStepSz<float2>(d_currPts0_final), N);
        gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
            cv::cuda::PtrStepSz<float2>(d_currPts1), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
            cv::cuda::PtrStepSz<float2>(d_currPts1_final), N);
        gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
            cv::cuda::PtrStepSz<float2>(d_prevPts0), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
            cv::cuda::PtrStepSz<float2>(d_prevPts0_final), N);
        gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
            cv::cuda::PtrStepSz<float2>(d_prevPts1), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
            cv::cuda::PtrStepSz<float2>(d_prevPts1_final), N);
    default:
        break;
    }
    d_prevPts0 = d_prevPts0_final;
    d_currPts0 = d_currPts0_final;
    d_prevPts1 = d_prevPts1_final;
    d_currPts1 = d_currPts1_final;

    thrust::device_vector<int> lifetime_final;
    thrust::device_vector<int> ids_final;
    thrust::device_vector<int> area_final;
    lifetime_final.assign(valid_count, 0);
    ids_final.assign(valid_count, 0);
    area_final.assign(valid_count, 0);

    gcuda::compactPoints_vec<int> <<<cuda_blocks, cuda_threads >>> (
        thrust::raw_pointer_cast(lifetime.data()), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        thrust::raw_pointer_cast(lifetime_final.data()), N);
    gcuda::compactPoints_vec<int> <<<cuda_blocks, cuda_threads >>> (
        thrust::raw_pointer_cast(ids.data()), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        thrust::raw_pointer_cast(ids_final.data()), N);
    gcuda::compactPoints_vec<int> <<<cuda_blocks, cuda_threads >>> (
        thrust::raw_pointer_cast(area.data()), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        thrust::raw_pointer_cast(area_final.data()), N);

    lifetime = lifetime_final;
    ids = ids_final;
    area = area_final;
    valid_flags.assign(valid_count, 0);
    cudaDeviceSynchronize();
}

void hwa_vis::vis_stereo_lk_gpu::InitNewPts() {
    base_scopedtimer timer("InitNewPts()", TimeCostDebugOutFile, TimeCostOut);
    int N = d_currPts0_new.cols;
    if (N == 0) return;
    valid_flags_newpts.assign(N, 0);
    lifetime_newpts.assign(N, 0);
    ids_newpts.assign(N, 0);
    area_newpts.assign(N, 0);
    cudaDeviceSynchronize();
}

void hwa_vis::vis_stereo_lk_gpu::Init() {
    int N = d_currPts0.cols;
    if (N == 0) return;

    d_status.create(1, N, CV_8UC1);
    d_status.setTo(cv::Scalar(1));
    d_currPts1.create(1, N, d_currPts0.type());
    d_prevPts0.create(1, N, d_currPts0.type());
    d_prevPts1.create(1, N, d_currPts0.type());

    valid_flags.assign(N, 0);
    lifetime.assign(N, 0);
    ids.assign(N, 0);
    area.assign(N, 0);
    cudaDeviceSynchronize();
}

void hwa_vis::vis_stereo_lk_gpu::InitializeGrid() {
    int N = d_currPts0.cols;
    if (N == 0) return;
    int cuda_threads = 256;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;
    int* area_ptr = thrust::raw_pointer_cast(area.data());
    const float2* prev_ptr = reinterpret_cast<const float2*>(d_currPts0.ptr<float>(0));
    gcuda::SeperatePointIntoGrid<<<cuda_blocks,cuda_threads>>>(prev_ptr, area_ptr,
        grid_row, grid_col, grid_height, grid_width, N);
    cudaDeviceSynchronize();
    valid_flags.assign(N, 0);
    thrust::host_vector<int> area_host(area.begin(), area.end());
    thrust::host_vector<int> _valid_flags(valid_flags.begin(), valid_flags.end());
    thrust::host_vector<int> _ids(ids.begin(), ids.end());
    thrust::host_vector<int> _lifetime(lifetime.begin(), lifetime.end());
    for (int i = 0; i < N; i++) {
        int _area = area_host[i];
        if (Grid.find(_area) == Grid.end()) {
            Grid[_area] = 1;
            _ids[i] = next_feature_id++;
            _lifetime[i]++;
            _valid_flags[i] = true;
        }
        else if (Grid[_area] >= grid_min_feature_num) {
            _valid_flags[i] = false;
        }
        else {
            Grid[_area]++;
            _ids[i] = next_feature_id++;
            _lifetime[i]++;
            _valid_flags[i] = true;
        }
    }
    valid_flags = _valid_flags;
    ids = _ids;
    lifetime = _lifetime;
    setTrackType(STEREO_MATCH);
    filterPointsGPU();
}

void hwa_vis::vis_stereo_lk_gpu::AddNewPts() {
    base_scopedtimer timer("AddNewPts()", TimeCostDebugOutFile, TimeCostOut);
    d_currPts0 = gcuda::concat(d_currPts0, d_currPts0_new, stream);
    d_currPts1 = gcuda::concat(d_currPts1, d_currPts1_new, stream);
    gcuda::appendDeviceVector(lifetime, lifetime_newpts, stream);
    gcuda::appendDeviceVector(ids, ids_newpts, stream);
    gcuda::appendDeviceVector(valid_flags, valid_flags_newpts, stream);
    gcuda::appendDeviceVector(area, area_newpts, stream);
    stream.waitForCompletion();
    cudaDeviceSynchronize();
}

void hwa_vis::vis_stereo_lk_gpu::InitializeNewPtsGrid() {
    base_scopedtimer timer("InitializeNewPtsGrid()", TimeCostDebugOutFile, TimeCostOut);
    InitNewPts();
    int N = d_currPts0_new.cols;
    if (N == 0) return;
    int cuda_threads = 256;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;
    int* area_ptr = thrust::raw_pointer_cast(area_newpts.data());
    const float2* prev_ptr = reinterpret_cast<const float2*>(d_currPts0_new.ptr<float>(0));
    const float2* curr_ptr = reinterpret_cast<const float2*>(d_currPts1_new.ptr<float>(0));
    gcuda::SeperatePointIntoGrid<<<cuda_blocks, cuda_threads>>> (prev_ptr, area_ptr,
        grid_row, grid_col, grid_height, grid_width, N);
    cudaDeviceSynchronize();
    thrust::host_vector<int> area_host(area_newpts.begin(), area_newpts.end());
    thrust::host_vector<int> _valid_flags(valid_flags_newpts.begin(), valid_flags_newpts.end());
    thrust::host_vector<int> _ids(ids_newpts.begin(), ids_newpts.end());
    thrust::host_vector<int> _lifetime(lifetime_newpts.begin(), lifetime_newpts.end());
   
    for (int i = 0; i < N; i++) {
        int _area = area_host[i];
        if (Grid.find(_area) == Grid.end()) {
            Grid[_area] = 1;
            _ids[i] = next_feature_id++;
            _lifetime[i]++;
            _valid_flags[i] = true;
        }
        else if (Grid[_area] >= grid_min_feature_num) {
            _valid_flags[i] = false;
        }
        else {
            Grid[_area]++;
            _ids[i] = next_feature_id++;
            _lifetime[i]++;
            _valid_flags[i] = true;
        }
    }
    valid_flags_newpts = _valid_flags;
    ids_newpts = _ids;
    lifetime_newpts = _lifetime;
    filterNewPointsGPU();
}

void hwa_vis::vis_stereo_lk_gpu::ManageGrid() {
    Grid.clear();
    int N = d_currPts0.cols;
    if (N == 0) return;
    thrust::host_vector<int> area_host(area.begin(), area.end());
    for (int i = N; i >= 0; i--) {
        int _area = area_host[i];
        if (Grid.find(_area) == Grid.end()) {
            Grid[_area] = 1;
        }
        else {
            Grid[_area]++;
        }
    }
}

void hwa_vis::vis_stereo_lk_gpu::PruneGrid() {
    int N = d_currPts0.cols;
    if (N == 0) return;
    thrust::host_vector<int> area_host(area.begin(), area.end());
    valid_flags.assign(N, 1);
    thrust::host_vector<int> _valid_flags(valid_flags.begin(),valid_flags.end());
    for (int i = N; i >= 0;  i--) {
        int area = area_host[i];
        if (Grid[area] >= grid_max_feature_num){
            _valid_flags[i] = 0;
            Grid[area]--;
        }
    }
    valid_flags = _valid_flags;
    setFilterType(MATCH_FILTER);
    filterPointsGPU();
}

void hwa_vis::vis_stereo_lk_gpu::DrawPts(std::shared_ptr<cv::cuda::GpuMat> _outimg) {
    uchar3* d_ptr = _outimg->ptr<uchar3>();
    uchar3 tracked = make_uchar3(0, 255, 255);
    uchar3 new_feature = make_uchar3(0, 255, 0); 

    const float2* curr0_ptr = reinterpret_cast<const float2*>(d_currPts0.ptr<float>(0));
    const float2* curr1_ptr = reinterpret_cast<const float2*>(d_currPts1.ptr<float>(0));
    const float2* prev0_ptr = reinterpret_cast<const float2*>(d_prevPts0.ptr<float>(0));
    const float2* prev1_ptr = reinterpret_cast<const float2*>(d_prevPts1.ptr<float>(0));
    int* lifetime_ptr = thrust::raw_pointer_cast(lifetime.data());
    int N = d_currPts0.cols;
    int cuda_threads = 256;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;

    gcuda::DrawLine<<<cuda_blocks, cuda_threads>>> (
        cv::cuda::PtrStepSz<uchar3>(*_outimg),
        prev0_ptr, curr0_ptr, 
        lifetime_ptr,
        tracked,
        N);

    gcuda::DrawLine<<<cuda_blocks, cuda_threads>>> (
        cv::cuda::PtrStepSz<uchar3>(*_outimg),
        prev1_ptr,curr1_ptr,
        lifetime_ptr,
        tracked,
        N, _outimg->cols / 2
        );

    gcuda::DrawCircle<<<cuda_blocks, cuda_threads>>>(
        cv::cuda::PtrStepSz<uchar3>(*_outimg),
        curr0_ptr,
        lifetime_ptr,
        tracked,
        new_feature,
        N, 3, 0
    );
    gcuda::DrawCircle<<<cuda_blocks, cuda_threads>>> (
        cv::cuda::PtrStepSz<uchar3>(*_outimg),
        curr1_ptr,
        lifetime_ptr,
        tracked,
        new_feature,
        N, 3, _outimg->cols / 2
        );

    cudaDeviceSynchronize();
}