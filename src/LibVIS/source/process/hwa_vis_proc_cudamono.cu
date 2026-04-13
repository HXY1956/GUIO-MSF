#include "hwa_vis_proc_monolk.h"

void hwa_vis::vis_mono_lk_gpu::download(std::vector<cv::Point2f>& cam0_inlier) {
    gcuda::_download(d_currPts0_new, cam0_inlier);
}

void hwa_vis::vis_mono_lk_gpu::download(std::vector<cv::Point2f>& cam0_inlier, std::vector<cv::Point2f>& cam1_inlier) { 
    gcuda::_download(d_prevPts0, cam0_inlier);
    gcuda::_download(d_currPts0, cam1_inlier);
}

void hwa_vis::vis_mono_lk_gpu::upload(std::vector<cv::Point2f>& cam0_inlier) {
    gcuda::_upload(d_currPts0_new, cam0_inlier);
}

void hwa_vis::vis_mono_lk_gpu::upload(std::vector<cv::Point2f>& cam0_inlier, std::vector<cv::Point2f>& cam1_inlier) {
    gcuda::_upload(d_prevPts0, cam0_inlier);
    gcuda::_upload(d_currPts0, cam1_inlier);
}

void hwa_vis::vis_mono_lk_gpu::setparam(cv::cuda::GpuMat& _d_status, cv::cuda::GpuMat& _cam0) {
    d_status = _d_status;
    int N = d_status.cols;
    valid_flags.resize(N);
    d_currPts0_new = _cam0;
}

void hwa_vis::vis_mono_lk_gpu::setparam(cv::cuda::GpuMat& _d_status, cv::cuda::GpuMat& _cam0, cv::cuda::GpuMat& _cam1) {
    d_status = _d_status;
    int N = d_status.cols;
    valid_flags.resize(N);
    d_prevPts0 = _cam0;
    d_currPts0 = _cam1;
}

void hwa_vis::vis_mono_lk_gpu::setparam_undistorted(cv::cuda::GpuMat& _cam0) {
    d_currPts0_new_undistorted = _cam0;
}

void hwa_vis::vis_mono_lk_gpu::setparam_undistorted(cv::cuda::GpuMat& _cam0, cv::cuda::GpuMat& _cam1) {
    d_prevPts0_undistorted = _cam0;
    d_currPts0_undistorted = _cam1;
}

void hwa_vis::vis_mono_lk_gpu::setnewptsflags() {
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
    gcuda::markValidPointsByRange<<<cuda_blocks, cuda_threads>>> (d_currPts0_new.ptr<float2>(), flags_ptr, N, width, height);
    std::cout << "Flag After SortByRange: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
}

void hwa_vis::vis_mono_lk_gpu::setflags() {
    int cuda_threads = 256;
    int N = d_currPts0.cols;
    if (N == 0) return;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;
    valid_flags.assign(N, 1);
    int* flags_ptr = thrust::raw_pointer_cast(valid_flags.data());
    std::cout << "Flag Before: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    gcuda::markValidPointsByStatus<<<cuda_blocks, cuda_threads>>> (cv::cuda::PtrStepSz<uchar>(d_status), flags_ptr, N);
    std::cout << "Flag After SortByStatus: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    gcuda::markValidPointsByRange<<<cuda_blocks, cuda_threads>>> (d_currPts0.ptr<float2>(), flags_ptr, N, width, height);
    std::cout << "Flag After SortByRange: " << gcuda::sum_flags(flags_ptr, N) << std::endl;

    if (useEpipolar) {
        const float2* prev_ptr = reinterpret_cast<const float2*>(d_prevPts0_undistorted.ptr<float>(0));
        const float2* curr_ptr = reinterpret_cast<const float2*>(d_currPts0_undistorted.ptr<float>(0));
        double hE[9] = {
            Epipolar(0,0), Epipolar(0,1), Epipolar(0,2),
            Epipolar(1,0), Epipolar(1,1), Epipolar(1,2),
            Epipolar(2,0), Epipolar(2,1), Epipolar(2,2)
        };
        double* dE;
        cudaMalloc(&dE, sizeof(hE));
        cudaMemcpy(dE, hE, sizeof(hE), cudaMemcpyHostToDevice);
        gcuda::markValidPointsByOutlier<<<cuda_blocks, cuda_threads>>> (prev_ptr, curr_ptr, flags_ptr, dE, N, threshold);
        std::cout << "Flag After SortByOutlier: " << gcuda::sum_flags(flags_ptr, N) << std::endl;
    } 
}

void hwa_vis::vis_mono_lk_gpu::filterNewPointsGPU()
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

    cv::cuda::GpuMat d_currPts0_final; 
    d_currPts0_final.create(1, valid_count, d_currPts0_new.type());
    gcuda::compactPoints<<<cuda_blocks, cuda_threads >>> (
        cv::cuda::PtrStepSz<float2>(d_currPts0_new), thrust::raw_pointer_cast(valid_flags_newpts.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        cv::cuda::PtrStepSz<float2>(d_currPts0_final), N);
    d_currPts0_new = d_currPts0_final;

    thrust::device_vector<int> lifetime_final;
    thrust::device_vector<int> ids_final;
    thrust::device_vector<int> area_final;
    lifetime_final.assign(valid_count, 0);
    ids_final.assign(valid_count, 0);
    area_final.assign(valid_count, 0);
    gcuda::compactPoints_vec<int> <<<cuda_blocks, cuda_threads>>> (
        thrust::raw_pointer_cast(lifetime_newpts.data()), thrust::raw_pointer_cast(valid_flags_newpts.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        thrust::raw_pointer_cast(lifetime_final.data()), N);
    gcuda::compactPoints_vec<int> <<<cuda_blocks, cuda_threads>>> (
        thrust::raw_pointer_cast(ids_newpts.data()), thrust::raw_pointer_cast(valid_flags_newpts.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        thrust::raw_pointer_cast(ids_final.data()), N);
    gcuda::compactPoints_vec<int> <<<cuda_blocks, cuda_threads>>> (
        thrust::raw_pointer_cast(area_newpts.data()), thrust::raw_pointer_cast(valid_flags_newpts.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        thrust::raw_pointer_cast(area_final.data()), N);
    lifetime_newpts = lifetime_final;
    ids_newpts = ids_final;
    area_newpts = area_final;
    valid_flags_newpts.assign(valid_count, 0);
    cudaDeviceSynchronize();
}

void hwa_vis::vis_mono_lk_gpu::filterPointsGPU()
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

    gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
        cv::cuda::PtrStepSz<float2>(d_prevPts0), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        cv::cuda::PtrStepSz<float2>(d_prevPts0_final), N);
    gcuda::compactPoints <<<cuda_blocks, cuda_threads >>> (
        cv::cuda::PtrStepSz<float2>(d_currPts0), thrust::raw_pointer_cast(valid_flags.data()), thrust::raw_pointer_cast(prefix_sum.data()),
        cv::cuda::PtrStepSz<float2>(d_currPts0_final), N);

    d_prevPts0 = d_prevPts0_final;
    d_currPts0 = d_currPts0_final;

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

void hwa_vis::vis_mono_lk_gpu::InitNewPts() {
    base_scopedtimer timer("InitNewPts()", TimeCostDebugOutFile, TimeCostOut);
    int N = d_currPts0_new.cols;
    if (N == 0) return;
    valid_flags_newpts.assign(N, 0);
    lifetime_newpts.assign(N, 0);
    ids_newpts.assign(N, 0);
    area_newpts.assign(N, 0);
    cudaDeviceSynchronize();
}

void hwa_vis::vis_mono_lk_gpu::Init() {
    int N = d_currPts0.cols;
    if (N == 0) return;
    d_prevPts0 = d_currPts0.clone();
    d_status.create(1, N, CV_8UC1);
    d_status.setTo(cv::Scalar(1));
    valid_flags.assign(N, 0);
    lifetime.assign(N, 0);
    ids.assign(N, 0);
    area.assign(N, 0);
    cudaDeviceSynchronize();
}

void hwa_vis::vis_mono_lk_gpu::InitializeGrid() {
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
    filterPointsGPU();
}

void hwa_vis::vis_mono_lk_gpu::AddNewPts() {
    base_scopedtimer timer("AddNewPts()", TimeCostDebugOutFile, TimeCostOut);
    d_currPts0 = gcuda::concat(d_currPts0, d_currPts0_new, stream);
    gcuda::appendDeviceVector(lifetime, lifetime_newpts, stream);
    gcuda::appendDeviceVector(ids, ids_newpts, stream);
    gcuda::appendDeviceVector(valid_flags, valid_flags_newpts, stream);
    gcuda::appendDeviceVector(area, area_newpts, stream);
    stream.waitForCompletion();
    cudaDeviceSynchronize();
}

void hwa_vis::vis_mono_lk_gpu::InitializeNewPtsGrid() {
    base_scopedtimer timer("InitializeNewPtsGrid()", TimeCostDebugOutFile, TimeCostOut);
    InitNewPts();
    int N = d_currPts0_new.cols;
    if (N == 0) return;
    int cuda_threads = 256;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;
    int* area_ptr = thrust::raw_pointer_cast(area_newpts.data());
    const float2* prev_ptr = reinterpret_cast<const float2*>(d_currPts0_new.ptr<float>(0));
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

void hwa_vis::vis_mono_lk_gpu::ManageGrid() {
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

void hwa_vis::vis_mono_lk_gpu::PruneGrid() {
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

void hwa_vis::vis_mono_lk_gpu::DrawPts(std::shared_ptr<cv::cuda::GpuMat> _outimg) {
    uchar3* d_ptr = _outimg->ptr<uchar3>();
    uchar3 tracked = make_uchar3(0, 255, 255);
    uchar3 new_feature = make_uchar3(0, 255, 0); 

    const float2* curr0_ptr = reinterpret_cast<const float2*>(d_currPts0.ptr<float>(0));
    const float2* prev0_ptr = reinterpret_cast<const float2*>(d_prevPts0.ptr<float>(0));
    int* lifetime_ptr = thrust::raw_pointer_cast(lifetime.data());
    int N = d_currPts0.cols;
    int cuda_threads = 256;
    int cuda_blocks = (N + cuda_threads - 1) / cuda_threads;

    gcuda::DrawLine<<<cuda_blocks, cuda_threads>>> (
        cv::cuda::PtrStepSz<uchar3>(*_outimg),
        prev0_ptr,
        curr0_ptr,
        lifetime_ptr,
        tracked,
        N, 0
        );

    gcuda::DrawCircle<<<cuda_blocks, cuda_threads>>>(
        cv::cuda::PtrStepSz<uchar3>(*_outimg),
        curr0_ptr,
        lifetime_ptr,
        tracked,
        new_feature,
        N, 3, 0
    );
    cudaDeviceSynchronize();
}