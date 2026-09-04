#pragma once

// Precompiled header for LibVIS.
// Stable third-party / standard headers only. Project headers (hwa_vis_* /
// hwa_set_*) stay out of the PCH so the PCH is not invalidated by LibVIS
// edits. NOTE: CMake applies the PCH to C++ sources only; .cu files are still
// compiled by the CUDA compiler without /Yu.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <QDateTime>
#include <QDebug>
#include <QMetaObject>
#include <QSize>
