#pragma once

// Precompiled header for LibFGO.
// Contains stable third-party headers that are transitively included by every
// LibFGO translation unit. Project headers are intentionally NOT listed here
// (the full project include chain is too large for one MSVC PCH and makes the
// PCH hit the compiler internal heap limit); they stay in the .cpp includes.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
