#pragma once

// Precompiled header for LibPG.
// Stable third-party / standard headers only. The DBoW2 headers in this tree
// are third-party (ORB-SLAM / DBoW2 upstream) and stable, so they are safe to
// precompile; own LibPG project headers stay out of the PCH.

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
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
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/dynamic_bitset.hpp>

#include "DBoW2.h"
#include "DVision.h"
