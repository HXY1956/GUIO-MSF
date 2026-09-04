#ifndef HWA_PG_INTERNAL_H
#define HWA_PG_INTERNAL_H

// Internal header shared by LibPG translation units only (never exposed to
// downstream libraries). Contains the DBoW2 payload of PGKeyFrame and the
// VINS-style BRIEF extractor.

#include "hwa_pg_types.h"
#include "DBoW2.h"
#include "DVision/DVision.h"
#include <vector>
#include <string>
#include <iostream>

namespace hwa_pg {

struct PGKeyFrameDBoW {
    std::vector<DBoW2::FBrief::TDescriptor> brief_descriptors;        // FAST corners
    std::vector<DBoW2::FBrief::TDescriptor> window_brief_descriptors; // tracked features
    DBoW2::BowVector bow;
    DBoW2::FeatureVector feat;
    int entry = -1;
};

// VINS-Mono BriefExtractor: loads the pair pattern from a yml file so that
// descriptors stay compatible with a pre-trained DBoW2 vocabulary. Falls back
// to a random pattern (consistent within one run) if the file is unavailable.
class PG_BriefExtractor {
public:
    explicit PG_BriefExtractor(const std::string& pattern_file) {
        if (!pattern_file.empty()) {
            try {
                cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
                if (fs.isOpened()) {
                    std::vector<int> x1, y1, x2, y2;
                    fs["x1"] >> x1;
                    fs["x2"] >> x2;
                    fs["y1"] >> y1;
                    fs["y2"] >> y2;
                    if (!x1.empty() && x1.size() == y1.size() &&
                        x1.size() == x2.size() && x1.size() == y2.size()) {
                        m_brief_.importPairs(x1, y1, x2, y2);
                        std::cout << "[PG] loaded BRIEF pattern: " << pattern_file
                                  << " (" << x1.size() << " pairs)" << std::endl;
                        return;
                    }
                }
                std::cerr << "[PG] failed to load BRIEF pattern " << pattern_file
                          << ", using random pattern" << std::endl;
            } catch (...) {
                std::cerr << "[PG] BRIEF pattern load failed, using random pattern" << std::endl;
            }
        }
        m_brief_ = DVision::BRIEF();
    }

    void operator()(const cv::Mat& image,
                    std::vector<cv::KeyPoint>& keys,
                    std::vector<DBoW2::FBrief::TDescriptor>& descriptors) const {
        descriptors.clear();
        m_brief_.compute(image, keys, descriptors);
    }

private:
    DVision::BRIEF m_brief_;
};

} // namespace hwa_pg

#endif
