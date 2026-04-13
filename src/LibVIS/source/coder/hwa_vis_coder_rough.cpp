#include "hwa_vis_coder_rough.h"

hwa_vis::vis_coder_rough::vis_coder_rough(hwa_set::set_base* _gset, const std::string& filepath_l, const std::string& filepath_r) :
_ts(0.1)
{
    fileL.open(filepath_l);
    fileR.open(filepath_r);
    if (filepath_r.empty()) stereo = false;
    else stereo = true;
    if (!fileL.is_open()) std::cerr << "LEFT IMG FILE NOT EXIST" << std::endl;
    if (stereo && !fileR.is_open()) std::cerr << "RIGHT IMG FILE NOT EXIST" << std::endl;
    imgs = std::make_unique<ImgMap>();
}

bool hwa_vis::vis_coder_rough::TimeDiff(double t1, double t2) {
    return abs(t1 - t2) <= _imu_ts / 2.0;
}

void hwa_vis::vis_coder_rough::add_IMG(const double& t, const std::string& img0_path, const std::string& img1_path)
{
    IMG_PATH tmp = {t, img0_path,img1_path };
    imgs->operator[](t) = tmp;
}

bool hwa_vis::vis_coder_rough::read() {
    std::string line;
    std::map<double, std::string> Lpath;
    std::map<double, std::string> Rpath;
    while (std::getline(fileL, line))
    {
        if (line.empty()) continue;
        for (auto& c : line) {
            if (c == ',' || c == ';') c = ' ';
        }
        std::string timestring;
        std::string filepath;
        std::stringstream ss(line);
        ss >> timestring >> filepath;
        double timestamp = std::stod(timestring);
        Lpath[timestamp] = filepath;
    }
    while (stereo && std::getline(fileR, line))
    {
        if (line.empty()) continue;
        for (auto& c : line) {
            if (c == ',' || c == ';') c = ' ';
        }
        std::string timestring;
        std::string filepath;
        std::stringstream ss(line);
        ss >> timestring >> filepath;
        double timestamp = std::stod(timestring);
        Rpath[timestamp] = filepath;
    }

    for (auto IterL : Lpath) {
        if (!stereo) {
            add_IMG(IterL.first, IterL.second);
            continue;
        }
        else {
            auto IterR = Rpath.find(IterL.first);
            if (IterR == Rpath.end()) continue;
            add_IMG(IterL.first, IterL.second, IterR->second);
        }
    }

    return true;
}

bool hwa_vis::vis_coder_rough::load(const double& target, IMG_PATH& img_path)
{
    auto iter = imgs->lower_bound(target);
    if (iter == imgs->end()) return false;
    if (iter == imgs->begin()) {
        double timestamp = iter->first;
        if (!TimeDiff(timestamp, target)) return false;
        img_path = iter->second;
        imgs->erase(imgs->begin(), std::next(iter));
        return true;
    }
    else {
        auto prev = std::prev(iter);
        double timestamp_prev = prev->first;
        double timestamp_curr = iter->first;
        if (TimeDiff(timestamp_prev, target)) {
            img_path = prev->second;
            imgs->erase(imgs->begin(), std::next(prev));
            return true;
        }
        else if (TimeDiff(timestamp_curr, target)) {
            img_path = iter->second;
            imgs->erase(imgs->begin(), std::next(iter));
            return true;
        }
        return false;
    }
}

