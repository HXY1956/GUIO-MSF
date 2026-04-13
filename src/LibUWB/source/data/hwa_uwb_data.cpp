#include "hwa_uwb_data.h"
using namespace hwa_uwb;

uwb_data::uwb_data()
    :base_data(),
    _node_num(0)

{
    id_type(ID_TYPE::UWBDATA);
}

int uwb_data::add_uwb(const double& t, const std::string& id, const UWB_NODE& node)
{
    _alldata.operator[](t)[id] = node;
    id_alldata.operator[](id)[t] = node;
    return 1;
}

bool uwb_data::load(const hwa_base::base_time& crt, double dt, hwa_map_id_uwbnode& val)
{
    double t = crt.sow() + crt.dsec();

    std::map<double, hwa_map_id_uwbnode>::iterator it = _alldata.lower_bound(t);

    if (it != _alldata.end() && fabs(t - it->first) <= dt && it->first >= t)
    {
        val = it->second;
        return true;
    }
    else return false;
}

bool uwb_data::load(const hwa_base::base_time& crt, double dt, hwa_map_id_uwbnode& val, double& intv)
{
    double t = crt.sow() + crt.dsec();

    std::map<double, hwa_map_id_uwbnode>::iterator it = _alldata.lower_bound(t);

    if (it != _alldata.end() && fabs(t - it->first) <= dt && it->first >= t)
    {

        intv = -(it->first - (++it)->first);
        --it;

        val = it->second;
        return true;
    }
    else return false;
}

bool uwb_data::loadUWB(const double& t, const double& intv, hwa_map_id_uwbnode& val)
{
    val.clear();
    double threshold = 0.4;

    for (int i = 0; i < _node_num; i++)
    {
        double min_dist = intv;
        std::map<double, hwa_map_id_uwbnode>::iterator it = _alldata.lower_bound(t - intv);
        while (1)
        {
            if (it == _alldata.end() || it->first - t > threshold * intv)
                break;
            if (it != _alldata.end() && fabs(t - it->first) <= threshold * intv)
            {
                if (it->second.find(_nodes[i]) != it->second.end() && fabs(t - it->first) < min_dist)
                {
                    min_dist = fabs(t - it->first);
                    //std::cerr << std::fixed<<setprecision(3)<<it->first << std::endl;
                    val[_nodes[i]] = it->second[_nodes[i]];
                    //break;
                }
            }
            it++;
        }
    }

    if (val.size() < 3 && _nodes.size() >= 3 && iflag && (t - floor(t) < 0.005 || ceil(t) - t < 0.005)) {
        std::vector<std::pair<std::string, double>> node;
        for (auto iter : _nodes) {
            if (val.find(iter) != val.end()) continue;
            if (!interpolation(id_alldata[iter], t)) continue;
            val[iter] = id_alldata[iter][t];
            node.push_back({ iter,val[iter].range });
            if (val.size() >= 3) break;
        }
        std::cout << "Time: " << t << " Interpolation Done; " << std::endl;
        for (auto i : node) {
            std::cout << "Node " << i.first << " Range: " << i.second << std::endl;
        }
    }

    return val.size() > 0 ? true : false;
}

void uwb_data::add_nodes(const std::vector<std::string>& nodes)
{
    _nodes = nodes;
    _node_num = nodes.size();
    return;
}

bool uwb_data::interpolation(hwa_map_time_uwbnode& df, double t) {
    std::vector<std::pair<double, double>> T;
    hwa_map_time_uwbnode::iterator it = df.lower_bound(t);
    hwa_map_time_uwbnode::iterator it_before, it_after;
    double t_min, del_t0, del_t1;
    double v = vel.norm();
    if (it != df.end()) {
        for (int i = 0; i < 4; i++) {
            it_before = it;  it_before--; if (it_before == df.end()) it_before = it;
            it_after = it;  it_after++; if (it_after == df.end()) it_after = it;
            del_t0 = -it_before->first + it->first;
            del_t1 = it_after->first - it->first;
            if (abs(it_before->second.range - it->second.range) < 3 * v * del_t0 || abs(it_after->second.range - it->second.range) < 3 * v * del_t1) {
                T.push_back({ it->first, it->second.range });
                t_min = it->first;
            }
            else {
                i--;
            }
            if (it == df.begin()) break;
            --it;
        }
    }
    it = df.lower_bound(t);
    if (it != df.end()) {
        for (int i = 0; i < 2; i++) {
            ++it;
            if (it == df.end()) break;
            it_before = it;  it_before--; if (it_before == df.end()) it_before = it;
            it_after = it;  it_after++; if (it_after == df.end()) it_after = it;
            del_t0 = -it_before->first + it->first;
            del_t1 = it_after->first - it->first;
            if (abs(it_before->second.range - it->second.range) < 3 * v * del_t0 || abs(it_after->second.range - it->second.range) < 3 * v * del_t1) {
                T.push_back({ it->first, it->second.range });
            }
            else {
                i--;
            }
        }
    }

    for (auto& iter : T) {
        iter.first -= t_min;
    }
    double mean_cost = 0;
    double range = prediction(T, 2, t - t_min, mean_cost);
    std::cout << t << " mean_cost: " << mean_cost << std::endl;

    if (range > 1000) return false;

    UWB_NODE tmp;
    tmp.range = range;
    if (t < 33210)
        tmp.noise = 0.03;
    else
        tmp.noise = noise;
    //tmp.noise = mean_cost;
    tmp.SNR = -20.0 * log10(tmp.noise);
    tmp.rxRSSI = tmp.SNR - 86.9897;
    tmp.fpRSSI = tmp.SNR - 86.9897;
    df[t] = tmp;

    return true;
}

double hwa_uwb::prediction(std::vector<std::pair<double, double>> points, int degree, double x, double& mean_cost) {
    size_t n = points.size();
    Matrix A(n, degree + 1);
    Vector b(n);
    mean_cost = 0;

    for (size_t i = 0; i < n; ++i) {
        double x = points[i].first;
        double y = points[i].second;
        b(i) = y;
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = std::pow(x, j);
        }
    }

    Vector coefficients = A.colPivHouseholderQr().solve(b);

    double y_predict = 0.0;
    double y;

    for (size_t i = 0; i < n; i++) {
        y = 0;
        for (size_t j = 0; j < coefficients.size(); ++j) {
            y += coefficients[j] * std::pow(points[i].first, j);
        }
        mean_cost += abs(y - points[i].second);
    }
    mean_cost /= n;

    for (size_t i = 0; i < coefficients.size(); ++i) {
        y_predict += coefficients[i] * std::pow(x, i);
    }

    return y_predict;
}