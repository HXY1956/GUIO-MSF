#ifndef hwa_base_string_h
#define hwa_base_string_h
#include <string>
#include <vector>
#include <set>

namespace hwa_base
{
    std::string trim(std::string s);
    std::string clean(std::string s);
    void split(const std::string &s, std::string delim, std::vector<std::string> &ret);

    std::string set2str(std::set<std::string> &s);
    std::string str2upper(const std::string &str);
    std::string str2lower(const std::string &str);
    std::string dofraction(long double value, int digits);
    std::string format(const char *fmt, ...);
    std::string remove_surplus_spaces(const std::string &src);

}

#endif
