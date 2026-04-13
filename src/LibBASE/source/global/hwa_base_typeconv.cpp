#include <algorithm>
#include <sstream>
#include <cmath>
#include <limits>
#include <stdlib.h>
#include "hwa_base_typeconv.h"

namespace hwa_base
{
    bool double_eq(const double& a, const double& b)
    {
        //  if ( fabs(a-b)< 1e-30) return true;   // must be editeed with machine epsilon (FOLLOWING FLOAT BELLOW!)
        if (fabs(a - b) < std::numeric_limits<double>::epsilon())
            return true; // must be editeed with machine epsilon (FOLLOWING FLOAT BELLOW!)
        else
            return false;
    }

    // float equivalence according to machine capability
    // ----------
    bool float_eq(const float& a, const float& b)
    {
        if (fabs(a - b) < std::numeric_limits<float>::epsilon())
            return true;
        else
            return false;
    }

    // round double
    // ----------
    double dround(double d)
    {
        return floor(d + 0.5);
    }

    // double to std::string conversion (width/digits by iomanip can be added !!!)
    // ----------
    std::string base_type_conv::dbl2str(const double& d, int prec)
    {
        std::ostringstream out;
        out << std::fixed << std::setprecision(prec) << " " << std::setw(0) << d;
        return out.str();
    }

    // std::string to double conversion (avoiding blanks)
    // ----------
    double base_type_conv::str2dbl(const std::string& s)
    {
        return strtod(s.c_str(), NULL);
    }

    double base_type_conv::strSci2dbl(const std::string& s)
    {
        double i = 0.0;
        std::string str(s);
        substitute(str, "d", "E");
        substitute(str, "D", "E");
        std::istringstream istr(str);
        istr >> i;
        return i;
    }

    bool base_type_conv::str2bool(const std::string& s)
    {
        std::string tmp = s;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
        if (tmp == "true" || tmp == "yes")
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    std::string base_type_conv::int2str(const int& i)
    {
        std::ostringstream out;
        out << i;
        return out.str();
    }

    // integer to std::string conversion (have width)
    // ----------
    std::string base_type_conv::int2str(const int& i, const int& width)
    {
        std::ostringstream out;
        out << std::setw(width) << std::setfill('0') << i;
        return out.str();
    }

    // std::string to integer conversion (avoiding blanks)
    // ----------
    int base_type_conv::str2int(const  std::string& s)
    {
        int i = 0;
        std::istringstream istr(s);
        istr >> i;
        return i;
    }

    std::string base_type_conv::bl2str(const bool& s)
    {
        if (s)
            return std::string("TRUE");
        else
            return std::string("FALSE");
    }

    // return base_type_conv::trimmed trailing spaces and changes the content
    // ----------
    std::string base_type_conv::rtrim(const  std::string& s)
    {
        std::string str;
        size_t endpos = s.find_last_not_of(" \t");
        if (std::string::npos != endpos)
            str = s.substr(0, endpos + 1);
        return str;
    }

    // return base_type_conv::trimmed std::string leading spaces
    // ----------
    std::string base_type_conv::ltrim(const std::string& s)
    {
        std::string str;
        size_t startpos = s.find_first_not_of(" \t");
        if (std::string::npos != startpos)
            str = s.substr(startpos);
        return str;
    }

    // return base_type_conv::trimmed leading and tailing spaces
    // ----------
    std::string base_type_conv::trim(const std::string& s)
    {
        return base_type_conv::ltrim(base_type_conv::rtrim(s));
    }

    // std::string base_type_conv::substitute
    // ----------
    std::size_t base_type_conv::substitute(std::string& line, const std::string& a, const std::string& b, bool caseSensitive)
    {
        size_t n = 0;

        if (caseSensitive)
        {
            std::string tmp(line);
            while ((n = line.find(a)) != std::string::npos)
            {
                tmp = line.substr(0, n) + b + line.substr(n + a.length());
                line = tmp;
            }
        }
        else
        {
            std::string lineLC(line);
            std::string findLC(a);
            transform(lineLC.begin(), lineLC.end(), lineLC.begin(), ::tolower);
            transform(findLC.begin(), findLC.end(), findLC.begin(), ::tolower);

            while ((n = lineLC.find(findLC)) != std::string::npos)
            {
                lineLC = line.substr(0, n) + b + line.substr(n + a.length());
                line = lineLC;
            }
        }

        return n + b.length();
    }

    // Frac part of double
    // ------------------------
    double base_type_conv::frac(double x)
    {
        return x - floor(x);
    }

    // Get Float without Leading Zero
    // ------------------------
    std::string base_type_conv::FloatWithoutZero(double x, int l)
    {
        std::stringstream ss;
        ss << std::setw(1 + l) << std::setprecision(l);
        ss << std::fixed << x;

        std::string str = ss.str();
        if (x > 0.f && x < 1.f)
            return str.substr(1, str.size() - 1);
        else if (x < 0.f && x > -1.f)
            return "-" + str.substr(2, str.size() - 1);

        return str;
    }

} // namespace
