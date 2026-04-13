#ifndef hwa_base_common_h
#define hwa_base_common_h

#include <iostream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <sstream>
#include <stdio.h>
#include <sys/stat.h>
#include <direct.h>
#include <Shlwapi.h>
#include <iomanip>
#define GET_CURRENT_PATH _getcwd
#pragma warning(disable : 4503) // suppress Visual Studio WARNINGS 4503 about truncated decorated names

#ifndef S_ISDIR
#define S_ISDIR(mode) (((mode)&S_IFMT) == S_IFDIR)
#endif

#ifndef S_ISREG
#define S_ISREG(mode) (((mode)&S_IFMT) == S_IFREG)
#endif

namespace hwa_base
{

    static const char lf = 0x0a; ///< = "\n" .. line feed
    static const char cr = 0x0d; ///< = "\r" .. carriage return

#if defined __linux__ || defined __APPLE__
    static const std::string crlf = std::string("") + lf; ///< "\n"
#endif

#if defined _WIN32 || defined _WIN64
    static const std::string crlf = std::string("") + lf; // "\r\n"
#endif

    /** @brief cut crlf. */
    inline std::string cut_crlf(std::string s)
    {
        if (!s.empty() && s[s.length() - 1] == lf)
            s.erase(s.length() - 1);
        if (!s.empty() && s[s.length() - 1] == cr)
            s.erase(s.length() - 1); // glfeng
        return s;
    }

    /** @brief get current path. */
    inline std::string get_current_path()
    {
        char cPath[FILENAME_MAX];
        if (!GET_CURRENT_PATH(cPath, sizeof(cPath)))
            return "./";

        cPath[sizeof(cPath) - 1] = '\0'; // not really required
        return std::string(cPath);
    }

    /**
    * @brief print std::string 
    * @param[in] dir        the dirrectory need to check
    * @return bool true:yes;false:no
    */
    inline bool chk_directory(const std::string &dir)
    {
        struct stat st;

        if (stat(dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode))
            return true;
        // if( stat(dir.c_str(), &st) == 0 && (((st.st_mode) & _S_IFMT) == _S_IFDIR) ) return true;
        //  if (stat(dir.substr(0,255).c_str(), &st) == 0 && st.st_mode == _S_IFDIR ) return true;

        //  if( stat(dir.c_str(),&st) == 0 )
        //    if( st.st_mode & S_IFDIR != 0 ) return true;  //  printf(" /tmp is present\n");

        return false;
    }

    /** 
    * @brief  get clk. 
    * usage:  clock_t clk; gclk("TEXT",clk);
    */
    inline void gclk(const std::string &str, time_t &clk) //
    {
#ifdef CLOCK
        std::cout << "#"
             << std::fixed << std::setprecision(4)
             << std::setw(7) << difftime(time(0) - clk) << " sec: "
             << str << " " << std::endl;
        clk = time(0); // clock();
#endif
    }

    /** @brief this is designed for main program. */
    inline std::string clk_string(clock_t clk)
    {
        clk = clock() - clk;
        std::ostringstream os;
        os << std::fixed << std::setprecision(3) << std::setw(7) << ((float)clk) / CLOCKS_PER_SEC << " sec ";
        return os.str();
    }

#ifdef DEBUG
#define ID_FUNC printf(" .. %s\n", __func__)
#else
#define ID_FUNC
#endif

#define SQR(x) ((x) * (x))
#define SQRT(x) ((x) <= 0.0 ? 0.0 : sqrt(x))

    /**
    * @brief tail of a std::string
    * @param[in] str        original std::string
    * @param[in] length        the need length
    * @return result std::string
    */
    inline std::string tail(const std::string &str, const size_t length)
    {
        if (length >= str.size())
            return str;
        else
            return str.substr(str.size() - length);
    }
} // namespace

#endif // # GCOMMON_H

