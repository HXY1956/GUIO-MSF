#ifndef hwa_fileconv_h
#define hwa_fileconv_h

#include <string>
#include <cstdio>        // mkdir/_mkdir
#include <sys/stat.h>    // mkdir mode flag
#include <direct.h>      // _mkdir (Windows)

#if defined _WIN32
#define PATH_SEPARATOR "\\"
#else
#define PATH_SEPARATOR "/"
#endif

#define HWA_FILE_PREFIX "file://"

#define MAX_PATH_LEN 256

#ifdef WIN32 //|| _WIN64
#include <io.h>
#define ACCESS(fileName, accessMode) _access(fileName, accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName, accessMode) access(fileName, accessMode)
#define MKDIR(path) mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif


namespace hwa_base
{
    std::string base_name(const std::string& path); ///< extract file base name
    std::string dir_name(const std::string& path);  ///< extract file dir  name
    std::string file_name(const std::string& path);
    bool dir_exists(const std::string& path); ///< check existance of path
    int make_path(const std::string& path);   ///< create path recursively
    int make_dir(const std::string& path);    ///< create single directory
} 

#endif
