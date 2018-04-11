#include "FileFunctions.h"
#include "STEP.h"
#include <string>
#include <algorithm>
#include <iomanip>
#include <sstream>

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#else
#define snprintf snprintf
#endif

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

#if defined(_MSC_VER)
#include <windows.h>
#endif

#if defined(__GNUG__)
#include <sys/types.h>
#include <dirent.h>
#endif

// Returns the file extension of FileName in lower case.

void FindFileExtension(std::string const &FileName,
                       std::string       &Extension)
    {
    std::string sTemp;
    if(FileName.find_last_of(".") != std::string::npos)
        {
        sTemp=FileName.substr(FileName.find_last_of(".")+1);
        }
    Extension.resize(sTemp.size());
    std::transform(sTemp.begin(),
                   sTemp.end(),
                   Extension.begin(),
                   ::tolower);
    }

// Get current date/time, format is YYYY-MM-DDTHH:mm:ss

std::string GetDateAndTime() 
    {
    std::stringstream ss;
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    ss << std::put_time(&tm, "%F %T"); // for example, 2018-04-21 16:45:06
    return ss.str();
    }

std::string GetFileName(std::string const &FileName) 
    {
    char sep='/';
    size_t Index1=FileName.rfind(sep,FileName.length());
    if(Index1!=std::string::npos) 
       {
       return(FileName.substr(Index1+1,FileName.length()-Index1));
       }
    return("");
    }

bool ReadToString(FILE              *pFile,
                  std::string const &sData)
    {
    char const *str=sData.c_str();
    size_t nLength=sData.length();
    size_t nCount=0;
    char data;
    while(nCount<nLength)
        {
        if(fread(&data,1,1,pFile))
            {
            if(str[nCount]==data)
                {
                ++nCount;
                }
            }
        else
            {
            break;
            }
        }
    return nCount==nLength;
    }

bool ReadFileLine(FILE        *pFile,
                  std::string &sFileLine)
    {
    char data;
    while(fread(&data,1,1,pFile))
        {
        if(data!='\n')
            {
            sFileLine+=data;
            }
        if(data==';')
            {
            return true;
            }
        }
    return false;
    }

void ReadDirectory(std::string        const &DirName, 
                   std::vector<std::string> &aFileNames)
    {
        aFileNames.clear();
#if defined(_WIN64)
    std::string pattern(DirName);
    pattern.append("\\*");
    _WIN32_FIND_DATAA data;
    HANDLE hFind;
    if ((hFind = FindFirstFileA(pattern.c_str(), &data)) != INVALID_HANDLE_VALUE) 
        {
        do 
            {
            aFileNames.push_back(data.cFileName);
            } while (FindNextFileA(hFind, &data) != 0);
        FindClose(hFind);
        }
#elif defined(__GNUG__)  // true on GCC or Clang
    if (DIR *directory = opendir(DirName.c_str()))
        {
            while(struct dirent *entity = readdir(directory))
                {
                    std::string filename(entity->d_name);
                    aFileNames.push_back(filename);
                }
            closedir(directory);
        }
#else
#   error No available implementation for ReadDirectory()
#endif
    }
