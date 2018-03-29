#include "FileFunctions.h"
#include "STEP.h"
#include <string>
#include <algorithm>
#include <time.h>
#include <windows.h>

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
    time_t seconds=time(NULL);
    struct tm TimeStruct;

    gmtime_s(&TimeStruct,&seconds);

    int nYear=TimeStruct.tm_year+1900;
    int nMonth=TimeStruct.tm_mon+1;
    int nDay=TimeStruct.tm_mday;
    int nHour=TimeStruct.tm_hour;
    int nMinute=TimeStruct.tm_min;
    int nSecond=TimeStruct.tm_sec;

    char Buf[25];
    sprintf_s(Buf,25,"%4d-%02d-%02dT%02d:%02d:%02d",nYear,nMonth,nDay,nHour,nMinute,nSecond);
    return Buf;
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
    }