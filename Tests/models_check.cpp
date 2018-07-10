#include <cmath>
#include <limits>
#include <string>
#include <gtest/gtest.h>
#include <SGMEntityClasses.h>
#include <SGMChecker.h>
#include <SGMPrimitives.h>
#include <SGMTranslators.h>

//#include "SGMVector.h"
//#include "SGMInterval.h"

//using SGM::UnitVector3D;
//using SGM::Interval3D;
//using SGM::Ray3D;

// absolute path to test/data directory must be defined on compile line with -D
#if !defined(SGM_MODELS_DIRECTORY)
#error Path to test data directory is undefined.
#endif

#ifdef _MSC_VER
#include <Windows.h>
#else
#include <dirent.h>
#endif

#define SGM_MODELS_PATH(filename) SGM_MODELS_DIRECTORY "/" filename

// return true the main string ends with the match string
bool ends_with(const std::string &main_str, const std::string &to_match_str) {
    return (main_str.size() >= to_match_str.size() &&
            main_str.compare(main_str.size() - to_match_str.size(), to_match_str.size(), to_match_str) == 0);
}

// return a list of STEP files in a directory
std::vector<std::string> get_stp_names_in_path(const std::string& dir)
{
    std::vector<std::string> names;

#ifdef _MSC_VER
    std::string search_path = dir + "/*.stp";
    WIN32_FIND_DATA fd;
    HANDLE hFind = ::FindFirstFile(search_path.c_str(), &fd);
    if (hFind != INVALID_HANDLE_VALUE) {
        do {
            // read all (real) files in current dir
            // , delete '!' read other 2 default dir . and ..
            if(! (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) ) {
                names.push_back(fd.cFileName);
            }
        } while(::FindNextFile(hFind, &fd));
        ::FindClose(hFind);
    }
#else
    DIR *dp;
    struct dirent *dirp;
    std::string extension(".stp");
    if ((dp  = opendir(dir.c_str())) == NULL)
        {
        std::cerr << "Error(" << errno << ") opening " << dir << std::endl;
        exit(errno);
        }
    while ((dirp = readdir(dp)) != NULL) {
        std::string name(dirp->d_name);
        if (ends_with(name, extension))
            {
            names.push_back(name);
            }
        }
    closedir(dp);
#endif
    
    return names;
}

int check_with_log(SGM::Result& result)
{
    std::vector<std::string> aLog;
    SGM::CheckOptions Options;
    SGM::CheckEntity(result,SGM::Thing(),Options,aLog);
    if(aLog.empty())
        {
        std::cerr << "Success" << std::endl;
        return 0;
        }
    std::cerr << "Error: ";
    for (std::string &log_item: aLog)
        {
        std::cerr << log_item << std::endl;
        }
    return 1;
}

void import_file_check_or_exit(std::string const &file_path)
{
    // we could do something like call check_file_is_readable(file_path); from Euclid here
    std::cerr << "Checking " << file_path << std::endl;
    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions const options;
    SGM::Result result(SGM::CreateThing());
    SGM::ReadFile(result, file_path, entities, log, options);
    if (result.GetResult() != SGM::ResultTypeOK)
        {
        std::cerr << "Error: Unable to read from file: " << file_path << std::endl;
        exit(1);
        }
    int failed = check_with_log(result);
    if (failed != 0)
        {
        exit(1);
        }
    else
        {
        exit(0);
        }
}

double square_root(double num) {
    if (num < 0.0) {
        std::cerr << "Error: Negative Input\n";
        exit(255);
        }
    return std::sqrt(num);
}


// test whether we can locate test/data subdirectory (in platform agnostic way)
TEST(DataDirectoriesCheck, test_data_exists) {
    const testing::internal::FilePath data_dir(std::string(SGM_MODELS_DIRECTORY));
    ASSERT_TRUE(data_dir.DirectoryExists());
}


TEST(ModelDeathTest, exit_code)
{
    EXPECT_EQ(0.0, square_root(0.0));
    EXPECT_EXIT(square_root(-22.0), ::testing::ExitedWithCode(255), "Error: Negative Input");
}

TEST(ModelDeathTest, sgm_models)
{
    std::string base_dir(SGM_MODELS_DIRECTORY);
    std::vector<std::string> names = get_stp_names_in_path(base_dir);
    for (const std::string& name : names)
    {
        std::string path_name = base_dir + "/" + name;
        EXPECT_EXIT(import_file_check_or_exit(path_name), ::testing::ExitedWithCode(0), "Success");
    }
}

