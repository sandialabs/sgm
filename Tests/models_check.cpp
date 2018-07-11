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
#include <future>

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

enum ModelsCheckResult: int { SUCCESS=0, FAIL_READ, FAIL_CHECK, FAIL_TIMEOUT };

static const size_t MODELS_CHECK_TIMEOUT = 60000; // milliseconds

int import_file(std::string const &file_path, SGM::Result& result)
{
    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions const options;
    SGM::ReadFile(result, file_path, entities, log, options);
    auto resultType = result.GetResult();
    if (resultType == SGM::ResultTypeOK)
        return ModelsCheckResult::SUCCESS;
    std::cerr << "Error: Unable to read from file: " << file_path << std::endl << std::flush;
    return ModelsCheckResult::FAIL_READ;
}

// perform check entity, return zero or non-zero (success or failure)
int check_with_log(std::string const &file_path, SGM::Result& result)
{
    std::vector<std::string> aLog;
    SGM::CheckOptions Options;
    SGM::CheckEntity(result,SGM::Thing(),Options,aLog);
    if (aLog.empty())
        return ModelsCheckResult::SUCCESS;
    std::cerr << "Error: check failed on file: " << file_path << std::endl << std::flush;
    for (std::string &log_item: aLog)
        std::cerr << log_item << std::endl;
    std::cerr << std::flush;
    return ModelsCheckResult::FAIL_CHECK;
}

// import file and check entity, return zero or non-zero (success or failure)
int import_check(std::string const &file_path)
{
    // this lambda will be run in another thread
    auto asyncFuture = std::async(std::launch::async, [&file_path]()->int {
        int status = ModelsCheckResult::SUCCESS;
        SGM::Result result(SGM::CreateThing());
        if (status == ModelsCheckResult::SUCCESS)
            status = import_file(file_path, result);
        if (status == ModelsCheckResult::SUCCESS)
            status = check_with_log(file_path, result);
        return status;
    });

    // run the lambda in a thread buy interrupt if it goes over the specified wait time
    if (asyncFuture.wait_for(std::chrono::milliseconds(MODELS_CHECK_TIMEOUT)) == std::future_status::timeout)
        {
        std::cerr << "Error: timed out after " << MODELS_CHECK_TIMEOUT/1000 <<
                  "s, on file: " << file_path << std::endl << std::flush;
        // forcefully exit the whole process because there is not a clean way to
        // kill the thread running the async task doing Reading and Checking
        exit(ModelsCheckResult::FAIL_TIMEOUT);
        }
    // the return value of the lambda
    int status = asyncFuture.get();
    return status;
}

void import_check_exit(std::string const &file_path)
{
    int status = import_check(file_path);
    if (status == ModelsCheckResult::SUCCESS)
        std::cerr << "Success" << std::endl;
    // if there were other reasons for failure a message will have already been written to std::cerr
    exit(status); // exit this process with the status code
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

TEST(ModelDeathTest, import_check_timeout)
{
    std::cout << std::endl << std::flush;
    std::string base_dir(SGM_MODELS_DIRECTORY);
    std::string path_name = base_dir + "/AR9 Ejector.stp";
    int status = import_check(path_name);
    if (status == ModelsCheckResult::SUCCESS)
        std::cerr << "Success" << std::endl;
    EXPECT_EQ(status, ModelsCheckResult::SUCCESS);
}

TEST(ModelDeathTest, sgm_models)
{
    std::cout << std::endl << std::flush;
    std::string base_dir(SGM_MODELS_DIRECTORY);
    std::vector<std::string> names = get_stp_names_in_path(base_dir);
    for (const std::string& name : names)
    {
        std::string path_name = base_dir + "/" + name;
        std::cout << "Checking " << path_name << std::endl << std::flush;
        EXPECT_EXIT(import_check_exit(path_name),
                    ::testing::ExitedWithCode(ModelsCheckResult::SUCCESS),
                    "Success");
    }
}

