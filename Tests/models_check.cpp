#include <cmath>
#include <limits>
#include <string>
#include <future>
#include <fstream>
#include <strstream>

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

static const std::vector<std::string> SGM_MODELS_EXTENSIONS = {".stp",".STEP",".STP",".step"};

// return true the main string ends with the match string
bool ends_with(const std::string &main_str, const std::string &to_match_str) {
    return (main_str.size() >= to_match_str.size() &&
            main_str.compare(main_str.size() - to_match_str.size(), to_match_str.size(), to_match_str) == 0);
}

// true if the file_path ends with one of the extensions
bool has_extension(const std::string & file_path)
{
    for (auto const &extension : SGM_MODELS_EXTENSIONS)
        if (ends_with(file_path, extension))
            return true;
    return false;
}

// if the path ends with one of the extensions,
// return the path with the extensions erased, else return the original path
std::string erase_extension(std::string const & file_path)
{
    std::string new_path(file_path);
    for (auto const &extension : SGM_MODELS_EXTENSIONS)
        {
        size_t pos = new_path.rfind(extension);
        if (pos != std::string::npos)
            {
            new_path.erase(pos, extension.length());
            return new_path;
            }
        }
    return new_path;
}

// filter out file names from list that do not have our extensions
void erase_no_extension(std::vector<std::string>& names)
{
    names.erase(std::remove_if(names.begin(), names.end(), [](const std::string& x)
                    {
                    return !has_extension(x);
                    }),
                names.end());
}

// return a list of STEP files in a directory
std::vector<std::string> get_names_with_extension(const std::string &dir)
{
    std::vector<std::string> names;

#ifdef _MSC_VER
    std::string search_path = dir + "/*.*"; // all files with any . extension
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
    erase_no_extension(names);
#else
    DIR *dp;
    struct dirent *dirp;
    if ((dp  = opendir(dir.c_str())) == NULL)
        {
        std::cerr << "Error(" << errno << ") opening " << dir << std::endl;
        exit(errno);
        }
    while ((dirp = readdir(dp)) != NULL) {
        std::string name(dirp->d_name);
        if (has_extension(name))
            names.push_back(name);
        }
    closedir(dp);
#endif

    return names;
}

// Get current date/time, format is YYYY-MM-DD HH:mm:ss
const std::string current_date_time()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
    return buf;
}

enum ModelsCheckResult: int { SUCCESS=0, FAIL_READ, FAIL_CHECK, FAIL_TIMEOUT };

static const size_t MODELS_CHECK_TIMEOUT = 60000; // milliseconds

int import_file(std::string const &file_path, SGM::Result& result, std::ofstream & log_file)
{
    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions const options;
    SGM::ReadFile(result, file_path, entities, log, options);
    auto resultType = result.GetResult();
    if (resultType == SGM::ResultTypeOK)
        return ModelsCheckResult::SUCCESS;
    log_file << "Error: Unable to read from file: " << file_path << std::endl;
    for (std::string &log_item: log)
        log_file << "    " << log_item << std::endl;
    log_file << std::flush;
    return ModelsCheckResult::FAIL_READ;
}

// perform check entity on already imported file,
// return zero or non-zero (success or failure)
int check_file(std::string const &file_path, SGM::Result &result, std::ofstream & log_file)
{
    std::vector<std::string> aLog;
    SGM::CheckOptions Options;
    SGM::CheckEntity(result,SGM::Thing(),Options,aLog);
    if (aLog.empty())
        return ModelsCheckResult::SUCCESS;
    log_file << "Error: CheckEntity failed on file: " << file_path << std::endl;
    for (std::string &log_item: aLog)
        log_file << "    " << log_item << std::endl;
    log_file << std::flush;
    return ModelsCheckResult::FAIL_CHECK;
}

//
// This is the function that captures the import/check happens before timeout
//
// 1. Open a import file and check entity, return zero or non-zero (success or failure)
int import_check_timeout(std::string const &file_path, std::ofstream & log_file)
{
    // this lambda will be run in another thread
    auto asyncFuture = std::async(std::launch::async, [&file_path,&log_file]()->int {
        int status = ModelsCheckResult::SUCCESS;
        SGM::Result result(SGM::CreateThing());
        if (status == ModelsCheckResult::SUCCESS)
            status = import_file(file_path, result, log_file);
        if (status == ModelsCheckResult::SUCCESS)
            status = check_file(file_path, result, log_file);
        return status;
    });

    // run the lambda in a thread buy interrupt if it goes over the specified wait time
    if (asyncFuture.wait_for(std::chrono::milliseconds(MODELS_CHECK_TIMEOUT)) == std::future_status::timeout)
        {
        std::strstream message;
        message << "Error: timed out after " << MODELS_CHECK_TIMEOUT/1000 <<
                "s, on file: " << file_path;
        std::cerr << message.str() << std::endl << std::flush;
        log_file << message.str() << std::endl << std::flush;
        // forcefully exit the whole process because there is not a clean way to
        // kill the thread running the async task doing Reading and Checking
        exit(ModelsCheckResult::FAIL_TIMEOUT);
        }
    // the return value of the lambda
    int status = asyncFuture.get();
    return status;
}

void open_log_file(std::string const &file_path, std::ofstream & log_file)
{
    std::string log_path = erase_extension(file_path) + ".log";
    log_file.open(log_path);
    log_file << file_path << std::endl;
    log_file << current_date_time() << std::endl << std::flush;
}

//
// This is the function that is run as an external process by GTest
//
// 1. Open a *.log file with same prefix as the file_path
// 2. Try to import/check within the timeout.
// 3. Write to stderr device "Success" if done.
// 4. Exit the process with status code of zero for success, non-zero otherwise.
//
void import_check_log_process(std::string const &file_path)
{
    std::ofstream log_file;
    open_log_file(file_path, log_file);

    int status = import_check_timeout(file_path, log_file);

    if (status == ModelsCheckResult::SUCCESS)
        {
        std::cerr << "Success" << std::endl; // gtest will EXPECT this
        log_file << "Success" << std::endl;
        }

    log_file.close();
    exit(status); // exit this process with the status code
}

double square_root(double num) {
    if (num < 0.0) {
        std::cerr << "Error: Negative Input\n";
        exit(255);
        }
    return std::sqrt(num);
}

///////////////////////////////////////////////////////////////////////////////
//
// Tests
//
///////////////////////////////////////////////////////////////////////////////

//
// test whether we can locate test/data subdirectory (in platform agnostic way)
//
TEST(DataDirectoriesCheck, test_data_exists) {
    const testing::internal::FilePath data_dir(std::string(SGM_MODELS_DIRECTORY));
    ASSERT_TRUE(data_dir.DirectoryExists());
}

//
// test whether we can handle the list of our file extensions properly
//
TEST(DataDirectoriesCheck, file_extensions)
{
    std::vector<std::string> names = {
            "testA.doc","testB.stp","testC.txt","testD","testE.STEP","testF.otl","testG.STP","testH.step"};
    ASSERT_FALSE(has_extension(names[0]));

    erase_no_extension(names);
    ASSERT_EQ(names.size(),4);
    ASSERT_EQ(names[0],"testB.stp");
    ASSERT_EQ(names[1],"testE.STEP");
    ASSERT_EQ(names[2],"testG.STP");
    ASSERT_EQ(names[3],"testH.step");
    ASSERT_TRUE(has_extension(names[0]));
    ASSERT_TRUE(has_extension(names[1]));
    ASSERT_TRUE(has_extension(names[2]));
    ASSERT_TRUE(has_extension(names[3]));
    ASSERT_EQ(erase_extension(names[0]),"testB");
    ASSERT_EQ(erase_extension(names[1]),"testE");
    ASSERT_EQ(erase_extension(names[2]),"testG");
    ASSERT_EQ(erase_extension(names[3]),"testH");
}

//
// test whether the Gtest exit function works properly
//
TEST(ModelDeathTest, exit_code)
{
    EXPECT_EQ(0.0, square_root(0.0));
    EXPECT_EXIT(square_root(-22.0), ::testing::ExitedWithCode(255), "Error: Negative Input");
}


TEST(ModelDeathTest, import_check_timeout)
{
    std::cout << std::endl << std::flush;
    std::string base_dir(SGM_MODELS_DIRECTORY);
    std::string file_path = base_dir + "/AR9 Ejector.stp";

    std::ofstream log_file;
    open_log_file(file_path, log_file);

    int status = import_check_timeout(file_path, log_file);

    log_file << "Success" << std::endl;

    log_file.close();

    EXPECT_EQ(status, ModelsCheckResult::SUCCESS);
}

TEST(ModelDeathTest, sgm_models)
{
    std::cout << std::endl << std::flush;
    std::string base_dir(SGM_MODELS_DIRECTORY);
    std::vector<std::string> names = get_names_with_extension(base_dir);
    for (const std::string& name : names)
    {
        std::string path_name = base_dir + "/" + name;
        std::cout << "Checking " << path_name << std::endl << std::flush;
        EXPECT_EXIT(import_check_log_process(path_name),
                    ::testing::ExitedWithCode(ModelsCheckResult::SUCCESS),
                    "Success");
    }
}


