#include <cmath>
#include <limits>
#include <string>
#include <future>
#include <fstream>

#include <gtest/gtest.h>

#include <SGMChecker.h>
#include <SGMPrimitives.h>
#include <SGMTranslators.h>

//#define PRSCOPY 1

#ifdef _MSC_VER
#include <Windows.h>
#else
#include <dirent.h>
#endif

#include "test_utility.h"

//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "cert-err58-cpp"

using testing::internal::FilePath;

// absolute path to test/data directory must be defined on compile line with -D
#if !defined(SGM_MODELS_DIRECTORY)
#error Path to test data directory is undefined.
#endif

///////////////////////////////////////////////////////////////////////////////
//
// Constants and strings
//
///////////////////////////////////////////////////////////////////////////////

// name of file which is the concatenated log file
std::string SGM_MODELS_LOG_FILE = {"models_check.log"};

// process exit codes for calls to built-in exit() function.
enum ModelsCheckResult: int { SUCCESS=0, FAIL_READ, FAIL_CHECK, FAIL_TIMEOUT };

// limit
static const size_t MODELS_CHECK_TIMEOUT = 10000; // milliseconds

// return true if the file_path has the *.log extension (and is not the concatenated summary SGM_MODELS_LOG_FILE).
bool has_log_extension(const std::string &file_path)
{
    return ends_with(file_path, ".log") && file_path != SGM_MODELS_LOG_FILE;
}

// Open the output stream log_file (using the prefix of the file_path and adding ".log")
// and write a header to it.
void open_log_file(std::string const &file_path, std::ofstream & log_file)
{
    std::string log_path = erase_model_extension(file_path) + ".log";
    std::string file_name = erase_directory(file_path);
    log_file.open(log_path);
    log_file << current_date_time() << ", " << file_name << std::endl << std::flush;
}

// Write a new summary SGM_MODELS_LOG_FILE, concatenating all log files (extension *.log) in the directory.
void concatenate_log_files()
{
    std::string const log_path = get_models_file_path(SGM_MODELS_LOG_FILE);

    std::ofstream output_log(log_path);
    if (!output_log.is_open() || !output_log.good())
        {
        std::cerr << "Error: could not open concatenated log file: '" << log_path << "'" << std::endl;
        return;
        }

    output_log.exceptions(std::ofstream::badbit | std::ofstream::failbit);

    // get a list of all the *.log files in the directory, besides the SGM_MODELS_LOG_FILE
    std::vector<std::string> log_files = get_file_names_if(SGM_MODELS_DIRECTORY, has_log_extension);

    // read each log file and append it
    for (auto const & input_log_name : log_files)
        {
        std::string path_name = get_models_file_path(input_log_name);;
        std::ifstream input_log(path_name);
        if (!input_log.is_open() || !input_log.good())
            {
            std::cerr << "Error: Could not open log file: '" << path_name << "'" << std::endl;
            continue;
            }
        input_log.exceptions(std::ifstream::badbit | std::ifstream::failbit);
        std::string str;

#if 1

        bool bFirst=true;
        std::string file_path = "c:/sgm-models/";
        bool bPassed=false;
        while (std::getline(input_log, str))
            {
            if(bFirst)
                {
                bFirst=false;
                file_path+=str.c_str()+21;
                }
            if(strstr(str.c_str(),"Success"))
                {
                bPassed=true;
                break;
                }
            }
        if(bPassed)
            {
            const testing::internal::FilePath file_path_object(file_path);
            auto file_name_only = file_path_object.RemoveFileName();
            std::string file_name = erase_directory(file_path);
            std::string PassedFileName="c:/sgm-models/Passed/"+file_name;
            int nError = rename(file_path.c_str(),PassedFileName.c_str());
            if(nError)
                {
                perror("bad thing");
                }
            }
#else
        while (std::getline(input_log, str))
            output_log << str << std::endl;
#endif
        input_log.close();
#if PRSCOPY
        remove(path_name.c_str());
#endif
        }
    output_log.close();
}

/**
 * Import a model file, write errors to a log file.
 *
 * @param file_path full path to the model file
 * @param result the result of the imported file
 * @param log_file open text stream for writing
 * @return ModelsCheckResult::SUCCESS or ModelsCheckResult::FAIL_READ
 */
int import_file(std::string const &file_path, SGM::Result& result, std::ofstream & log_file)
{
    std::vector<SGM::Entity> entities;
    std::vector<std::string> log;
    SGM::TranslatorOptions const options;
    SGM::ReadFile(result, file_path, entities, log, options);
    auto resultType = result.GetResult();
    if (resultType == SGM::ResultTypeOK)
        return ModelsCheckResult::SUCCESS;
    log_file << current_date_time() << ", Error: Unable to read from file." << std::endl;
    for (std::string &log_item: log)
        log_file << current_date_time() << ",    " << strip_new_line(log_item) << std::endl;
    log_file << std::flush;
    return ModelsCheckResult::FAIL_READ;
}

/**
 * CheckEntity on a model (result), that has already been imported (from file_path);
 * write any warnings/errors to a log_file.
 *
 * @param file_path original path to the imported model
 * @param result the result of the imported file
 * @param log_file open text stream for writing
 * @return ModelsCheckResult::SUCCESS or ModelsCheckResult::FAIL_CHECK
 */
int check_file(std::string const &/*file_path*/, SGM::Result &result, std::ofstream & log_file)
{
    std::vector<std::string> aLog;
    SGM::CheckOptions Options;
    SGM::CheckEntity(result,SGM::Thing(),Options,aLog);
    if (aLog.empty())
        {
        return ModelsCheckResult::SUCCESS;
        }
#if PRSCOPY
    else
        {
        // Copy passed files to the "Check Errors" directory.  PRS
        std::string file_path("C:/sgm-models");
        const testing::internal::FilePath file_path_object(file_path);
        auto file_name_only = file_path_object.RemoveFileName();
        std::string BaseDir=file_name_only.string();
        std::string file_name = erase_directory(file_path);
        std::string PassedFileName=BaseDir+"Check Errors"+'/'+file_name;
        rename(file_path.c_str(),PassedFileName.c_str());
        }
#endif
    log_file << current_date_time() << ", Error: CheckEntity failed on file. " << std::endl;
    for (std::string &log_item: aLog)
        log_file << current_date_time() << ",    " << strip_new_line(log_item) << std::endl;
    log_file << std::flush;
    return ModelsCheckResult::FAIL_CHECK;
}

/**
 * Import a model given by the file_path and run CheckEntity while monitoring the time used;
 * call system exit() if it exceeds MODELS_CHECK_TIMEOUT; write any errors to a log file.
 *
 * NOTE: this function is meant to be called from inside a forked process because
 * it may call system exit() to kill the process.
 *
 * @param file_path full path to the model file
 * @param log_file open text stream for writing
 * @return one of the values of ModelsCheckResult
 */
int import_check_timeout(std::string const &file_path, std::ofstream & log_file)
{
    // launch a lambda to do the actual work (run in another thread so we can monitor time spent)
    auto asyncFuture = std::async(std::launch::async, [&file_path,&log_file]()->int {
        int status = ModelsCheckResult::SUCCESS;
        SGM::Result result(SGM::CreateThing());
        if (status == ModelsCheckResult::SUCCESS)
            status = import_file(file_path, result, log_file);
        if (status == ModelsCheckResult::SUCCESS)
            status = check_file(file_path, result, log_file);
        return status;
    });

    // if required time is exceeded, write error and kill our process by calling exit()
    if (asyncFuture.wait_for(std::chrono::milliseconds(MODELS_CHECK_TIMEOUT)) == std::future_status::timeout)
        {
#if PRSCOPY
        // Copy passed files to the "Check Errors" directory.  PRS
        const testing::internal::FilePath file_path_object(file_path);
        auto file_name_only = file_path_object.RemoveFileName();
        std::string BaseDir=file_name_only.string();
        std::string file_name = erase_directory(file_path);
        std::string PassedFileName=BaseDir+"Timed Out"+'/'+file_name;
        rename(file_path.c_str(),PassedFileName.c_str());
#endif
        int seconds = ((int)MODELS_CHECK_TIMEOUT) / 1000;
        std::cerr << "Error: timed out after " << seconds << "s." << std::endl << std::flush;
        log_file << current_date_time() << ", Error: timed out after " << seconds << "s." << std::endl << std::flush;
        // forcefully exit the whole process because there is not a clean way to
        // kill the thread running the async task doing Reading and Checking
        exit(ModelsCheckResult::FAIL_TIMEOUT);
        }

    // otherwise return the value of success or failure for any other reason
    int status = asyncFuture.get();
    return status;
}


/**
 * Import a model given by the file_path and run CheckEntity while monitoring the time used,
 * write any errors to a log file with the same prefix, and exit the process.

 * NOTE: this function is meant to be called by gtest EXPECT_EXIT() running from inside a forked process
 * and will call system exit() with an exit code.
 *
 * @param file_path full path to the model file
 */
void import_check_log_process(std::string const &file_path)
{
    std::chrono::steady_clock::time_point start;
    //std::chrono::steady_clock::time_point::duration sum;

    std::ofstream log_file;
    open_log_file(file_path, log_file);

    // start timer
    start = std::chrono::steady_clock::now();

    int status = import_check_timeout(file_path, log_file);

    // end timer
    std::chrono::steady_clock::time_point::duration diff = std::chrono::steady_clock::now() - start;
    double time_in_milliseconds = std::chrono::duration<double, std::milli>(diff).count();

    // write success
    if (status == ModelsCheckResult::SUCCESS)
        {
        std::cerr << "Success" << std::endl; // gtest will EXPECT this
        log_file << current_date_time() << ", Success " << time_in_milliseconds << "ms." << std::endl;

        /*
        // Copy passed files to the passed directory.  PRS
        const testing::internal::FilePath file_path_object(file_path);
        auto file_name_only = file_path_object.RemoveFileName();
        std::string BaseDir=file_name_only.string();
        std::string file_name = erase_directory(file_path);
        std::string PassedFileName=BaseDir+"Passed"+'/'+file_name;
        rename(file_path.c_str(),PassedFileName.c_str());
        */
        }

    log_file.close();
    exit(status); // exit this process with the status code
}

double square_root(double num) {
    if (num < 0.0)
        {
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
    const FilePath data_dir(std::string(SGM_MODELS_DIRECTORY));
    ASSERT_TRUE(data_dir.DirectoryExists());
}

//
// test whether we can handle the list of our file extensions properly
//
TEST(DataDirectoriesCheck, file_extensions)
{
    std::vector<std::string> names = {
            "testA.doc","testB.stp","testC.txt","testD","testE.STEP","testF.otl","testG.STP","testH.step"};
    ASSERT_FALSE(has_model_extension(names[0]));

    filter_model_extension(names);
    ASSERT_EQ(names.size(),4);
    ASSERT_EQ(names[0],"testB.stp");
    ASSERT_EQ(names[1],"testE.STEP");
    ASSERT_EQ(names[2],"testG.STP");
    ASSERT_EQ(names[3],"testH.step");
    ASSERT_TRUE(has_model_extension(names[0]));
    ASSERT_TRUE(has_model_extension(names[1]));
    ASSERT_TRUE(has_model_extension(names[2]));
    ASSERT_TRUE(has_model_extension(names[3]));
    ASSERT_EQ(erase_model_extension(names[0]),"testB");
    ASSERT_EQ(erase_model_extension(names[1]),"testE");
    ASSERT_EQ(erase_model_extension(names[2]),"testG");
    ASSERT_EQ(erase_model_extension(names[3]),"testH");
}

// import and check a single file
TEST(DataDirectoriesCheck, import_check_single)
{
    std::cout << std::endl << std::flush;
    //std::string file_path = get_models_file_path("Default.stp");
    //std::string file_path = get_models_file_path("1 Blade1-A3.STEP");
    //std::string file_path = get_models_file_path("pencil 15.STEP");
    //std::string file_path = get_models_file_path("knuckle.stp");

    //std::string file_path = get_models_file_path("Large Files/Part1.STEP"); // reads but FindCachedData hangs
    //std::string file_path = get_models_file_path("Large Files/47M Anchor Handling TUG-STEP.STEP");
    //std::string file_path = get_models_file_path("Large Files/T-34.STEP"); // read but no FindCachedData

    //std::string file_path = get_models_file_path("Grand Piano 2.STEP");
    //std::string file_path = get_models_file_path("Large Files/Globe 01 (Solid).stp"); // hangs in FindCachedData facet face
    //std::string file_path = get_models_file_path("Large Files/Globe (1).stp");
    //std::string file_path = get_models_file_path("Large Files/USSnathonjames01assem.STEP");
    //std::string file_path = get_models_file_path("_Assy, Porcupine - 01.STEP");
    std::string file_path = get_models_file_path("EPF-E16-4450-50C.STEP");
    //std::string file_path = get_models_file_path("Pencil.STEP");
    //std::string file_path = get_models_file_path("../sgm-models-ouo/OUO_Cone_definition.stp");
    //std::string file_path = get_models_file_path("../sgm-models-ouo/OUO_Cone_definition.stp");
    //std::string file_path = get_models_file_path("../sgm-models-ouo/OUO_glom4/0016-_Bspring.stp");
    //std::string file_path = get_models_file_path("../sgm-models-ouo/OUO_Cone_definition.stp");
    //std::string file_path = get_models_file_path("../sgm-models-ouo/OUO_ZGeom.stp"); // hangs in FindCachedData facet face
    //std::string file_path = get_models_file_path("../sgm-models-ouo/OUO_TSLhousingGeom.stp");
    //std::string file_path = get_models_file_path("../sgm-models-ouo/OUO_full_model_volume1.stp");
    //std::string file_path = get_models_file_path("../sgm-models-ouo/OUO_full_model_volume1x64.stp");

    std::ofstream log_file;

    open_log_file(file_path, log_file);
    EXPECT_TRUE(log_file.is_open());

    SGM::Result result(SGM::CreateThing());

    int status = import_file(file_path, result, log_file);
    EXPECT_EQ(status,ModelsCheckResult::SUCCESS);

//    if (status == ModelsCheckResult::SUCCESS)
//        status = check_file(file_path, result, log_file);
//    EXPECT_EQ(status,ModelsCheckResult::SUCCESS);
}



//
// test whether the Gtest exit function works properly
//
TEST(ModelDeathTest, exit_code)
{
    EXPECT_EQ(0.0, square_root(0.0));
    EXPECT_EXIT(square_root(-22.0), ::testing::ExitedWithCode(255), "Error: Negative Input");
}


// Test just a single file using our timeout wrapper, but it better not timeout
// or whole process will exit().
TEST(ModelDeathTest, import_check_timeout)
{
    std::cout << std::endl << std::flush;
    std::string base_dir(SGM_MODELS_DIRECTORY);
    std::string file_path = get_models_file_path("ball.stp");
    std::ofstream log_file;
    open_log_file(file_path, log_file);
    int status = import_check_timeout(file_path, log_file);
    log_file << "Success" << std::endl;
    log_file.close();
    EXPECT_EQ(status, ModelsCheckResult::SUCCESS);
}

TEST(ModelDeathTest, sgm_models)
{
    std::string base_dir(SGM_MODELS_DIRECTORY);

    std::vector<std::string> names = get_file_names_if(base_dir, has_model_extension);
#if PRSCOPY
    // Create a directory for the passed files.  PRS
    std::string passed_name = std::string(SGM_MODELS_DIRECTORY) + "/" + "Passed";
    CreateDirectory(passed_name.c_str(),nullptr);

    // Create a directory for files that read in but do not check.  PRS
    std::string check_error_name = std::string(SGM_MODELS_DIRECTORY) + "/" + "Check Errors";
    CreateDirectory(check_error_name.c_str(),nullptr);

    // Create a directory for files that timed out.  PRS
    std::string timed_out_name = std::string(SGM_MODELS_DIRECTORY) + "/" + "Timed Out";
    CreateDirectory(timed_out_name.c_str(),nullptr);
#endif
    for (const std::string& name : names)
        {
        std::string path_name = std::string(SGM_MODELS_DIRECTORY) + "/" + name;
        printf("Checking %s\n",path_name.c_str());
        EXPECT_EXIT(import_check_log_process(path_name),
                    ::testing::ExitedWithCode(ModelsCheckResult::SUCCESS),
                    "Success");
        }

    concatenate_log_files();
    }

//#pragma clang diagnostic pop
