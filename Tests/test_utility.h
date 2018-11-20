#ifndef SGM_TEST_UTILITY_H
#define SGM_TEST_UTILITY_H

///////////////////////////////////////////////////////////////////////////////
//
// Utilities for gtest unit tests.
//
///////////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <string>
#include <gtest/gtest.h>

#ifdef _MSC_VER
#include <Windows.h>
#else
#include <dirent.h>
#endif

#include <SGMChecker.h>
#include <SGMPrimitives.h>
#include <SGMTranslators.h>

///////////////////////////////////////////////////////////////////////////////
//
// For reading from our external data directories and other utilities
//
///////////////////////////////////////////////////////////////////////////////

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

// list of file extensions of models we try to read
extern const std::vector<std::string> SGM_MODELS_EXTENSIONS;

///////////////////////////////////////////////////////////////////////////////
//
// Utility helper functions
//
///////////////////////////////////////////////////////////////////////////////

// replace all instances of 'from' in the 'str' with 'to'
inline void replace_all_substring(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos)
        {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
        }
}

// return a full path to a given file name in the models directory
inline std::string get_models_file_path(std::string const &file_name)
{
    static const FilePath models_directory(SGM_MODELS_DIRECTORY);
    FilePath full_path = FilePath::ConcatPaths(models_directory, FilePath(file_name));
    return full_path.string();
}

// return a path to the models directory
inline std::string get_models_path()
{
    static const FilePath models_directory(SGM_MODELS_DIRECTORY);
    return models_directory.string();
}

// return a full path to a given file name in the models directory
inline std::string get_models_ouo_file_path(std::string const &file_name)
{
    static const FilePath models_directory(SGM_MODELS_OUO_DIRECTORY);
    FilePath full_path = FilePath::ConcatPaths(models_directory, FilePath(file_name));
    return full_path.string();
}

// return true the main string ends with the match string
inline bool ends_with(const std::string &main_str, const std::string &to_match_str) {
    return (main_str.size() >= to_match_str.size() &&
            main_str.compare(main_str.size() - to_match_str.size(), to_match_str.size(), to_match_str) == 0);
}

// true if the file_path ends with one of the extensions
inline bool has_model_extension(const std::string &file_path)
{
    for (auto const &extension : SGM_MODELS_EXTENSIONS)
        if (ends_with(file_path, extension))
            return true;
    return false;
}

// if the path ends with one of the extensions,
// return the path with the extensions erased, else return the original path
inline std::string erase_model_extension(std::string const &file_path)
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

// remove the directory from a file path, leaving only the file name
inline std::string erase_directory(std::string const &file_path)
{
    const FilePath file_path_object(file_path);
    auto file_name_only = file_path_object.RemoveDirectoryName();
    return file_name_only.string();
}

// filter out file names from list that do not have our extensions
inline void filter_model_extension(std::vector<std::string> &names)
{
    names.erase(std::remove_if(names.begin(), names.end(), [](const std::string& x)
                    {
                    return !has_model_extension(x);
                    }),
                names.end());
}

// if string ends with newline ("\n") remove last the newline
inline std::string strip_new_line(const std::string & line)
{
    std::string s(line);
    if (!s.empty() && s[s.length()-1] == '\n')
        s.erase(s.length()-1);
    return s;
}

/**
 * Return the subset of files in a directory that match a condition.
 *
 * @param dir - name of the directory containing all the files to
 * @param fileNameKeep - a function that returns true or false given a filename
 * @return a vector holding the  filenames
 */
std::vector<std::string> get_file_names_if(const std::string &dir, bool (*fileNameKeep)(const std::string&));

// Get current date/time, format is YYYY-MM-DD HH:mm:ss
inline const std::string current_date_time()
{
    time_t     now = time(0);
    struct tm  tstruct{};
    char       buf[80];
#ifdef _MSC_VER
    localtime_s(&tstruct,&now);
#else
    tstruct = *localtime(&now);
#endif
    strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
    return buf;
}


///////////////////////////////////////////////////////////////////////////////
//
// For running tests in ModelViewer
//
///////////////////////////////////////////////////////////////////////////////

namespace SGMInternal
{
   class thing;
}

namespace testing
{
    class EmptyTestEventListener;
}

namespace SGMTesting {

    /**
     * The ModelViewer calls this to run specific test(s) specified by the argument and display result.
     *
     * Two files will be written holding the contents of stderr and stdout:
     *      "~gtest_stdout.log" and "~gtest_stderr.log".
     *
     * Example:
     *
     *   int result = PerformViewerTest(mModel->GetThing(), "--filter=volume_check")
     *
     * @param pThing environment that the ModelViewer is using to display
     * @param arg command line like argument to specify the test(s) to run
     * @param listener callback class for responding to test run events
     *
     * @return 0 for success, 1 for one or more test failures
     */
    int PerformViewerTest(SGMInternal::thing *pThing,
                          const char* arg,
                          testing::EmptyTestEventListener *listener);

    // A test calls this to get the environment used by the ModelViewer to display a model.
    SGMInternal::thing* AcquireTestThing();

    // A test calls this to release the environment, if it is local it will be deleted.
    void ReleaseTestThing(SGMInternal::thing* pThing);
}

#endif //SGM_TEST_UTILITY_H
