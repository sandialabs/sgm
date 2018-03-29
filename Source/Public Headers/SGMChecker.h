#ifndef SGM_CHECKER_H
#define SGM_CHECKER_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include <vector>

namespace SGM
    {
    ///////////////////////////////////////////////////////////////////////////
    //
    //  Functions to check test results.  The following functions cause a
    //  test script to fail if they return false.
    //
    ///////////////////////////////////////////////////////////////////////////

    bool CheckEntity(SGM::Result              &rResult,
                     SGM::Entity              &EntityID,
                     SGM::CheckLevel           Level,
                     std::vector<std::string> &aCheckStrings);
    
    bool CompareFiles(SGM::Result       &rResult,
                      std::string const &sFile1,
                      std::string const &sFile2);

    bool CompareSizes(size_t nSize1,size_t nSize2);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Functions to run tests.
    //
    ///////////////////////////////////////////////////////////////////////////

    bool RunTestFile(SGM::Result       &rResult,
                     std::string const &sTestDirectory,
                     std::string const &sTestFileName,
                     std::string const &sOutputFileName);

    // If nTestNumber is zero, then all the CPP tests are run.

    bool RunCPPTest(SGM::Result       &rResult,
                    size_t             nTestNumber, 
                    std::string const &sOutputFileName);

    void RunTestDirectory(SGM::Result       &rResult,
                          std::string const &sTestDirectory,
                          std::string const &sOutputFileName);

    } // End of SGM namespace

#endif // SGM_CHECKER_H