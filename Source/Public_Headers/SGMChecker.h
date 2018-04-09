#ifndef SGM_CHECKER_H
#define SGM_CHECKER_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include <vector>

namespace SGM
    {
    
    class CheckOptions
        {
        public:

            CheckOptions():
                m_bDerivatives(false),
                m_bEvaluateors(false) {}

            bool m_bDerivatives;
            bool m_bEvaluateors;
        };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Functions to check test results.  The following functions cause a
    //  test script to fail if they return false.
    //
    ///////////////////////////////////////////////////////////////////////////

    bool CheckEntity(SGM::Result              &rResult,
                     SGM::Entity        const &EntityID,
                     SGM::CheckOptions  const &Options,
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

    bool RunCPPTest(SGM::Result &rResult,
                    size_t       nTestNumber);

    void RunTestDirectory(SGM::Result       &rResult,
                          std::string const &sTestDirectory,
                          std::string const &sOutputFileName);

    } // End of SGM namespace

#endif // SGM_CHECKER_H