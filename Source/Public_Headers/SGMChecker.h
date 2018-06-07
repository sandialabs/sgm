#ifndef SGM_CHECKER_H
#define SGM_CHECKER_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMResult.h"
#include <vector>

#include "sgm_export.h"

namespace SGM
    {
    
    class SGM_EXPORT CheckOptions
        {
        public:

            CheckOptions():
                m_bDerivatives(false),
                m_bEvaluaters(false) {}

            explicit CheckOptions(std::string);

            std::string FindString() const;

            bool m_bDerivatives;
            bool m_bEvaluaters;
        };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Functions to check test results.  The following functions cause a
    //  test script to fail if they return false.
    //
    ///////////////////////////////////////////////////////////////////////////

    SGM_EXPORT bool CheckEntity(SGM::Result              &rResult,
                                SGM::Entity        const &EntityID,
                                SGM::CheckOptions  const &Options,
                                std::vector<std::string> &aCheckStrings);
    
    SGM_EXPORT bool CompareFiles(SGM::Result       &rResult,
                                 std::string const &sFile1,
                                 std::string const &sFile2);

    SGM_EXPORT bool CompareSizes(size_t nSize1,size_t nSize2);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Functions to run tests.
    //
    ///////////////////////////////////////////////////////////////////////////

    SGM_EXPORT bool RunTestFile(SGM::Result       &rResult,
                                std::string const &sTestDirectory,
                                std::string const &sTestFileName,
                                std::string const &sOutputFileName);

    SGM_EXPORT bool RunCPPTest(SGM::Result &rResult,
                               size_t       nTestNumber);

    SGM_EXPORT void RunTestDirectory(SGM::Result       &rResult,
                                     std::string const &sTestDirectory,
                                     std::string const &sOutputFileName);

    } // End of SGM namespace

#endif // SGM_CHECKER_H