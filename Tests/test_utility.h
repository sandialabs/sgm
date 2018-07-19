#ifndef SGM_TEST_UTILITY_H
#define SGM_TEST_UTILITY_H

#include "sgm_export.h"

namespace SGMInternal
{
   class thing;
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
     *
     * @return 0 for success, 1 for one or more test failures
     */
    int PerformViewerTest(SGMInternal::thing *pThing, const char* arg);

    // A test calls this to get the environment used by the ModelViewer to display a model.
    SGMInternal::thing* AcquireTestThing();

    // A test calls this to release the environment, if it is local it will be deleted.
    void ReleaseTestThing(SGMInternal::thing* pThing);
}

#endif //SGM_TEST_UTILITY_H
