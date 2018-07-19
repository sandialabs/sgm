#include <cstdio>
#include <gtest/gtest.h>
#include <SGMPrimitives.h>

#include "assert.h"
#include "test_utility.h"

namespace SGMTesting {

    // A static variable used to capture the environment used by the ModelViewer to display.
    // This is internal only to this module and not global.
    static SGMInternal::thing *viewer_thing = nullptr;

    ///////////////////////////////////////////////////////////////////////////
    //
    // Public functions
    //
    ///////////////////////////////////////////////////////////////////////////

    int PerformViewerTest(SGMInternal::thing *pThing, const char* arg)
    {
        // set the environment to use that given to us by the viewer
        assert(pThing != nullptr);
        viewer_thing = pThing;
        FILE *file_out, *file_err;
#ifdef _MSC_VER
        freopen_s(&file_out, "~gtest_stdout.log", "w", stdout);
        freopen_s(&file_err, "~gtest_stderr.log", "w", stderr);
#else
        // start capturing stdout/stderr
        file_out = freopen("~gtest_stdout.log", "w", stdout);
        file_err = freopen("~gtest_stderr.log", "w", stderr);
#endif
        assert(file_out);
        assert(file_err);

        int argc = 2;
        const char *argv[2];
        argv[0] = "sgm_viewer";
        argv[1] = arg;

        ::testing::InitGoogleTest(&argc, const_cast<char**>(argv));
        int ret_value =  RUN_ALL_TESTS();

        // done capturing stdout/stderr
        fclose(file_out);
        fclose(file_err);

        return ret_value;
    }

    ///////////////////////////////////////////////////////////////////////////
    //
    // Inside gtests only functions
    //
    ///////////////////////////////////////////////////////////////////////////

    SGMInternal::thing* AcquireTestThing()
    {
        // if this is not null, the viewer has requested we display using its environment
        SGMInternal::thing *pThing = viewer_thing;
        if (pThing == nullptr)
            // we were not called by the viewer, create a new environment
            pThing = SGM::CreateThing();
        return pThing;
    }

    void ReleaseTestThing(SGMInternal::thing* pThing)
    {
        // only delete if we are not using the one from the ModelViewer
        if (viewer_thing == nullptr)
            SGM::DeleteThing(pThing);
    }

}