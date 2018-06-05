
#include "SGMChecker.h"
#include "SGMDataClasses.h"
#include "SGMPrimitives.h"

int main(int /*argc*/, char ** /*argv[]*/)
    {
    SGMInternal::thing *pThing = SGM::CreateThing();
    SGM::Result rResult(pThing);
    std::string sInputDirectoryName = SGM_TEST_INPUT_DIR;
    std::string sOutputDirectoryName = SGM_TEST_OUTPUT_DIR;
    std::string sOutputFileName = sOutputDirectoryName + "/output.txt";
    SGM::RunTestDirectory(rResult,sInputDirectoryName,sOutputFileName);
    SGM::DeleteThing(pThing);
    return 0;
    }

