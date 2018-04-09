
#include "SGMChecker.h"
#include "SGMDataClasses.h"
#include "SGMPrimatives.h"

int main(int argc, char *argv[])
    {
    SGM::Result rResult=SGM::CreateResult();
    std::string sInputDirectoryName = SGM_TEST_INPUT_DIR;
    std::string sOutputDirectoryName = SGM_TEST_OUTPUT_DIR;
    std::string sOutputFileName = sOutputDirectoryName + "/output.txt";
    SGM::RunTestDirectory(rResult,sInputDirectoryName,sOutputFileName);

	return 0;
    }

