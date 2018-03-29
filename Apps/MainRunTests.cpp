
#include "SGMChecker.h"
#include "SGMDataClasses.h"
#include "SGMPrimatives.h"

int main(int argc, char *argv[])
    {
    SGM::Result rResult=SGM::CreateResult();
    std::string sDirectoryName("c:/sgm/Tests");
    std::string sOutputFileName("c:/sgm/output.txt");
    SGM::RunTestDirectory(rResult,sDirectoryName,sOutputFileName);

	return 0;
    }

