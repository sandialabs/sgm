#ifndef SGM_FILE_IO_H
#define SGM_FILE_IO_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include <string>

namespace SGM
    {
    
    void SaveAsSGM(SGM::Result       &rResult,
                   std::string const &FileName,
                   SGM::Thing  const &ThingID);

    void ReadSGMFile(SGM::Result       &rResult,
                     std::string const &FileName,
                     SGM::Thing        &ThingID);

    void SaveAsSTL(SGM::Result       &rResult,
                   std::string const &FileName,
                   SGM::Thing  const &ThingID,
                   bool               bBinary);

    void ReadSTLFile(SGM::Result       &rResult,
                     std::string const &FileName,
                     SGM::Thing        &ThingID);

    void SaveAsSTEP(SGM::Result       &rResult,
                    std::string const &FileName,
                    SGM::Thing  const &ThingID);

    void ReadSTEPFile(SGM::Result       &rResult,
                      std::string const &FileName,
                      SGM::Thing        &ThingID);

    } // End of SGM namespace

#endif // SGM_FILE_IO_H
