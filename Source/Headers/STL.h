#ifndef SGM_STL_H
#define SGM_STL_H

#include "SGMResult.h"
#include "SGMTranslators.h"

namespace SGMInternal
{

class entity;

void ParseSTLTextSerial(SGM::Result &rResult,
                        std::string const &FileName,
                        std::vector<entity *> &aEntities,
                        bool bMerge);

void ParseSTLTextConcurrent(SGM::Result &rResult,
                            std::string const &FileName,
                            std::vector<entity*> &aEntities,
                            bool bMerge);

void SaveSTL(SGM::Result &rResult,
             std::string const &FileName,
             entity *pEntity,
             SGM::TranslatorOptions const &Options);
}

#endif //SGM_STL_H
