#include "EntityClasses.h"
#include "SGMEntityFunctions.h"
#include "Curve.h"
#include "Surface.h"

namespace SGMInternal
{

void DeleteEntity(SGM::Result &rResult,
                  entity      *pEntity)
    {
    std::set<entity *,EntityCompare> sChildern;
    pEntity->FindAllChildern(sChildern);
    std::set<entity *,EntityCompare>::iterator iter=sChildern.begin();
    while(iter!=sChildern.end())
        {
        entity *pEntity=*iter;
        rResult.GetThing()->DeleteEntity(pEntity);
        ++iter;
        }
    rResult.GetThing()->DeleteEntity(pEntity);
    }

entity *CopyEntity(SGM::Result &,//rResult,
                   entity      *)//pEntity)
    {
    return nullptr;
    }

void TransformEntity(SGM::Result            &,//rResult,
                     SGM::Transform3D const &,//Trans,
                     entity                 *)//pEntity)
    {
    
    }

SGM::Interval3D const &GetBoundingBox(SGM::Result  &,//rResult,
                                      entity const *pEntity)
    {
    return pEntity->GetBox();
    }

} // End of SGMInternal namespace