#include "EntityClasses.h"
#include "SGMEntityFunctions.h"
#include "Curve.h"
#include "Surface.h"

namespace SGMInternal
{

void DeleteEntity(SGM::Result &rResult,
                  entity      *pEntity)
    {
    auto pThing = rResult.GetThing();
    std::set<entity *,EntityCompare> sChildren;
    pEntity->FindAllChildren(sChildren);
    for (auto pChild : sChildren) // access by value, the type of pChild is entity*
        pThing->DeleteEntity(pChild);
    pThing->DeleteEntity(pEntity);
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