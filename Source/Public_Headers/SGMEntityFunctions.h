#ifndef SGM_ENTITY_FUNCTIONS_H
#define SGM_ENTITY_FUNCTIONS_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"

namespace SGM
{
    
void DeleteEntity(SGM::Result &rResult,
                  SGM::Entity &EntityID);

SGM::Entity CopyEntity(SGM::Result       &rResult,
                       SGM::Entity const &EntityID);

void TransformEntity(SGM::Result            &rResult,
                     SGM::Transform3D const &Trans,
                     SGM::Entity            &EntityID);

SGM::Interval3D const &GetBoundingBox(SGM::Result       &rResult,
                                      SGM::Entity const &EntityID);

}// End SGM namespace

#endif // SGM_ENTITY_FUNCTIONS_H