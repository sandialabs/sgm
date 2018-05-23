#ifndef SGM_ENTITY_FUNCTIONS_H
#define SGM_ENTITY_FUNCTIONS_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMResult.h"

#include "sgm_export.h"

namespace SGM
    {
    
    SGM_EXPORT void DeleteEntity(SGM::Result &rResult,
                                 SGM::Entity &EntityID);

    SGM_EXPORT SGM::Entity CopyEntity(SGM::Result       &rResult,
                                      SGM::Entity const &EntityID);

    SGM_EXPORT void TransformEntity(SGM::Result            &rResult,
                                    SGM::Transform3D const &Trans,
                                    SGM::Entity            &EntityID);

    SGM_EXPORT SGM::Interval3D const &GetBoundingBox(SGM::Result       &rResult,
                                                     SGM::Entity const &EntityID);

    }// End SGM namespace

#endif // SGM_ENTITY_FUNCTIONS_H