#ifndef SGM_ENTITY_FUNCTIONS_H
#define SGM_ENTITY_FUNCTIONS_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMResult.h"

#include "sgm_export.h"

namespace SGM
    {

    SGM_EXPORT SGM::EntityType GetType(SGM::Result       &rResult,
                                       SGM::Entity const &EntityID);

    SGM_EXPORT size_t GetOwners(SGM::Result       &rResult,
                                SGM::Entity const &EntityID,
                                std::set<Entity>  &sOwners);

    SGM_EXPORT size_t GetAttributes(SGM::Result         &rResult,
                                    SGM::Entity   const &EntityID,
                                    std::set<Attribute> &sAttributes);
    
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