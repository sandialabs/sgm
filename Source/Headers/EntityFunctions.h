#ifndef SGM_INTERNAL_ENTITY_FUNCTIONS_H
#define SGM_INTERNAL_ENTITY_FUNCTIONS_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMBoxTree.h"
#include "SGMResult.h"

#include "EntityClasses.h"

#include <vector>

namespace SGMInternal
{

void DeleteEntity(SGM::Result &rResult,
                  entity      *pEntity);

entity *CopyEntity(SGM::Result &rResult,
                   entity      *pEntity);

void TransformEntity(SGM::Result            &rResult,
                     SGM::Transform3D const &Trans,
                     entity                 *pEntity);

SGM::Interval3D const &GetBoundingBox(SGM::Result  &rResult,
                                      entity const *pEntity);

void Heal(SGM::Result           &rResult,
          std::vector<entity *> &aEntities);

/// Return a box that contains all the entity objects in the range [first,last) using the entity->GetBox()
template< class InputIt >
inline void StretchBox(SGM::Result &rResult, SGM::Interval3D &box, InputIt first, InputIt last)
    {
    for (InputIt iter = first; iter != last; ++iter)
        box.operator+=((*iter)->GetBox(rResult));
    }

/// Add all the entity objects (with GetBox member function) in the range [first,last) to the BoxTree.
template< class InputIt >
inline void BoxTreeInsert(SGM::Result &rResult, SGM::BoxTree& rTree, InputIt first, InputIt last)
    {
    for (InputIt iter = first; iter != last; ++iter)
        rTree.Insert(*iter,(*iter)->GetBox(rResult));
    }

}  // End of SGMInternal namespace

#endif // SGM_INTERNAL_ENTITY_FUNCTIONS_H