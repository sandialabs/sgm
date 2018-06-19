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

template< class InputIt >
inline void BoxTreeInsert(SGM::Result &rResult, SGM::BoxTree& rTree, InputIt first, InputIt last)
    {
        for (InputIt iter = first; iter != last; ++iter)
            rTree.Insert(*iter,(*iter)->GetBox(rResult));
    }

}  // End of SGMInternal namespace

#endif // SGM_INTERNAL_ENTITY_FUNCTIONS_H