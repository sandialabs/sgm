#ifndef FACET_BREP_H
#define FACET_BREP_H

#include "SGMVector.h"
#include "EntityClasses.h"
#include <vector>
namespace SGMInternal
{
curve *FindConic(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 double                           dTolerance);
}
#endif // FACET_BREP_H
