#ifndef INTERROGATE_H
#define INTERROGATE_H

#include "SGMResult.h"
#include "SGMVector.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

namespace SGMInternal
{

bool PointInEntity(SGM::Result        &rResult,
                   SGM::Point3D const &Point,
                   entity       const *pEntity,
                   double              dTolerance=SGM_MIN_TOL);

void PointsInVolumes(SGM::Result                         &rResult,
                     std::vector<SGM::Point3D>     const &aPoints,
                     std::vector<std::vector<volume *> > &aaVolumes,
                     double                               dTolerance=SGM_MIN_TOL);

} // End of SGMInternal namespace


#endif // INTERROGATE_H