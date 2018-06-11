#ifndef INTERROGATE_H
#define INTERROGATE_H

#include "SGMDataClasses.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

namespace SGMInternal
{

bool PointInEntity(SGM::Result        &rResult,
                   SGM::Point3D const &Point,
                   entity       const *pEntity,
                   double              dTolerance=SGM_MIN_TOL);

} // End of SGMInternal namespace


#endif // INTERROGATE_H