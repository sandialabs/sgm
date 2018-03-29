#ifndef SGM_GEOMETRY_H
#define SGM_GEOMETRY_H

#include "SGMEntityClasses.h"
#include "SGMDataClasses.h"

SGM::Surface CreatePlane(SGM::Result        &rResult,
                         SGM::Thing         &ThingID,
                         SGM::Point3D const &Origin,
                         SGM::Point3D const &XPos,
                         SGM::Point3D const &YPos);

SGM::Curve CreateLine(SGM::Result        &rResult,
                      SGM::Thing         &ThingID,
                      SGM::Point3D const &Origin,
                      SGM::Point3D const &Axis);

#endif // SGM_GEOMETRY_H