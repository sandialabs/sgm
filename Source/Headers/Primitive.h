#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include "SGMDataClasses.h"
#include "EntityClasses.h"

body *CreateBlock(SGM::Result        &rResult,
                  thing              *pThing,
                  SGM::Point3D const &Point1,
                  SGM::Point3D const &Point2);

body *CreateSphere(SGM::Result        &rResult,
                   thing              *pThing,
                   SGM::Point3D const &Center,
                   double              dRadius);

body *CreateCylinder(SGM::Result        &rResult,
                     thing              *pThing,
                     SGM::Point3D const &BottomCenter,
                     SGM::Point3D const &TopCenter,
                     double              dRadius);

body *CreateTorus(SGM::Result             &rResult,
                  thing                   *pThing,
                  SGM::Point3D      const &Center,
                  SGM::UnitVector3D const &Axis,
                  double                   dMajorRadius,
                  double                   dMinorRadius,
                  bool                     bApple);

#endif // PRIMITIVE_H