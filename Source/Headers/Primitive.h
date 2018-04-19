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

 NUBcurve *CreateNUBCurve(SGM::Result                     &rResult,
                          std::vector<SGM::Point3D> const &aInterpolate,
                          std::vector<double>       const *pParams=nullptr);

 NUBcurve *CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                        std::vector<SGM::Point3D> const &aInterpolate,
                                        SGM::Vector3D             const &StartVec,
                                        SGM::Vector3D             const &EndVec,
                                        std::vector<double>       const *pParams);

#endif // PRIMITIVE_H