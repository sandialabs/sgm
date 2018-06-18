#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include "Curve.h"

#include "SGMVector.h"
#include "EntityClasses.h"

namespace SGMInternal
{
body *CreateBlock(SGM::Result        &rResult,
                  SGM::Point3D const &Point1,
                  SGM::Point3D const &Point2);

body *CreateSphere(SGM::Result        &rResult,
                   SGM::Point3D const &Center,
                   double              dRadius);

body *CreateCylinder(SGM::Result        &rResult,
                     SGM::Point3D const &BottomCenter,
                     SGM::Point3D const &TopCenter,
                     double              dRadius);

body *CreateCone(SGM::Result        &rResult,
                 SGM::Point3D const &BottomCenter,
                 SGM::Point3D const &TopCenter,
                 double              dBottomRadius,
                 double              dTopRadius);

body *CreateTorus(SGM::Result             &rResult,
                  SGM::Point3D      const &Center,
                  SGM::UnitVector3D const &Axis,
                  double                   dMinorRadius,
                  double                   dMajorRadius,
                  bool                     bApple);

body *CreateRevolve(SGM::Result             &rResult,
                    SGM::Point3D      const &Origin,
                    SGM::UnitVector3D const &Axis,
                    curve                   *pCurve);

surface *CreateRevolveSurface(SGM::Result             &rResult,
                              SGM::Point3D      const &Origin,
                              SGM::UnitVector3D const &Axis,
                              curve                   *pCurve);

edge *CreateEdge(SGM::Result           &rResult,
                 curve                 *pCurve,
                 SGM::Interval1D const *pDomain);

NUBcurve *CreateNUBCurve(SGM::Result                     &rResult,
                         std::vector<SGM::Point3D> const &aPoints,
                         std::vector<double>       const *pParams=nullptr);

NUBcurve *CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                       std::vector<SGM::Point3D> const &aPoints,
                                       SGM::Vector3D             const &StartVec,
                                       SGM::Vector3D             const &EndVec,
                                       std::vector<double>       const *pParams);

body *CreateSheetBody(SGM::Result                    &rResult,
                      surface                        *pSurface,
                      std::vector<edge *>            &aEdges,
                      std::vector<SGM::EdgeSideType> &aTypes);

body *CreateDisk(SGM::Result             &rResult,
                 SGM::Point3D      const &Center,
                 SGM::UnitVector3D const &Normal,
                 double                   dRadius);
    }
#endif // PRIMITIVE_H
