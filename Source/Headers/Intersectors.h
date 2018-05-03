#ifndef INTERSECTOR_H
#define INTERSECTOR_H

#include "SGMDataClasses.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

///////////////////////////////////////////////////////////////////////////////
//
//  General Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectCurves(SGM::Result                        &rResult,
                       curve                        const *pCurve1,
                       curve                        const *pCurve2,
                       std::vector<SGM::Point3D>          &aPoints,
                       std::vector<SGM::IntersectionType> &aTypes,
                       edge                         const *pEdge1,
                       edge                         const *pEdge2);

size_t IntersectCurveAndSurface(SGM::Result                        &rResult,
                                curve                        const *pCurve,
                                surface                      const *pSurface,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes,
                                edge                         const *pEdge,
                                face                         const *pFace);

size_t IntersectSurfaces(SGM::Result                &rResult,
                         surface              const *pSurface1,
                         surface              const *pSurface2,
                         std::vector<curve *>       &aCurves,
                         face                 const *pFace1,
                         face                 const *pFace2);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Surface Surface Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectPlaneCone(SGM::Result                &rResult,
                          plane                const *pPlane,
                          cone                 const *pCone,
                          std::vector<curve *>       &aCurves,
                          face                 const *pFace1,
                          face                 const *pFace2);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific General Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectLineAndCurve(SGM::Result                        &rResult,
                             SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             curve                        const *pCurve,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectCircleAndCurve(SGM::Result                        &rResult,
                               circle                       const *pCircle,
                               curve                        const *pCurve,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectEllipseAndCurve(SGM::Result                        &rResult,
                                ellipse                      const *pEllipse,
                                curve                        const *pCurve,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Line Curve Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectLineAndLine(SGM::Point3D                 const &Origin,
                            SGM::UnitVector3D            const &Axis,
                            SGM::Interval1D              const &Domain,
                            line                         const *pLine,
                            double                              dTolerance,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndCircle(SGM::Point3D                 const &Origin,
                              SGM::UnitVector3D            const &Axis,
                              SGM::Interval1D              const &Domain,
                              circle                       const *pCircle,
                              double                              dTolerance,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndEllipse(SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               ellipse                      const *pEllipse,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndHyperbola(SGM::Point3D                 const &Origin,
                                 SGM::UnitVector3D            const &Axis,
                                 SGM::Interval1D              const &Domain,
                                 hyperbola                    const *pHyperbola,
                                 double                              dTolerance,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndParabola(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                SGM::Interval1D              const &Domain,
                                parabola                     const *pParabola,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Line Surface Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectLineAndSurface(SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               surface                      const *pSurface,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndCylinder(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                SGM::Interval1D              const &Domain,
                                cylinder                     const *pCylinder,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndPlane(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             plane                        const *pPlane,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndPlane(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             SGM::Point3D                 const &PlaneOrigin,
                             SGM::UnitVector3D            const &PlaneNorm,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndSphere(SGM::Point3D                  const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               sphere                       const *pSphere,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndCone(SGM::Point3D                 const &Origin,
                            SGM::UnitVector3D            const &Axis,
                            SGM::Interval1D              const &Domain,
                            cone                         const *pCone,
                            double                              dTolerance,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndTorus(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             torus                        const *pTorus,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndNUBSurface(SGM::Point3D                 const &Origin,
                                  SGM::UnitVector3D            const &Axis,
                                  SGM::Interval1D              const &Domain,
                                  NUBsurface                   const *pNUBSurface,
                                  double                              dTolerance,
                                  std::vector<SGM::Point3D>          &aPoints,
                                  std::vector<SGM::IntersectionType> &aTypes);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Circle Surface Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectCircleAndSurface(SGM::Point3D                 const &Center,
                                 SGM::UnitVector3D            const &Normal,
                                 double                              dRadius,
                                 surface                      const *pSurface,
                                 double                              dTolerance,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectCircleAndCylinder(SGM::Point3D                 const &Center,
                                  SGM::UnitVector3D            const &Normal,
                                  double                              dRadius,
                                  cylinder                     const *pCylinder,
                                  double                              dTolerance,
                                  std::vector<SGM::Point3D>          &aPoints,
                                  std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectCircleAndPlane(SGM::Point3D                 const &Center,
                               SGM::UnitVector3D            const &Normal,
                               double                              dRadius,
                               plane                        const *pPlane,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectCircleAndSphere(SGM::Point3D                 const &Center,
                                SGM::UnitVector3D            const &Normal,
                                double                              dRadius,
                                sphere                       const *pSphere,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectCircleAndTorus(SGM::Point3D                 const &Center,
                               SGM::UnitVector3D            const &Normal,
                               double                              dRadius,
                               torus                        const *pTorus,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

///////////////////////////////////////////////////////////////////////////////
//
//  Other Suporting Intersection Functions
//
///////////////////////////////////////////////////////////////////////////////

void IntersectNonParallelPlanes(SGM::Point3D      const &Origin1,
                                SGM::UnitVector3D const &Normal1,
                                SGM::Point3D      const &Origin2,
                                SGM::UnitVector3D const &Normal2,
                                SGM::Point3D            &Origin,
                                SGM::UnitVector3D       &Axis);

// Returns the local minimum distance pair of points, from the given t1, and t2,
// on the the given curves as Pos1, and Pos1.  In addition, the corresponding
// parameters of the two points are returned as t1, and t2.  The functions 
// returns the distance between to two points.

double FindLocalMin(curve  const *pCurve1,    // Input
                    curve  const *pCurve2,    // Input
                    double       &t1,         // Input and output
                    double       &t2,         // Input and output
                    SGM::Point3D &Pos1,       // Output
                    SGM::Point3D &Pos2);      // Output

#endif // INTERSECTOR_H
