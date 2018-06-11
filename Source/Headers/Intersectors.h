#ifndef INTERSECTOR_H
#define INTERSECTOR_H

#include "SGMVector.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"
namespace SGMInternal
{
///////////////////////////////////////////////////////////////////////////////
//
//  General Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t RayFire(SGM::Result                        &rResult,
               SGM::Point3D                 const &Origin,
               SGM::UnitVector3D            const &Axis,
               entity                       const *pEntity,
               std::vector<SGM::Point3D>          &aPoints,
               std::vector<SGM::IntersectionType> &aTypes,
               double                              dTolerance=SGM_ZERO);

size_t RayFireBody(SGM::Result                        &rResult,
                   SGM::Point3D                 const &Origin,
                   SGM::UnitVector3D            const &Axis,
                   body                         const *pBody,
                   std::vector<SGM::Point3D>          &aPoints,
                   std::vector<SGM::IntersectionType> &aTypes,
                   double                       const dTolerance);

size_t RayFireVolume(SGM::Result                        &rResult,
                     SGM::Point3D                 const &Origin,
                     SGM::UnitVector3D            const &Axis,
                     volume                       const *pVolume,
                     std::vector<SGM::Point3D>          &aPoints,
                     std::vector<SGM::IntersectionType> &aTypes,
                     double                       const dTolerance);

size_t IntersectCurves(SGM::Result                        &rResult,
                       curve                        const *pCurve1,
                       curve                        const *pCurve2,
                       std::vector<SGM::Point3D>          &aPoints,
                       std::vector<SGM::IntersectionType> &aTypes,
                       edge                         const *pEdge1,
                       edge                         const *pEdge2,
                       double                       const dTolerance);

size_t IntersectCurveAndSurface(SGM::Result                        &rResult,
                                curve                        const *pCurve,
                                surface                      const *pSurface,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes,
                                edge                         const *pEdge,
                                face                         const *pFace,
                                double                              dTolerance);

size_t IntersectSurfaces(SGM::Result                &rResult,
                         surface              const *pSurface1,
                         surface              const *pSurface2,
                         std::vector<curve *>       &aCurves,
                         face                 const *pFace1,
                         face                 const *pFace2,
                         double                      dTolerance);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Surface Surface Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectPlanePlane(SGM::Result                &rResult,
                           plane                const *pPlane1,
                           plane                const *pPlane2,
                           std::vector<curve *>       &aCurves,
                           face                 const *pFace1,
                           face                 const *pFace2,
                           double                      dTolerance);

size_t IntersectPlaneSphere(SGM::Result                &rResult,
                            plane                const *pPlane,
                            sphere               const *pSphere,
                            std::vector<curve *>       &aCurves,
                            face                 const *pFace1,
                            face                 const *pFace2,
                            double                      dTolerance);

size_t IntersectPlaneCylinder(SGM::Result                &rResult,
                              plane                const *pPlane,
                              cylinder             const *pCylinder,
                              std::vector<curve *>       &aCurves,
                              face                 const *pFace1,
                              face                 const *pFace2,
                              double                      dTolerance);

size_t IntersectPlaneCone(SGM::Result                &rResult,
                          plane                const *pPlane,
                          cone                 const *pCone,
                          std::vector<curve *>       &aCurves,
                          face                 const *pFace1,
                          face                 const *pFace2,
                          double                      dTolerance);

size_t IntersectPlaneTorus(SGM::Result                &rResult,
                           plane                const *pPlane,
                           torus                const *pTorus,
                           std::vector<curve *>       &aCurves,
                           face                 const *pFace1,
                           face                 const *pFace2,
                           double                      dTolerance);

size_t IntersectPlaneSurface(SGM::Result                &rResult,
                             plane                const *pPlane,
                             surface              const *pSurface,
                             std::vector<curve *>       &aCurves,
                             face                 const *pFace1,
                             face                 const *pFace2,
                             double                      dTolerance);

size_t IntersectSphereSphere(SGM::Result                &rResult,
                             sphere               const *pSphere1,
                             sphere               const *pSphere2,
                             std::vector<curve *>       &aCurves,
                             face                 const *pFace1,
                             face                 const *pFace2,
                             double                      dTolerance);

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

size_t IntersectLineAndNUBCurve(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                SGM::Interval1D              const &Domain,
                                NUBcurve                     const *pNUBCurve,
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

size_t IntersectLineAndNURBSurface(SGM::Point3D                 const &Origin,
                                   SGM::UnitVector3D            const &Axis,
                                   SGM::Interval1D              const &Domain,
                                   NURBsurface                  const *pNURBSurface,
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
                               SGM::Point3D                 const &PlaneOrigin,
                               SGM::UnitVector3D            const &PlaneNormal,
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
//  Other Supporting Intersection Functions
//
///////////////////////////////////////////////////////////////////////////////

void IntersectNonParallelPlanes(SGM::Point3D      const &Origin1,
                                SGM::UnitVector3D const &Normal1,
                                SGM::Point3D      const &Origin2,
                                SGM::UnitVector3D const &Normal2,
                                SGM::Point3D            &Origin,
                                SGM::UnitVector3D       &Axis);

hermite *WalkFromTo(SGM::Result        &rResult,
                    SGM::Point3D const &StartPos,
                    SGM::Point3D const &EndPos,
                    surface      const *pSurface1,
                    surface      const *pSurface2);

SGM::Point3D ZoomInFrom(SGM::Point3D const &Pos,
                        surface      const *pSurface1,
                        surface      const *pSurface2);

}
#endif // INTERSECTOR_H
