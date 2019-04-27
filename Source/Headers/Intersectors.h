#ifndef INTERSECTOR_H
#define INTERSECTOR_H

#include "SGMVector.h"
#include "SGMSegment.h"

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
               std::vector<entity *>              &aEntites,
               double                              dTolerance=SGM_ZERO,
               bool                                bUseWholeLine=false);

size_t RayFireBody(SGM::Result                        &rResult,
                   SGM::Point3D                 const &Origin,
                   SGM::UnitVector3D            const &Axis,
                   body                         const *pBody,
                   std::vector<SGM::Point3D>          &aPoints,
                   std::vector<SGM::IntersectionType> &aTypes,
                   std::vector<entity *>              &aEntites,
                   double                              dTolerance,
                   bool                                bUseWholeLine=false);

// Supply the candidate faces, or an empty list of faces

size_t RayFireVolume(SGM::Result                                      &rResult,
                     SGM::Point3D                               const &Origin,
                     SGM::UnitVector3D                          const &Axis,
                     volume                                     const *pVolume,
                     std::vector<face*>                               &aHitFacesSupplied,
                     std::vector<SGM::Point3D>                        &aPoints,
                     std::vector<SGM::IntersectionType>               &aTypes,
                     std::vector<entity *>                            &aEntites,
                     double                                            dTolerance,
                     bool                                              bUseWholeLine=false);


size_t RayFireFace(SGM::Result                        &rResult,
                   SGM::Point3D                 const &Origin,
                   SGM::UnitVector3D            const &Axis,
                   face                         const *pFace,
                   std::vector<SGM::Point3D>          &aPoints,
                   std::vector<SGM::IntersectionType> &aTypes,
                   std::vector<entity *>              &aEntites,
                   double                              dTolerance,
                   bool                                bUseWholeLine);

size_t IntersectSegment(SGM::Result               &rResult,
                        SGM::Segment3D      const &Segment,
                        entity              const *pEntity,
                        std::vector<SGM::Point3D> &aPoints,
                        double                     dTolerance);

size_t IntersectCurves(SGM::Result                        &rResult,
                       curve                        const *pCurve1,
                       curve                        const *pCurve2,
                       std::vector<SGM::Point3D>          &aPoints,
                       std::vector<SGM::IntersectionType> &aTypes,
                       double                              dTolerance);

size_t IntersectCurveAndSurface(SGM::Result                        &rResult,
                                curve                        const *pCurve,
                                surface                      const *pSurface,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes,
                                double                              dTolerance);

size_t IntersectCurveAndPlane(SGM::Result                        &rResult,
                              curve                        const *pCurve,
                              plane                        const *pPlane,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes,
                              double                              dTolerance);

size_t IntersectCurveAndPlane(SGM::Result                        &rResult,
                              curve                        const *pCurve,
                              SGM::Point3D                 const &PlaneOrigin,
                              SGM::UnitVector3D            const &PlaneNorm,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes,
                              double                              dTolerance);

size_t IntersectEdgeAndPlane(SGM::Result                        &rResult,
                             edge                         const *pEdge,
                             SGM::Point3D                 const &PlaneOrigin,
                             SGM::UnitVector3D            const &PlaneNorm,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes,
                             double                              dTolerance);

// Returns true if coinicident.

bool IntersectSurfaces(SGM::Result                &rResult,
                       surface              const *pSurface1,
                       surface              const *pSurface2,
                       std::vector<curve *>       &aCurves,
                       double                      dTolerance);

void IntersectThreeSurfaces(SGM::Result               &rResult,
                            surface             const *pSurface1,
                            surface             const *pSurface2,
                            surface             const *pSurface3,
                            std::vector<SGM::Point3D> &aPoints);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Surface Surface Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

bool IntersectPlaneAndPlane(SGM::Result                &rResult,
                            plane                const *pPlane1,
                            plane                const *pPlane2,
                            std::vector<curve *>       &aCurves,
                            double                      dTolerance);
 
bool IntersectPlaneAndSphere(SGM::Result                &rResult,
                             plane                const *pPlane,
                             sphere               const *pSphere,
                             std::vector<curve *>       &aCurves,
                             double                      dTolerance);
     
bool IntersectPlaneAndCylinder(SGM::Result                &rResult,
                               plane                const *pPlane,
                               cylinder             const *pCylinder,
                               std::vector<curve *>       &aCurves,
                               double                      dTolerance);
 
bool IntersectPlaneAndCone(SGM::Result                &rResult,
                           plane                const *pPlane,
                           cone                 const *pCone,
                           std::vector<curve *>       &aCurves,
                           double                      dTolerance);
 
bool IntersectPlaneAndTorus(SGM::Result                &rResult,
                            plane                const *pPlane,
                            torus                const *pTorus,
                            std::vector<curve *>       &aCurves,
                            double                      dTolerance);
        
bool IntersectPlaneAndSurface(SGM::Result                &rResult,
                              plane                const *pPlane,
                              surface              const *pSurface,
                              std::vector<curve *>       &aCurves,
                              double                      dTolerance);
     
bool IntersectSphereAndSphere(SGM::Result                &rResult,
                              sphere               const *pSphere1,
                              sphere               const *pSphere2,
                              std::vector<curve *>       &aCurves,
                              double                      dTolerance);
     
bool IntersectSphereAndCylinder(SGM::Result                &rResult,
                                sphere               const *pSphere,
                                cylinder             const *pCylinder,
                                std::vector<curve *>       &aCurves,
                                double                      dTolerance);
     
bool IntersectSphereAndSurface(SGM::Result                &rResult,
                               sphere               const *pSphere,
                               surface              const *pSurface,
                               std::vector<curve *>       &aCurves,
                               double                      dTolerance);
     
bool IntersectCylinderAndSurface(SGM::Result                &rResult,
                                 cylinder             const *pCylinder,
                                 surface              const *pSurface,
                                 std::vector<curve *>       &aCurves,
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

size_t IntersectLineAndLine(SGM::Point3D                 const &Origin1,
                            SGM::UnitVector3D            const &Axis1,
                            SGM::Interval1D              const &Domain1,
                            SGM::Point3D                 const &Origin2,
                            SGM::UnitVector3D            const &Axis2,
                            SGM::Interval1D              const &Domain2,
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

size_t IntersectLineAndCircle(SGM::Point3D                 const &Origin,
                              SGM::UnitVector3D            const &Axis,
                              SGM::Interval1D              const &Domain,
                              SGM::Point3D                 const &Center,
                              SGM::UnitVector3D            const &Normal,
                              double                              dRadius,
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

size_t IntersectLineAndParabola(SGM::Point3D                 const &LineOrigin,
                                SGM::UnitVector3D            const &LineAxis,
                                SGM::Interval1D              const &LineDomain,
                                SGM::Point3D                 const &ParabolaCenter,
                                SGM::UnitVector3D            const &ParabolaXAxis,
                                SGM::UnitVector3D            const &ParabolaYAxis,
                                double                              ParabolaA,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectCoplanarLineAndParabola(SGM::Point3D                 const &LineOrigin,
                                        SGM::UnitVector3D            const &LineAxis,
                                        SGM::Interval1D              const &LineDomain,
                                        SGM::Point3D                 const &ParabolaCenter,
                                        SGM::UnitVector3D            const &ParabolaXAxis,
                                        SGM::UnitVector3D            const &ParabolaYAxis,
                                        double                              ParabolaA,
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

size_t IntersectLineAndSurface(SGM::Result                        &rResult,
                               SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               surface                      const *pSurface,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndSurface(SGM::Result                        &rResult,
                               line                         const *pLine,
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

size_t IntersectLineAndTorus(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             double                              dMajorRadius,
                             double                              dMinorRadius,
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

size_t IntersectLineAndRevolve(SGM::Result                        &rResult,
                               SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               revolve                      const *pRevolve,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Circle Surface Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectCircleAndSurface(SGM::Result                        &rResult,
                                 SGM::Point3D                 const &Center,
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

size_t IntersectNUBCurveAndPlane(SGM::Result                        &rResult,
                                 NUBcurve                     const *pCurve,
                                 SGM::Point3D                 const &PlaneOrigin,
                                 SGM::UnitVector3D            const &PlaneNorm,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes,
                                 double                              dTolerance);

size_t IntersectNURBCurveAndPlane(SGM::Result                         &rResult,
                                  NURBcurve                     const *pCurve,
                                  SGM::Point3D                  const &PlaneOrigin,
                                  SGM::UnitVector3D             const &PlaneNorm,
                                  std::vector<SGM::Point3D>           &aPoints,
                                  std::vector<SGM::IntersectionType>  &aTypes,
                                  double                               dTolerance);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Ellipse Surface Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectEllipseAndPlane(ellipse                      const *pEllipse,
                                SGM::Point3D                 const &PlaneOrigin,
                                SGM::UnitVector3D            const &PlaneNormal,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Parabola Surface Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectParabolaAndPlane(SGM::Point3D                 const &Center,
                                 SGM::UnitVector3D            const &XAxis,
                                 SGM::UnitVector3D            const &YAxis,
                                 double                             dA,
                                 SGM::Point3D                 const &PlaneOrigin,
                                 SGM::UnitVector3D            const &PlaneNormal,
                                 double                              dTolerance,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectParabolaAndPlane(parabola                     const *pParabola,
                                 SGM::Point3D                 const &PlaneOrigin,
                                 SGM::UnitVector3D            const &PlaneNormal,
                                 double                              dTolerance,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes);

///////////////////////////////////////////////////////////////////////////////
//
//  Specific Hyperbola Surface Intersectors.
//
///////////////////////////////////////////////////////////////////////////////

size_t IntersectHyperbolaAndPlane(hyperbola                     const *pHyperbola,
                                  SGM::Point3D                  const &PlaneOrigin,
                                  SGM::UnitVector3D             const &PlaneNormal,
                                  double                               dTolerance,
                                  std::vector<SGM::Point3D>           &aPoints,
                                  std::vector<SGM::IntersectionType>  &aTypes);

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

curve *WalkFromTo(SGM::Result                     &rResult,
                  SGM::Point3D              const &StartPos,
                  std::vector<SGM::Point3D> const &aEndPoints,
                  surface                   const *pSurface1,
                  surface                   const *pSurface2);

SGM::Point3D ZoomInFrom(SGM::Point3D const &Pos,
                        surface      const *pSurface1,
                        surface      const *pSurface2);

 curve *FindConicCurve(SGM::Result             &rResult,
                       SGM::Point3D      const &Pos,
                       SGM::UnitVector3D const &Norm,
                       cone              const *pCone);

 // Returns <A,B,C,D,E,F> such that A*x^2+B*y^2+C*x*y+D*x+E*y+F=0.

bool FindConicCoefficient(std::vector<SGM::Point2D> const &aPoints,
                          std::vector<double>             &aCoefficients);

size_t SolveTwoConics(std::vector<double>        aCoefficients1,
                      std::vector<double>        aCoefficients2,
                      std::vector<SGM::Point2D> &aPoints,
                      double                     dTolerance);

bool PointOnCurves(SGM::Point3D         const &Pos,
                   std::vector<curve *> const &aCurves,
                   surface              const *pSurface1,
                   surface              const *pSurface2);

size_t OrderAndRemoveDuplicates(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                double                              dTolerance,
                                bool                                bUseWholeLine,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes,
                                std::vector<entity *>              &aEntities);

 SGM::Point3D ClosestPointOnLine(SGM::Point3D      const &Pos,
                                 SGM::Point3D      const &LineOrigin,
                                 SGM::UnitVector3D const &LineAxis);
}
#endif // INTERSECTOR_H
