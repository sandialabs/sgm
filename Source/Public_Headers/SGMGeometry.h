#ifndef SGM_GEOMETRY_H
#define SGM_GEOMETRY_H

#include "SGMEntityClasses.h"
#include "SGMVector.h"
#include "SGMResult.h"

#include <vector>

#include "sgm_export.h"

namespace SGM
    {

///////////////////////////////////////////////////////////////////////////////
//
//  Curve Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGM::Curve CreateLine(SGM::Result             &rResult,
                                 SGM::Point3D      const &Origin,
                                 SGM::UnitVector3D const &Axis);

SGM_EXPORT SGM::Curve CreateCircle(SGM::Result             &rResult,
                                   SGM::Point3D      const &Center,
                                   SGM::UnitVector3D const &Normal,
                                   double                   dRadius);

SGM_EXPORT SGM::Curve CreateEllipse(SGM::Result             &rResult,
                                    SGM::Point3D      const &Center,
                                    SGM::UnitVector3D const &XAxis,
                                    SGM::UnitVector3D const &YAxis,
                                    double                   dXRadius,
                                    double                   dYRadius);

SGM_EXPORT SGM::Curve CreateParabola(SGM::Result             &rResult,
                                     SGM::Point3D      const &Center,
                                     SGM::UnitVector3D const &XAxis,
                                     SGM::UnitVector3D const &YAxis,
                                     double                   dA);

SGM_EXPORT SGM::Curve CreateHyperbola(SGM::Result             &rResult,
                                      SGM::Point3D      const &Center,
                                      SGM::UnitVector3D const &XAxis,
                                      SGM::UnitVector3D const &YAxis,
                                      double                   dA,
                                      double                   dB);

SGM_EXPORT SGM::Curve CreateTorusKnot(SGM::Result             &rResult,
                                      SGM::Point3D      const &Center,
                                      SGM::UnitVector3D const &XAxis,
                                      SGM::UnitVector3D const &YAxis,
                                      double                   dMinorRadius,
                                      double                   dMajorRadius,
                                      size_t                   nA,
                                      size_t                   nB);

SGM_EXPORT SGM::Curve CreateNUBCurve(SGM::Result                     &rResult,
                                     std::vector<SGM::Point3D> const &aInterpolatePoints,
                                     std::vector<double>       const *pParams=nullptr);

SGM_EXPORT SGM::Curve CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                                   std::vector<SGM::Point3D> const &aInterpolatePoints,
                                                   SGM::Vector3D             const &StartVec,
                                                   SGM::Vector3D             const &EndVec,
                                                   std::vector<double>       const *pParams=nullptr);

SGM_EXPORT SGM::Curve CreateNUBCurveWithControlPointsAndKnots(SGM::Result                     &rResult,
                                                              std::vector<SGM::Point3D> const &aControlPoints,
                                                              std::vector<double>       const &aKnots);

SGM_EXPORT SGM::Curve CreateNURBCurve(SGM::Result                     &rResult,
                                      std::vector<SGM::Point4D> const &aControlPoints,
                                      std::vector<double>       const &aKnots);

SGM_EXPORT SGM::Curve CreatePointCurve(SGM::Result  &rResult,
                                       SGM::Point3D &Pos);

// Fits a conic curve to five points returning a line, circle, ellipse, parabola or hyperbola.

SGM_EXPORT SGM::Curve FindConic(SGM::Result                     &rResult,
                                std::vector<SGM::Point3D> const &aPoints,
                                double                           dTolerance);

///////////////////////////////////////////////////////////////////////////////
//
//  Surface Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGM::Surface CreatePlane(SGM::Result        &rResult,
                                    SGM::Point3D const &Origin,
                                    SGM::Point3D const &XPos,
                                    SGM::Point3D const &YPos);

SGM_EXPORT SGM::Surface CreateSphereSurface(SGM::Result        &rResult,
                                            SGM::Point3D const &Center,
                                            double              dRadius,
                                            SGM::UnitVector3D  *pXAxis=nullptr,
                                            SGM::UnitVector3D  *pYAxis=nullptr);

SGM_EXPORT SGM::Surface CreateCylinderSurface(SGM::Result        &rResult,
                                              SGM::Point3D const &Bottom,
                                              SGM::Point3D const &Top,
                                              double              dRadius);

SGM_EXPORT SGM::Surface CreateConeSurface(SGM::Result             &rResult,
                                          SGM::Point3D      const &Origin,
                                          SGM::UnitVector3D const &Axis,
                                          double                   dRadius,
                                          double                   dHalfAngle);

SGM_EXPORT SGM::Surface CreateTorusSurface(SGM::Result             &rResult,
                                           SGM::Point3D      const &Center,
                                           SGM::UnitVector3D const &Axis,
                                           double                   dMinorRadius,
                                           double                   dMajorRadius,
                                           bool                     bApple=true);

SGM_EXPORT SGM::Surface CreateNUBSurface(SGM::Result                                   &rResult,
                                         std::vector<std::vector<SGM::Point3D> > const &aaInterpolatePoints,
                                         std::vector<SGM::Vector3D>              const *paStartVecs=nullptr,
                                         std::vector<SGM::Vector3D>              const *paEndVecs=nullptr,
                                         std::vector<double>                     const *pUParams=nullptr,
                                         std::vector<double>                     const *pVParams=nullptr);

SGM_EXPORT SGM::Surface CreateNUBSurfaceFromControlPoints(SGM::Result                                   &rResult,
                                                          std::vector<std::vector<SGM::Point3D> > const &aaControlPoints,
                                                          std::vector<double>                     const &aUKnots,
                                                          std::vector<double>                     const &aVKnots);

SGM_EXPORT SGM::Surface CreateNURBSurface(SGM::Result                                   &rResult,
                                          std::vector<std::vector<SGM::Point4D> > const &aaControlPoints,
                                          std::vector<double>                     const &aUKnots,
                                          std::vector<double>                     const &aVKnots);

SGM_EXPORT SGM::Surface CreateRevolveSurface(SGM::Result             &rResult,
                                             SGM::Point3D      const &Origin,
                                             SGM::UnitVector3D const &Axis,
                                             SGM::Curve              &CurveID);

///////////////////////////////////////////////////////////////////////////////
//
//  Interrogation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGM::Interval1D const &GetCurveDomain(SGM::Result      &rResult,
                                                 SGM::Curve const &CurveID);

SGM_EXPORT void EvaluateCurve(SGM::Result      &rResult,
                              SGM::Curve const &CurveID, 
                              double            dt,
                              SGM::Point3D     *pPos=nullptr,
                              SGM::Vector3D    *pVec1=nullptr,
                              SGM::Vector3D    *pVec2=nullptr);

SGM_EXPORT double CurveInverse(SGM::Result        &rResult,
                               SGM::Curve   const &CurveID,
                               SGM::Point3D const &Pos,
                               SGM::Point3D       *pClosePos=nullptr,
                               double       const *pGuess=nullptr);

SGM_EXPORT SGM::Vector3D CurveCurvature(SGM::Result        &rResult,
                                        SGM::Curve   const &CurveID,
                                        double              t);

SGM_EXPORT void PrincipleCurvature(SGM::Result        &rResult,
                                   SGM::Surface const &SurfaceID,
                                   SGM::Point2D const &uv,
                                   SGM::UnitVector3D  &Vec1,
                                   SGM::UnitVector3D  &Vec2,
                                   double             &dk1,
                                   double             &dk2);

SGM_EXPORT void EvaluateSurface(SGM::Result             &rResult,
                                SGM::Surface      const &SurfaceID,
                                SGM::Point2D      const &uv,
                                SGM::Point3D            *pPos=nullptr,
                                SGM::Vector3D           *pDu=nullptr,
                                SGM::Vector3D           *pDv=nullptr,
                                SGM::UnitVector3D       *pNorm=nullptr,
                                SGM::Vector3D           *pDuu=nullptr,
                                SGM::Vector3D           *pDuv=nullptr,
                                SGM::Vector3D           *pDvv=nullptr);

SGM_EXPORT SGM::Point2D SurfaceInverse(SGM::Result        &rResult,
                                       SGM::Surface const &SurfaceID,
                                       SGM::Point3D const &Pos,
                                       SGM::Point3D       *pClosePos=nullptr,
                                       SGM::Point2D const *pGuess=nullptr);

//////////////////////////////////////////////////////////////////////////////
//
//  Get Curve Data
//
//////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGM::EntityType GetCurveType(SGM::Result      &rResult,
                                        SGM::Curve const &CurveID);

SGM_EXPORT SGM::Interval1D const &GetDomainOfCurve(SGM::Result      &rResult,
                                                   SGM::Curve const &CurveID);

SGM_EXPORT bool GetLineData(SGM::Result       &rResult,
                            SGM::Curve  const &CurveID,
                            SGM::Point3D      &Origin,
                            SGM::UnitVector3D &Axis);

SGM_EXPORT bool GetCircleData(SGM::Result       &rResult,
                              SGM::Curve  const &CurveID,
                              SGM::Point3D      &Center,
                              SGM::UnitVector3D &Normal,
                              SGM::UnitVector3D &XAxis,
                              SGM::UnitVector3D &YAxis,
                              double            &dRadius);
                
SGM_EXPORT bool GetEllipseData(SGM::Result       &rResult,
                               SGM::Curve  const &CurveID,
                               SGM::Point3D      &Center,
                               SGM::UnitVector3D &XAxis,
                               SGM::UnitVector3D &YAxis,
                               SGM::UnitVector3D &Normal,
                               double            &dA,
                               double            &dB);
             
SGM_EXPORT bool GetParabolaData(SGM::Result       &rResult,
                                SGM::Curve  const &CurveID,
                                SGM::Point3D      &Center,
                                SGM::UnitVector3D &XAxis,
                                SGM::UnitVector3D &YAxis,
                                SGM::UnitVector3D &Normal,
                                double            &dA);
             
SGM_EXPORT bool GetHyperbolaData(SGM::Result       &rResult,
                                 SGM::Curve  const &CurveID,
                                 SGM::Point3D      &Center,
                                 SGM::UnitVector3D &XAxis,
                                 SGM::UnitVector3D &YAxis,
                                 SGM::UnitVector3D &Normal,
                                 double            &dA,
                                 double            &dB); 
             
SGM_EXPORT bool GetNUBCurveData(SGM::Result               &rResult,
                                SGM::Curve          const &CurveID,
                                std::vector<SGM::Point3D> &aControlPoints,
                                std::vector<double>       &aKnots);
             
SGM_EXPORT bool GetNURBCurveData(SGM::Result               &rResult,
                                 SGM::Curve          const &CurveID,
                                 std::vector<SGM::Point4D> &aControlPoints,
                                 std::vector<double>       &aKnots);  

SGM_EXPORT bool GetPointCurveData(SGM::Result      &rResult,
                                  SGM::Curve const &CurveID,
                                  SGM::Point3D     &Pos);

//////////////////////////////////////////////////////////////////////////////
//
//  Get Surface Data
//
//////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGM::EntityType GetSurfaceType(SGM::Result        &rResult,
                                          SGM::Surface const &SurfaceID);

SGM_EXPORT SGM::Interval2D const &GetDomainOfSurface(SGM::Result        &rResult,
                                                     SGM::Surface const &SurfaceID);
             
SGM_EXPORT bool GetPlaneData(SGM::Result        &rResult,
                             SGM::Surface const &SurfaceID,
                             SGM::Point3D       &Origin,
                             SGM::UnitVector3D  &XAxis,
                             SGM::UnitVector3D  &YAxis,
                             SGM::UnitVector3D  &ZAxis);   
             
SGM_EXPORT bool GetCylinderData(SGM::Result        &rResult,
                                SGM::Surface const &SurfaceID,
                                SGM::Point3D       &Origin,
                                SGM::UnitVector3D  &XAxis,
                                SGM::UnitVector3D  &YAxis,
                                SGM::UnitVector3D  &ZAxis,
                                double             &dRadius);
            
SGM_EXPORT bool GetConeData(SGM::Result        &rResult,
                            SGM::Surface const &SurfaceID,
                            SGM::Point3D       &Origin,
                            SGM::UnitVector3D  &XAxis,
                            SGM::UnitVector3D  &YAxis,
                            SGM::UnitVector3D  &ZAxis,  // Points from center to apex.
                            double             &dHalfAngle,
                            double             &dRadius);
            
SGM_EXPORT bool GetSphereData(SGM::Result        &rResult,
                              SGM::Surface const &SurfaceID,
                              SGM::Point3D       &Center,
                              SGM::UnitVector3D  &XAxis,
                              SGM::UnitVector3D  &YAxis,
                              SGM::UnitVector3D  &ZAxis,
                              double             &dRadius);
            
SGM_EXPORT bool GetTorusData(SGM::Result        &rResult,
                             SGM::Surface const &SurfaceID,
                             SGM::Point3D       &Center,
                             SGM::UnitVector3D  &XAxis,
                             SGM::UnitVector3D  &YAxis,
                             SGM::UnitVector3D  &ZAxis,
                             double             &dMinorRadius,
                             double             &dMajorRadius,
                             SGM::TorusKindType &nKind);

SGM_EXPORT bool GetRevolveData(SGM::Result       &rResult,
                               SGM::Surface      &SurfaceID,
                               SGM::Point3D      &Origin,
                               SGM::UnitVector3D &Axis,
                               SGM::Curve        &CurveID);
            
SGM_EXPORT bool GetNUBSurfaceData(SGM::Result                             &rResult,
                                  SGM::Surface                      const &SurfaceID,
                                  std::vector<std::vector<SGM::Point3D> > &aaControlPoints,
                                  std::vector<double>                     &aUKnots,
                                  std::vector<double>                     &aVKnots);
             
SGM_EXPORT bool GetNURBSurfaceData(SGM::Result                             &rResult,
                                   SGM::Surface                      const &SurfaceID,
                                   std::vector<std::vector<SGM::Point4D> > &aaControlPoints,
                                   std::vector<double>                     &aUKnots,
                                   std::vector<double>                     &aVKnots);

    } // End of SGM namespace

#endif // SGM_GEOMETRY_H