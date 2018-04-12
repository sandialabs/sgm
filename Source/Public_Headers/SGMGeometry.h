#ifndef SGM_GEOMETRY_H
#define SGM_GEOMETRY_H

#include "SGMEntityClasses.h"
#include "SGMDataClasses.h"
#include <vector>

namespace SGM
    {
///////////////////////////////////////////////////////////////////////////////
//
//  Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM::Surface CreatePlane(SGM::Result        &rResult,
                         SGM::Thing         &ThingID,
                         SGM::Point3D const &Origin,
                         SGM::Point3D const &XPos,
                         SGM::Point3D const &YPos);

SGM::Curve CreateLine(SGM::Result        &rResult,
                      SGM::Thing         &ThingID,
                      SGM::Point3D const &Origin,
                      SGM::Point3D const &Axis);


SGM::Curve CreateNUBCurve(SGM::Result                     &rResult,
                          std::vector<SGM::Point3D> const &aInterpolate,
                          std::vector<double>       const *pParams=nullptr);

SGM::Curve CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                        std::vector<SGM::Point3D> const &aInterpolate,
                                        SGM::Vector3D             const &StartVec,
                                        SGM::Vector3D             const &EndVec,
                                        std::vector<double>       const *pParams=nullptr);

///////////////////////////////////////////////////////////////////////////////
//
//  Interrogation Functions
//
///////////////////////////////////////////////////////////////////////////////

void EvaluateCurve(SGM::Result      &rResult,
                   SGM::Curve const &CurveID, 
                   double            dt,
                   SGM::Point3D     *pPos=nullptr,
                   SGM::Vector3D    *pVec1=nullptr,
                   SGM::Vector3D    *pVec2=nullptr);

double CurveInverse(SGM::Result        &rResult,
                    SGM::Curve   const &CurveID,
                    SGM::Point3D const &Pos,
                    SGM::Point3D       *pClosePos=nullptr,
                    double       const *pGuess=nullptr);

void EvaluateSurface(SGM::Result             &rResult,
                     SGM::Surface      const &SurfaceID,
                     SGM::Point2D      const &uv,
                     SGM::Point3D            *pPos=nullptr,
                     SGM::Vector3D           *pDu=nullptr,
                     SGM::Vector3D           *pDv=nullptr,
                     SGM::UnitVector3D       *pNorm=nullptr,
                     SGM::Vector3D           *pDuu=nullptr,
                     SGM::Vector3D           *pDuv=nullptr,
                     SGM::Vector3D           *pDvv=nullptr);

SGM::Point2D SurfaceInverse(SGM::Result        &rResult,
                            SGM::Surface const &SurfaceID,
                            SGM::Point3D const &Pos,
                            SGM::Point3D       *pClosePos=nullptr,
                            SGM::Point2D const *pGuess=nullptr);

    } // End of SGM namespace

#endif // SGM_GEOMETRY_H