#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMPrimatives.h"
#include "SGMGeometry.h"
#include "SGMChecker.h"
#include "Primitive.h"

SGM::Result SGM::CreateResult()
    {
    thing *pThing=new thing();
    return SGM::Result(pThing); 
    }

SGM::Body SGM::CreateBlock(SGM::Result        &rResult,
                           SGM::Point3D const &Point1,
                           SGM::Point3D const &Point2)
    {
    thing *pThing=rResult.GetThing();
    body *pBody=CreateBlock(rResult,pThing,Point1,Point2);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateSphere(SGM::Result        &rResult,
                            SGM::Point3D const &Center,
                            double              dRadius)
    {
    thing *pThing=rResult.GetThing();
    body *pBody=CreateSphere(rResult,pThing,Center,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateCylinder(SGM::Result        &rResult,
                              SGM::Point3D const &BottomCenter,
                              SGM::Point3D const &TopCenter,
                              double              dRadius)
    {
    thing *pThing=rResult.GetThing();
    body *pBody=CreateCylinder(rResult,pThing,BottomCenter,TopCenter,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateTorus(SGM::Result             &rResult,
                           SGM::Point3D      const &Center,
                           SGM::UnitVector3D const &Axis,
                           double                   dMajorRadius,
                           double                   dMinorRadius,
                           bool                     bApple)
    {
    thing *pThing=rResult.GetThing();
    body *pBody=CreateTorus(rResult,pThing,Center,Axis,dMajorRadius,dMinorRadius,bApple);
    return SGM::Body(pBody->GetID());
    }

bool SGM::CheckEntity(SGM::Result              &rResult,
                      SGM::Entity        const &EntityID,
                      SGM::CheckOptions  const &Options,
                      std::vector<std::string> &aCheckStrings)
    {
    thing *pThing=rResult.GetThing();
    entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    return pEntity->Check(rResult,Options,aCheckStrings);
    }

void SGM::EvaluateCurve(SGM::Result      &rResult,
                        SGM::Curve const &CurveID, 
                        double            dt,
                        SGM::Point3D     *pPos,
                        SGM::Vector3D    *pVec1,
                        SGM::Vector3D    *pVec2)
    {
    thing *pThing=rResult.GetThing();
    curve *pCurve=(curve *)(pThing->FindEntity(CurveID.m_ID));
    pCurve->Evaluate(dt,pPos,pVec1,pVec2);
    }

double SGM::CurveInverse(SGM::Result        &rResult,
                         SGM::Curve   const &CurveID,
                         SGM::Point3D const &Pos,
                         SGM::Point3D       *pClosePos,
                         double       const *pGuess)
    {
    thing *pThing=rResult.GetThing();
    curve *pCurve=(curve *)(pThing->FindEntity(CurveID.m_ID));
    return pCurve->Inverse(Pos,pClosePos,pGuess);
    }

void SGM::EvaluateSurface(SGM::Result             &rResult,
                          SGM::Surface      const &SurfaceID,
                          SGM::Point2D      const &uv,
                          SGM::Point3D            *pPos,
                          SGM::Vector3D           *pDu,
                          SGM::Vector3D           *pDv,
                          SGM::UnitVector3D       *pNorm,
                          SGM::Vector3D           *pDuu,
                          SGM::Vector3D           *pDuv,
                          SGM::Vector3D           *pDvv)
    {
    thing *pThing=rResult.GetThing();
    surface *pSurface=(surface *)(pThing->FindEntity(SurfaceID.m_ID));
    pSurface->Evaluate(uv,pPos,pDu,pDv,pNorm,pDuu,pDuv,pDvv);
    }

SGM::Point2D SGM::SurfaceInverse(SGM::Result        &rResult,
                                 SGM::Surface const &SurfaceID,
                                 SGM::Point3D const &Pos,
                                 SGM::Point3D       *pClosePos,
                                 SGM::Point2D const *pGuess)
    {
    thing *pThing=rResult.GetThing();
    surface *pSurface=(surface *)(pThing->FindEntity(SurfaceID.m_ID));
    return pSurface->Inverse(Pos,pClosePos,pGuess);
    }

SGM::Curve SGM::CreateNUBCurve(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aInterpolate,
                               std::vector<double>       const *pParams)
    {
    curve *pCurve=::CreateNUBCurve(rResult,aInterpolate,pParams);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                             std::vector<SGM::Point3D> const &aInterpolate,
                                             SGM::Vector3D             const &StartVec,
                                             SGM::Vector3D             const &EndVec,
                                             std::vector<double>       const *pParams)
    {
    curve *pCurve=::CreateNUBCurveWithEndVectors(rResult,aInterpolate,StartVec,EndVec,pParams);
    return SGM::Curve(pCurve->GetID());
    }