#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMChecker.h"
#include "SGMComplex.h"
#include "SGMTopology.h"
#include "SGMIntersecors.h"
#include "Primitive.h"
#include "EntityClasses.h"
#include "Topology.h"
#include "Intersectors.h"

size_t SGM::IntersectCurves(SGM::Result                        &rResult,
                            SGM::Curve                   const &CurveID1,
                            SGM::Curve                   const &CurveID2,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes,
                            SGM::Edge                    const *pEdge1,
                            SGM::Edge                    const *pEdge2)
    {
    curve const *pCurve1=(curve const *)rResult.GetThing()->FindEntity(CurveID1.m_ID);
    curve const *pCurve2=(curve const *)rResult.GetThing()->FindEntity(CurveID2.m_ID);
    edge const *pedge1=NULL;
    edge const *pedge2=NULL;
    if(pEdge1)
        {
        pedge1=(edge const *)rResult.GetThing()->FindEntity(pEdge1->m_ID);
        }
    if(pEdge2)
        {
        pedge2=(edge const *)rResult.GetThing()->FindEntity(pEdge2->m_ID);
        }
    return ::IntersectCurves(rResult,pCurve1,pCurve2,aPoints,aTypes,pedge1,pedge2);
    }

 size_t SGM::IntersectCurveAndSurface(SGM::Result                        &rResult,
                                      SGM::Curve                   const &CurveID,
                                      SGM::Surface                 const &SurfaceID,
                                      std::vector<SGM::Point3D>          &aPoints,
                                      std::vector<SGM::IntersectionType> &aTypes,
                                      SGM::Edge                    const *pEdge,
                                      SGM::Face                    const *pFace)
     {
     curve const *pCurve=(curve const *)rResult.GetThing()->FindEntity(CurveID.m_ID);
     surface const *pSurface=(surface const *)rResult.GetThing()->FindEntity(SurfaceID.m_ID);
     edge const *pedge=NULL;
     face const *pface=NULL;
     if(pEdge)
         {
         pedge=(edge const *)rResult.GetThing()->FindEntity(pEdge->m_ID);
         }
     if(pFace)
         {
         pface=(face const *)rResult.GetThing()->FindEntity(pFace->m_ID);
         }
     return ::IntersectCurveAndSurface(rResult,pCurve,pSurface,aPoints,aTypes,pedge,pface);
     }

SGM::Complex SGM::CreatePoints(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aPoints)
    {
    complex *pComplex=new complex(rResult,aPoints);
    return SGM::Complex(pComplex->GetID());
    }

SGM::Complex SGM::CreateSegments(SGM::Result                    &rResult,
                                 std::vector<SGM::Point3D> const &aPoints,
                                 std::vector<size_t>       const &aSegments)
    {
    complex *pComplex=new complex(rResult,aSegments,aPoints);
    return SGM::Complex(pComplex->GetID());
    }

void SGM::FindBodies(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<SGM::Body> &sBodies)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<body *> sbodies;
    FindBodies(rResult,pEntity,sbodies);
    std::set<body *>::iterator iter=sbodies.begin();
    while(iter!=sbodies.end())
        {
        sBodies.insert(SGM::Body((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindComplexes(SGM::Result            &rResult,
                   SGM::Entity      const &EntityID,
                   std::set<SGM::Complex> &sComplexes)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<complex *> sComplex;
    FindComplexes(rResult,pEntity,sComplex);
    std::set<complex *>::iterator iter=sComplex.begin();
    while(iter!=sComplex.end())
        {
        sComplexes.insert(SGM::Complex((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindVolumes(SGM::Result           &rResult,
                 SGM::Entity     const &EntityID,
                 std::set<SGM::Volume> &sVolumes)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<volume *> sVolume;
    FindVolumes(rResult,pEntity,sVolume);
    std::set<volume *>::iterator iter=sVolume.begin();
    while(iter!=sVolume.end())
        {
        sVolumes.insert(SGM::Volume((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindFaces(SGM::Result         &rResult,
               SGM::Entity   const &EntityID,
               std::set<SGM::Face> &sFaces)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<face *> sFace;
    FindFaces(rResult,pEntity,sFace);
    std::set<face *>::iterator iter=sFace.begin();
    while(iter!=sFace.end())
        {
        sFaces.insert(SGM::Face((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindEdges(SGM::Result         &rResult,
               SGM::Entity   const &EntityID,
               std::set<SGM::Edge> &sEdges)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<edge *> sEdge;
    FindEdges(rResult,pEntity,sEdge);
    std::set<edge *>::iterator iter=sEdge.begin();
    while(iter!=sEdge.end())
        {
        sEdges.insert(SGM::Edge((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindVertices(SGM::Result           &rResult,
                  SGM::Entity     const &EntityID,
                  std::set<SGM::Vertex> &sVertices)
    {
    entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<vertex *> sVertex;
    FindVertices(rResult,pEntity,sVertex);
    std::set<vertex *>::iterator iter=sVertex.begin();
    while(iter!=sVertex.end())
        {
        sVertices.insert(SGM::Vertex((*iter)->GetID()));
        ++iter;
        }
    }

SGM::Result SGM::CreateResult()
    {
    thing *pThing=new thing();
    return SGM::Result(pThing); 
    }

thing *SGM::CreateThing()
    {
    return new thing();
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
