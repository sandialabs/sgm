#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMChecker.h"
#include "SGMComplex.h"
#include "SGMTopology.h"
#include "SGMIntersecors.h"
#include "SGMDisplay.h"
#include "SGMEntityFunctions.h"
#include "SGMMathematics.h"
#include "SGMQuery.h"
#include "SGMTranslators.h"

#include "Primitive.h"
#include "EntityClasses.h"
#include "Topology.h"
#include "Intersectors.h"

std::vector<size_t> const &SGM::GetFaceTriangles(SGM::Result     &rResult,
                                                 SGM::Face const &FaceID)
    {
    Impl::face *pFace=(Impl::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetTriangles(rResult);    
    }

std::vector<SGM::Point3D> const &SGM::GetFacePoints(SGM::Result     &rResult,
                                                    SGM::Face const &FaceID)
    {
    Impl::face *pFace=(Impl::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetPoints3D(rResult);
    }

std::vector<SGM::Point3D> const &SGM::GetEdgePoints(SGM::Result     &rResult,
                                                    SGM::Edge const &EdgeID)
    {
    Impl::edge *pEdge=(Impl::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);    
    return pEdge->GetFacets();
    }

std::vector<SGM::UnitVector3D> const &SGM::GetFaceNormals(SGM::Result     &rResult,
                                                          SGM::Face const &FaceID)
    {
    Impl::face *pFace=(Impl::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetNormals(rResult);
    }

SGM::Interval3D const &SGM::GetBoundingBox(SGM::Result       &rResult,
                                           SGM::Entity const &EntityID)
    {
    Impl::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    return pEntity->GetBox();
    }

void SGM::DeleteEntity(SGM::Result &rResult,
                       SGM::Entity &EntityID)
    {
    Impl::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    rResult.GetThing()->DeleteEntity(pEntity);
    }


size_t SGM::IntersectCurves(SGM::Result                        &rResult,
                            SGM::Curve                   const &CurveID1,
                            SGM::Curve                   const &CurveID2,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes,
                            SGM::Edge                    const *pEdge1,
                            SGM::Edge                    const *pEdge2)
    {
    using namespace Impl;
    curve const *pCurve1=(curve const *)rResult.GetThing()->FindEntity(CurveID1.m_ID);
    curve const *pCurve2=(curve const *)rResult.GetThing()->FindEntity(CurveID2.m_ID);
    edge const *pedge1=nullptr;
    edge const *pedge2=nullptr;
    if(pEdge1)
        {
        pedge1=(edge const *)rResult.GetThing()->FindEntity(pEdge1->m_ID);
        }
    if(pEdge2)
        {
        pedge2=(edge const *)rResult.GetThing()->FindEntity(pEdge2->m_ID);
        }
    return ::SGM::Impl::IntersectCurves(rResult,pCurve1,pCurve2,aPoints,aTypes,pedge1,pedge2);
    }

 size_t SGM::IntersectCurveAndSurface(SGM::Result                        &rResult,
                                      SGM::Curve                   const &CurveID,
                                      SGM::Surface                 const &SurfaceID,
                                      std::vector<SGM::Point3D>          &aPoints,
                                      std::vector<SGM::IntersectionType> &aTypes,
                                      SGM::Edge                    const *pEdge,
                                      SGM::Face                    const *pFace)
     {
     using namespace Impl;
     curve const *pCurve=(curve const *)rResult.GetThing()->FindEntity(CurveID.m_ID);
     surface const *pSurface=(surface const *)rResult.GetThing()->FindEntity(SurfaceID.m_ID);
     edge const *pedge=nullptr;
     face const *pface=nullptr;
     if(pEdge)
         {
         pedge=(edge const *)rResult.GetThing()->FindEntity(pEdge->m_ID);
         }
     if(pFace)
         {
         pface=(face const *)rResult.GetThing()->FindEntity(pFace->m_ID);
         }
     return Impl::IntersectCurveAndSurface(rResult,pCurve,pSurface,aPoints,aTypes,pedge,pface);
     }

SGM::Point3D const &SGM::GetPointOfVertex(SGM::Result       &rResult,
                                          SGM::Vertex const &VertexID)
    {
    Impl::vertex const *pVertex=(Impl::vertex const *)rResult.GetThing()->FindEntity(VertexID.m_ID);
    return pVertex->GetPoint();
    }

SGM::Complex SGM::CreatePoints(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aPoints)
    {
    Impl::complex *pComplex=new Impl::complex(rResult,aPoints);
    return SGM::Complex(pComplex->GetID());
    }

SGM::Complex SGM::CreateSegments(SGM::Result                    &rResult,
                                 std::vector<SGM::Point3D> const &aPoints,
                                 std::vector<size_t>       const &aSegments)
    {
    Impl::complex *pComplex=new Impl::complex(rResult,aSegments,aPoints);
    return SGM::Complex(pComplex->GetID());
    }

void SGM::FindBodies(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<SGM::Body> &sBodies)
    {
    using namespace Impl;
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
    using namespace Impl;
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
    using namespace Impl;
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
    using namespace Impl;
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

void SGM::FindSurfaces(SGM::Result         &rResult,
                       SGM::Entity   const &EntityID,
                       std::set<SGM::Surface> &sSurfaces)
    {
      entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
      std::set<surface *> sSurface;
      FindSurfaces(rResult,pEntity,sSurface);
      std::set<surface *>::iterator iter=sSurface.begin();
      while(iter!=sSurface.end())
      {
        sSurfaces.insert(SGM::Surface((*iter)->GetID()));
        ++iter;
      }
}

void SGM::FindEdges(SGM::Result         &rResult,
               SGM::Entity   const &EntityID,
               std::set<SGM::Edge> &sEdges)
    {
    using namespace Impl;
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

void SGM::FindCurves(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<Curve> &sCurves)
{
  entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
  std::set<curve *> sCurve;
  FindCurves(rResult,pEntity,sCurve);
  std::set<curve *>::iterator iter=sCurve.begin();
  while(iter!=sCurve.end())
  {
    sCurves.insert(SGM::Curve((*iter)->GetID()));
    ++iter;
  }
}

void SGM::FindVertices(SGM::Result           &rResult,
                  SGM::Entity     const &EntityID,
                  std::set<SGM::Vertex> &sVertices)
    {
    using namespace Impl;
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

SGM::Impl::thing *SGM::CreateThing()
    {
    return new Impl::thing();
    }

void SGM::DeleteThing(Impl::thing *pThing)
    {
    delete pThing;
    }

SGM::Body SGM::CreateBlock(SGM::Result        &rResult,
                           SGM::Point3D const &Point1,
                           SGM::Point3D const &Point2)
    {
    Impl::thing *pThing=rResult.GetThing();
    Impl::body *pBody=CreateBlock(rResult,pThing,Point1,Point2);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateSphere(SGM::Result        &rResult,
                            SGM::Point3D const &Center,
                            double              dRadius)
    {
    Impl::thing *pThing=rResult.GetThing();
    Impl::body *pBody=CreateSphere(rResult,pThing,Center,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateCylinder(SGM::Result        &rResult,
                              SGM::Point3D const &BottomCenter,
                              SGM::Point3D const &TopCenter,
                              double              dRadius)
    {
    Impl::thing *pThing=rResult.GetThing();
    Impl::body *pBody=CreateCylinder(rResult,pThing,BottomCenter,TopCenter,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateTorus(SGM::Result             &rResult,
                           SGM::Point3D      const &Center,
                           SGM::UnitVector3D const &Axis,
                           double                   dMajorRadius,
                           double                   dMinorRadius,
                           bool                     bApple)
    {
    Impl::thing *pThing=rResult.GetThing();
    Impl::body *pBody=CreateTorus(rResult,pThing,Center,Axis,dMajorRadius,dMinorRadius,bApple);
    return SGM::Body(pBody->GetID());
    }

bool SGM::CheckEntity(SGM::Result              &rResult,
                      SGM::Entity        const &EntityID,
                      SGM::CheckOptions  const &Options,
                      std::vector<std::string> &aCheckStrings)
    {
    Impl::thing *pThing=rResult.GetThing();
    Impl::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    return pEntity->Check(rResult,Options,aCheckStrings);
    }

void SGM::EvaluateCurve(SGM::Result      &rResult,
                        SGM::Curve const &CurveID, 
                        double            dt,
                        SGM::Point3D     *pPos,
                        SGM::Vector3D    *pVec1,
                        SGM::Vector3D    *pVec2)
    {
    using namespace Impl;
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
    using namespace Impl;
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
    using namespace Impl;
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
    using namespace Impl;
    thing *pThing=rResult.GetThing();
    surface *pSurface=(surface *)(pThing->FindEntity(SurfaceID.m_ID));
    return pSurface->Inverse(Pos,pClosePos,pGuess);
    }

SGM::Curve SGM::CreateNUBCurve(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aInterpolate,
                               std::vector<double>       const *pParams)
    {
    Impl::curve *pCurve=Impl::CreateNUBCurve(rResult,aInterpolate,pParams);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                             std::vector<SGM::Point3D> const &aInterpolate,
                                             SGM::Vector3D             const &StartVec,
                                             SGM::Vector3D             const &EndVec,
                                             std::vector<double>       const *pParams)
    {
    Impl::curve *pCurve=Impl::CreateNUBCurveWithEndVectors(rResult,aInterpolate,StartVec,EndVec,pParams);
    return SGM::Curve(pCurve->GetID());
    }
