#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMInterval.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMChecker.h"
#include "SGMComplex.h"
#include "SGMTopology.h"
#include "SGMIntersector.h"
#include "SGMDisplay.h"
#include "SGMEntityFunctions.h"
#include "SGMMathematics.h"
#include "SGMMeasure.h"
#include "SGMInterrogate.h"
#include "SGMTranslators.h"

#include "Primitive.h"
#include "EntityClasses.h"
#include "Topology.h"
#include "Intersectors.h"
#include "STEP.h"
#include "Query.h"
#include "FileFunctions.h"
#include "EntityFunctions.h"
#include "Curve.h"
#include "Faceter.h"
#include "Surface.h"
#include "Interrogate.h"

#include <algorithm>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

size_t SGM::RayFire(SGM::Result                        &rResult,
                    SGM::Point3D                 const &Origin,
                    SGM::UnitVector3D            const &Axis,
                    SGM::Entity                  const &EntityID,
                    std::vector<SGM::Point3D>          &aPoints,
                    std::vector<SGM::IntersectionType> &aTypes,
                    double                              dTolerance)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);  
    return SGMInternal::RayFire(rResult,Origin,Axis,pEntity,aPoints,aTypes,dTolerance);
    }

SGM::Complex SGM::CreateTriangles(SGM::Result                     &rResult,
                                  std::vector<SGM::Point3D> const &aPoints,
                                  std::vector<size_t>       const &aTriangles)
    {
    if(aPoints.empty() || aTriangles.empty())
        {
        rResult.SetResult(SGM::ResultTypeInsufficientData);
        return SGM::Complex(0);
        }
    SGMInternal::complex *pComplex=new SGMInternal::complex(rResult,aPoints,aTriangles);
    return SGM::Complex(pComplex->GetID());
    }

size_t SGM::FindComponents(SGM::Result               &,//rResult,
                           SGM::Complex const        &,//ComplexID,
                           std::vector<SGM::Complex> &)//aComponents)
    {
    return 0;
    }

size_t SGM::FindBoundary(SGM::Result               &,//rResult,
                         SGM::Complex        const &,//ComplexID,
                         std::vector<SGM::Complex> &)//aBoundary)
    {
    return 0;
    }

size_t SGM::FindGenus(SGM::Result        &,//rResult,
                      SGM::Complex const &)//ComplexID)
    {
    return 0;
    }

size_t SGM::SplitWithPlane(SGM::Result               &,//rResult,
                           SGM::Complex        const &,//ComplexID,
                           SGM::Point3D        const &,//Point,
                           SGM::UnitVector3D   const &,//Normal,
                           std::vector<SGM::Complex> &)//aComponents)
    {
    return 0;
    }

size_t SGM::SplitWithSlices(SGM::Result                     &,//rResult,
                            SGM::Complex              const &,//ComplexID,
                            std::vector<SGM::Complex> const &,//aSlices,
                            std::vector<SGM::Complex>       &)//aComponents)
    {
    return 0;
    }

size_t SGM::SplitWithComplex(SGM::Result               &,//rResult,
                             SGM::Complex        const &,//ComplexID,
                             SGM::Complex        const &,//SliceID,
                             std::vector<SGM::Complex> &)//aComponents)
    {
    return 0;
    }

SGM::Complex SGM::CreateRectangle(SGM::Result        &rResult,
                                  SGM::Point2D const &Pos0,
                                  SGM::Point2D const &Pos1,
                                  bool                bFilled)
    {
    double dMinX=std::min(Pos0.m_u,Pos1.m_u);
    double dMaxX=std::max(Pos0.m_u,Pos1.m_u);
    double dMinY=std::min(Pos0.m_v,Pos1.m_v);
    double dMaxY=std::max(Pos0.m_v,Pos1.m_v);

    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(4);
    aPoints.emplace_back(dMinX,dMinY,0);
    aPoints.emplace_back(dMaxX,dMinY,0);
    aPoints.emplace_back(dMaxX,dMaxY,0);
    aPoints.emplace_back(dMinX,dMaxY,0);

    SGMInternal::complex *pComplex;
    if(bFilled)
        {
        std::vector<size_t> aTriangles;
        aTriangles.reserve(6);
        aTriangles.push_back(0);
        aTriangles.push_back(1);
        aTriangles.push_back(3);
        aTriangles.push_back(3);
        aTriangles.push_back(1);
        aTriangles.push_back(2);
        pComplex=new SGMInternal::complex(rResult,aPoints,aTriangles);
        }
    else
        {
        std::vector<size_t> aSegments;
        aSegments.reserve(8);
        aSegments.push_back(0);
        aSegments.push_back(1);
        aSegments.push_back(1);
        aSegments.push_back(2);
        aSegments.push_back(2);
        aSegments.push_back(3);
        aSegments.push_back(3);
        aSegments.push_back(0);
        pComplex=new SGMInternal::complex(rResult,aSegments,aPoints);
        }

    return Complex(pComplex->GetID());
    }

SGM::Complex SGM::CreateSlice(SGM::Result   &,//rResult,
                    SGM::Complex      const &,//ComplexID,
                    SGM::Point3D      const &,//Point,
                    SGM::UnitVector3D const &,//Normal,
                    bool                     )//bLocal)
    {
    return Complex(0);
    }

SGM::Complex SGM::CreatePolygon(SGM::Result                &,//rResult,
                                std::vector<Point3D> const &,//aPoints,
                                bool                        )//bFilled)
    {
    return Complex(0);
    }

std::vector<size_t> const &SGM::GetFaceTriangles(SGM::Result     &rResult,
                                                 SGM::Face const &FaceID)
    {
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetTriangles(rResult);    
    }

std::vector<SGM::Point2D> const &SGM::GetFacePoints2D(SGM::Result     &rResult,
                                                      SGM::Face const &FaceID)
    {
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetPoints2D(rResult);
    }

std::vector<SGM::Point3D> const &SGM::GetFacePoints3D(SGM::Result     &rResult,
                                                      SGM::Face const &FaceID)
    {
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetPoints3D(rResult);
    }

std::vector<SGM::Point3D> const &SGM::GetEdgePoints(SGM::Result     &rResult,
                                                    SGM::Edge const &EdgeID)
    {
    SGMInternal::edge *pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);    
    return pEdge->GetFacets(rResult);
    }

std::vector<SGM::UnitVector3D> const &SGM::GetFaceNormals(SGM::Result     &rResult,
                                                          SGM::Face const &FaceID)
    {
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);    
    return pFace->GetNormals(rResult);
    }

SGM::Interval3D const &SGM::GetBoundingBox(SGM::Result       &rResult,
                                           SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    return pEntity->GetBox();
    }

void SGM::DeleteEntity(SGM::Result &rResult,
                       SGM::Entity &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    SGMInternal::DeleteEntity(rResult,pEntity);
    }


size_t SGM::IntersectCurves(SGM::Result                        &rResult,
                            SGM::Curve                   const &CurveID1,
                            SGM::Curve                   const &CurveID2,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes,
                            SGM::Edge                    const *pEdge1,
                            SGM::Edge                    const *pEdge2,
                            double                              dTolerance)
    {
    SGMInternal::curve const *pCurve1=(SGMInternal::curve const *)rResult.GetThing()->FindEntity(CurveID1.m_ID);
    SGMInternal::curve const *pCurve2=(SGMInternal::curve const *)rResult.GetThing()->FindEntity(CurveID2.m_ID);
    SGMInternal::edge const *pedge1=nullptr;
    SGMInternal::edge const *pedge2=nullptr;
    if(pEdge1)
        {
        pedge1=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(pEdge1->m_ID);
        }
    if(pEdge2)
        {
        pedge2=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(pEdge2->m_ID);
        }
    return IntersectCurves(rResult,pCurve1,pCurve2,aPoints,aTypes,pedge1,pedge2,dTolerance);
    }

 size_t SGM::IntersectCurveAndSurface(SGM::Result                        &rResult,
                                      SGM::Curve                   const &CurveID,
                                      SGM::Surface                 const &SurfaceID,
                                      std::vector<SGM::Point3D>          &aPoints,
                                      std::vector<SGM::IntersectionType> &aTypes,
                                      SGM::Edge                    const *pEdge,
                                      SGM::Face                    const *pFace,
                                      double                              dTolerance)
     {
     SGMInternal::curve const *pCurve=(SGMInternal::curve const *)rResult.GetThing()->FindEntity(CurveID.m_ID);
     SGMInternal::surface const *pSurface=(SGMInternal::surface const *)rResult.GetThing()->FindEntity(SurfaceID.m_ID);
     SGMInternal::edge const *pedge=nullptr;
     SGMInternal::face const *pface=nullptr;
     if(pEdge)
         {
         pedge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(pEdge->m_ID);
         }
     if(pFace)
         {
         pface=(SGMInternal::face const *)rResult.GetThing()->FindEntity(pFace->m_ID);
         }
     return IntersectCurveAndSurface(rResult,pCurve,pSurface,aPoints,aTypes,pedge,pface,dTolerance);
     }

 
size_t SGM::IntersectSurfaces(SGM::Result               &rResult,
                              SGM::Surface        const &SurfaceID1,
                              SGM::Surface        const &SurfaceID2,
                              std::vector<SGM::Curve>   &aCurves,
                              SGM::Face           const *pFace1,
                              SGM::Face           const *pFace2,
                              double                     dTolerance)
    {
    SGMInternal::surface const *pSurface1=(SGMInternal::surface const *)rResult.GetThing()->FindEntity(SurfaceID1.m_ID);
    SGMInternal::surface const *pSurface2=(SGMInternal::surface const *)rResult.GetThing()->FindEntity(SurfaceID2.m_ID);
    SGMInternal::face const *pface1=nullptr;
    SGMInternal::face const *pface2=nullptr;
    if(pFace1)
        {
        pface1=(SGMInternal::face const *)rResult.GetThing()->FindEntity(pFace1->m_ID);
        }
    if(pFace2)
        {
        pface2=(SGMInternal::face const *)rResult.GetThing()->FindEntity(pFace2->m_ID);
        }
    std::vector<SGMInternal::curve *> acurves;
    size_t nAnswer=IntersectSurfaces(rResult,pSurface1,pSurface2,acurves,pface1,pface2,dTolerance);
    size_t Index1;
    for(Index1=0;Index1<nAnswer;++Index1)
        {
        aCurves.push_back(SGM::Curve(acurves[Index1]->GetID()));
        }
    return nAnswer;
    }


SGM::Point3D const &SGM::GetPointOfVertex(SGM::Result       &rResult,
                                          SGM::Vertex const &VertexID)
    {
    SGMInternal::vertex const *pVertex=(SGMInternal::vertex const *)rResult.GetThing()->FindEntity(VertexID.m_ID);
    return pVertex->GetPoint();
    }

SGM::Complex SGM::CreatePoints(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aPoints)
    {
    SGMInternal::complex *pComplex=new SGMInternal::complex(rResult,aPoints);
    return SGM::Complex(pComplex->GetID());
    }

SGM::Complex SGM::CreateSegments(SGM::Result                    &rResult,
                                 std::vector<SGM::Point3D> const &aPoints,
                                 std::vector<size_t>       const &aSegments)
    {
    SGMInternal::complex *pComplex=new SGMInternal::complex(rResult,aSegments,aPoints);
    return SGM::Complex(pComplex->GetID());
    }

void SGM::FindBodies(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<SGM::Body> &sBodies,
                     bool                 bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::body *,SGMInternal::EntityCompare> sbodies;
    FindBodies(rResult,pEntity,sbodies,bTopLevel);
    std::set<SGMInternal::body *>::iterator iter=sbodies.begin();
    while(iter!=sbodies.end())
        {
        sBodies.insert(SGM::Body((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindComplexes(SGM::Result            &rResult,
                        SGM::Entity      const &EntityID,
                        std::set<SGM::Complex> &sComplexes,
                        bool                    bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::complex *,SGMInternal::EntityCompare> sComplex;
    FindComplexes(rResult,pEntity,sComplex,bTopLevel);
    std::set<SGMInternal::complex *,SGMInternal::EntityCompare>::iterator iter=sComplex.begin();
    while(iter!=sComplex.end())
        {
        sComplexes.insert(SGM::Complex((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindVolumes(SGM::Result           &rResult,
                      SGM::Entity     const &EntityID,
                      std::set<SGM::Volume> &sVolumes,
                      bool                   bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::volume *,SGMInternal::EntityCompare> sVolume;
    FindVolumes(rResult,pEntity,sVolume,bTopLevel);
    std::set<SGMInternal::volume *,SGMInternal::EntityCompare>::iterator iter=sVolume.begin();
    while(iter!=sVolume.end())
        {
        sVolumes.insert(SGM::Volume((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindFaces(SGM::Result         &rResult,
                    SGM::Entity   const &EntityID,
                    std::set<SGM::Face> &sFaces,
                    bool                 bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::face *,SGMInternal::EntityCompare> sFace;
    FindFaces(rResult,pEntity,sFace,bTopLevel);
    std::set<SGMInternal::face *>::iterator iter=sFace.begin();
    while(iter!=sFace.end())
        {
        sFaces.insert(SGM::Face((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindSurfaces(SGM::Result            &rResult,
                       SGM::Entity      const &EntityID,
                       std::set<SGM::Surface> &sSurfaces,
                       bool                    bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::surface *,SGMInternal::EntityCompare> sSurface;
    FindSurfaces(rResult,pEntity,sSurface,bTopLevel);
    std::set<SGMInternal::surface *>::iterator iter=sSurface.begin();
    while(iter!=sSurface.end())
        {
        sSurfaces.insert(SGM::Surface((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindEdges(SGM::Result         &rResult,
                    SGM::Entity   const &EntityID,
                    std::set<SGM::Edge> &sEdges,
                    bool                 bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::edge *,SGMInternal::EntityCompare> sEdge;
    FindEdges(rResult,pEntity,sEdge,bTopLevel);
    std::set<SGMInternal::edge *>::iterator iter=sEdge.begin();
    while(iter!=sEdge.end())
        {
        sEdges.insert(SGM::Edge((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindCurves(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<Curve>     &sCurves,
                     bool                 bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::curve *,SGMInternal::EntityCompare> sCurve;
    FindCurves(rResult,pEntity,sCurve,bTopLevel);
    std::set<SGMInternal::curve *>::iterator iter=sCurve.begin();
    while(iter!=sCurve.end())
        {
        sCurves.insert(SGM::Curve((*iter)->GetID()));
        ++iter;
        }
    }

void SGM::FindVertices(SGM::Result           &rResult,
                       SGM::Entity     const &EntityID,
                       std::set<SGM::Vertex> &sVertices,
                       bool                   bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::vertex *,SGMInternal::EntityCompare> sVertex;
    FindVertices(rResult,pEntity,sVertex,bTopLevel);
    std::set<SGMInternal::vertex *>::iterator iter=sVertex.begin();
    while(iter!=sVertex.end())
        {
        sVertices.insert(SGM::Vertex((*iter)->GetID()));
        ++iter;
        }
    }

SGM::Surface SGM::GetSurfaceOfFace(SGM::Result     &rResult,
                                   SGM::Face const &FaceID)
    {
    SGMInternal::face const *pFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return SGM::Surface(pFace->GetSurface()->GetID());
    }

SGMInternal::thing *SGM::CreateThing()
    {
    return new SGMInternal::thing();
    }

void SGM::DeleteThing(SGMInternal::thing *pThing)
    {
    delete pThing;
    }

SGM::Body SGM::CreateBlock(SGM::Result        &rResult,
                           SGM::Point3D const &Point1,
                           SGM::Point3D const &Point2)
    {
    SGMInternal::body *pBody=SGMInternal::CreateBlock(rResult,Point1,Point2);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateSphere(SGM::Result        &rResult,
                            SGM::Point3D const &Center,
                            double              dRadius)
    {
    SGMInternal::body *pBody=SGMInternal::CreateSphere(rResult,Center,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateCylinder(SGM::Result        &rResult,
                              SGM::Point3D const &BottomCenter,
                              SGM::Point3D const &TopCenter,
                              double              dRadius)
    {
    SGMInternal::body *pBody=SGMInternal::CreateCylinder(rResult,BottomCenter,TopCenter,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateCone(SGM::Result        &rResult,
                          SGM::Point3D const &BottomCenter,
                          SGM::Point3D const &TopCenter,
                          double              dBottomRadius,
                          double              dTopRadius)
    {
    SGMInternal::body *pBody=SGMInternal::CreateCone(rResult,BottomCenter,TopCenter,dBottomRadius,dTopRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateTorus(SGM::Result             &rResult,
                           SGM::Point3D      const &Center,
                           SGM::UnitVector3D const &Axis,
                           double                   dMinorRadius,
                           double                   dMajorRadius,
                           bool                     bApple)
    {
    SGMInternal::body *pBody=SGMInternal::CreateTorus(rResult,Center,Axis,dMinorRadius,dMajorRadius,bApple);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateRevolve(SGM::Result             &rResult,
                             SGM::Point3D      const &Origin,
                             SGM::UnitVector3D const &Axis,
                             SGM::Curve        const &IDCurve)
    {
    SGMInternal::curve const *pCurve = (SGMInternal::curve const *)rResult.GetThing()->FindEntity(IDCurve.m_ID);
    SGMInternal::body *pBody=SGMInternal::CreateRevolve(rResult, Origin, Axis, pCurve);

    return SGM::Body(pBody->GetID());
    }

SGM_EXPORT SGM::Body SGM::CreateDisk(SGM::Result             &rResult,
                                     SGM::Point3D      const &Center,
                                     SGM::UnitVector3D const &Normal,
                                     double                   dRadius)
    {
    SGMInternal::body *pBody=SGMInternal::CreateDisk(rResult,Center,Normal,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateSheetBody(SGM::Result                    &rResult,
                               SGM::Surface                   &SurfaceID,
                               std::vector<SGM::Edge>         &aEdges,
                               std::vector<SGM::EdgeSideType> &aTypes)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::surface *pSurface=(SGMInternal::surface *)pThing->FindEntity(SurfaceID.m_ID);
    std::vector<SGMInternal::edge *> aedges;
    size_t Index1;
    size_t nEdges=aEdges.size();
    aedges.reserve(nEdges);
    for(Index1=0;Index1<nEdges;++Index1)
        {
        SGMInternal::edge *pEdge=(SGMInternal::edge *)pThing->FindEntity(aEdges[Index1].m_ID);
        aedges.push_back(pEdge);
        }
    SGMInternal::body *pBody=SGMInternal::CreateSheetBody(rResult,pSurface,aedges,aTypes);
    return SGM::Body(pBody->GetID());
    }

SGM::Curve SGM::CreateLine(SGM::Result             &rResult,
                           SGM::Point3D      const &Origin,
                           SGM::UnitVector3D const &Axis)
    {
    SGMInternal::curve *pCurve=new SGMInternal::line(rResult,Origin,Axis,1.0);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateCircle(SGM::Result             &rResult,
                             SGM::Point3D      const &Center,
                             SGM::UnitVector3D const &Normal,
                             double                   dRadius)
    {
    SGMInternal::curve *pCurve=new SGMInternal::circle(rResult,Center,Normal,dRadius);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateEllipse(SGM::Result             &rResult,
                              SGM::Point3D      const &Center,
                              SGM::UnitVector3D const &XAxis,
                              SGM::UnitVector3D const &YAxis,
                              double                   dXRadius,
                              double                   dYRadius)
    {
    SGMInternal::curve *pCurve=new SGMInternal::ellipse(rResult,Center,XAxis,YAxis,dXRadius,dYRadius);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateParabola(SGM::Result             &rResult,
                               SGM::Point3D      const &Center,
                               SGM::UnitVector3D const &XAxis,
                               SGM::UnitVector3D const &YAxis,
                               double                   dA)
    {
    SGMInternal::curve *pCurve=new SGMInternal::parabola(rResult,Center,XAxis,YAxis,dA);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateHyperbola(SGM::Result             &rResult,
                                SGM::Point3D      const &Center,
                                SGM::UnitVector3D const &XAxis,
                                SGM::UnitVector3D const &YAxis,
                                double                   dA,
                                double                   dB)
    {
    SGMInternal::curve *pCurve=new SGMInternal::hyperbola(rResult,Center,XAxis,YAxis,dA,dB);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateTorusKnot(SGM::Result             &rResult,
                                SGM::Point3D      const &Center,
                                SGM::UnitVector3D const &XAxis,
                                SGM::UnitVector3D const &YAxis,
                                double                   dMinorRadius,
                                double                   dMajorRadius,
                                size_t                   nA,
                                size_t                   nB)
    {
    SGMInternal::curve *pCurve=new SGMInternal::TorusKnot(rResult,Center,XAxis,YAxis,dMinorRadius,dMajorRadius,nA,nB);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Edge SGM::CreateEdge(SGM::Result           &rResult,
                          SGM::Curve            &CurveID,
                          SGM::Interval1D const *pDomain)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::curve *pCurve=(SGMInternal::curve *)pThing->FindEntity(CurveID.m_ID);
    SGMInternal::edge *pEdge=CreateEdge(rResult,pCurve,pDomain);
    return SGM::Edge(pEdge->GetID());
    }

SGM::Entity SGM::CopyEntity(SGM::Result       &rResult,
                            SGM::Entity const &EntityID)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    return SGM::Entity(pEntity->Copy(rResult)->GetID());
    }

void SGM::TransformEntity(SGM::Result            &rResult,
                          SGM::Transform3D const &Trans,
                          SGM::Entity            &EntityID)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    pEntity->Transform(rResult,Trans);
    }

bool SGM::CheckEntity(SGM::Result              &rResult,
                      SGM::Entity        const &EntityID,
                      SGM::CheckOptions  const &Options,
                      std::vector<std::string> &aCheckStrings)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    return pEntity->Check(rResult,Options,aCheckStrings);
    }

SGM::Interval1D const &SGM::GetCurveDomain(SGM::Result      &rResult,
                                      SGM::Curve const &CurveID)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::curve *pCurve=(SGMInternal::curve *)(pThing->FindEntity(CurveID.m_ID));
    return pCurve->GetDomain();
    }

void SGM::EvaluateCurve(SGM::Result      &rResult,
                        SGM::Curve const &CurveID, 
                        double            dt,
                        SGM::Point3D     *pPos,
                        SGM::Vector3D    *pVec1,
                        SGM::Vector3D    *pVec2)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::curve *pCurve=(SGMInternal::curve *)(pThing->FindEntity(CurveID.m_ID));
    pCurve->Evaluate(dt,pPos,pVec1,pVec2);
    }

double SGM::CurveInverse(SGM::Result        &rResult,
                         SGM::Curve   const &CurveID,
                         SGM::Point3D const &Pos,
                         SGM::Point3D       *pClosePos,
                         double       const *pGuess)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::curve *pCurve=(SGMInternal::curve *)(pThing->FindEntity(CurveID.m_ID));
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
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::surface *pSurface=(SGMInternal::surface *)(pThing->FindEntity(SurfaceID.m_ID));
    pSurface->Evaluate(uv,pPos,pDu,pDv,pNorm,pDuu,pDuv,pDvv);
    }

SGM::Point2D SGM::SurfaceInverse(SGM::Result        &rResult,
                                 SGM::Surface const &SurfaceID,
                                 SGM::Point3D const &Pos,
                                 SGM::Point3D       *pClosePos,
                                 SGM::Point2D const *pGuess)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::surface *pSurface=(SGMInternal::surface *)(pThing->FindEntity(SurfaceID.m_ID));
    return pSurface->Inverse(Pos,pClosePos,pGuess);
    }

SGM::Curve SGM::CreateNUBCurve(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aInterpolate,
                               std::vector<double>       const *pParams)
    {
    SGMInternal::curve *pCurve=SGMInternal::CreateNUBCurve(rResult,aInterpolate,pParams);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Curve SGM::CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                             std::vector<SGM::Point3D> const &aInterpolate,
                                             SGM::Vector3D             const &StartVec,
                                             SGM::Vector3D             const &EndVec,
                                             std::vector<double>       const *pParams)
    {
    SGMInternal::curve *pCurve=SGMInternal::CreateNUBCurveWithEndVectors(rResult,aInterpolate,StartVec,EndVec,pParams);
    return SGM::Curve(pCurve->GetID());
    }

SGM::Surface SGM::CreateTorusSurface(SGM::Result             &rResult,
                                     SGM::Point3D      const &Center,
                                     SGM::UnitVector3D const &Axis,
                                     double                   dMinorRadius,
                                     double                   dMajorRadius,
                                     bool                     bApple)
    {
    SGMInternal::surface *pSurface=new SGMInternal::torus(rResult,Center,Axis,dMinorRadius,dMajorRadius,bApple);
    return SGM::Surface(pSurface->GetID());
    }

SGM::Surface SGM::CreateSphereSurface(SGM::Result        &rResult,
                                      SGM::Point3D const &Center,
                                      double              dRadius)
    {
    SGMInternal::surface *pSurface=new SGMInternal::sphere(rResult,Center,dRadius);
    return SGM::Surface(pSurface->GetID());
    }

bool SGM::GetLineData(SGM::Result       &rResult,
                      SGM::Curve  const &CurveID,
                      SGM::Point3D      &Origin,
                      SGM::UnitVector3D &Axis,
                      double            &dScale)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::LineType)
        {
        return false;
        }
    SGMInternal::line const *pLine=(SGMInternal::line const *)pCurve;
    Origin =pLine->m_Origin;
    Axis   =pLine->m_Axis;
    dScale =pLine->m_dScale;
    return true;
    }

bool SGM::GetCircleData(SGM::Result       &rResult,
                        SGM::Curve  const &CurveID,
                        SGM::Point3D      &Center,
                        SGM::UnitVector3D &Normal,
                        SGM::UnitVector3D &XAxis,
                        SGM::UnitVector3D &YAxis,
                        double            &dRadius)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::CircleType)
        {
        return false;
        }
    SGMInternal::circle const *pCircle=(SGMInternal::circle const *)pCurve;
    Center =pCircle->m_Center;
    Normal =pCircle->m_Normal;
    XAxis  =pCircle->m_XAxis;
    YAxis  =pCircle->m_YAxis;
    dRadius=pCircle->m_dRadius;
    return true;
    }
     
bool SGM::GetEllipseData(SGM::Result       &rResult,
                         SGM::Curve  const &CurveID,
                         SGM::Point3D      &Center,
                         SGM::UnitVector3D &XAxis,
                         SGM::UnitVector3D &YAxis,
                         SGM::UnitVector3D &Normal,
                         double            &dA,
                         double            &dB)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::EllipseType)
        {
        return false;
        }
    SGMInternal::ellipse const *pEllipse=(SGMInternal::ellipse const *)pCurve;
    Center=pEllipse->m_Center;
    Normal=pEllipse->m_Normal;
    XAxis =pEllipse->m_XAxis;
    YAxis =pEllipse->m_YAxis;
    dA    =pEllipse->m_dA;
    dB    =pEllipse->m_dB;
    return true;
    }
  
bool SGM::GetParabolaData(SGM::Result       &rResult,
                          SGM::Curve  const &CurveID,
                          SGM::Point3D      &Center,
                          SGM::UnitVector3D &XAxis,
                          SGM::UnitVector3D &YAxis,
                          SGM::UnitVector3D &Normal,
                          double            &dA)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::ParabolaType)
        {
        return false;
        }
    SGMInternal::parabola const *pParabola=(SGMInternal::parabola const *)pCurve;
    Center=pParabola->m_Center;
    Normal=pParabola->m_Normal;
    XAxis =pParabola->m_XAxis;
    YAxis =pParabola->m_YAxis;
    dA    =pParabola->m_dA;
    return true;
    }
  
bool SGM::GetHyperbolaData(SGM::Result       &rResult,
                           SGM::Curve  const &CurveID,
                           SGM::Point3D      &Center,
                           SGM::UnitVector3D &XAxis,
                           SGM::UnitVector3D &YAxis,
                           SGM::UnitVector3D &Normal,
                           double            &dA,
                           double            &dB)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::HyperbolaType)
        {
        return false;
        }
    SGMInternal::hyperbola const *pHyperbola=(SGMInternal::hyperbola const *)pCurve;
    Center=pHyperbola->m_Center;
    Normal=pHyperbola->m_Normal;
    XAxis =pHyperbola->m_XAxis;
    YAxis =pHyperbola->m_YAxis;
    dA    =pHyperbola->m_dA;
    dB    =pHyperbola->m_dB;
    return true;
    }
  
bool SGM::GetNUBCurveData(SGM::Result               &rResult,
                          SGM::Curve          const &CurveID,
                          std::vector<SGM::Point3D> &aControlPoints,
                          std::vector<double>       &aKnots)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::NUBCurveType)
        {
        return false;
        }
    SGMInternal::NUBcurve const *pNUBCurve=(SGMInternal::NUBcurve const *)pCurve;
    aControlPoints=pNUBCurve->m_aControlPoints;
    aKnots        =pNUBCurve->m_aKnots;
    return true;
    }
  
bool SGM::GetNURBCurveData(SGM::Result               &rResult,
                           SGM::Curve          const &CurveID,
                           std::vector<SGM::Point4D> &aControlPoints,
                           std::vector<double>       &aKnots)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::NURBCurveType)
        {
        return false;
        }
    SGMInternal::NURBcurve const *pNURBCurve=(SGMInternal::NURBcurve const *)pCurve;
    aControlPoints=pNURBCurve->m_aControlPoints;
    aKnots        =pNURBCurve->m_aKnots;
    return true;
    }

bool SGM::GetPointCurveData(SGM::Result      &rResult,
                            SGM::Curve const &CurveID,
                            SGM::Point3D     &Pos)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::PointCurveType)
        {
        return false;
        }
    SGMInternal::PointCurve const *pPointCurve=(SGMInternal::PointCurve const *)pCurve;
    Pos=pPointCurve->m_Pos;
    return true;
    }
          
bool SGM::GetPlaneData(SGM::Result        &rResult,
                       SGM::Surface const &SurfaceID,
                       SGM::Point3D       &Origin,
                       SGM::UnitVector3D  &XAxis,
                       SGM::UnitVector3D  &YAxis,
                       SGM::UnitVector3D  &ZAxis,
                       double             &dScale)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::PlaneType)
        {
        return false;
        }
    SGMInternal::plane const *pPlane=(SGMInternal::plane const *)pSurface;
    Origin=pPlane->m_Origin;
    XAxis =pPlane->m_XAxis;
    YAxis =pPlane->m_YAxis;
    ZAxis =pPlane->m_ZAxis;
    dScale=pPlane->m_dScale;
    return true;
    }
  
bool SGM::GetCylinderData(SGM::Result        &rResult,
                          SGM::Surface const &SurfaceID,
                          SGM::Point3D       &Origin,
                          SGM::UnitVector3D  &XAxis,
                          SGM::UnitVector3D  &YAxis,
                          SGM::UnitVector3D  &ZAxis,
                          double             &dRadius)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::CylinderType)
        {
        return false;
        }
    SGMInternal::cylinder const *pCylinder=(SGMInternal::cylinder const *)pSurface;
    Origin =pCylinder->m_Origin;
    XAxis  =pCylinder->m_XAxis;
    YAxis  =pCylinder->m_YAxis;
    ZAxis  =pCylinder->m_ZAxis;
    dRadius=pCylinder->m_dRadius;
    return true;
    }
 
bool SGM::GetConeData(SGM::Result        &rResult,
                      SGM::Surface const &SurfaceID,
                      SGM::Point3D       &Origin,
                      SGM::UnitVector3D  &XAxis,
                      SGM::UnitVector3D  &YAxis,
                      SGM::UnitVector3D  &ZAxis,  
                      double             &dHalfAngle,
                      double             &dRadius)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::ConeType)
        {
        return false;
        }
    SGMInternal::cone const *pCone=(SGMInternal::cone const *)pSurface;
    Origin    =pCone->m_Origin;
    XAxis     =pCone->m_XAxis; 
    YAxis     =pCone->m_YAxis; 
    ZAxis     =pCone->m_ZAxis; 
    dHalfAngle=SAFEacos(pCone->m_dCosHalfAngle);
    dRadius   =pCone->m_dRadius;
    return true;
    }
 
bool SGM::GetSphereData(SGM::Result        &rResult,
                        SGM::Surface const &SurfaceID,
                        SGM::Point3D       &Center,
                        SGM::UnitVector3D  &XAxis,
                        SGM::UnitVector3D  &YAxis,
                        SGM::UnitVector3D  &ZAxis,
                        double             &dRadius)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::SphereType)
        {
        return false;
        }
    SGMInternal::sphere const *pSphere=(SGMInternal::sphere const *)pSurface;
    Center =pSphere->m_Center;
    XAxis  =pSphere->m_XAxis; 
    YAxis  =pSphere->m_YAxis; 
    ZAxis  =pSphere->m_ZAxis; 
    dRadius=pSphere->m_dRadius;
    return true;
    }

bool SGM::GetRevolveData(SGM::Result       &rResult,
                         SGM::Surface      &SurfaceID,
                         SGM::Point3D      &Origin,
                         SGM::UnitVector3D &Axis,
                         SGM::Curve        &CurveID)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::RevolveType)
        {
        return false;
        }
    SGMInternal::revolve const *pRevolve=(SGMInternal::revolve const *)pSurface;

    Origin=pRevolve->m_Origin;
    Axis=pRevolve->m_ZAxis;
    CurveID=SGM::Curve(pRevolve->m_pCurve->GetID());
    return true;
    }
 
bool SGM::GetTorusData(SGM::Result        &rResult,
                       SGM::Surface const &SurfaceID,
                       SGM::Point3D       &Center,
                       SGM::UnitVector3D  &XAxis,
                       SGM::UnitVector3D  &YAxis,
                       SGM::UnitVector3D  &ZAxis,
                       double             &dMinorRadius,
                       double             &dMajorRadius,
                       SGM::TorusKindType &nKind)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::TorusType)
        {
        return false;
        }
    SGMInternal::torus const *pTorus=(SGMInternal::torus const *)pSurface;

    Center=pTorus->m_Center;
    XAxis=pTorus->m_XAxis;
    YAxis=pTorus->m_YAxis;
    ZAxis=pTorus->m_ZAxis;
    dMinorRadius=pTorus->m_dMinorRadius;
    dMajorRadius=pTorus->m_dMajorRadius;
    nKind=pTorus->m_nKind;

    return true;
    }
 
bool SGM::GetNUBSurfaceData(SGM::Result                             &rResult,
                            SGM::Surface                      const &SurfaceID,
                            std::vector<std::vector<SGM::Point3D> > &aaControlPoints,
                            std::vector<double>                     &aUKnots,
                            std::vector<double>                     &aVKnots)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::NUBSurfaceType)
        {
        return false;
        }
    SGMInternal::NUBsurface const *pNUBSurface=(SGMInternal::NUBsurface const *)pSurface;
    aaControlPoints=pNUBSurface->m_aaControlPoints;
    aUKnots=pNUBSurface->m_aUKnots;
    aVKnots=pNUBSurface->m_aVKnots;
    return true;
    }
  
bool SGM::GetNURBSurfaceData(SGM::Result                             &rResult,
                             SGM::Surface                      const &SurfaceID,
                             std::vector<std::vector<SGM::Point4D> > &aaControlPoints,
                             std::vector<double>                     &aUKnots,
                             std::vector<double>                     &aVKnots)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::NURBSurfaceType)
        {
        return false;
        }
    SGMInternal::NURBsurface const *pNURBSurface=(SGMInternal::NURBsurface const *)pSurface;
    aaControlPoints=pNURBSurface->m_aaControlPoints;
    aUKnots=pNURBSurface->m_aUKnots;
    aVKnots=pNURBSurface->m_aVKnots;
    return true;
    }

void SGM::SaveSTEP(SGM::Result                  &rResult,
                   std::string            const &FileName,
                   SGM::Entity            const &EntityID,
                   SGM::TranslatorOptions const &Options)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::SaveSTEP(rResult,FileName,pThing->FindEntity(EntityID.m_ID),Options); 
    }


void SGM::SaveSTL(SGM::Result                  &rResult,
                  std::string            const &sFileName,
                  SGM::Entity            const &EntityID,
                  SGM::TranslatorOptions const &Options)
    {
    SGMInternal::SaveSTL(rResult,sFileName,rResult.GetThing()->FindEntity(EntityID.m_ID),Options);
    }

SGM::Body SGM::CoverPlanarWire(SGM::Result &rResult,
                               SGM::Body   &PlanarWireID)
    {
    // Create the new body

    SGMInternal::thing  *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(PlanarWireID.m_ID);
    SGMInternal::body   *pBody=new SGMInternal::body(rResult); 
    SGMInternal::volume *pVolume=new SGMInternal::volume(rResult);
    SGMInternal::face   *pFace=new SGMInternal::face(rResult);
    pVolume->AddFace(pFace);
    pBody->AddVolume(pVolume);

    // Copy the bounding edges

    std::set<SGMInternal::edge *,SGMInternal::EntityCompare> sEdges;
    std::set<SGMInternal::vertex *,SGMInternal::EntityCompare> sVertices;
    std::map<SGMInternal::vertex const *,SGMInternal::vertex *> mVertices;
    FindEdges(rResult,pEntity,sEdges);
    FindVertices(rResult,pEntity,sVertices);
    std::set<SGMInternal::vertex *,SGMInternal::EntityCompare>::iterator VertexIter=sVertices.begin();
    while(VertexIter!=sVertices.end())
        {
        SGMInternal::vertex *pVertex=*VertexIter;
        mVertices[pVertex]=new SGMInternal::vertex(rResult,pVertex);
        ++VertexIter;
        }
    std::set<SGMInternal::edge *,SGMInternal::EntityCompare>::iterator EdgeIter=sEdges.begin();
    while(EdgeIter!=EdgeIter)  //TODO: always false. fix? (kdcopps)
        {
        SGMInternal::edge *pEdge=*EdgeIter;
        SGMInternal::curve const *pCurve=pEdge->GetCurve();
        SGMInternal::vertex const *pStart=pEdge->GetStart();
        SGMInternal::vertex const *pEnd=pEdge->GetEnd();
        SGMInternal::vertex *pNewStart=mVertices[pStart];
        SGMInternal::vertex *pNewEnd=mVertices[pEnd];
        SGMInternal::curve *pNewCurve=pCurve->MakeCopy(rResult);
        SGM::Interval1D const &Domain=pEdge->GetDomain();
        SGMInternal::edge *pNewEdge=new SGMInternal::edge(rResult);
        pNewEdge->SetStart(pNewStart);
        pNewEdge->SetEnd(pNewEnd);
        pNewEdge->SetCurve(pNewCurve);
        pNewEdge->SetDomain(rResult,Domain);
        pFace->AddEdge(pNewEdge,SGM::FaceOnLeftType);
        }

    // Create the plane.

    return SGM::Body(pBody->GetID());
    }


void SGM::FindClosestPointOnEntity(SGM::Result        &rResult,
                                   SGM::Point3D const &Point,
                                   SGM::Entity  const &EntityID,
                                   SGM::Point3D       &ClosestPoint,
                                   SGM::Entity        &ClosestEntity,
                                   bool                bBoundary)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    if (nullptr == pEntity)
        {
        rResult.SetResult(ResultType::ResultTypeUnknownEntityID);
        rResult.SetMessage("Given EntityID does not exist.");
        }
    else
        {
        SGMInternal::entity *pCloseEntity;
        SGMInternal::FindClosestPointOnEntity(rResult, Point, pEntity, ClosestPoint, pCloseEntity, bBoundary);
        ClosestEntity = Entity(pCloseEntity->GetID());
        }
    }

size_t SGM::FindCloseEdges(SGM::Result            &rResult,
                           SGM::Point3D     const &Point,
                           SGM::Entity      const &EntityID,
                           double                  dMaxDistance,
                           std::vector<SGM::Edge> &aEdges)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::edge *,SGMInternal::EntityCompare> sEdges;
    SGMInternal::FindEdges(rResult,pEntity,sEdges);
    double dTol=dMaxDistance*dMaxDistance;
    std::set<SGMInternal::edge *,SGMInternal::EntityCompare>::iterator iter=sEdges.begin();
    while(iter!=sEdges.end())
        {
        SGM::Point3D ClosestPoint;
        SGMInternal::entity *pCloseEntity;
        SGMInternal::edge *pEdge=*iter;
        SGMInternal::FindClosestPointOnEdge(rResult,Point,pEdge,ClosestPoint,pCloseEntity);
        if(Point.DistanceSquared(ClosestPoint)<dTol)
            {
            aEdges.push_back(SGM::Edge(pEdge->GetID()));
            }
        ++iter;
        }
    return aEdges.size();
    }

bool SGM::PointInEntity(SGM::Result        &rResult,
                        SGM::Point3D const &Point,
                        SGM::Entity  const &EntityID,
                        double              dTolerance)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity const *pEntity=pThing->FindEntity(EntityID.m_ID);
    return SGMInternal::PointInEntity(rResult,Point,pEntity,dTolerance);
    }

size_t SGM::FindCloseFaces(SGM::Result            &rResult,
                           SGM::Point3D     const &Point,
                           SGM::Entity      const &EntityID,
                           double                  dMaxDistance,
                           std::vector<SGM::Face> &aFaces)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::face *,SGMInternal::EntityCompare> sFaces;
    FindFaces(rResult,pEntity,sFaces);
    double dTol=dMaxDistance*dMaxDistance;
    std::set<SGMInternal::face *,SGMInternal::EntityCompare>::iterator iter=sFaces.begin();
    while(iter!=sFaces.end())
        {
        SGM::Point3D ClosestPoint;
        SGMInternal::entity *pCloseEntity;
        SGMInternal::face *pFace=*iter;
        SGMInternal::FindClosestPointOnFace(rResult,Point,pFace,ClosestPoint,pCloseEntity);
        if(Point.DistanceSquared(ClosestPoint)<dTol)
            {
            aFaces.push_back(SGM::Face(pFace->GetID()));
            }
        ++iter;
        }
    return aFaces.size();
    }

size_t SGM::ReadFile(SGM::Result                  &rResult,
                     std::string            const &FileName,
                     std::vector<SGM::Entity>     &aEntities,
                     std::vector<std::string>     &aLog,
                     SGM::TranslatorOptions const &Options)
    {
    // Find the file type.

    SGMInternal::thing *pThing=rResult.GetThing();
    std::string Extension;
    SGMInternal::FindFileExtension(FileName,Extension);
    std::vector<SGMInternal::entity *> aEnts;

    if(Extension=="stp" || Extension=="step")
        {
        SGMInternal::ReadStepFile(rResult,FileName,pThing,aEnts,aLog,Options);
        }
    else if(Extension=="stl")
        {
        SGMInternal::ReadSTLFile(rResult,FileName,pThing,aEnts,aLog,Options);
        }
    else
        {
        rResult.SetResult(ResultType::ResultTypeUnknownFileType);
        return 0;
        }

    size_t Index1;
    size_t nEnts=aEnts.size();
    aEntities.reserve(nEnts);
    for(Index1=0;Index1<nEnts;++Index1)
        {
        aEntities.emplace_back(aEnts[Index1]->GetID());
        }
    return nEnts;
    }

void SGM::ScanDirectory(SGM::Result       &rResult,
                        std::string const &sDirName,
                        std::string const &sOutputName)
    {
    std::vector<std::string> aFileNames;
    SGMInternal::ReadDirectory(sDirName,aFileNames);
    size_t nFileNames=aFileNames.size();
    SGM::TranslatorOptions Options;
    Options.m_bScan=true;
    std::vector<std::string> aLog;
    std::vector<SGM::Entity> aEnts;
    size_t Index1;
    for(Index1=0;Index1<nFileNames;++Index1)
        {
        std::string sExt;
        SGMInternal::FindFileExtension(aFileNames[Index1],sExt);
        if(sExt=="STEP" || sExt=="step" || sExt=="stp" || sExt=="STP")
            {
            std::string sFullName=sDirName+"/"+aFileNames[Index1];
            SGM::ReadFile(rResult,sFullName,aEnts,aLog,Options);
            std::sort(aLog.begin(),aLog.end());
            aLog.erase(unique( aLog.begin(),aLog.end() ),aLog.end());
            }
        }
    FILE *pFile=fopen(sOutputName.c_str(),"wt");
    size_t nLog=aLog.size();
    for(Index1=0;Index1<nLog;++Index1)
        {
        fprintf(pFile,"%s\n",aLog[Index1].c_str());
        }
    fclose(pFile);
    }

void SGM::SaveSGM(SGM::Result                  &rResult,
                  std::string            const &sFileName,
                  SGM::Entity            const &EntityID,
                  SGM::TranslatorOptions const &Options)
    {
    SGMInternal::SaveSGM(rResult,sFileName,rResult.GetThing()->FindEntity(EntityID.m_ID),Options);
    }

double SGM::FindLength(SGM::Result     &rResult,
                       SGM::Edge const &EdgeID,
                       double           dTolerance)
    {
    SGMInternal::edge const *pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->FindLength(dTolerance);
    }

double SGM::FindArea(SGM::Result     &rResult,
                     SGM::Face const &FaceID)
    {
    SGMInternal::face const *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->FindArea(rResult);
    }

double SGM::FindVolume(SGM::Result       &rResult,
                       SGM::Entity const &EntityID,
                       bool               bApproximate)
    {
    double dAnswer=0;
    SGMInternal::entity const *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    if(pEntity->GetType()==SGM::EntityType::BodyType)
        {
        dAnswer=((SGMInternal::body *)pEntity)->FindVolume(rResult,bApproximate);
        }
    else if(pEntity->GetType()==SGM::EntityType::VolumeType)
        {
        dAnswer=((SGMInternal::volume *)pEntity)->FindVolume(rResult,bApproximate);
        }
    return dAnswer;
    }

void SGM::ImprintVerticesOnClosedEdges(SGM::Result &rResult)
    {
      SGMInternal::ImprintVerticesOnClosedEdges(rResult);
    }
