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
#include "SGMAttribute.h"
#include "SGMModify.h"

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
#include "Modify.h"
#include "FacetToBRep.h"

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
                    double                              dTolerance,
                    bool                                bUseWholeLine)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);  
    return SGMInternal::RayFire(rResult,Origin,Axis,pEntity,aPoints,aTypes,dTolerance,bUseWholeLine);
    }

size_t SGM::IntersectSegment(SGM::Result               &rResult,
                             SGM::Segment3D      const &Segment,
                             SGM::Entity         const &EntityID,
                             std::vector<SGM::Point3D> &aPoints,
                             double                     dTolerance)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);  
    return SGMInternal::IntersectSegment(rResult,Segment,pEntity,aPoints,dTolerance);
    }

SGM::Complex SGM::CreateTriangles(SGM::Result                     &rResult,
                                  std::vector<SGM::Point3D> const &aPoints,
                                  std::vector<unsigned int> const &aTriangles)
    {
    if(aPoints.empty() || aTriangles.empty())
        {
        rResult.SetResult(SGM::ResultTypeInsufficientData);
        return {0};
        }
    SGMInternal::complex *pComplex=new SGMInternal::complex(rResult,aPoints,aTriangles);
    return SGM::Complex(pComplex->GetID());
    }

size_t SGM::FindComponents(SGM::Result               &rResult,
                           SGM::Complex const        &ComplexID,
                           std::vector<SGM::Complex> &aComponents)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    std::vector<SGMInternal::complex *> aComps=pComplex->FindComponents(rResult);
    size_t nComps=aComps.size();
    for(auto pComp : aComps)
        {
        aComponents.push_back(SGM::Complex(pComp->GetID()));
        }
    return nComps;
    }

size_t SGM::FindPlanarParts(SGM::Result               &rResult,
                            SGM::Complex const        &ComplexID,
                            std::vector<SGM::Complex> &aPlanarParts,
                            double                     dTolerance)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    std::vector<SGMInternal::complex *> aParts=pComplex->SplitByPlanes(rResult,dTolerance);
    size_t nParts=aParts.size();
    for(auto pPart : aParts)
        {
        aPlanarParts.push_back(SGM::Complex(pPart->GetID()));
        }
    return nParts;
    }

bool SGM::IsLinear(SGM::Result        &rResult,
                   SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    unsigned int nStart,nEnd;
    return pComplex->IsLinear(nStart,nEnd);
    }

bool SGM::IsCycle(SGM::Result        &rResult,
                  SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsCycle();
    }

bool SGM::IsConnected(SGM::Result        &rResult,
                      SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsConnected();
    }

bool SGM::IsPlanar(SGM::Result        &rResult,
                   SGM::Complex const &ComplexID,
                   SGM::Point3D       &Origin,
                   SGM::UnitVector3D  &Normal,
                   double              dTolerance)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsPlanar(Origin,Normal,dTolerance);
    }

bool SGM::IsOriented(SGM::Result        &rResult,
                     SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsOriented();
    }

bool SGM::IsManifold(SGM::Result        &rResult,
                     SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsManifold();
    }

SGM::Complex SGM::FindBoundary(SGM::Result        &rResult,
                               SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    SGMInternal::complex *pBoundary=pComplex->FindBoundary(rResult);
    return SGM::Complex(pBoundary->GetID());
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

double SGM::FindComplexLength(SGM::Result        &rResult,
                              SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->FindLength();
    }

double SGM::FindComplexArea(SGM::Result        &rResult,
                            SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->Area();
    }

double SGM::FindAverageEdgeLength(SGM::Result        &rResult,
                                  SGM::Complex const &ComplexID,
                                  double             *dMaxLength)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->FindAverageEdgeLength(dMaxLength);
    }

SGM::Complex SGM::CreateComplex(SGM::Result       &rResult,
                                SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    return SGM::Complex(SGMInternal::CreateComplex(rResult,pEntity)->GetID());
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
        std::vector<unsigned int> aTriangles;
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
        std::vector<unsigned int> aSegments;
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

    return {pComplex->GetID()};
    }

SGM::Complex SGM::CreateSlice(SGM::Result   &,//rResult,
                    SGM::Complex      const &,//ComplexID,
                    SGM::Point3D      const &,//Point,
                    SGM::UnitVector3D const &,//Normal,
                    bool                     )//bLocal)
    {
    return {0};
    }

SGM::Complex SGM::CreatePolygon(SGM::Result                &,//rResult,
                                std::vector<Point3D> const &,//aPoints,
                                bool                        )//bFilled)
    {
    return {0};
    }

std::vector<unsigned int> const &SGM::GetFaceTriangles(SGM::Result     &rResult,
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

bool SGM::GetColor(SGM::Result       &rResult,
                   SGM::Entity const &EntityID,
                   int               &nRed,
                   int               &nGreen,
                   int               &nBlue)
    {
    SGMInternal::entity *pEntity=(SGMInternal::face *)rResult.GetThing()->FindEntity(EntityID.m_ID);    
    return pEntity->GetColor(nRed,nGreen,nBlue);
    }

void SGM::ChangeColor(SGM::Result       &rResult,
                      SGM::Entity const &EntityID,
                      int                nRed,
                      int                nGreen,
                      int                nBlue)
    {
    SGMInternal::entity *pEntity=(SGMInternal::face *)rResult.GetThing()->FindEntity(EntityID.m_ID);  
    if(pEntity)
        return pEntity->ChangeColor(rResult,nRed,nGreen,nBlue);
    }

void SGM::RemoveColor(SGM::Result       &rResult,
                      SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=(SGMInternal::face *)rResult.GetThing()->FindEntity(EntityID.m_ID);    
    if(pEntity)
        return pEntity->RemoveColor(rResult);
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

std::vector<SGM::Point3D> const &SGM::GetComplexPoints(SGM::Result        &rResult,
                                                       SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);    
    return pComplex->GetPoints();
    }

std::vector<unsigned int> const &SGM::GetComplexSegments(SGM::Result        &rResult,
                                                         SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);    
    return pComplex->GetSegments();
    }

std::vector<unsigned int> const &SGM::GetComplexTriangles(SGM::Result        &rResult,
                                                          SGM::Complex const &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);    
    return pComplex->GetTriangles();
    }

SGM::Interval3D const &SGM::GetBoundingBox(SGM::Result       &rResult,
                                           SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    return pEntity->GetBox(rResult);
    }

void SGM::DeleteEntity(SGM::Result &rResult,
                       SGM::Entity &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    if(pEntity)
        SGMInternal::DeleteEntity(rResult,pEntity);
    }

SGM::EntityType SGM::GetType(SGM::Result       &rResult,
                             SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    return pEntity->GetType();
    }

size_t SGM::GetAttributes(SGM::Result         &rResult,
                          SGM::Entity   const &EntityID,
                          std::set<Attribute> &sAttributes)
    {
    SGMInternal::entity const *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::attribute *,SGMInternal::EntityCompare> const &sTempAttributes=pEntity->GetAttributes();
    auto iter=sTempAttributes.begin();
    while(iter!=sTempAttributes.end())
        {
        sAttributes.insert(SGM::Attribute((*iter)->GetID()));
        ++iter;
        }
    return sAttributes.size();
    }

size_t SGM::GetOwners(SGM::Result       &rResult,
                      SGM::Entity const &EntityID,
                      std::set<Entity>  &sOwners)
    {
    SGMInternal::entity const *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::entity *,SGMInternal::EntityCompare> const &sTempOwners=pEntity->GetOwners();
    auto iter=sTempOwners.begin();
    while(iter!=sTempOwners.end())
        {
        sOwners.insert(SGM::Entity((*iter)->GetID()));
        ++iter;
        }
    return sOwners.size();
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
        aCurves.emplace_back(acurves[Index1]->GetID());
        }
    return nAnswer;
    }

SGM::Complex SGM::CreatePoints(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aPoints)
    {
    SGMInternal::complex *pComplex=new SGMInternal::complex(rResult,aPoints);
    return {pComplex->GetID()};
    }

SGM::Complex SGM::CreateSegments(SGM::Result                    &rResult,
                                 std::vector<SGM::Point3D> const &aPoints,
                                 std::vector<unsigned int> const &aSegments)
    {
    SGMInternal::complex *pComplex=new SGMInternal::complex(rResult,aSegments,aPoints);
    return {pComplex->GetID()};
    }

SGM::Complex SGM::CoverComplex(SGM::Result        &rResult,
                               SGM::Complex const &ComplexID)
    {
    SGMInternal::complex const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    SGMInternal::complex *pAnswer=pComplex->Cover(rResult);
    return SGM::Complex(pAnswer->GetID());
    }

SGM::Complex SGM::MergePoints(SGM::Result        &rResult,
                              SGM::Complex const &ComplexID,
                              double              dTolerance)
    {
    SGMInternal::complex const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    SGMInternal::complex *pAnswer=pComplex->Merge(rResult,dTolerance);
    return SGM::Complex(pAnswer->GetID());
    }

SGM::Complex SGM::FindSharpEdges(SGM::Result        &rResult,
                                 SGM::Complex const &ComplexID,
                                 double              dAngle,
                                 bool                bIncludeBoundary)
    {
    SGMInternal::complex const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    SGMInternal::complex *pAnswer=pComplex->FindSharpEdges(rResult,dAngle,bIncludeBoundary);
    return SGM::Complex(pAnswer->GetID());
    }

SGM::Complex SGM::FindDegenerateTriangles(SGM::Result        &rResult,
                                          SGM::Complex const &ComplexID)
    {
    SGMInternal::complex const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    SGMInternal::complex *pAnswer=pComplex->FindDegenerateTriangles(rResult);
    return SGM::Complex(pAnswer->GetID());
    }

std::vector<double> SGM::FindTriangleAreas(SGM::Result        &rResult,
                                           SGM::Complex const &ComplexID)
    {
    SGMInternal::complex const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    return pComplex->FindTriangleAreas();
    }

size_t SGM::FindHoles(SGM::Result               &rResult,
                      SGM::Complex        const &ComplexID,
                      std::vector<SGM::Complex> &aHoles)
    {
    SGMInternal::complex const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    std::vector<SGMInternal::complex *> aInternalHoles;
    pComplex->FindHoles(rResult,aInternalHoles);
    for(auto pHole : aInternalHoles)
        {
        aHoles.push_back(SGM::Complex(pHole->GetID()));
        }
    return aHoles.size();
    }

SGM::Complex SGM::MergeComplexes(SGM::Result                     &rResult,
                                 std::vector<SGM::Complex> const &aComplexIDs)
    {
    std::vector<SGMInternal::complex *> aComplexes;
    size_t nComplexes=aComplexIDs.size();
    aComplexes.reserve(nComplexes);
    size_t Index1;
    for(Index1=0;Index1<nComplexes;++Index1)
        {
        aComplexes.push_back((SGMInternal::complex *)(rResult.GetThing()->FindEntity(aComplexIDs[Index1].m_ID)));
        }
    SGMInternal::complex *pComplex=aComplexes.back();
    aComplexes.pop_back();
    SGMInternal::complex *pAnswer=pComplex->Merge(rResult,aComplexes);
    return SGM::Complex(pAnswer->GetID());
    }

void SGM::SplitComplexAtPoints(SGM::Result                     &rResult,
                               SGM::Complex                    &ComplexID,
                               std::vector<SGM::Point3D> const &aPoints,
                               double                           dTolerance)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    pComplex->SplitAtPoints(rResult,aPoints,dTolerance);
    }

void SGM::ReduceToUsedPoints(SGM::Result  &rResult,
                             SGM::Complex &ComplexID)
    {
    SGMInternal::complex *pComplex=(SGMInternal::complex *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    pComplex->ReduceToUsedPoints();
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

SGM::Body SGM::FindBody(SGM::Result       &rResult,
                        SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::body *,SGMInternal::EntityCompare> sBodies;
    FindBodies(rResult,pEntity,sBodies,false);
    auto iter=sBodies.begin();
    return SGM::Body((*iter)->GetID());
    }

SGM::Volume SGM::FindVolume(SGM::Result       &rResult,
                            SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::volume *,SGMInternal::EntityCompare> sVolume;
    FindVolumes(rResult,pEntity,sVolume,false);
    auto iter=sVolume.begin();
    return SGM::Volume((*iter)->GetID());
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

SGM::Attribute SGM::CreateIntegerAttribute(SGM::Result            &rResult,
                                           std::string      const &Name,
                                           std::vector<int> const &aData)
    {
    SGMInternal::attribute *pAttribute=new SGMInternal::IntegerAttribute(rResult,Name,aData);
    return {pAttribute->GetID()};
    }

SGM::Attribute SGM::CreateDoubleAttribute(SGM::Result               &rResult,
                                          std::string         const &Name,
                                          std::vector<double> const &aData)
    {
    SGMInternal::attribute *pAttribute=new SGMInternal::DoubleAttribute(rResult,Name,aData);
    return {pAttribute->GetID()};
    }

SGM::Attribute SGM::CreateCharAttribute(SGM::Result             &rResult,
                                        std::string       const &Name,
                                        std::vector<char> const &aData)
    {
    SGMInternal::attribute *pAttribute=new SGMInternal::CharAttribute(rResult,Name,aData);
    return {pAttribute->GetID()};
    }

SGM::Attribute SGM::CreateStringAttribute(SGM::Result       &rResult,
                                          std::string const &Name,
                                          std::string const &Data)
    {
    SGMInternal::attribute *pAttribute=new SGMInternal::StringAttribute(rResult,Name,Data);
    return {pAttribute->GetID()};
    }

SGM::Attribute SGM::CreateAttribute(SGM::Result       &rResult,
                                    std::string const &Name)
    {
    SGMInternal::attribute *pAttribute=new SGMInternal::attribute(rResult,Name);
    return {pAttribute->GetID()};
    }


void SGM::FindAttributes(SGM::Result              &rResult,
                         SGM::Entity        const &EntityID,
                         std::set<SGM::Attribute> &sAttributes,
                         bool                      bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::attribute *,SGMInternal::EntityCompare> sAttribute;
    FindAttributes(rResult,pEntity,sAttribute,bTopLevel);
    std::set<SGMInternal::attribute *>::iterator iter=sAttribute.begin();
    while(iter!=sAttribute.end())
        {
        sAttributes.insert(SGM::Attribute((*iter)->GetID()));
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

void SGM::FindWireEdges(SGM::Result         &rResult,
                        SGM::Entity   const &EntityID,
                        std::set<SGM::Edge> &sOutEdges)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::edge *,SGMInternal::EntityCompare> sEdges;
    FindWireEdges(rResult,pEntity,sEdges);
    for(auto pEdge : sEdges)
        {
        sOutEdges.insert(SGM::Edge(pEdge->GetID()));
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

bool SGM::IsSheetBody(SGM::Result     &rResult,
                      SGM::Body const &BodyID)
    {
    SGMInternal::body const *pBody=(SGMInternal::body const *)rResult.GetThing()->FindEntity(BodyID.m_ID);
    return pBody->IsSheetBody(rResult);
    }

bool SGM::IsWireBody(SGM::Result     &rResult,
                      SGM::Body const &BodyID)
    {
    SGMInternal::body const *pBody=(SGMInternal::body const *)rResult.GetThing()->FindEntity(BodyID.m_ID);
    return pBody->IsWireBody(rResult);
    }

SGM::Surface SGM::GetSurfaceOfFace(SGM::Result     &rResult,
                                   SGM::Face const &FaceID)
    {
    SGMInternal::face const *pFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetSurface()->GetID();
    }

SGM::Curve SGM::GetCurveOfEdge(SGM::Result     &rResult,
                               SGM::Edge const &EdgeID)
    {
    SGMInternal::edge const *pEdge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->GetCurve()->GetID();
    }

int SGM::GetSidesOfFace(SGM::Result     &rResult,
                        SGM::Face const &FaceID)
    {
    SGMInternal::face const *pFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetSides();
    }

bool SGM::IsFaceFlipped(SGM::Result     &rResult,
                        SGM::Face const &FaceID)
    {
    SGMInternal::face const *pFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetFlipped();
    }

double SGM::GetToleranceOfEdge(SGM::Result     &rResult,
                                SGM::Edge const &EdgeID)
    {
    SGMInternal::edge const *pEdge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->GetTolerance();
    }

SGM::Interval1D const &SGM::GetDomainOfEdge(SGM::Result     &rResult,
                                            SGM::Edge const &EdgeID)
    {
    SGMInternal::edge const *pEdge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->GetDomain();
    }

SGM::Point3D const &SGM::GetPointOfVertex(SGM::Result       &rResult,
                                          SGM::Vertex const &VertexID)
    {
    SGMInternal::vertex const *pVertex=(SGMInternal::vertex const *)rResult.GetThing()->FindEntity(VertexID.m_ID);
    return pVertex->GetPoint();
    }

std::vector<SGM::Point3D> const &SGM::GetPointsOfBody(SGM::Result     &rResult,
                                                      SGM::Body const &BodyID)
    {
    SGMInternal::body const *pBody=(SGMInternal::body const *)rResult.GetThing()->FindEntity(BodyID.m_ID);
    return pBody->GetPoints();
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
    return {pBody->GetID()};
    }

SGM::Body SGM::CreateSphere(SGM::Result        &rResult,
                            SGM::Point3D const &Center,
                            double              dRadius)
    {
    SGMInternal::body *pBody=SGMInternal::CreateSphere(rResult,Center,dRadius);
    return {pBody->GetID()};
    }

SGM::Body SGM::CreateCylinder(SGM::Result        &rResult,
                              SGM::Point3D const &BottomCenter,
                              SGM::Point3D const &TopCenter,
                              double              dRadius)
    {
    SGMInternal::body *pBody=SGMInternal::CreateCylinder(rResult,BottomCenter,TopCenter,dRadius);
    return {pBody->GetID()};
    }

SGM::Body SGM::CreateCone(SGM::Result        &rResult,
                          SGM::Point3D const &BottomCenter,
                          SGM::Point3D const &TopCenter,
                          double              dBottomRadius,
                          double              dTopRadius)
    {
    SGMInternal::body *pBody=SGMInternal::CreateCone(rResult,BottomCenter,TopCenter,dBottomRadius,dTopRadius);
    return {pBody->GetID()};
    }

SGM::Body SGM::CreateTorus(SGM::Result             &rResult,
                           SGM::Point3D      const &Center,
                           SGM::UnitVector3D const &Axis,
                           double                   dMinorRadius,
                           double                   dMajorRadius,
                           bool                     bApple)
    {
    SGMInternal::body *pBody=SGMInternal::CreateTorus(rResult,Center,Axis,dMinorRadius,dMajorRadius,bApple);
    return {pBody->GetID()};
    }

SGM::Body SGM::CreateRevolve(SGM::Result             &rResult,
                             SGM::Point3D      const &Origin,
                             SGM::UnitVector3D const &Axis,
                             SGM::Curve              &IDCurve)
    {
    SGMInternal::curve *pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(IDCurve.m_ID);
    SGMInternal::body *pBody=SGMInternal::CreateRevolve(rResult, Origin, Axis, pCurve);

    return {pBody->GetID()};
    }

SGM::Complex SGM::CreateComplex(SGM::Result                     &rResult,
                                std::vector<SGM::Point3D> const &aPoints,
                                std::vector<unsigned int> const &aSegments,
                                std::vector<unsigned int> const &aTriangles)
    {
    SGMInternal::complex *pComplex=SGMInternal::CreateComplex(rResult,aPoints,aSegments,aTriangles);
    return {pComplex->GetID()};
    }

SGM::Body SGM::CreateDisk(SGM::Result             &rResult,
                          SGM::Point3D      const &Center,
                          SGM::UnitVector3D const &Normal,
                          double                   dRadius)
    {
    SGMInternal::body *pBody=SGMInternal::CreateDisk(rResult,Center,Normal,dRadius);
    return {pBody->GetID()};
    }

SGM::Surface SGM::CreateRevolveSurface(SGM::Result             &rResult,
                                       SGM::Point3D      const &Origin,
                                       SGM::UnitVector3D const &Axis,
                                       SGM::Curve              &IDCurve)
    {
    SGMInternal::curve *pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(IDCurve.m_ID);
    SGMInternal::surface *pSurface=SGMInternal::CreateRevolveSurface(rResult, Origin, Axis, pCurve);

    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreateExtrudeSurface(SGM::Result             &rResult,
                                       SGM::UnitVector3D const &Axis,
                                       SGM::Curve              &CurveID)
    {
    SGMInternal::curve *pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(CurveID.m_ID);
    SGMInternal::surface *pSurface=SGMInternal::CreateExtrudeSurface(rResult, Axis, pCurve);

    return {pSurface->GetID()};
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
    return {pBody->GetID()};
    }

SGM::Curve SGM::CreateLine(SGM::Result             &rResult,
                           SGM::Point3D      const &Origin,
                           SGM::UnitVector3D const &Axis)
    {
    SGMInternal::curve *pCurve=new SGMInternal::line(rResult,Origin,Axis);
    return {pCurve->GetID()};
    }

SGM::Curve SGM::CreateCircle(SGM::Result             &rResult,
                             SGM::Point3D      const &Center,
                             SGM::UnitVector3D const &Normal,
                             double                   dRadius)
    {
    SGMInternal::curve *pCurve=new SGMInternal::circle(rResult,Center,Normal,dRadius);
    return {pCurve->GetID()};
    }

SGM::Curve SGM::CreateEllipse(SGM::Result             &rResult,
                              SGM::Point3D      const &Center,
                              SGM::UnitVector3D const &XAxis,
                              SGM::UnitVector3D const &YAxis,
                              double                   dXRadius,
                              double                   dYRadius)
    {
    SGMInternal::curve *pCurve=new SGMInternal::ellipse(rResult,Center,XAxis,YAxis,dXRadius,dYRadius);
    return {pCurve->GetID()};
    }

SGM::Curve SGM::CreateParabola(SGM::Result             &rResult,
                               SGM::Point3D      const &Center,
                               SGM::UnitVector3D const &XAxis,
                               SGM::UnitVector3D const &YAxis,
                               double                   dA)
    {
    SGMInternal::curve *pCurve=new SGMInternal::parabola(rResult,Center,XAxis,YAxis,dA);
    return {pCurve->GetID()};
    }

SGM::Curve SGM::CreateHyperbola(SGM::Result             &rResult,
                                SGM::Point3D      const &Center,
                                SGM::UnitVector3D const &XAxis,
                                SGM::UnitVector3D const &YAxis,
                                double                   dA,
                                double                   dB)
    {
    SGMInternal::curve *pCurve=new SGMInternal::hyperbola(rResult,Center,XAxis,YAxis,dA,dB);
    return {pCurve->GetID()};
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
    return {pCurve->GetID()};
    }

SGM::Edge SGM::CreateEdge(SGM::Result           &rResult,
                          SGM::Curve            &CurveID,
                          SGM::Interval1D const *pDomain)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::curve *pCurve=(SGMInternal::curve *)pThing->FindEntity(CurveID.m_ID);
    SGMInternal::edge *pEdge=SGMInternal::CreateEdge(rResult,pCurve,pDomain);
    return {pEdge->GetID()};
    }

SGM::Edge SGM::CreateLinearEdge(SGM::Result        &rResult,
                                SGM::Point3D const &StartPos,
                                SGM::Point3D const &EndPos)
    {
    SGMInternal::edge *pEdge=SGMInternal::CreateEdge(rResult,StartPos,EndPos);
    return {pEdge->GetID()};
    }

SGM::Body SGM::CreateWireBody(SGM::Result               &rResult,
                              std::set<SGM::Edge> const &sEdges)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    std::set<SGMInternal::edge *> spEdges;
    for(auto EdgeID : sEdges)
        {
        SGMInternal::edge *pEdge=(SGMInternal::edge *)pThing->FindEntity(EdgeID.m_ID);
        spEdges.insert(pEdge);
        }
    SGMInternal::body *pBody=SGMInternal::CreateWireBody(rResult,spEdges);
    return {pBody->GetID()};
    }

SGM::Body SGM::CreatePointBody(SGM::Result                  &rResult,
                               std::set<SGM::Point3D> const &sPoints)
    {
    return {SGMInternal::CreatePointBody(rResult,sPoints)->GetID()};
    }

SGM::Entity SGM::CopyEntity(SGM::Result       &rResult,
                            SGM::Entity const &EntityID)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    return {SGMInternal::CopyEntity(rResult,pEntity)->GetID()};
    }

void SGM::TransformEntity(SGM::Result            &rResult,
                          SGM::Transform3D const &Trans,
                          SGM::Entity            &EntityID)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    SGMInternal::TransformEntity(rResult,Trans,pEntity);
    }

bool SGM::CheckEntity(SGM::Result              &rResult,
                      SGM::Entity        const &EntityID,
                      SGM::CheckOptions  const &Options,
                      std::vector<std::string> &aCheckStrings)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    return pEntity->Check(rResult,Options,aCheckStrings,true);
    }

bool SGM::TestCurve(SGM::Result      &rResult,
                    SGM::Curve const &CurveID,
                    double            dT)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::curve *pCurve=(SGMInternal::curve *)pThing->FindEntity(CurveID.m_ID);
    return SGMInternal::TestCurve(rResult,pCurve,dT);
    }

bool SGM::TestSurface(SGM::Result        &rResult,
                      SGM::Surface const &SurfaceID,
                      SGM::Point2D const &uv)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::surface *pSurface=(SGMInternal::surface *)pThing->FindEntity(SurfaceID.m_ID);
    return SGMInternal::TestSurface(rResult,pSurface,uv);
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

SGM::Vector3D SGM::CurveCurvature(SGM::Result        &rResult,
                                  SGM::Curve   const &CurveID,
                                  double              t)
{
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::curve *pCurve=(SGMInternal::curve *)(pThing->FindEntity(CurveID.m_ID));
    return pCurve->Curvature(t);
}

void SGM::PrincipleCurvature(SGM::Result        &rResult,
                             SGM::Surface const &SurfaceID,
                             SGM::Point2D const &uv,
                             SGM::UnitVector3D  &Vec1,
                             SGM::UnitVector3D  &Vec2,
                             double             &dk1,
                             double             &dk2)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::surface *pSurface=(SGMInternal::surface *)(pThing->FindEntity(SurfaceID.m_ID));
    pSurface->PrincipleCurvature(uv,Vec1,Vec2,dk1,dk2);
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
    return {pCurve->GetID()};
    }

SGM::Curve SGM::CreateNUBCurveWithEndVectors(SGM::Result                     &rResult,
                                             std::vector<SGM::Point3D> const &aInterpolate,
                                             SGM::Vector3D             const &StartVec,
                                             SGM::Vector3D             const &EndVec,
                                             std::vector<double>       const *pParams)
    {
    SGMInternal::curve *pCurve=SGMInternal::CreateNUBCurveWithEndVectors(rResult,aInterpolate,StartVec,EndVec,pParams);
    return {pCurve->GetID()};
    }

SGM::Curve SGM::CreateNURBCurve(SGM::Result                     &rResult,
                                std::vector<SGM::Point4D> const &aControlPoints,
                                std::vector<double>       const &aKnots)
    {
    SGMInternal::NURBcurve *pNURB=new SGMInternal::NURBcurve(rResult, aControlPoints, aKnots);
    return {pNURB->GetID()};
    }

SGM::Curve SGM::CreateHermitCurve(SGM::Result                      &rResult,
                                  std::vector<SGM::Point3D>  const &aPoints,
                                  std::vector<SGM::Vector3D> const &aVectors,
                                  std::vector<double>        const &aParams)
    {
    SGMInternal::hermite *pHermite=new SGMInternal::hermite(rResult,aPoints,aVectors,aParams);
    return {pHermite->GetID()};
    }

SGM::Curve SGM::CreateNUBCurveWithControlPointsAndKnots(SGM::Result                     &rResult,
                                                        std::vector<SGM::Point3D> const &aControlPoints,
                                                        std::vector<double>       const &aKnots)
    {
    SGMInternal::NUBcurve *pNUB=new SGMInternal::NUBcurve(rResult, aControlPoints, aKnots);
    return {pNUB->GetID()};
    }

SGM::Surface SGM::CreateTorusSurface(SGM::Result             &rResult,
                                     SGM::Point3D      const &Center,
                                     SGM::UnitVector3D const &Axis,
                                     double                   dMinorRadius,
                                     double                   dMajorRadius,
                                     bool                     bApple,
                                     SGM::UnitVector3D const *pXAxis)
    {
    SGMInternal::surface *pSurface;
    if(pXAxis)
        {
        pSurface=new SGMInternal::torus(rResult,Center,Axis,dMinorRadius,dMajorRadius,bApple,pXAxis);
        }
    else
        {
        pSurface=new SGMInternal::torus(rResult,Center,Axis,dMinorRadius,dMajorRadius,bApple);
        }
    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreateNUBSurfaceFromControlPoints(SGM::Result                                   &rResult,
                                                    std::vector<std::vector<SGM::Point3D> > const &aaControlPoints,
                                                    std::vector<double>                     const &aUKnots,
                                                    std::vector<double>                     const &aVKnots)
    {
    SGMInternal::surface *pSurface=new SGMInternal::NUBsurface(rResult,aaControlPoints,aUKnots,aVKnots);
    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreateNURBSurface(SGM::Result                                   &rResult,
                                    std::vector<std::vector<SGM::Point4D> > const &aaControlPoints,
                                    std::vector<double>                     const &aUKnots,
                                    std::vector<double>                     const &aVKnots)
    {
    SGMInternal::surface *pSurface=new SGMInternal::NURBsurface(rResult,aaControlPoints,aUKnots,aVKnots);
    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreateSphereSurface(SGM::Result        &rResult,
                                      SGM::Point3D const &Center,
                                      double              dRadius,
                                      SGM::UnitVector3D  *pXAxis,
                                      SGM::UnitVector3D  *pYAxis)
    {
    SGMInternal::surface *pSurface=new SGMInternal::sphere(rResult,Center,dRadius,pXAxis,pYAxis);
    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreateCylinderSurface(SGM::Result        &rResult,
                                        SGM::Point3D const &Bottom,
                                        SGM::Point3D const &Top,
                                        double              dRadius)
    {
    SGMInternal::surface *pSurface=new SGMInternal::cylinder(rResult,Bottom,Top,dRadius);
    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreateConeSurface(SGM::Result             &rResult,
                                    SGM::Point3D      const &Origin,
                                    SGM::UnitVector3D const &Axis,
                                    double                   dRadius,
                                    double                   dHalfAngle)
    {
    SGMInternal::surface *pSurface=new SGMInternal::cone(rResult,Origin,Axis,dRadius,dHalfAngle);
    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreatePlane(SGM::Result        &rResult,
                              SGM::Point3D const &Origin,
                              SGM::Point3D const &XPos,
                              SGM::Point3D const &YPos)
    {
    SGMInternal::plane *pPlane = new SGMInternal::plane(rResult, Origin, XPos, YPos);
    return {pPlane->GetID()};
    }

SGM::EntityType SGM::GetCurveType(SGM::Result      &rResult,
                                  SGM::Curve const &CurveID)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve const *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    return pCurve->GetCurveType();
    }

SGM::EntityType SGM::GetAttributeType(SGM::Result          &rResult,
                                      SGM::Attribute const &AttributeID)
    {
    SGMInternal::attribute const *pAttribute=(SGMInternal::attribute const *)(rResult.GetThing()->FindEntity(AttributeID.m_ID));
    return pAttribute->GetAttributeType();
    }

std::string const &SGM::GetAttributeName(SGM::Result          &rResult,
                                         SGM::Attribute const &AttributeID)
    {
    SGMInternal::attribute const *pAttribute=(SGMInternal::attribute *)(rResult.GetThing()->FindEntity(AttributeID.m_ID));
    return pAttribute->GetName();
    }

std::vector<int> const &SGM::GetIntegerAttributeData(SGM::Result          &rResult,
                                                     SGM::Attribute const &AttributeID)
    {
    SGMInternal::IntegerAttribute const *pAttribute=(SGMInternal::IntegerAttribute const *)(rResult.GetThing()->FindEntity(AttributeID.m_ID));
    return pAttribute->GetData();
    }

SGM::Interval1D const &SGM::GetDomainOfCurve(SGM::Result      &rResult,
                                             SGM::Curve const &CurveID)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    return pCurve->GetDomain();
    }

SGM::EntityType SGM::GetSurfaceType(SGM::Result        &rResult,
                                    SGM::Surface const &SurfaceID)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    return pSurface->GetSurfaceType();
    }

SGM::Interval2D const &SGM::GetDomainOfSurface(SGM::Result        &rResult,
                                               SGM::Surface const &SurfaceID)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    return pSurface->GetDomain();
    }

bool SGM::GetLineData(SGM::Result       &rResult,
                      SGM::Curve  const &CurveID,
                      SGM::Point3D      &Origin,
                      SGM::UnitVector3D &Axis)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::LineType)
        {
        return false;
        }
    SGMInternal::line const *pLine=(SGMInternal::line const *)pCurve;
    Origin =pLine->m_Origin;
    Axis   =pLine->m_Axis;
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

bool SGM::GetHermiteCurveData(SGM::Result                &rResult,
                              SGM::Curve           const &CurveID,
                              std::vector<SGM::Point3D>  &aPoints,
                              std::vector<SGM::Vector3D> &aVectors,
                              std::vector<double>        &aParams)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::HermiteCurveType)
        {
        return false;
        }
    SGMInternal::hermite const *pHermite=(SGMInternal::hermite const *)pCurve;
    aPoints=pHermite->m_aPoints;
    aVectors=pHermite->m_aTangents;
    aParams=pHermite->m_aParams;
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
                       SGM::UnitVector3D  &ZAxis)
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
        mVertices[pVertex]=pVertex->Clone(rResult);
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
        SGMInternal::curve *pNewCurve=pCurve->Clone(rResult);
        SGM::Interval1D const &Domain=pEdge->GetDomain();
        SGMInternal::edge *pNewEdge=new SGMInternal::edge(rResult);
        pNewEdge->SetStart(pNewStart);
        pNewEdge->SetEnd(pNewEnd);
        pNewEdge->SetCurve(pNewCurve);
        pNewEdge->SetDomain(rResult,Domain);
        pFace->AddEdge(rResult,pNewEdge,SGM::FaceOnLeftType);
        }

    // Create the plane.

    return {pBody->GetID()};
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
        SGMInternal::FindClosestPointOnEdge3D(rResult,Point,pEdge,ClosestPoint,pCloseEntity);
        if(Point.DistanceSquared(ClosestPoint)<dTol)
            {
            aEdges.emplace_back(pEdge->GetID());
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

void SGM::PointsInVolumes(SGM::Result                            &rResult,
                          std::vector<SGM::Point3D>        const &aPoints,
                          std::vector<std::vector<SGM::Volume> > &aaVolumeIDs,
                          double                                  dTolerance)
    {
    std::vector<std::vector<SGMInternal::volume *> > aaVolumes;
    SGMInternal::PointsInVolumes(rResult,aPoints,aaVolumes,dTolerance);
    size_t Index1,Index2;
    size_t nPoints=aPoints.size();
    aaVolumes.reserve(nPoints);
    for(Index1=0;Index1<nPoints;++Index1)
        {
        std::vector<SGMInternal::volume *> const &aVolumes=aaVolumes[Index1];
        size_t nVolumes=aVolumes.size();
        std::vector<SGM::Volume> aVolumeIDs;
        aVolumeIDs.reserve(nVolumes);
        for(Index2=0;Index2<nVolumes;++Index2)
            {
            aVolumeIDs.push_back(aVolumes[Index2]->GetID());
            }
        aaVolumeIDs.push_back(aVolumeIDs);
        }
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
            aFaces.emplace_back(pFace->GetID());
            }
        ++iter;
        }
    return aFaces.size();
    }

SGM::FileType SGM::GetFileType(std::string const &sFileName)
    {
    std::string Extension;
    SGMInternal::FindFileExtension(sFileName,Extension);

    if(Extension=="stp" || Extension=="step")
        {
        return SGM::STEPFileType;
        }
    else if(Extension=="stl")
        {
        return SGM::STLFileType;
        }
    else if(Extension=="sgm")
        {
        return SGM::SGMFileType;
        }
    else
        {
        return SGM::UnknownFileType;
        }
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
    else if(Extension=="sgm")
        {
        SGMInternal::ReadSGMFile(rResult,FileName,aEnts,aLog,Options);
        }
    else if(Extension=="txt")
        {
        SGMInternal::ReadTXTFile(rResult,FileName,aEnts,aLog,Options);
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

double SGM::FindEdgeLength(SGM::Result     &rResult,
                           SGM::Edge const &EdgeID,
                           double           dTolerance)
    {
    SGMInternal::edge const *pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->FindLength(dTolerance);
    }

double SGM::FindCurveLength(SGM::Result           &rResult,
                            SGM::Interval1D const &Domain,
                            SGM::Curve      const &CurveID,
                            double                 dTolerance)
    {
    SGMInternal::curve const *pCurve=(SGMInternal::curve *)rResult.GetThing()->FindEntity(CurveID.m_ID);
    return pCurve->FindLength(Domain,dTolerance);
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

/*
std::vector<SGM::Face> SGM::ImprintEdgeOnFace(SGM::Result &rResult,
                                              SGM::Edge   &EdgeID,
                                              SGM::Face   &FaceID)
    {
    SGMInternal::edge *pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    std::vector<SGMInternal::face *> aFaces=SGMInternal::ImprintEdgeOnFace(rResult,pEdge,pFace);
    std::vector<SGM::Face> aAnswer;
    aAnswer.reserve(aFaces.size());
    for(auto pFace : aFaces)
        {
        aAnswer.push_back(SGM::Face(pFace->GetID()));
        }
    return aAnswer;
    }

void SGM::UniteBodies(SGM::Result &rResult,
                      SGM::Body   &ReturnedBodyID,
                      SGM::Body   &DeletedBodyID)
    {
    SGMInternal::body *pReturnedBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(ReturnedBodyID.m_ID);
    SGMInternal::body *pDeletedBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(DeletedBodyID.m_ID);
    SGMInternal::UniteBodies(rResult,pReturnedBody,pDeletedBody);
    }
*/

void SGM::ImprintVerticesOnClosedEdges(SGM::Result &rResult)
    {
    SGMInternal::ImprintVerticesOnClosedEdges(rResult);
    }

SGM::Vertex SGM::ImprintPoint(SGM::Result        &rResult,
                              SGM::Point3D const &Pos,
                              SGM::Topology      &TopologyID)
    {
    SGMInternal::topology *pTopology=(SGMInternal::topology *)rResult.GetThing()->FindEntity(TopologyID.m_ID);
    return {SGMInternal::ImprintPoint(rResult,Pos,pTopology)->GetID()};
    }

void SGM::Merge(SGM::Result &rResult,
                SGM::Entity &EntityID)
    {
    SGMInternal::entity *pEntity=(SGMInternal::entity *)rResult.GetThing()->FindEntity(EntityID.m_ID);
    SGMInternal::Merge(rResult,pEntity);
    }

/*
SGM::Body SGM::UnhookFaces(SGM::Result            &rResult,
                           std::vector<SGM::Face> &aFaceIDs)
    {
    size_t nFaceIDs=aFaceIDs.size();
    size_t Index1;
    std::vector<SGMInternal::face *> aFaces;
    aFaces.reserve(nFaceIDs);
    for(Index1=0;Index1<nFaceIDs;++Index1)
        {
        SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(aFaceIDs[Index1].m_ID);
        aFaces.push_back(pFace);
        }
    return SGM::Body(SGMInternal::UnhookFaces(rResult,aFaces)->GetID());
    }
*/

size_t SGM::IntersectCurveAndPlane(SGM::Result                        &rResult,
                                   SGM::Curve                   const &CurveID,
                                   SGM::Point3D                 const &PlaneOrigin,
                                   SGM::UnitVector3D            const &PlaneNorm,
                                   std::vector<SGM::Point3D>          &aPoints,
                                   std::vector<SGM::IntersectionType> &aTypes,
                                   double                              dTolerance)
{
    SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(CurveID.m_ID);
    SGMInternal::curve *pCurve = (SGMInternal::curve *)pEntity;
    return SGMInternal::IntersectCurveAndPlane(rResult, pCurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);
}

size_t SGM::IntersectEdgeAndPlane(SGM::Result                        &rResult,
                                  SGM::Edge                    const &EdgeID,
                                  SGM::Point3D                 const &PlaneOrigin,
                                  SGM::UnitVector3D            const &PlaneNorm,
                                  std::vector<SGM::Point3D>          &aPoints,
                                  std::vector<SGM::IntersectionType> &aTypes,
                                  double                              dTolerance)
{
    SGMInternal::entity *pEntity = rResult.GetThing()->FindEntity(EdgeID.m_ID);
    SGMInternal::edge *pEdge = (SGMInternal::edge *)pEntity;
    return SGMInternal::IntersectEdgeAndPlane(rResult, pEdge, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);
}


SGM::Curve SGM::CreatePointCurve(SGM::Result  &rResult,
                                 SGM::Point3D &Pos)
{
    SGMInternal::curve *pCurve=new SGMInternal::PointCurve(rResult,Pos);
    return {pCurve->GetID()};
}


SGM::Curve SGM::FindConic(SGM::Result                     &rResult,
                          std::vector<SGM::Point3D> const &aPoints,
                          double                           dTolerance)
    {
    SGMInternal::curve *pCurve=SGMInternal::FindConic(rResult,aPoints,dTolerance);
    return {pCurve->GetID()};
    }