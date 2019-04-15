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
                    std::vector<SGM::Entity>           &aEntities,
                    double                              dTolerance,
                    bool                                bUseWholeLine)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID); 
    std::vector<SGMInternal::entity *> aentities;
    size_t nHits=SGMInternal::RayFire(rResult,Origin,Axis,pEntity,aPoints,aTypes,aentities,dTolerance,bUseWholeLine);
    for(auto pEnt : aentities)
        {
        aEntities.push_back(SGM::Entity(pEnt ? pEnt->GetID() : 0));
        }
    return nHits;
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
                                  std::vector<unsigned int> const &aTriangles,
                                  bool                             bMerge)
    {
    if(aPoints.empty() || aTriangles.empty())
        {
        rResult.SetResult(SGM::ResultTypeInsufficientData);
        return {0};
        }
    SGMInternal::complex *pComplex;
    if (bMerge)
        {
        std::vector<unsigned> aSegments; // empty segments
        pComplex = new SGMInternal::complex(rResult, aPoints, aSegments, aTriangles, SGM_ZERO);
        }
    else
        {
        pComplex = new SGMInternal::complex(rResult, aPoints, aTriangles);
        }
    return {pComplex->GetID()};
    }

size_t SGM::FindComponents(SGM::Result               &rResult,
                           SGM::Complex const        &ComplexID,
                           std::vector<SGM::Complex> &aComponents)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    std::vector<SGMInternal::complex *> aComps=pComplex->FindComponents(rResult);
    size_t nComponents = aComps.size();
    aComponents.reserve(nComponents);
    for(auto pComp : aComps)
        {
        aComponents.emplace_back(pComp->GetID());
        }
    return nComponents;
    }

size_t SGM::FindPlanarParts(SGM::Result               &rResult,
                            SGM::Complex const        &ComplexID,
                            std::vector<SGM::Complex> &aPlanarParts,
                            double                     dTolerance)
    {
    auto *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    std::vector<SGMInternal::complex *> aParts=pComplex->SplitByPlanes(rResult,dTolerance);
    size_t nParts=aParts.size();
    aPlanarParts.reserve(nParts);
    for(auto pPart : aParts)
        {
        aPlanarParts.emplace_back(pPart->GetID());
        }
    return nParts;
    }

bool SGM::IsLinear(SGM::Result        &rResult,
                   SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    unsigned int nStart,nEnd;
    return pComplex->IsLinear(nStart,nEnd);
    }

bool SGM::IsCycle(SGM::Result        &rResult,
                  SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsCycle();
    }

bool SGM::IsConnected(SGM::Result        &rResult,
                      SGM::Complex const &ComplexID)
    {
    auto *pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsConnected();
    }

bool SGM::IsPlanar(SGM::Result        &rResult,
                   SGM::Complex const &ComplexID,
                   SGM::Point3D       &Origin,
                   SGM::UnitVector3D  &Normal,
                   double              dTolerance)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsPlanar(Origin,Normal,dTolerance);
    }

bool SGM::IsOriented(SGM::Result        &rResult,
                     SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsOriented();
    }

bool SGM::IsManifold(SGM::Result        &rResult,
                     SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->IsManifold();
    }

SGM::Complex SGM::FindBoundary(SGM::Result        &rResult,
                               SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    SGMInternal::complex *pBoundary=pComplex->FindBoundary(rResult);
    return {pBoundary->GetID()};
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
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->FindLength();
    }

double SGM::FindComplexArea(SGM::Result        &rResult,
                            SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->Area();
    }

double SGM::FindAverageEdgeLength(SGM::Result        &rResult,
                                  SGM::Complex const &ComplexID,
                                  double             *dMaxLength)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->FindAverageEdgeLength(dMaxLength);
    }

SGM::Complex SGM::CreateComplex(SGM::Result       &rResult,
                                SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    return {SGMInternal::CreateComplex(rResult,pEntity)->GetID()};
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

SGM::Complex SGM::CreatePolygon(SGM::Result                &rResult,
                                std::vector<Point3D> const &aPoints,
                                bool                        bFilled)
    {
    auto pComplex=new SGMInternal::complex(rResult, aPoints, bFilled);
    return {pComplex->GetID()};
    }

std::vector<unsigned int> const &SGM::GetFaceTriangles(SGM::Result     &rResult,
                                                       SGM::Face const &FaceID)
    {
    auto pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetTriangles(rResult);    
    }

std::vector<SGM::Point2D> const &SGM::GetFacePoints2D(SGM::Result     &rResult,
                                                      SGM::Face const &FaceID)
    {
    auto pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetPoints2D(rResult);
    }

std::vector<SGM::Point3D> const &SGM::GetFacePoints3D(SGM::Result     &rResult,
                                                      SGM::Face const &FaceID)
    {
    auto pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetPoints3D(rResult);
    }

std::vector<SGM::Entity> SGM::FindPointEntities(SGM::Result     &rResult,
                                                SGM::Face const &FaceID)
    {
    auto pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    std::vector<SGM::Entity> aAnswer;

    auto pSurface=pFace->GetSurface();
    if (pSurface)
        {
        std::vector<SGMInternal::entity *> aEntities;
        pFace->FindPointEntities(rResult, aEntities);
        aAnswer.reserve(aEntities.size());
        for(auto *pEnt : aEntities)
            {
            aAnswer.emplace_back(pEnt->GetID());
            }
        }
    else
        {
        rResult.SetResult(SGM::ResultType::ResultTypeInconsistentData);
        rResult.SetMessage(std::string("No surface on face ID ") + std::to_string(pFace->GetID()));
        }
    return aAnswer;
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
    auto pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->GetFacets(rResult);
    }

std::vector<double> const &SGM::GetEdgeParams(SGM::Result     &rResult,
                                              SGM::Edge const &EdgeID)
    {
    auto pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->GetParams(rResult);
    }

std::vector<SGM::UnitVector3D> const &SGM::GetFaceNormals(SGM::Result     &rResult,
                                                          SGM::Face const &FaceID)
    {
    auto pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetNormals(rResult);
    }

std::vector<SGM::Point3D> const &SGM::GetComplexPoints(SGM::Result        &rResult,
                                                       SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->GetPoints();
    }

std::vector<unsigned int> const &SGM::GetComplexSegments(SGM::Result        &rResult,
                                                         SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
    return pComplex->GetSegments();
    }

std::vector<unsigned int> const &SGM::GetComplexTriangles(SGM::Result        &rResult,
                                                          SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)rResult.GetThing()->FindEntity(ComplexID.m_ID);
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

const char* SGM::EntityTypeName(SGM::EntityType entityType)
    {
    switch(entityType)
        {
        case SGM::AssemblyType:
            return "assembly";
        case SGM::ReferenceType:
            return "reference";
        case SGM::ComplexType:
            return "complex";
        case SGM::BodyType:
            return "body";
        case SGM::VolumeType:
            return "volume";
        case SGM::FaceType:
            return "face";
        case SGM::EdgeType:
            return "edge";
        case SGM::VertexType:
            return "vertex";
        case SGM::CurveType:
            return "curve";
        case SGM::LineType:
            return "line";
        case SGM::CircleType:
            return "circle";
        case SGM::EllipseType:
            return "ellipse";
        case SGM::ParabolaType:
            return "parabola";
        case SGM::HyperbolaType:
            return "hyperbola";
        case SGM::NUBCurveType:
            return "NUBcurve";
        case SGM::NURBCurveType:
            return "NURBcurve";
        case SGM::PointCurveType:
            return "PointCurve";
        case SGM::HelixCurveType:
            return "helix";
        case SGM::HermiteCurveType:
            return "hermite";
        case SGM::TorusKnotCurveType:
            return "torus";
        case SGM::SurfaceType:
            return "surface";
        case SGM::PlaneType:
            return "plane";
        case SGM::CylinderType:
            return "cylinder";
        case SGM::ConeType:
            return "cone";
        case SGM::SphereType:
            return "sphere";
        case SGM::TorusType:
            return "torus";
        case SGM::NUBSurfaceType:
            return "NUBsurface";
        case SGM::NURBSurfaceType:
            return "NURBsurface";
        case SGM::RevolveType:
            return "revolve";
        case SGM::ExtrudeType:
            return "extrude";
        case SGM::OffsetType:
            return "offset";
        case SGM::AttributeType:
            return "attribute";
        case SGM::StringAttributeType:
            return "StringAttribute";
        case SGM::IntegerAttributeType:
            return "IntegerAttribute";
        case SGM::DoubleAttributeType:
            return "DoubleAttribute";
        case SGM::CharAttributeType:
            return "CharAttribute";
        default:
            throw std::logic_error("unknown SGM::EntityType has no associated name implemented");
        }
    }

size_t SGM::GetAttributes(SGM::Result         &rResult,
                          SGM::Entity   const &EntityID,
                          std::set<Attribute> &sAttributes,
                          bool                 bTopLevel)
    {
    SGMInternal::entity const *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    if(bTopLevel)
        {
        std::unordered_set<SGMInternal::attribute *> usAttributes=rResult.GetThing()->GetAttributes(true);
        for(auto pAttribute : usAttributes)
            {
            sAttributes.insert(SGM::Attribute(pAttribute->GetID()));
            }
        }
    else
        {
        std::set<SGMInternal::attribute *,SGMInternal::EntityCompare> const &sTempAttributes=pEntity->GetAttributes();
        auto iter=sTempAttributes.begin();
        while(iter!=sTempAttributes.end())
            {
            sAttributes.insert(SGM::Attribute((*iter)->GetID()));
            ++iter;
            }
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
                            double                              dTolerance)
    {
    auto pCurve1=(SGMInternal::curve const *)rResult.GetThing()->FindEntity(CurveID1.m_ID);
    auto pCurve2=(SGMInternal::curve const *)rResult.GetThing()->FindEntity(CurveID2.m_ID);
    return IntersectCurves(rResult,pCurve1,pCurve2,aPoints,aTypes,dTolerance);
    }

 size_t SGM::IntersectCurveAndSurface(SGM::Result                        &rResult,
                                      SGM::Curve                   const &CurveID,
                                      SGM::Surface                 const &SurfaceID,
                                      std::vector<SGM::Point3D>          &aPoints,
                                      std::vector<SGM::IntersectionType> &aTypes,
                                      double                              dTolerance)
     {
     auto pCurve=(SGMInternal::curve const *)rResult.GetThing()->FindEntity(CurveID.m_ID);
     auto pSurface=(SGMInternal::surface const *)rResult.GetThing()->FindEntity(SurfaceID.m_ID);
     return IntersectCurveAndSurface(rResult,pCurve,pSurface,aPoints,aTypes,dTolerance);
     }

bool SGM::IntersectSurfaces(SGM::Result               &rResult,
                            SGM::Surface        const &SurfaceID1,
                            SGM::Surface        const &SurfaceID2,
                            std::vector<SGM::Curve>   &aCurves,
                            double                     dTolerance)
    {
    auto const *pSurface1=(SGMInternal::surface const *)rResult.GetThing()->FindEntity(SurfaceID1.m_ID);
    auto const *pSurface2=(SGMInternal::surface const *)rResult.GetThing()->FindEntity(SurfaceID2.m_ID);
    std::vector<SGMInternal::curve *> acurves;
    bool bAnswer=IntersectSurfaces(rResult,pSurface1,pSurface2,acurves,dTolerance);
    for(auto pCurve : acurves)
        {
        aCurves.emplace_back(pCurve->GetID());
        }
    return bAnswer;
    }

SGM::Complex SGM::CreatePoints(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aPoints)
    {
    auto pComplex=new SGMInternal::complex(rResult, aPoints);
    return {pComplex->GetID()};
    }

SGM::Complex SGM::CreateSegments(SGM::Result                    &rResult,
                                 std::vector<SGM::Point3D> const &aPoints,
                                 std::vector<unsigned int> const &aSegments)
    {
    auto pComplex=new SGMInternal::complex(rResult, aSegments, aPoints);
    return {pComplex->GetID()};
    }

SGM::Complex SGM::CoverComplex(SGM::Result        &rResult,
                               SGM::Complex const &ComplexID)
    {
    auto pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    SGMInternal::complex *pAnswer=pComplex->Cover(rResult);
    return {pAnswer->GetID()};
    }

SGM::Complex SGM::MergePoints(SGM::Result        &rResult,
                              SGM::Complex const &ComplexID,
                              double              dTolerance)
    {
    auto const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    SGMInternal::complex *pAnswer=pComplex->Merge(rResult,dTolerance);
    return {pAnswer->GetID()};
    }

SGM::Complex SGM::FindSharpEdges(SGM::Result        &rResult,
                                 SGM::Complex const &ComplexID,
                                 double              dAngle,
                                 bool                bIncludeBoundary)
    {
    auto const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    SGMInternal::complex *pAnswer=pComplex->FindSharpEdges(rResult,dAngle,bIncludeBoundary);
    return {pAnswer->GetID()};
    }

SGM::Complex SGM::FindDegenerateTriangles(SGM::Result        &rResult,
                                          SGM::Complex const &ComplexID)
    {
    auto const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    SGMInternal::complex *pAnswer=pComplex->FindDegenerateTriangles(rResult);
    return {pAnswer->GetID()};
    }

std::vector<double> SGM::FindTriangleAreas(SGM::Result        &rResult,
                                           SGM::Complex const &ComplexID)
    {
    auto const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    return pComplex->FindTriangleAreas();
    }

size_t SGM::FindHoles(SGM::Result               &rResult,
                      SGM::Complex        const &ComplexID,
                      std::vector<SGM::Complex> &aHoles)
    {
    auto const *pComplex=(SGMInternal::complex const *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    std::vector<SGMInternal::complex *> aInternalHoles;
    pComplex->FindHoles(rResult,aInternalHoles);
    aHoles.reserve(aInternalHoles.size());
    for(auto pHole : aInternalHoles)
        {
        aHoles.emplace_back(pHole->GetID());
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
    return {pAnswer->GetID()};
    }

void SGM::SplitComplexAtPoints(SGM::Result                     &rResult,
                               SGM::Complex                    &ComplexID,
                               std::vector<SGM::Point3D> const &aPoints,
                               double                           dTolerance)
    {
    auto pComplex=(SGMInternal::complex *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    pComplex->SplitAtPoints(rResult,aPoints,dTolerance);
    }

void SGM::ReduceToUsedPoints(SGM::Result  &rResult,
                             SGM::Complex &ComplexID)
    {
    auto pComplex=(SGMInternal::complex *)(rResult.GetThing()->FindEntity(ComplexID.m_ID));
    pComplex->ReduceToUsedPoints();
    }

std::vector<SGM::Entity> SGM::FindTopLevelEntities(SGM::Result &rResult)
    {
    std::vector<SGMInternal::entity *> aEnts=rResult.GetThing()->GetTopLevelEntities();
    std::vector<SGM::Entity> aAnswer;
    aAnswer.reserve(aEnts.size());
    for(auto pEnt : aEnts)
        {
        aAnswer.emplace_back(pEnt->GetID());
        }
    return aAnswer;
    }

void SGM::FindBodies(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<SGM::Body> &sBodies,
                     bool                 bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::body *,SGMInternal::EntityCompare> sBodyPointers;
    FindBodies(rResult,pEntity,sBodyPointers,bTopLevel);
    for (auto pBody : sBodyPointers)
        {
        sBodies.insert(SGM::Body(pBody->GetID()));
        }
    }

SGM::Body SGM::FindBody(SGM::Result       &rResult,
                        SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::body *,SGMInternal::EntityCompare> sBodies;
    FindBodies(rResult,pEntity,sBodies,false);
    auto iter=sBodies.begin();
    return {(*iter)->GetID()};
    }

SGM::Volume SGM::FindVolume(SGM::Result       &rResult,
                            SGM::Entity const &EntityID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::volume *,SGMInternal::EntityCompare> sVolume;
    FindVolumes(rResult,pEntity,sVolume,false);
    auto iter=sVolume.begin();
    return {(*iter)->GetID()};
    }

void SGM::FindComplexes(SGM::Result            &rResult,
                        SGM::Entity      const &EntityID,
                        std::set<SGM::Complex> &sComplexes,
                        bool                    bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::complex *,SGMInternal::EntityCompare> sComplexPointers;
    FindComplexes(rResult,pEntity,sComplexPointers,bTopLevel);
    for (auto pComplex : sComplexPointers)
        {
        sComplexes.insert(SGM::Complex(pComplex->GetID()));
        }
    }

void SGM::FindVolumes(SGM::Result           &rResult,
                      SGM::Entity     const &EntityID,
                      std::set<SGM::Volume> &sVolumes,
                      bool                   bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::volume *,SGMInternal::EntityCompare> sVolumePointers;
    FindVolumes(rResult,pEntity,sVolumePointers,bTopLevel);
    for (auto pVolume : sVolumePointers)
        {
        sVolumes.insert(SGM::Volume(pVolume->GetID()));
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

void SGM::AddAttribute(SGM::Result    &rResult,
                       SGM::Entity    &EntityID,
                       SGM::Attribute &AttributeID)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    auto pAttribute=(SGMInternal::attribute *)rResult.GetThing()->FindEntity(AttributeID.m_ID);
    pEntity->AddAttribute(pAttribute);
    }

SGM::Attribute SGM::CreateAttribute(SGM::Result       &rResult,
                                    std::string const &Name)
    {
    SGMInternal::attribute *pAttribute=new SGMInternal::attribute(rResult,Name);
    return {pAttribute->GetID()};
    }

void SGM::FindFaces(SGM::Result         &rResult,
                    SGM::Entity   const &EntityID,
                    std::set<SGM::Face> &sFaces,
                    bool                 bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::face *,SGMInternal::EntityCompare> sFacePointers;
    FindFaces(rResult,pEntity,sFacePointers,bTopLevel);
    for (auto pFace : sFacePointers)
        {
        sFaces.insert(SGM::Face(pFace->GetID()));
        }
    }

void SGM::FindSurfaces(SGM::Result            &rResult,
                       SGM::Entity      const &EntityID,
                       std::set<SGM::Surface> &sSurfaces,
                       bool                    bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::surface *,SGMInternal::EntityCompare> sSurfacePointers;
    FindSurfaces(rResult,pEntity,sSurfacePointers,bTopLevel);
    for(auto pSurface : sSurfacePointers)
        {
        sSurfaces.insert(SGM::Surface(pSurface->GetID()));
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
    std::set<SGMInternal::edge *,SGMInternal::EntityCompare> sEdgePointers;
    FindEdges(rResult,pEntity,sEdgePointers,bTopLevel);
    for (auto pEdge : sEdgePointers)
        {
        sEdges.insert(SGM::Edge(pEdge->GetID()));
        }
    }

void SGM::FindCurves(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<Curve>     &sCurves,
                     bool                 bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::curve *,SGMInternal::EntityCompare> sCurvePointers;
    FindCurves(rResult,pEntity,sCurvePointers,bTopLevel);
    for (auto pCurve : sCurvePointers)
        {
        sCurves.insert(SGM::Curve(pCurve->GetID()));
        }
    }

void SGM::FindVertices(SGM::Result           &rResult,
                       SGM::Entity     const &EntityID,
                       std::set<SGM::Vertex> &sVertices,
                       bool                   bTopLevel)
    {
    SGMInternal::entity *pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
    std::set<SGMInternal::vertex *,SGMInternal::EntityCompare> sVertexPointers;
    FindVertices(rResult,pEntity,sVertexPointers,bTopLevel);
    for (auto pVertex : sVertexPointers)
        {
        sVertices.insert(SGM::Vertex(pVertex->GetID()));
        }
    }

bool SGM::IsSheetBody(SGM::Result     &rResult,
                      SGM::Body const &BodyID)
    {
    auto pBody=(SGMInternal::body const *)rResult.GetThing()->FindEntity(BodyID.m_ID);
    return pBody->IsSheetBody(rResult);
    }

bool SGM::IsWireBody(SGM::Result     &rResult,
                      SGM::Body const &BodyID)
    {
    auto pBody=(SGMInternal::body const *)rResult.GetThing()->FindEntity(BodyID.m_ID);
    return pBody->IsWireBody(rResult);
    }

SGM::Surface SGM::GetSurfaceOfFace(SGM::Result     &rResult,
                                   SGM::Face const &FaceID)
    {
    auto pFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetSurface()->GetID();
    }

SGM::UnitVector3D SGM::FindNormal(SGM::Result        &rResult,
                                        SGM::Face    const &FaceID,
                                        SGM::Point3D const &Pos)
    {
    auto pFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->FindNormal(Pos);
    }

SGM::Curve SGM::GetCurveOfEdge(SGM::Result     &rResult,
                               SGM::Edge const &EdgeID)
    {
    auto pEdge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->GetCurve()->GetID();
    }

int SGM::GetSidesOfFace(SGM::Result     &rResult,
                        SGM::Face const &FaceID)
    {
    auto pFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetSides();
    }

SGM::EdgeSideType SGM::GetEdgeSideType(SGM::Result     &rResult,
                                       SGM::Edge const &EdgeID,
                                       SGM::Face const &FaceID)
    {
    auto pFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    auto pEdge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pFace->GetSideType(pEdge);
    }

bool SGM::IsFaceFlipped(SGM::Result     &rResult,
                        SGM::Face const &FaceID)
    {
    auto pFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    return pFace->GetFlipped();
    }

double SGM::GetToleranceOfEdge(SGM::Result     &rResult,
                                SGM::Edge const &EdgeID)
    {
    auto pEdge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->GetTolerance();
    }

SGM::Body SGM::CreateBody(SGM::Result &rResult)
    {
    SGMInternal::body *pBody=new SGMInternal::body(rResult);
    return pBody->GetID();
    }

SGM::Volume SGM::CreateVolume(SGM::Result &rResult)
    {
    SGMInternal::volume *pVolume=new SGMInternal::volume(rResult);
    return pVolume->GetID();
    }

SGM::Face SGM::CreateFace(SGM::Result &rResult)
    {
    SGMInternal::face *pFace=new SGMInternal::face(rResult);
    return pFace->GetID();
    }

SGM::Edge SGM::CreateEdge(SGM::Result &rResult)
    {
    SGMInternal::edge *pEdge=new SGMInternal::edge(rResult);
    return pEdge->GetID();
    }

SGM::Vertex SGM::CreateVertex(SGM::Result        &rResult,
                              SGM::Point3D const &Pos)
    {
    SGMInternal::vertex *pVertex=new SGMInternal::vertex(rResult,Pos);
    return pVertex->GetID();
    }

void SGM::AddVolumeToBody(SGM::Result &rResult,
                          SGM::Volume &VolumeID,
                          SGM::Body   &BodyID)
    {
    SGMInternal::body *pBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(BodyID.m_ID);
    SGMInternal::volume *pVolume=(SGMInternal::volume *)rResult.GetThing()->FindEntity(VolumeID.m_ID);
    pBody->AddVolume(pVolume);
    }

void SGM::SetPointsOfBody(SGM::Result                     &rResult,
                          std::vector<SGM::Point3D> const &aPoints,
                          SGM::Body                       &BodyID)
    {
    SGMInternal::body *pBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(BodyID.m_ID);
    pBody->SetPoints(aPoints);
    }

void SGM::AddFaceToVolume(SGM::Result &rResult,
                          SGM::Face   &FaceID,
                          SGM::Volume &VolumeID)
    {
    SGMInternal::volume *pVolume=(SGMInternal::volume *)rResult.GetThing()->FindEntity(VolumeID.m_ID);
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    pVolume->AddFace(rResult,pFace);
    }

void SGM::SetSurfaceOfFace(SGM::Result  &rResult,
                           SGM::Surface &SurfaceID,
                           SGM::Face    &FaceID)
    {
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    SGMInternal::surface *pSurface=(SGMInternal::surface *)rResult.GetThing()->FindEntity(SurfaceID.m_ID);
    pFace->SetSurface(rResult,pSurface);
    }

void SGM::SetSidesOfFace(SGM::Result  &rResult,
                         int           nSides,        
                         SGM::Face    &FaceID)
    {
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    pFace->SetSides(nSides);
    }

void SGM::SetFlippedOfFace(SGM::Result  &rResult,
                           bool          bFlipped,        
                           SGM::Face    &FaceID)
    {
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    pFace->SetFlipped(bFlipped);
    }

void SGM::AddEdgeToFace(SGM::Result       &rResult,
                        SGM::Edge         &EdgeID,
                        SGM::EdgeSideType  nType,
                        SGM::Face         &FaceID)
    {
    SGMInternal::face *pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    SGMInternal::edge *pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    pFace->AddEdge(rResult,pEdge,nType);
    }

void SGM::SetStartOfEdge(SGM::Result &rResult,
                         SGM::Vertex &VertexID,
                         SGM::Edge   &EdgeID)
    {
    SGMInternal::edge *pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    SGMInternal::vertex *pVertex=(SGMInternal::vertex *)rResult.GetThing()->FindEntity(VertexID.m_ID);
    pEdge->SetStart(rResult,pVertex);
    }

void SGM::SetEndOfEdge(SGM::Result &rResult,
                       SGM::Vertex &VertexID,
                       SGM::Edge   &EdgeID)
    {
    SGMInternal::edge *pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    SGMInternal::vertex *pVertex=(SGMInternal::vertex *)rResult.GetThing()->FindEntity(VertexID.m_ID);
    pEdge->SetEnd(rResult,pVertex);
    }

void SGM::SetCurveOfEdge(SGM::Result &rResult,
                         SGM::Curve  &CurveID,
                         SGM::Edge   &EdgeID)
    {
    SGMInternal::edge *pEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    SGMInternal::curve *pCurve=(SGMInternal::curve *)rResult.GetThing()->FindEntity(CurveID.m_ID);
    pEdge->SetCurve(rResult,pCurve);
    }

size_t SGM::FindAdjacentFaces(SGM::Result            &rResult,
                              SGM::Face        const &FaceID,
                              std::vector<SGM::Face> &aFaces)
    {
    auto pThisFace=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    std::set<SGMInternal::face *,SGMInternal::EntityCompare> sFaces;
    size_t nAnswer=SGMInternal::FindAdjacentFaces(rResult,pThisFace,sFaces);
    aFaces.reserve(sFaces.size());
    for(auto pFace : sFaces)
        {
        aFaces.emplace_back(pFace->GetID());
        }
    return nAnswer;
    }

size_t SGM::FindCommonEdgesFromFaces(SGM::Result            &rResult,
                                     SGM::Face        const &FaceID1,   
                                     SGM::Face        const &FaceID2,   
                                     std::vector<SGM::Edge> &aEdges)
    {
    auto pFace1=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID1.m_ID);
    auto pFace2=(SGMInternal::face const *)rResult.GetThing()->FindEntity(FaceID2.m_ID);
    std::vector<SGMInternal::edge *> aEdgePointers;
    size_t nAnswer=SGMInternal::FindCommonEdgesFromFaces(rResult,pFace1,pFace2,aEdgePointers);
    for(auto pEdge : aEdgePointers)
        {
        aEdges.emplace_back(pEdge->GetID());
        }
    return nAnswer;
    }

SGM::Interval1D const &SGM::GetDomainOfEdge(SGM::Result     &rResult,
                                            SGM::Edge const &EdgeID)
    {
    auto pEdge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->GetDomain();
    }

SGM::Point3D SGM::GetStartPointOfEdge(SGM::Result     &rResult,
                                      SGM::Edge const &EdgeID)
    {
    auto pEdge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->FindStartPoint();
    }

SGM::Point3D SGM::GetEndPointOfEdge(SGM::Result     &rResult,
                                    SGM::Edge const &EdgeID)
    {
    auto pEdge=(SGMInternal::edge const *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    return pEdge->FindEndPoint();
    }

SGM::Point3D const &SGM::GetPointOfVertex(SGM::Result       &rResult,
                                          SGM::Vertex const &VertexID)
    {
    auto pVertex=(SGMInternal::vertex const *)rResult.GetThing()->FindEntity(VertexID.m_ID);
    return pVertex->GetPoint();
    }

std::vector<SGM::Point3D> const &SGM::GetPointsOfBody(SGM::Result     &rResult,
                                                      SGM::Body const &BodyID)
    {
    auto pBody=(SGMInternal::body const *)rResult.GetThing()->FindEntity(BodyID.m_ID);
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
                              double              dRadius,
                              bool                bSheetBody)
    {
    SGMInternal::body *pBody=SGMInternal::CreateCylinder(rResult,BottomCenter,TopCenter,dRadius,bSheetBody);
    return {pBody->GetID()};
    }

SGM::Body SGM::CreateCone(SGM::Result        &rResult,
                          SGM::Point3D const &BottomCenter,
                          SGM::Point3D const &TopCenter,
                          double              dBottomRadius,
                          double              dTopRadius,
                          bool                bSheetBody)
    {
    SGMInternal::body *pBody=SGMInternal::CreateCone(rResult,BottomCenter,TopCenter,dBottomRadius,dTopRadius,bSheetBody);
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
    auto pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(IDCurve.m_ID);
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
    auto pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(IDCurve.m_ID);
    SGMInternal::surface *pSurface=SGMInternal::CreateRevolveSurface(rResult, Origin, Axis, pCurve);

    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreateExtrudeSurface(SGM::Result             &rResult,
                                       SGM::UnitVector3D const &Axis,
                                       SGM::Curve              &CurveID)
    {
    auto pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(CurveID.m_ID);
    SGMInternal::surface *pSurface=SGMInternal::CreateExtrudeSurface(rResult, Axis, pCurve);

    return {pSurface->GetID()};
    }

SGM::Body SGM::CreateSheetBody(SGM::Result                    &rResult,
                               SGM::Surface                   &SurfaceID,
                               std::vector<SGM::Edge>         &aEdges,
                               std::vector<SGM::EdgeSideType> &aTypes)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pSurface=(SGMInternal::surface *)pThing->FindEntity(SurfaceID.m_ID);
    std::vector<SGMInternal::edge *> aedges;
    size_t Index1;
    size_t nEdges=aEdges.size();
    aedges.reserve(nEdges);
    for(Index1=0;Index1<nEdges;++Index1)
        {
        auto pEdge=(SGMInternal::edge *)pThing->FindEntity(aEdges[Index1].m_ID);
        aedges.push_back(pEdge);
        }
    SGMInternal::body *pBody=SGMInternal::CreateSheetBody(rResult,pSurface,aedges,aTypes);
    return {pBody->GetID()};
    }

SGM::Body SGM::CreateSheetBody(SGM::Result           &rResult,
                               SGM::Surface          &SurfaceID,
                               SGM::Interval2D const &Domain)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pSurface=(SGMInternal::surface *)pThing->FindEntity(SurfaceID.m_ID);
    SGMInternal::body *pBody=SGMInternal::CreateSheetBody(rResult,pSurface,Domain);
    return {pBody->GetID()};
    }

SGM::Face SGM::CreateFaceFromSurface(SGM::Result                    &rResult,
                                     SGM::Surface                   &SurfaceID,
                                     std::vector<SGM::Edge>         &aEdges,
                                     std::vector<SGM::EdgeSideType> &aTypes)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pSurface=(SGMInternal::surface *)pThing->FindEntity(SurfaceID.m_ID);
    std::vector<SGMInternal::edge *> aedges;
    size_t Index1;
    size_t nEdges=aEdges.size();
    aedges.reserve(nEdges);
    for(Index1=0;Index1<nEdges;++Index1)
        {
        auto pEdge=(SGMInternal::edge *)pThing->FindEntity(aEdges[Index1].m_ID);
        aedges.push_back(pEdge);
        }
    SGMInternal::face *pFace=SGMInternal::CreateFaceFromSurface(rResult,pSurface,aedges,aTypes);
    return {pFace->GetID()};
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
                               double                   dA,
                               SGM::Interval1D   const *pDomain)
    {
    SGMInternal::curve *pCurve=new SGMInternal::parabola(rResult,Center,XAxis,YAxis,dA);
    if(pDomain)
        {
        pCurve->SetDomain(*pDomain);
        }
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
    auto pCurve=(SGMInternal::curve *)pThing->FindEntity(CurveID.m_ID);
    SGMInternal::edge *pEdge=SGMInternal::CreateEdge(rResult,pCurve,pDomain);
    return {pEdge->GetID()};
    }

SGM::Edge SGM::CreateEdge(SGM::Result &rResult,
                          SGM::Curve  &CurveID,
                          SGM::Vertex &Start,
                          SGM::Vertex &End)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pCurve=(SGMInternal::curve *)pThing->FindEntity(CurveID.m_ID);
    auto pStart=(SGMInternal::vertex *)pThing->FindEntity(Start.m_ID);
    auto pEnd=(SGMInternal::vertex *)pThing->FindEntity(End.m_ID);
    if (pStart == pEnd)
    {
        if (!pCurve->GetClosed())
        {
            throw std::logic_error("Matching start and end vertices requires a closed curve");
        }
    }
    SGMInternal::edge *pEdge=SGMInternal::CreateEdge(rResult, pCurve, pStart, pEnd);
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
        auto pEdge=(SGMInternal::edge *)pThing->FindEntity(EdgeID.m_ID);
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
    //SGMInternal::CheckPreexistingConditions(rResult, aCheckStrings);
    //if (rResult.GetResult() != ResultTypeOK)
    //    return false;

    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    return pEntity->Check(rResult,Options,aCheckStrings,true);
    }

bool SGM::TestCurve(SGM::Result      &rResult,
                    SGM::Curve const &CurveID,
                    double            dT)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pCurve=(SGMInternal::curve *)pThing->FindEntity(CurveID.m_ID);
    return SGMInternal::TestCurve(rResult,pCurve,dT);
    }

bool SGM::TestSurface(SGM::Result        &rResult,
                      SGM::Surface const &SurfaceID,
                      SGM::Point2D const &uv)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pSurface=(SGMInternal::surface *)pThing->FindEntity(SurfaceID.m_ID);
    return SGMInternal::TestSurface(rResult,pSurface,uv);
    }

SGM::Interval1D const &SGM::GetCurveDomain(SGM::Result      &rResult,
                                      SGM::Curve const &CurveID)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pCurve=(SGMInternal::curve *)(pThing->FindEntity(CurveID.m_ID));
    return pCurve->GetDomain();
    }

SGM_EXPORT bool SGM::IsCurveClosed(SGM::Result      &rResult,
                                   SGM::Curve const &CurveID)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pCurve=(SGMInternal::curve *)(pThing->FindEntity(CurveID.m_ID));
    return pCurve->GetClosed();
    }

void SGM::EvaluateCurve(SGM::Result      &rResult,
                        SGM::Curve const &CurveID, 
                        double            dt,
                        SGM::Point3D     *pPos,
                        SGM::Vector3D    *pVec1,
                        SGM::Vector3D    *pVec2)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pCurve=(SGMInternal::curve *)(pThing->FindEntity(CurveID.m_ID));
    pCurve->Evaluate(dt,pPos,pVec1,pVec2);
    }

double SGM::CurveInverse(SGM::Result        &rResult,
                         SGM::Curve   const &CurveID,
                         SGM::Point3D const &Pos,
                         SGM::Point3D       *pClosePos,
                         double       const *pGuess)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pCurve=(SGMInternal::curve *)(pThing->FindEntity(CurveID.m_ID));
    return pCurve->Inverse(Pos,pClosePos,pGuess);
    }

SGM::Vector3D SGM::CurveCurvature(SGM::Result        &rResult,
                                  SGM::Curve   const &CurveID,
                                  double              t)
{
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pCurve=(SGMInternal::curve *)(pThing->FindEntity(CurveID.m_ID));
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
    auto pSurface=(SGMInternal::surface *)(pThing->FindEntity(SurfaceID.m_ID));
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
    auto pSurface=(SGMInternal::surface *)(pThing->FindEntity(SurfaceID.m_ID));
    pSurface->Evaluate(uv,pPos,pDu,pDv,pNorm,pDuu,pDuv,pDvv);
    }

bool SGM::SameSurface(SGM::Result        &rResult,
                      SGM::Surface const &SurfaceID1,
                      SGM::Surface const &SurfaceID2,
                      double              dTolerance)
    {
    SGMInternal::thing   *pThing=rResult.GetThing();
    auto pSurface1=(SGMInternal::surface *)(pThing->FindEntity(SurfaceID1.m_ID));
    auto pSurface2=(SGMInternal::surface *)(pThing->FindEntity(SurfaceID2.m_ID));
    return pSurface1->IsSame(pSurface2,dTolerance);
    }

bool SGM::SameCurve(SGM::Result      &rResult,
                    SGM::Curve const &CurveID1,
                    SGM::Curve const &CurveID2,
                    double            dTolerance)
    {
    SGMInternal::thing   *pThing=rResult.GetThing();
    auto pCurve1=(SGMInternal::curve *)(pThing->FindEntity(CurveID1.m_ID));
    auto pCurve2=(SGMInternal::curve *)(pThing->FindEntity(CurveID2.m_ID));
    return pCurve1->IsSame(pCurve2,dTolerance);
    }

SGM::Point2D SGM::SurfaceInverse(SGM::Result        &rResult,
                                 SGM::Surface const &SurfaceID,
                                 SGM::Point3D const &Pos,
                                 SGM::Point3D       *pClosePos,
                                 SGM::Point2D const *pGuess)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    auto pSurface=(SGMInternal::surface *)(pThing->FindEntity(SurfaceID.m_ID));
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
    auto pNURB=new SGMInternal::NURBcurve(rResult, aControlPoints, aKnots);
    return {pNURB->GetID()};
    }

SGM::Curve SGM::CreateHermiteCurve(SGM::Result                      &rResult,
                                   std::vector<SGM::Point3D>  const &aPoints,
                                   std::vector<SGM::Vector3D> const &aVectors,
                                   std::vector<double>        const &aParams)
    {
    auto pHermite=new SGMInternal::hermite(rResult,aPoints,aVectors,aParams);
    return {pHermite->GetID()};
    }

SGM::Curve SGM::CreateNUBCurveWithControlPointsAndKnots(SGM::Result                     &rResult,
                                                        std::vector<SGM::Point3D> const &aControlPoints,
                                                        std::vector<double>       const &aKnots)
    {
    auto pNUB=new SGMInternal::NUBcurve(rResult, aControlPoints, aKnots);
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

SGM::Surface SGM::CreateNUBSurface(SGM::Result                                   &rResult,
                                   std::vector<std::vector<SGM::Point3D> > const &aaInterpolatePoints,
                                   std::vector<SGM::Vector3D>              const *,//paStartVecs,
                                   std::vector<SGM::Vector3D>              const *,//paEndVecs,
                                   std::vector<double>                     const *,//pUParams,
                                   std::vector<double>                     const *)//pVParams)
    {
    SGMInternal::surface *pSurface=new SGMInternal::NUBsurface(rResult,aaInterpolatePoints);
    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreateOffsetSurface(SGM::Result  &rResult,
                                      SGM::Surface &BaseSurfaceID,
                                      double        dOffset)
    {
    SGMInternal::surface *pBaseSurface=(SGMInternal::surface *)rResult.GetThing()->FindEntity(BaseSurfaceID.m_ID);
    SGMInternal::surface *pSurface=new SGMInternal::offset(rResult,dOffset,pBaseSurface);
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

SGM::Surface SGM::CreateConeSurfaceFromPoints(SGM::Result        &rResult,
                                              SGM::Point3D const &Bottom,
                                              SGM::Point3D const &Top,
                                              double              dBottomRadius,
                                              double              dTopRadius)
    {
    SGMInternal::surface *pSurface=new SGMInternal::cone(rResult,Bottom,Top,dBottomRadius,dTopRadius);
    return {pSurface->GetID()};
    }

SGM::Surface SGM::CreatePlane(SGM::Result        &rResult,
                              SGM::Point3D const &Origin,
                              SGM::Point3D const &XPos,
                              SGM::Point3D const &YPos)
    {
    auto pPlane = new SGMInternal::plane(rResult,Origin,XPos,YPos);
    return {pPlane->GetID()};
    }

SGM::Surface SGM::CreatePlane(SGM::Result             &rResult,
                              SGM::Point3D      const &Origin,
                              SGM::UnitVector3D const &Normal)
    {
    auto pPlane = new SGMInternal::plane(rResult,Origin,Normal);
    return {pPlane->GetID()};
    }

SGM::Surface SGM::CreatePlaneFromOriginAndNormal(SGM::Result             &rResult,
                                                 SGM::Point3D      const &Origin,
                                                 SGM::UnitVector3D const &Normal)
    {
    auto pPlane = new SGMInternal::plane(rResult,Origin,Normal);
    return {pPlane->GetID()};
    }

SGM::EntityType SGM::GetCurveType(SGM::Result      &rResult,
                                  SGM::Curve const &CurveID)
    {
    auto pCurve=(SGMInternal::curve const *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    return pCurve->GetCurveType();
    }

SGM::EntityType SGM::GetAttributeType(SGM::Result          &rResult,
                                      SGM::Attribute const &AttributeID)
    {
    auto const *pAttribute=(SGMInternal::attribute const *)(rResult.GetThing()->FindEntity(AttributeID.m_ID));
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
    auto const *pAttribute=(SGMInternal::IntegerAttribute const *)(rResult.GetThing()->FindEntity(AttributeID.m_ID));
    return pAttribute->GetData();
    }

std::string const &SGM::GetStringAttributeData(SGM::Result          &rResult,
                                                     SGM::Attribute const &AttributeID)
    {
    auto const *pAttribute=(SGMInternal::StringAttribute const *)(rResult.GetThing()->FindEntity(AttributeID.m_ID));
    return pAttribute->GetData();
    }

SGM::Interval1D const &SGM::GetDomainOfCurve(SGM::Result      &rResult,
                                             SGM::Curve const &CurveID)
    {
    auto const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    return pCurve->GetDomain();
    }

SGM::EntityType SGM::GetSurfaceType(SGM::Result        &rResult,
                                    SGM::Surface const &SurfaceID)
    {
    auto const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    return pSurface->GetSurfaceType();
    }

SGM::Interval2D const &SGM::GetDomainOfSurface(SGM::Result        &rResult,
                                               SGM::Surface const &SurfaceID)
    {
    auto const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    return pSurface->GetDomain();
    }

void SGM::SetDomainOfSurface(SGM::Result           &rResult,
                             SGM::Surface          &SurfaceID,
                             SGM::Interval2D const &Domain)
    {
    auto pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    pSurface->SetDomain(Domain);
    }

bool SGM::IsSurfaceSingularHighU(SGM::Result        &rResult,
                                 SGM::Surface const &SurfaceID)
    {
    auto const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    return pSurface->SingularHighU();
    }

bool SGM::IsSurfaceSingularHighV(SGM::Result        &rResult,
                                 SGM::Surface const &SurfaceID)
    {
    auto const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    return pSurface->SingularHighV();
    }

bool SGM::IsSurfaceSingularLowU(SGM::Result        &rResult,
                                SGM::Surface const &SurfaceID)
    {
    auto const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    return pSurface->SingularLowU();
    }

bool SGM::IsSurfaceSingularLowV(SGM::Result        &rResult,
                                SGM::Surface const &SurfaceID)
    {
    auto const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    return pSurface->SingularLowV();
    }

bool SGM::IsSingularity(SGM::Result        &rResult,
                        SGM::Surface const &SurfaceID,
                        SGM::Point2D const &uv,
                        double              dTolerance)
    {
    auto const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    return pSurface->IsSingularity(uv,dTolerance);
    }

void SGM::FindWindingNumbers(SGM::Result                     &rResult,
                             SGM::Surface              const &SurfaceID,
                             std::vector<SGM::Point3D> const &aPolygon3D,
                             int                             &nUWinds,
                             int                             &nVWinds)
    {
    auto const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    SGMInternal::FindWindingNumbers(pSurface,aPolygon3D,nUWinds,nVWinds);
    }

bool SGM::GetLineData(SGM::Result       &rResult,
                      SGM::Curve  const &CurveID,
                      SGM::Point3D      &Origin,
                      SGM::UnitVector3D &Axis)
    {
    auto const *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    if(pCurve->GetCurveType()!=SGM::EntityType::LineType)
        {
        return false;
        }
    auto pLine=(SGMInternal::line const *)pCurve;
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
    auto pCircle=(SGMInternal::circle const *)pCurve;
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
    auto pEllipse=(SGMInternal::ellipse const *)pCurve;
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
    auto pParabola=(SGMInternal::parabola const *)pCurve;
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
    auto pHyperbola=(SGMInternal::hyperbola const *)pCurve;
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
    auto const *pNUBCurve=(SGMInternal::NUBcurve const *)pCurve;
    aControlPoints=pNUBCurve->GetControlPoints();
    aKnots        =pNUBCurve->GetKnots();
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
    auto const *pNURBCurve=(SGMInternal::NURBcurve const *)pCurve;
    aControlPoints=pNURBCurve->GetControlPoints();
    aKnots        =pNURBCurve->GetKnots();
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
    auto const *pHermite=(SGMInternal::hermite const *)pCurve;
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
    auto const *pPointCurve=(SGMInternal::PointCurve const *)pCurve;
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
    auto const *pPlane=(SGMInternal::plane const *)pSurface;
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
    auto const *pCylinder=(SGMInternal::cylinder const *)pSurface;
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
                      double             &dRadius,
                      SGM::Point3D       &Apex)
    {
    SGMInternal::surface const *pSurface=(SGMInternal::surface *)(rResult.GetThing()->FindEntity(SurfaceID.m_ID));
    if(pSurface->GetSurfaceType()!=SGM::EntityType::ConeType)
        {
        return false;
        }
    auto const *pCone=(SGMInternal::cone const *)pSurface;
    Origin    =pCone->m_Origin;
    XAxis     =pCone->m_XAxis; 
    YAxis     =pCone->m_YAxis; 
    ZAxis     =pCone->m_ZAxis; 
    dHalfAngle=SAFEacos(pCone->m_dCosHalfAngle);
    dRadius   =pCone->m_dRadius;
    Apex      =pCone->FindApex();
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
    auto const *pSphere=(SGMInternal::sphere const *)pSurface;
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
    auto const *pRevolve=(SGMInternal::revolve const *)pSurface;

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
    auto const *pTorus=(SGMInternal::torus const *)pSurface;

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
    auto const *pNUBSurface=(SGMInternal::NUBsurface const *)pSurface;
    aaControlPoints=pNUBSurface->m_aaControlPoints;
    aUKnots=pNUBSurface->m_aUKnots;
    aVKnots=pNUBSurface->m_aVKnots;
    return true;
    }

void SGM::NegateCurve(SGM::Result &rResult,
                      SGM::Curve  &CurveID)
    {
    SGMInternal::curve *pCurve=(SGMInternal::curve *)(rResult.GetThing()->FindEntity(CurveID.m_ID));
    pCurve->Negate();
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
    auto const *pNURBSurface=(SGMInternal::NURBsurface const *)pSurface;
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
    SGMInternal::body *pWireBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(PlanarWireID.m_ID);
    SGMInternal::body *pCoveredBody=SGMInternal::CoverPlanarWire(rResult,pWireBody);
    return {pCoveredBody->GetID()};
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
    for (auto pEdge : sEdges)
        {
        SGM::Point3D ClosestPoint;
        SGMInternal::entity *pCloseEntity;
        SGMInternal::FindClosestPointOnEdge3D(rResult,Point,pEdge,ClosestPoint,pCloseEntity);
        if(Point.DistanceSquared(ClosestPoint)<dTol)
            {
            aEdges.emplace_back(pEdge->GetID());
            }
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

std::vector<bool> SGM::PointsInVolume(SGM::Result                     &rResult,
                                      std::vector<SGM::Point3D> const &aPoints,
                                      SGM::Volume               const &VolumeID,
                                      double                           dTolerance)
    {
    SGMInternal::thing *pThing=rResult.GetThing();
    SGMInternal::volume const *pVolume=(SGMInternal::volume*)pThing->FindEntity(VolumeID.m_ID);
    return SGMInternal::PointsInVolume(rResult,aPoints,pVolume,dTolerance);
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
            aVolumeIDs.emplace_back(aVolumes[Index2]->GetID());
            }
        aaVolumeIDs.emplace_back(std::move(aVolumeIDs));
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
    for (auto pFace : sFaces)
        {
        SGM::Point3D ClosestPoint;
        SGMInternal::entity *pCloseEntity;
        SGMInternal::FindClosestPointOnFace(rResult,Point,pFace,ClosestPoint,pCloseEntity);
        if(Point.DistanceSquared(ClosestPoint)<dTol)
            {
            aFaces.emplace_back(pFace->GetID());
            }
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

void SGM::ReduceToVolumes(SGM::Result           &rResult,
                          SGM::Body             &BodyID,
                          std::set<SGM::Volume> &sVolumes)
    {
    auto pBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(BodyID.m_ID);
    std::set<SGMInternal::volume *,SGMInternal::EntityCompare> svolumes;
    SGMInternal::ReduceToVolumes(rResult,pBody,svolumes);
    for(auto pVolume : svolumes)
        {
        sVolumes.insert(SGM::Volume(pVolume->GetID()));
        }
    }

std::vector<SGM::Face> SGM::ImprintEdgeOnFace(SGM::Result &rResult,
                                              SGM::Edge   &EdgeID,
                                              SGM::Face   &FaceID)
    {
    auto pTheEdge=(SGMInternal::edge *)rResult.GetThing()->FindEntity(EdgeID.m_ID);
    auto pTheFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    std::vector<SGMInternal::face *> aFaces=SGMInternal::ImprintEdgeOnFace(rResult,pTheEdge,pTheFace);
    std::vector<SGM::Face> aAnswer;
    aAnswer.reserve(aFaces.size());
    for(auto pFace : aFaces)
        {
        aAnswer.emplace_back(pFace->GetID());
        }
    return aAnswer;
    }

void SGM::TrimCurveWithFace(SGM::Result               &rResult,
                            SGM::Curve                &CurveID,
                            SGM::Face           const &FaceID,
                            std::vector<SGM::Edge>    &aEdges)
    {
    auto pCurve=(SGMInternal::curve *)rResult.GetThing()->FindEntity(CurveID.m_ID);
    auto pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    std::vector<SGMInternal::edge *> aedges;
    std::map<double,SGMInternal::entity *> mHitMap1,aHitMap2;
    SGMInternal::TrimCurveWithFaces(rResult,pCurve,pFace,nullptr,aedges,SGM_MIN_TOL,nullptr);
    for(auto pEdge : aedges)
        {
        aEdges.push_back(SGM::Edge(pEdge->GetID()));
        }
    }

void SGM::UniteBodies(SGM::Result &rResult,
                      SGM::Body   &KeepBodyID,
                      SGM::Body   &DeletedBodyID)
    {
    auto pKeepBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(KeepBodyID.m_ID);
    auto pDeletedBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(DeletedBodyID.m_ID);
    SGMInternal::UniteBodies(rResult,pKeepBody,pDeletedBody);
    }

void SGM::SubtractBodies(SGM::Result &rResult,
                         SGM::Body   &KeepBodyID,
                         SGM::Body   &DeletedBodyID)
    {
    auto pKeepBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(KeepBodyID.m_ID);
    auto pDeletedBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(DeletedBodyID.m_ID);
    SGMInternal::SubtractBodies(rResult,pKeepBody,pDeletedBody);
    }

void SGM::IntersectBodies(SGM::Result &rResult,
                          SGM::Body   &KeepBodyID,
                          SGM::Body   &DeletedBodyID)
    {
    auto pKeepBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(KeepBodyID.m_ID);
    auto pDeletedBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(DeletedBodyID.m_ID);
    SGMInternal::IntersectBodies(rResult,pKeepBody,pDeletedBody);
    }

void SGM::ImprintBodies(SGM::Result &rResult,
                        SGM::Body   &KeepBodyID,
                        SGM::Body   &DeletedBodyID)
    {
    auto pKeepBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(KeepBodyID.m_ID);
    auto pDeletedBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(DeletedBodyID.m_ID);
    SGMInternal::ImprintBodies(rResult,pKeepBody,pDeletedBody);
    }

void SGM::ImprintVerticesOnClosedEdges(SGM::Result &rResult)
    {
    SGMInternal::ImprintVerticesOnClosedEdges(rResult);
    }

void SGM::TweakFace(SGM::Result  &rResult,
                    SGM::Face    &FaceID,
                    SGM::Surface &SurfaceID)
    {
    auto pFace=(SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    auto pSurface=(SGMInternal::surface *)rResult.GetThing()->FindEntity(SurfaceID.m_ID);
    SGMInternal::TweakFace(rResult,pFace,pSurface);
    }

SGM_EXPORT void SGM::Repair(SGM::Result            &rResult,
                            std::vector<SGM::Body> &aBodies,
                            SGM::RepairOptions     *pOptions)
    {
    std::vector<SGMInternal::body *> abodies;
    abodies.reserve(aBodies.size());
    for(auto BodyID : aBodies)
        {
        auto pBody=(SGMInternal::body *)rResult.GetThing()->FindEntity(BodyID.m_ID);
        abodies.push_back(pBody);
        }
    SGMInternal::Repair(rResult,abodies,pOptions);
    }

SGM::Vertex SGM::ImprintPoint(SGM::Result        &rResult,
                              SGM::Point3D const &Pos,
                              SGM::Topology      &TopologyID)
    {
    auto pTopology=(SGMInternal::topology *)rResult.GetThing()->FindEntity(TopologyID.m_ID);
    SGMInternal::vertex *pVertex=SGMInternal::ImprintPoint(rResult,Pos,pTopology);
    if(pVertex==nullptr)
        {
        return {0};
        }
    return {pVertex->GetID()};
    }

void SGM::Merge(SGM::Result &rResult,
                SGM::Entity &EntityID)
    {
    auto pEntity=rResult.GetThing()->FindEntity(EntityID.m_ID);
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
    auto pCurve = (SGMInternal::curve *)pEntity;
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
    auto pEdge = (SGMInternal::edge *)pEntity;
    return SGMInternal::IntersectEdgeAndPlane(rResult, pEdge, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);
}

void SGM::IntersectThreeSurfaces(SGM::Result                &rResult,
                                 SGM::Surface         const &SurfaceID1,
                                 SGM::Surface         const &SurfaceID2,
                                 SGM::Surface         const &SurfaceID3,
                                 std::vector<SGM::Point3D>  &aPoints)
{
    auto pSurf1 = (SGMInternal::surface *)rResult.GetThing()->FindEntity(SurfaceID1.m_ID);
    auto pSurf2 = (SGMInternal::surface *)rResult.GetThing()->FindEntity(SurfaceID2.m_ID);
    auto pSurf3 = (SGMInternal::surface *)rResult.GetThing()->FindEntity(SurfaceID3.m_ID);
    return SGMInternal::IntersectThreeSurfaces(rResult,pSurf1,pSurf2,pSurf3,aPoints);
}

SGM::Curve SGM::CreatePointCurve(SGM::Result           &rResult,
                                 SGM::Point3D    const &Pos,
                                 SGM::Interval1D const *pDomain)
{
    SGMInternal::curve *pCurve=new SGMInternal::PointCurve(rResult,Pos,pDomain);
    return {pCurve->GetID()};
}


SGM::Curve SGM::FindConic(SGM::Result                     &rResult,
                          std::vector<SGM::Point3D> const &aPoints,
                          double                           dTolerance)
    {
    SGMInternal::curve *pCurve=SGMInternal::FindConic(rResult,aPoints,dTolerance);
    if(pCurve==nullptr)
        {
        return {0};
        }
    return {pCurve->GetID()};
    }

SGM::Curve SGM::FindUParamCurve(SGM::Result        &rResult,
                                SGM::Surface const &SurfaceID,
                                double              dU)
    {
    auto pSurface=(SGMInternal::surface *)rResult.GetThing()->FindEntity(SurfaceID.m_ID);
    SGMInternal::curve *pCurve=pSurface->UParamLine(rResult,dU);
    return {pCurve->GetID()};
    }

SGM::Curve SGM::FindVParamCurve(SGM::Result        &rResult,
                                SGM::Surface const &SurfaceID,
                                double              dV)
    {
    auto pSurface=(SGMInternal::surface *)rResult.GetThing()->FindEntity(SurfaceID.m_ID);
    SGMInternal::curve *pCurve=pSurface->VParamLine(rResult,dV);
    return {pCurve->GetID()};
    }

void SGM::FindSimilarFaces(SGM::Result            &rResult,
                           SGM::Face        const &FaceID,
                           std::vector<SGM::Face> &aSimilar,
                           bool                    bIgnoreScale)
{
    auto pFace = (SGMInternal::face *)rResult.GetThing()->FindEntity(FaceID.m_ID);
    std::vector<SGMInternal::face *> aFaces;
    SGMInternal::FindSimilarFaces(rResult, pFace, aFaces, bIgnoreScale);
    for (auto pEnt : aFaces)
    {
        aSimilar.emplace_back(SGM::Face(pEnt->GetID()));
    }
}
