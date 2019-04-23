#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMEntityClasses.h"
#include "SGMMathematics.h"

#include "Topology.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Interrogate.h"
#include "Intersectors.h"
#include "Signature.h"

#if !defined( _MSC_VER ) || _MSC_VER >= 1900
#define NOEXCEPT noexcept
#define NOEXCEPT_ARGS(ARGS) noexcept((ARGS))
#else
#define NOEXCEPT
#define NOEXCEPT_ARGS(ARGS)
#endif

namespace SGMInternal
{

bool IsGoodRay(std::vector<SGM::IntersectionType> aTypes,
               std::vector<entity *>              aEntity)
{
    size_t nSize = aEntity.size();
    for (size_t Index1 = 0; Index1 < nSize; ++Index1)
        {
        if (aTypes[Index1]!=SGM::IntersectionType::PointType)
            {
            return false;
            }
        if (aEntity[Index1]->GetType()!=SGM::FaceType)
            {
            return false;
            }
        }
    return true;
}

bool IsRayExpensive(SGM::Result                                &rResult,
                    SGM::Point3D      const                    &Origin,
                    SGM::UnitVector3D const                    &Axis,
                    volume            const                    *pVolume,
                    double                                      dTolerance,
                    std::vector<SGM::BoxTree::BoundedItemType> &aHitFaces)
    {
    SGM::BoxTree const &FaceTree = pVolume->GetFaceTree(rResult);
    SGM::Ray3D Ray(Origin, Axis);
    aHitFaces = FaceTree.FindIntersectsRay(Ray, dTolerance);

    // skip if debugging ray firing
    if (rResult.GetDebugFlag()==6)
        {
        return false;
        }

    for (auto boundedItem : aHitFaces)
        {
        face *pFace = (face *) boundedItem.first;
        auto FaceType = pFace->GetSurface()->GetSurfaceType();
        if (FaceType == SGM::NURBSurfaceType || FaceType == SGM::NUBSurfaceType) // || FaceType == SGM::CylinderType)
            {
            return true;
            }
        }
        return false;
    }

struct RayFaceBoxIntersections
    {
    double             m_dCost;
    SGM::Ray3D         m_Ray;
    std::vector<face*> m_aHitFaces;

    RayFaceBoxIntersections()
        : m_dCost(0.0), m_Ray({0,0,0},{1,0,0}), m_aHitFaces()
        {}

    RayFaceBoxIntersections(const RayFaceBoxIntersections& other) = default;

    RayFaceBoxIntersections(RayFaceBoxIntersections&& other) NOEXCEPT
        : m_dCost(other.m_dCost), m_Ray(other.m_Ray), m_aHitFaces(std::move(other.m_aHitFaces))
    {}

    RayFaceBoxIntersections(SGM::Point3D const &origin, SGM::UnitVector3D const &dir)
        : m_Ray(origin,dir), m_dCost(0.0), m_aHitFaces()
    {}

    RayFaceBoxIntersections &operator=(const RayFaceBoxIntersections &other)
        {
        if (&other != this)
            {
            m_dCost = other.m_dCost;
            m_Ray = other.m_Ray;
            m_aHitFaces = other.m_aHitFaces;
            }
        return *this;
        }

    RayFaceBoxIntersections &operator=(RayFaceBoxIntersections &&other) NOEXCEPT
        {
        if (&other != this)
            {
            m_dCost = other.m_dCost;
            m_Ray = other.m_Ray;
            m_aHitFaces = std::move(other.m_aHitFaces);
            }
        return *this;
        }

    ~RayFaceBoxIntersections() = default;
    };

inline bool DoesRayMissFaceFacets(SGM::Result &rResult,face const* pFace, SGM::Ray3D const &Ray)
    {
    auto const & Tree = pFace->GetFacetTree(rResult);
    size_t count = Tree.CountIntersectsRayTight(Ray);
    return count == 0;
    }

double CostOfFaceIntersection(SGM::Result &rResult,face const* pFace, SGM::Ray3D const &Ray)
    {
    double dCost;
    auto SurfaceType = pFace->GetSurface()->GetSurfaceType();

    // use underlying surface type for Offset
    if (SurfaceType == SGM::OffsetType)
        {
        auto *pOffset = dynamic_cast<offset const*>(pFace->GetSurface());
        SurfaceType = pOffset->GetSurface()->GetSurfaceType();
        }

    // increment cost of this surface
    if (SurfaceType == SGM::PlaneType)
        {
        dCost = 1;
        auto pPlane = (plane const *)pFace->GetSurface();
        double dDot = std::abs(pPlane->m_ZAxis % Ray.m_Direction);
        if (dDot < SGM_MIN_TOL)
            {
            dCost = 10000; // glancing hit on surface, assign it to be expensive
            }
        else
            {
            if (DoesRayMissFaceFacets(rResult,pFace,Ray))
                dCost = 0;
            }
        }
    else if (SurfaceType == SGM::CylinderType)
        {
        dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 10;
        }
    else if (SurfaceType == SGM::TorusType || SurfaceType == SGM::SphereType)
        {
        dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 25;
        }
    else if (SurfaceType == SGM::NURBSurfaceType || SurfaceType == SGM::NUBSurfaceType)
        {
        dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 1000;
        }
    else if (SurfaceType == SGM::RevolveType)
        {
        auto *pRevolve = dynamic_cast<revolve const*>(pFace->GetSurface());
        curve const *pCurve = pRevolve->GetCurve();
        auto CurveType = pCurve->GetCurveType();
        if (CurveType == SGM::EntityType::LineType)
            {
            dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 15;
            }
        else if (CurveType == SGM::EntityType::CircleType)
            {
            dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 30;
            }
        else if (CurveType == SGM::EntityType::NUBCurveType || CurveType == SGM::EntityType::NURBCurveType)
            {
            dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 1000;
            }
        else
            {
            dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 100; // guess
            }
        }
    else if (SurfaceType == SGM::ExtrudeType)
        {
        auto *pExtrude = dynamic_cast<extrude const*>(pFace->GetSurface());
        curve const *pCurve = pExtrude->GetCurve();
        auto CurveType = pCurve->GetCurveType();
        if (CurveType == SGM::EntityType::LineType)
            {
            dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 2;
            }
        else if (CurveType == SGM::EntityType::CircleType)
            {
            dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 11;
            }
        else if (CurveType == SGM::EntityType::NUBCurveType || CurveType == SGM::EntityType::NURBCurveType)
            {
            dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 1000;
            }
        else
            {
            dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 10; // guess
            }
        }
    else
        {
        dCost = DoesRayMissFaceFacets(rResult,pFace,Ray) ? 0 : 100; // guess
        }
    return dCost;
    }

RayFaceBoxIntersections FindRayFacesCost(SGM::Result &rResult,
                                         volume const *pVolume,
                                         double dTolerance,
                                         SGM::Point3D const &Origin,
                                         SGM::UnitVector3D const &Direction)
    {
    SGM::BoxTree const &FaceTree = pVolume->GetFaceTree(rResult);
    std::vector<SGM::BoxTree::BoundedItemType> aHitFaces;
    RayFaceBoxIntersections RayFaceIntersection(Origin,Direction);
    aHitFaces = FaceTree.FindIntersectsRay(RayFaceIntersection.m_Ray, dTolerance);
    RayFaceIntersection.m_dCost = 0;
    RayFaceIntersection.m_aHitFaces.clear();
    RayFaceIntersection.m_aHitFaces.reserve(aHitFaces.size());
    for (auto boundedItem : aHitFaces)
        {
        face *pFace = (face *) boundedItem.first;
        double dCost = CostOfFaceIntersection(rResult,pFace,RayFaceIntersection.m_Ray);
        if (dCost > 0)
            {
            RayFaceIntersection.m_dCost += dCost;
            RayFaceIntersection.m_aHitFaces.push_back(pFace);
            }
        }
    return std::move(RayFaceIntersection);
    }

RayFaceBoxIntersections FindCheapRay(SGM::Result                &rResult,
                                      SGM::Point3D       const &Point,
                                      volume             const *pVolume,
                                      double                    dTolerance)
    {
/*
    // construct a set of guess rays in various directions using Icosahedron vertices
#define NUM_TEST_RAYS 12
    static const double dA=0.52573111211913360602566908484788; // 1/sqrt(1+G^2)
    static const double dB=dA*SGM_GOLDEN_RATIO;
    static double const aaDirections[NUM_TEST_RAYS][3] = {
        {0, dA, dB},
        {0,-dA, dB},
        {0, dA,-dB},
        {0,-dA,-dB},
        { dA, dB,0},
        {-dA, dB,0},
        { dA,-dB,0},
        {-dA,-dB,0},
        { dB,0, dA},
        { dB,0,-dA},
        {-dB,0, dA},
        {-dB,0,-dA}
    };
*/
    //  draw rays through vertices of dodecahedron
    // where \phi is golden ratio
#define NUM_TEST_RAYS 20
#define NUM_PRIMARY_RAYS 8
    static const double A = 0.57735026918962576451; // 1 / \sqrt(3)
    static const double B = 0.35682208977308993194; // 1 / (\phi * \sqrt(3)
    static const double C = 0.93417235896271569645; // \phi / \sqrt(3)
    static const double aaDirections[NUM_TEST_RAYS][3] =
        {
            {  A,  A,  A}, { -A, -A, -A},
            {  A,  A, -A}, { -A, -A,  A},
            {  A, -A,  A}, { -A,  A, -A},
            { -A,  A,  A}, {  A, -A, -A},
            {  0,  B,  C}, {  0, -B, -C},
            {  B,  C,  0}, { -B, -C,  0},
            {  C,  0,  B}, { -C,  0, -B},
            {  0,  B, -C}, {  0, -B,  C},
            {  B, -C,  0}, { -B,  C,  0},
            { -C,  0,  B}, {  C,  0, -B}
        };

    double dMinimumCost = std::numeric_limits<double>::max();
    RayFaceBoxIntersections MinimumIntersections;

    for (unsigned i = 0; i < NUM_TEST_RAYS; ++i)
        {
        double const *aDirection = aaDirections[i];
        SGM::UnitVector3D Direction;
        Direction.m_x = aDirection[0];
        Direction.m_y = aDirection[1];
        Direction.m_z = aDirection[2];
        RayFaceBoxIntersections Intersections(FindRayFacesCost(rResult, pVolume, dTolerance, Point, Direction));
        if (Intersections.m_dCost < dMinimumCost)
            {
            dMinimumCost = Intersections.m_dCost;
            MinimumIntersections = std::move(Intersections);
            if (dMinimumCost == 0)
                {
                break; // not going to get better than this, we are finished
                }
            if (i >= NUM_PRIMARY_RAYS-1 && dMinimumCost < 25)
                {
                break; // good enough
                }
            }
        }
    return MinimumIntersections;
    }

void FindRaysForPoints(SGM::Result                           &rResult,
                       std::vector<SGM::Point3D>       const &aPoints,
                       volume                          const *pVolume,
                       double                                 dTolerance,
                       buffer<unsigned>                      &aIndexOrdered,
                       std::vector<RayFaceBoxIntersections>  &aRayFaceBoxIntersections)
    {
    size_t nPoints = aPoints.size();
    aIndexOrdered = SGMInternal::OrderPointsZorder(aPoints);
    aRayFaceBoxIntersections.clear();
    aRayFaceBoxIntersections.reserve(nPoints);

//    for (size_t i = 0; i < nPoints; ++i)
//        {
//        aRayFaceBoxIntersections.emplace_back(FindCheapRay(rResult, aPoints[aIndexOrdered[i]], pVolume, dTolerance));
//        }


    // search for cheapest array for our first point
    RayFaceBoxIntersections FirstRayIntersects = FindCheapRay(rResult, aPoints[aIndexOrdered[0]], pVolume, dTolerance);
    double dCost = FirstRayIntersects.m_dCost;
    SGM::UnitVector3D Direction = FirstRayIntersects.m_Ray.m_Direction;
    aRayFaceBoxIntersections.emplace_back(FirstRayIntersects);

    for (size_t i = 1; i < nPoints; ++i)
        {
        // for followings points
        SGM::Point3D const &NextPoint = aPoints[aIndexOrdered[i]];
        // reuse the last direction
        RayFaceBoxIntersections NextRayIntersects(FindRayFacesCost(rResult, pVolume, dTolerance, NextPoint, Direction));
        if (NextRayIntersects.m_dCost <= dCost)
            {
            // if it is the still cheap use it
            dCost =  NextRayIntersects.m_dCost;
            }
        else
            {
            // unless it is expensive, find a better cheap ray by searching again
            NextRayIntersects = FindCheapRay(rResult, NextPoint, pVolume, dTolerance);
            dCost = NextRayIntersects.m_dCost;
            Direction = NextRayIntersects.m_Ray.m_Direction;
            }
        aRayFaceBoxIntersections.emplace_back(NextRayIntersects);
        }
    }

bool PointInVolume(SGM::Point3D const &Point,
                   SGM::Point3D const &HitPoint,
                   entity       const *pEntity)
    {
    SGM::UnitVector3D UVec(0,0,1);
    if(pEntity->GetType()==SGM::FaceType)
        {
        face *pFace=(face *)pEntity;
        UVec=pFace->FindNormal(Point);
        }
    else
        {
        edge *pEdge=(edge *)pEntity;
        std::set<face *,EntityCompare> const &aFaces=pEdge->GetFaces();
        SGM::Vector3D Vec(0,0,0);
        for(auto pFace : aFaces)
            {
            Vec=Vec+pFace->FindNormal(Point);
            }
        UVec=Vec;
        }
    return UVec%(Point-HitPoint)<0;
    }

bool IsRayInVolume(SGM::Result                   &rResult,
                   RayFaceBoxIntersections const &RayIntersections,
                   volume                  const *pVolume,
                   double                        dTolerance)
    {
    SGM::Point3D const &Point = RayIntersections.m_Ray.m_Origin;
    SGM::UnitVector3D Axis(RayIntersections.m_Ray.m_Direction);
    std::vector<face*> aHitFaces = RayIntersections.m_aHitFaces;
    if(rResult.GetDebugFlag()==6)
        {
        std::vector<double> const &aData=rResult.GetDebugData();
        Axis.m_x=aData[0]; Axis.m_y=aData[1]; Axis.m_z=aData[2];
        }
    size_t nCount=1;
    size_t nHits=0;
    bool bFound=true;
    while(bFound)
        {
        bFound=false;
        std::vector<SGM::Point3D>          aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        std::vector<entity *>              aEntity;
        nHits=RayFireVolume(rResult,Point,Axis,pVolume,aPoints,aTypes,aEntity,dTolerance,false,&aHitFaces);
        if(nHits)
            {
            if(SGM::NearEqual(Point,aPoints[0],dTolerance))
                {
                return true;
                }
            if(!IsGoodRay(aTypes,aEntity))
                {
                if(4<nCount)
                    {
                    return PointInVolume(Point,aPoints[0],aEntity[0]);
                    }
                Axis=SGM::UnitVector3D (cos(nCount),sin(nCount),cos(nCount+17));
                bFound=true;
                }
            }
        ++nCount;
        }
    return nHits%2==1;
    }


std::vector<bool> PointsInVolume(SGM::Result                     &rResult,
                                 std::vector<SGM::Point3D> const &aPoints,
                                 volume                    const *pVolume,
                                 double                           dTolerance)
    {
    buffer<unsigned>                     aIndexOrdered;
    std::vector<RayFaceBoxIntersections> aRayFaceBoxIntersections;
    FindRaysForPoints(rResult,aPoints,pVolume,dTolerance,aIndexOrdered,aRayFaceBoxIntersections);
    size_t nPoints = aPoints.size();
    std::vector<bool> aIsInside(nPoints,false);
    // loop over rays in Z-order
    for (size_t i = 0; i < nPoints; ++i)
        {
        RayFaceBoxIntersections const & RayIntersections = aRayFaceBoxIntersections[i];
        if (RayIntersections.m_dCost != 0)
            {
            aIsInside[aIndexOrdered[i]] = IsRayInVolume(rResult,RayIntersections,pVolume,dTolerance);
            }
        }
    return std::move(aIsInside);
    }

bool PointInVolume(SGM::Result        &rResult,
                   SGM::Point3D const &Point,
                   volume       const *pVolume,
                   double              dTolerance)
    {
    size_t nHits=0;
    bool bFound=true;
    size_t nCount=1;
    SGM::UnitVector3D Axis(0,0,1);
    if(rResult.GetDebugFlag()==6)
        {
        std::vector<double> aData=rResult.GetDebugData();
        Axis.m_x=aData[0];
        Axis.m_y=aData[1];
        Axis.m_z=aData[2];
        }
    while(bFound)
        {
        std::vector<SGM::BoxTree::BoundedItemType> aHitFacesTree;
        bool bIsRayExpensive=true;
        while (nCount<=12 && bIsRayExpensive)
            {
            aHitFacesTree.clear();
            bIsRayExpensive=IsRayExpensive(rResult,Point,Axis,pVolume,dTolerance,aHitFacesTree);
            if (bIsRayExpensive)
                {
                Axis=SGM::UnitVector3D(cos(nCount),sin(nCount),cos(nCount+17));
                ++nCount;
                }
            }
        std::vector<face*> aHitsFaces;
        for (auto const &PairIter : aHitFacesTree)
            {
            aHitsFaces.push_back((face*)PairIter.first);
            }
        bFound=false;
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        std::vector<entity *> aEntity;
        nHits=RayFireVolume(rResult,Point,Axis,pVolume,aPoints,aTypes,aEntity,dTolerance,false,&aHitsFaces);
        if(rResult.GetDebugFlag()==6)
            {
            std::vector<double> aData;
            aData.push_back(Axis.m_x);
            aData.push_back(Axis.m_y);
            aData.push_back(Axis.m_z);
            rResult.SetDebugData(aData);
            }
        if(nHits)
            {
            if(SGM::NearEqual(Point,aPoints[0],dTolerance))
                {
                return true;
                }
            if(!IsGoodRay(aTypes,aEntity))
                {
                if(4<nCount)
                    {
                    return PointInVolume(Point,aPoints[0],aEntity[0]);
                    }
                Axis=SGM::UnitVector3D (cos(nCount),sin(nCount),cos(nCount+17));
                bFound=true;
                }
            }
        ++nCount;
        }

    return nHits%2==1;
    }

bool PointInBody(SGM::Result        &rResult,
                 SGM::Point3D const &Point,
                 body         const *pBody,
                 double              dTolerance)
    {
    bool bAnswer=false;
    std::set<volume *,EntityCompare> const &sVolumes=pBody->GetVolumes();
    for (auto pVolume : sVolumes)
        {
        if(PointInVolume(rResult,Point,pVolume,dTolerance))
            {
            return true;
            }
        }
    return bAnswer;
    }

void PointsInVolumes(SGM::Result                         &rResult,
                     std::vector<SGM::Point3D>     const &aPoints,
                     std::vector<std::vector<volume *> > &aaVolumes,
                     double                               dTolerance)
    {
    thing *pThing=rResult.GetThing();
    std::set<volume *,EntityCompare> sVolumes;
    FindVolumes(rResult,pThing,sVolumes);
    size_t nPoints=aPoints.size();
    aaVolumes.reserve(nPoints);
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        std::vector<volume *> aVolumes;
        for(auto pVolume : sVolumes)
            {
            if(PointInEntity(rResult,Pos,pVolume,dTolerance))
                {
                aVolumes.push_back(pVolume);
                }
            }
        aaVolumes.push_back(aVolumes);
        }
    }

bool PointInEntity(SGM::Result        &rResult,
                   SGM::Point3D const &Point,
                   entity       const *pEntity,
                   double              dTolerance)
    {
    bool bAnswer=false;
    switch(pEntity->GetType())
        {
        case SGM::BodyType:
            {
            bAnswer=PointInBody(rResult,Point,(body const *)pEntity,dTolerance);
            break;
            }
        case SGM::VolumeType:
            {
            bAnswer=PointInVolume(rResult,Point,(volume const *)pEntity,dTolerance);
            break;
            }
        case SGM::FaceType:
            {
            auto pFace=(face const *)pEntity;
            SGM::Point3D ClosePos;
            SGM::Point2D uv=pFace->GetSurface()->Inverse(Point,&ClosePos);
            if(Point.Distance(ClosePos)<dTolerance && pFace->PointInFace(rResult,uv))
                {
                bAnswer=true;
                }
            break;
            }
        case SGM::EdgeType:
            {
            break;
            }
        case SGM::VertexType:
            {
            break;
            }
        case SGM::SurfaceType:
            {
            break;
            }
        case SGM::CurveType:
            {
            break;
            }
        default:
            {
            break;
            }
        }

    return bAnswer;
    }

void FindSimilarFaces(SGM::Result         &rResult,
                      face          const *pFace,
                      std::vector<face *> &aSimilar,
                      bool                 bIgnoreScale)
{
    Signature signature = pFace->GetSignature(rResult);

    std::set<face *, EntityCompare> sFaces;
    FindFaces(rResult, rResult.GetThing(), sFaces);

    for (auto pEnt : sFaces)
    {
        if (pEnt != pFace)
        {
            Signature sigCheck = pEnt->GetSignature(rResult);
            if (signature.Matches(sigCheck, bIgnoreScale))
            {
                aSimilar.emplace_back(pEnt);
            }
        }
    }


}

} // End SGMInternal namespace
