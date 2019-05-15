#include <SGMEntityFunctions.h>
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

#include <queue>

#ifdef SGM_MULTITHREADED
#include "SGMThreadPool.h"
#endif

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

bool IsRayExpensive(SGM::Result              &rResult,
                    SGM::Point3D      const  &Origin,
                    SGM::UnitVector3D const  &Axis,
                    volume            const  *pVolume,
                    std::vector<void const*> &aHitFaces)
    {
    SGM::BoxTree const &FaceTree = pVolume->GetFaceTree(rResult);
    SGM::Ray3D Ray(Origin, Axis);
    aHitFaces = FaceTree.FindIntersectsRay(Ray);

    for (void const* pVoid : aHitFaces)
        {
        face *pFace = (face *)pVoid;
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
        : m_dCost(0.0), m_Ray(origin,dir), m_aHitFaces()
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

inline bool operator<(RayFaceBoxIntersections const& lhs, RayFaceBoxIntersections const& rhs)
    {
    return lhs.m_dCost < rhs.m_dCost;
    }

double CostOfFaceIntersection(face const* pFace, SGM::Ray3D const &Ray)
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
        }
    else if (SurfaceType == SGM::CylinderType)
        {
        dCost = 10;
        }
    else if (SurfaceType == SGM::TorusType || SurfaceType == SGM::SphereType)
        {
        dCost = 25;
        }
    else if (SurfaceType == SGM::NURBSurfaceType || SurfaceType == SGM::NUBSurfaceType)
        {
        dCost = 1000;
        }
    else if (SurfaceType == SGM::RevolveType)
        {
        auto *pRevolve = dynamic_cast<revolve const*>(pFace->GetSurface());
        curve const *pCurve = pRevolve->GetCurve();
        auto CurveType = pCurve->GetCurveType();
        if (CurveType == SGM::EntityType::LineType)
            {
            dCost = 15;
            }
        else if (CurveType == SGM::EntityType::CircleType)
            {
            dCost = 30;
            }
        else if (CurveType == SGM::EntityType::NUBCurveType || CurveType == SGM::EntityType::NURBCurveType)
            {
            dCost = 1000;
            }
        else
            {
            dCost = 100; // guess
            }
        }
    else if (SurfaceType == SGM::ExtrudeType)
        {
        auto *pExtrude = dynamic_cast<extrude const*>(pFace->GetSurface());
        curve const *pCurve = pExtrude->GetCurve();
        auto CurveType = pCurve->GetCurveType();
        if (CurveType == SGM::EntityType::LineType)
            {
            dCost = 2;
            }
        else if (CurveType == SGM::EntityType::CircleType)
            {
            dCost = 11;
            }
        else if (CurveType == SGM::EntityType::NUBCurveType || CurveType == SGM::EntityType::NURBCurveType)
            {
            dCost = 1000;
            }
        else
            {
            dCost = 10; // guess
            }
        }
    else
        {
        dCost = 100; // guess
        }
    return dCost;
    }

// A Filter that matches:
//      1) BoxTree::Node when the given ray intersects,
//      2) BoxTree::Leaf (containing a face*) when the given ray intersects face->FacetTree.
struct IsIntersectingRayFacetTree
    {
    SGM::Result *m_rResult;
    SGM::Ray3D const *m_pRay;

    IsIntersectingRayFacetTree() = delete;

    inline IsIntersectingRayFacetTree(SGM::Result &rResult, SGM::Ray3D const &Ray)
        : m_rResult(&rResult), m_pRay(&Ray) { }

    inline bool operator()(SGM::BoxTree::Node const * node) const
        {
        return node->m_Bound.IntersectsRay(*m_pRay);
        }

    inline bool operator()(SGM::BoxTree::Leaf const * leaf) const
        {
        if (leaf->m_Bound.IntersectsRay(*m_pRay))
            {
            face* pFace = (face*)leaf->m_pObject;
            if (pFace->GetFacetTree(*m_rResult).AnyIntersectsRay(*m_pRay))
                {
                return true;
                }
            };
        return false;
        }
    };

// fill a vector with faces who have a facet that intersects the given ray

inline void FindFacetsIntersectsRay(SGM::Result &rResult,
                                    SGM::BoxTree const &FaceTree,
                                    SGM::Ray3D const &Ray,
                                    std::vector<face*> &FacesHit)
    {
    FaceTree.Query(IsIntersectingRayFacetTree(rResult,Ray), SGM::BoxTree::PushLeafObject<face>(FacesHit));
    }

RayFaceBoxIntersections FindRayFacesCost(SGM::Result &rResult,
                                         volume const *pVolume,
                                         SGM::Point3D const &Origin,
                                         SGM::UnitVector3D const &Direction)
    {
    SGM::BoxTree const &FaceTree = pVolume->GetFaceTree(rResult);
    RayFaceBoxIntersections RayFaceIntersection(Origin,Direction);
    RayFaceIntersection.m_aHitFaces.reserve(SGM_BOX_MAX_RAY_HITS);
    FindFacetsIntersectsRay(rResult,FaceTree,RayFaceIntersection.m_Ray,RayFaceIntersection.m_aHitFaces);
    for (face* pFace : RayFaceIntersection.m_aHitFaces)
        {
        RayFaceIntersection.m_dCost += CostOfFaceIntersection(pFace, RayFaceIntersection.m_Ray);
        }
    return RayFaceIntersection;
    }

RayFaceBoxIntersections FindCheapRay(SGM::Result                          &rResult,
                                     SGM::Point3D                   const &Point,
                                     volume                         const *pVolume,
                                     std::vector<SGM::UnitVector3D> const &GuessDirections)
    {

//    static const size_t NUM_TEST_RAYS=12; // Icosahedron vertices
//    static const double dA=0.52573111211913360602566908484788; // 1/sqrt(1+G^2)
//    static const double dB=dA*SGM_GOLDEN_RATIO;
//    static double const aaDirections[NUM_TEST_RAYS][3] = {
//        {0, dA, dB},{0,-dA, dB},{0, dA,-dB},{0,-dA,-dB},
//        { dA, dB,0},{-dA, dB,0},{ dA,-dB,0},{-dA,-dB,0},
//        { dB,0, dA},{ dB,0,-dA},{-dB,0, dA},{-dB,0,-dA}
//    };

    //  draw rays through vertices of dodecahedron
    // where \phi is golden ratio

    static const size_t NUM_PRIMARY_RAYS = 8;
    static const size_t NUM_SECONDARY_RAYS  = 12;
    static const size_t NUM_TEST_RAYS = NUM_PRIMARY_RAYS+NUM_SECONDARY_RAYS;

    static const double COST_THRESHOLD_1 = 25.0;   // slightly less than a torus
    static const double COST_THRESHOLD_2 = 1000.0; // cost of a nurb

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

    const size_t NUM_GUESS_RAYS = GuessDirections.size();
    const size_t NUM_TOTAL_RAYS = 2*NUM_GUESS_RAYS + NUM_TEST_RAYS;
    size_t nStartIndex = 0;

    // get estimated cost of a set of rays
    std::vector<RayFaceBoxIntersections> aIntersections;
    aIntersections.reserve(NUM_TOTAL_RAYS);

    // use the given guess directions first
    for (size_t i = 0; i < NUM_GUESS_RAYS; ++i)
        {
        SGM::UnitVector3D const &Direction = GuessDirections[i];
        aIntersections.emplace_back(FindRayFacesCost(rResult, pVolume, Point, Direction));
        if (aIntersections.back().m_dCost == 0)
            return aIntersections.back();
        }

    std::sort(aIntersections.begin(),aIntersections.end());

    nStartIndex += aIntersections.size();

    // try those guesses in the opposite direction
    for (size_t i = 0; i < NUM_GUESS_RAYS; ++i)
        {
        SGM::UnitVector3D Direction(-GuessDirections[i]);
        aIntersections.emplace_back(FindRayFacesCost(rResult, pVolume, Point, Direction));
        if (aIntersections.back().m_dCost < COST_THRESHOLD_1)
            return aIntersections.back();
        }

    std::sort(aIntersections.begin()+nStartIndex,aIntersections.end());

    nStartIndex += aIntersections.size();

    // try harder with additional hard coded ray directions
    for (size_t i = 0; i < NUM_PRIMARY_RAYS; ++i)
        {
        double const *aDirection = aaDirections[i];
        SGM::UnitVector3D Direction;
        SGM::Vector3D &VectorDirection = Direction; // so we can change member data directly
        VectorDirection.m_x = aDirection[0];
        VectorDirection.m_y = aDirection[1];
        VectorDirection.m_z = aDirection[2];
        aIntersections.emplace_back(FindRayFacesCost(rResult, pVolume, Point, Direction));
        auto &rayFaceIntersection = aIntersections.back();
        if (rayFaceIntersection.m_dCost < COST_THRESHOLD_1)
            {
            return rayFaceIntersection;
            }
        }

    // Relax our requirements to a new threshold.
    // See the new threshold is met by any existing rays.
    std::sort(aIntersections.begin(),aIntersections.end());
    if (aIntersections[0].m_dCost < COST_THRESHOLD_2)
        return aIntersections[0];

    nStartIndex += aIntersections.size();

    // add additional rays
    for (size_t i = NUM_PRIMARY_RAYS; i < NUM_TEST_RAYS; ++i)
        {
        double const *aDirection = aaDirections[i];
        SGM::UnitVector3D Direction;
        SGM::Vector3D &VectorDirection = Direction;
        VectorDirection.m_x = aDirection[0];
        VectorDirection.m_y = aDirection[1];
        VectorDirection.m_z = aDirection[2];
        aIntersections.emplace_back(FindRayFacesCost(rResult, pVolume, Point, Direction));
        auto &rayFaceIntersection = aIntersections.back();
        if (rayFaceIntersection.m_dCost < COST_THRESHOLD_2)
            {
            return rayFaceIntersection;
            }
        }

    // return the best we have
    std::sort(aIntersections.begin(), aIntersections.end());
    return aIntersections[0];
    }

// Return an ordering of the lengths of the interval.

inline void FindShortestLengths(SGM::Interval3D const &BoundingBox,
                                unsigned               aShortestLengths[3])
    {
    aShortestLengths[0] = 0;
    aShortestLengths[1] = 1;
    aShortestLengths[2] = 2;

    double aLengths[3] =
        {
            BoundingBox.m_XDomain.Length(),
            BoundingBox.m_YDomain.Length(),
            BoundingBox.m_ZDomain.Length()
        };
    //    SWAP(1, 2);
    //    SWAP(0, 2);
    //    SWAP(0, 1);
    if (aLengths[aShortestLengths[1]] > aLengths[aShortestLengths[2]])
        std::swap(aShortestLengths[1],aShortestLengths[2]);
    if (aLengths[aShortestLengths[0]] > aLengths[aShortestLengths[2]])
        std::swap(aShortestLengths[0],aShortestLengths[2]);
    if (aLengths[aShortestLengths[0]] > aLengths[aShortestLengths[1]])
        std::swap(aShortestLengths[0],aShortestLengths[1]);
    }

void FindGuessDirections(unsigned                       const aShortestLengths[3],
                         SGM::Point3D                   const &Centroid,
                         SGM::Point3D                   const &Point,
                         std::vector<SGM::UnitVector3D>       &GuessDirections)
    {
    SGM::Vector3D CentroidDirection(Point - Centroid);

    // initialize all the entries of all the unit vectors to zero
    assert(GuessDirections.size()==3);
    SGM::Vector3D &StartVector = GuessDirections[0];
    double *pStart = &(StartVector[0]);
    std::fill(pStart,pStart+9, 0.0);

    // put a -1 or 1 in the directions of the Point is away from the Centroid
    // (in order of x,y,z of the the given shortest lengths
    for (unsigned i = 0; i < 3; ++i)
        {
        size_t iDirection = aShortestLengths[i];
        SGM::Vector3D &GuessDirectionVector = GuessDirections[i];
        GuessDirectionVector[iDirection] = std::signbit(CentroidDirection[iDirection]) ? -1.0 : 1.0;
        }
    }

// Return the number of new directions tried, zero if the previous direction was satisfactory

void FindNextRayFaceBoxIntersections(SGM::Result &rResult,
                                     volume const *pVolume,
                                     const unsigned int *aVolumeShortestLengths,
                                     SGM::Point3D const VolumeCentroid,
                                     SGM::Point3D const &Point,
                                     bool bTryPrevious,
                                     double &dCost,
                                     SGM::UnitVector3D &Direction,
                                     size_t &nCountSinceNewRay,
                                     RayFaceBoxIntersections &NextRayFaceBoxIntersections)
    {
    if (bTryPrevious)
        {
        NextRayFaceBoxIntersections = FindRayFacesCost(rResult, pVolume, Point, Direction);
        double dNextCost = NextRayFaceBoxIntersections.m_dCost;  
        if ( dNextCost == 0                                                 ||
            (nCountSinceNewRay <  12 && dNextCost <= std::min(dCost,1000.)) ||
            (nCountSinceNewRay < 128 && dNextCost < 10                    ) ||
            (nCountSinceNewRay <  64 && dNextCost < 20                    ) ||
            (nCountSinceNewRay <  32 && dNextCost < 50                    ) ||
            (nCountSinceNewRay <  16 && dNextCost < 100                   ) ||
            (nCountSinceNewRay <   8 && dNextCost < 200                   ) ||
            (nCountSinceNewRay <   4 && dNextCost < 500                   ) ||
            (nCountSinceNewRay <   2 && dNextCost < 1000                  ))
            {
            ++nCountSinceNewRay; // re-use the previous direction
            dCost = dNextCost;
            return;
            }
        }
    // we are not using the previous direction, find a new one
    nCountSinceNewRay = 0;
    std::vector<SGM::UnitVector3D> GuessDirections(3);
    FindGuessDirections(aVolumeShortestLengths, VolumeCentroid, Point, GuessDirections);
    NextRayFaceBoxIntersections = FindCheapRay(rResult, Point, pVolume, GuessDirections);
    dCost = NextRayFaceBoxIntersections.m_dCost;
    Direction = NextRayFaceBoxIntersections.m_Ray.m_Direction;
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

//#define IS_RAY_IN_VOLUME_LOG

bool IsRayInVolume(SGM::Result                   &rResult,
                   RayFaceBoxIntersections const &RayIntersections,
                   volume                  const *pVolume,
                   double                        dTolerance,
                   bool                          &bReuseDirection,
                   size_t                        &nHits,
                   SGM::Point3D                  &FirstFacePoint)
    {
    SGM::Point3D const &Point = RayIntersections.m_Ray.m_Origin;
    SGM::UnitVector3D Direction(RayIntersections.m_Ray.m_Direction);
    std::vector<face*> aHitFaces = RayIntersections.m_aHitFaces;
#ifdef IS_RAY_IN_VOLUME_LOG
    //std::cout.setf(std::ios::floatfield,std::ios::scientific);
    std::cout << std::setprecision(15);
    std::cout << "P{" << std::setw(19) << Point.m_x << ',' << std::setw(19) << Point.m_y << ',' << std::setw(19) << Point.m_z << '}';
#endif
    bReuseDirection = true;
    size_t nCount = 1;
    size_t nBadRays = 0;
    nHits = 0;
    bool bContinue = true;
    std::vector<SGM::Point3D>          aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    std::vector<entity *>              aEntity;
    while(bContinue)
        {
        bContinue=false;
#ifdef  IS_RAY_IN_VOLUME_LOG
        std::cout << " D{" << Direction.m_x << ',' << Direction.m_y << ',' << Direction.m_z << '}';
#endif
        nHits=RayFireVolume(rResult,Point,Direction,pVolume,aHitFaces,aPoints,aTypes,aEntity,dTolerance,false);
        if (nHits)
            {
            FirstFacePoint = aPoints[0];
            if(SGM::NearEqual(Point,aPoints[0],dTolerance))
                {
#ifdef          IS_RAY_IN_VOLUME_LOG
                std::cout << "on top of " << SGM::EntityTypeName(aEntity[0]->GetType()) << " ID " << aEntity[0]->GetID() << std::endl;
#endif
                return true;
                }
            if(!IsGoodRay(aTypes,aEntity))
                {
                ++nBadRays;
#ifdef          IS_RAY_IN_VOLUME_LOG
                std::cout << " N" << nBadRays;
#endif
                if(5<nBadRays && aEntity[0]->GetType()==SGM::FaceType)
                    {
#ifdef              IS_RAY_IN_VOLUME_LOG
                    std::cout << ' ' << SGM::EntityTypeName(aEntity[0]->GetType()) << " ID " << aEntity[0]->GetID() << std::endl;
#endif
                    bReuseDirection = false;
                    nHits = 0;
                    return PointInVolume(Point,aPoints[0],aEntity[0]);
                    }
                if(20<nBadRays)
                    {
                    return true;
                    }
                // look for a new ray
                size_t nNextMaxCount = nCount+4;
                RayFaceBoxIntersections TempRayIntersections;
                TempRayIntersections.m_dCost = std::numeric_limits<double>::max();
                while (TempRayIntersections.m_dCost > 1000 && nCount < nNextMaxCount)
                    {
                    Direction = SGM::UnitVector3D(cos(nCount), sin(nCount), cos(nCount + 17));
                    ++nCount;
                    TempRayIntersections = FindRayFacesCost(rResult, pVolume, Point, Direction);
                    }
                aHitFaces.swap(TempRayIntersections.m_aHitFaces);
                bContinue=true;
                }
            }
        }
#ifdef IS_RAY_IN_VOLUME_LOG
    std::cout << ' ' << ((nHits%2==1) ? "true" : "false") << std::endl;
#endif
    return nHits%2==1;
    }

inline void FindVolumeTreeLengths(SGM::Result &rResult,
                                  const volume *pVolume,
                                  unsigned int aVolumeShortestLengths[3],
                                  SGM::Point3D &VolumeCentroid)
    {
    VolumeCentroid = pVolume->GetFaceTree(rResult).FindCenterOfMass();
    FindShortestLengths(pVolume->GetBox(rResult), aVolumeShortestLengths);
    }

// A Filter that matches:
//      1) BoxTree::Node when the given bounding box intersects,
//      2) BoxTree::Leaf (containing a face*) when the given bounding box intersects face->FacetTree.
struct IsOverlappingFacetTree
    {
    SGM::Result *m_rResult;
    SGM::Interval3D const *m_pInterval; // implicitly constant will not be changed

    IsOverlappingFacetTree() = delete;

    inline explicit IsOverlappingFacetTree(SGM::Result &rResult, SGM::Interval3D const & bound)
        : m_rResult(&rResult), m_pInterval(&bound) { }

    inline bool operator()(SGM::BoxTree::Node const * node) const
        {
        return m_pInterval->IntersectsBox(node->m_Bound);
        }

    inline bool operator()(SGM::BoxTree::Leaf const * leaf) const
        {
        if (m_pInterval->IntersectsBox(leaf->m_Bound))
            {
            face* pFace = (face*)leaf->m_pObject;
            if (pFace->GetFacetTree(*m_rResult).AnyIntersectsBox(*m_pInterval))
                {
                return true;
                }
            }
        return false;
        }
    };

// A Filter that matches:
//      1) Node when the AABB around Segment overlaps Node box
//      2) Leaf when
//         a) the AABB around Segment overlaps a Leaf box, and
//         c) the Segment intersects that Leaf box
struct IsIntersectingBoxSegment
    {
    SGM::Result *m_pResult;
    SGM::Interval3D const *m_pInterval;
    SGM::Ray3D const *m_pRay;
    double m_dLength;

    IsIntersectingBoxSegment() = delete;

    inline IsIntersectingBoxSegment(SGM::Result &rResult, SGM::Interval3D const & Bound, SGM::Ray3D const & Ray, double dLength)
        : m_pResult(&rResult),
          m_pInterval(&Bound),
          m_pRay(&Ray),
          m_dLength(dLength)
    { }

    inline bool operator()(SGM::BoxTree::Node const * node) const
        {
        return m_pInterval->IntersectsBox(node->m_Bound);
        }

    inline bool operator()(SGM::BoxTree::Leaf const * leaf) const
        {
        if (m_pInterval->IntersectsBox(leaf->m_Bound))
            {
            if (leaf->m_Bound.IntersectsSegment(*m_pRay,m_dLength))
                {
                return true;
                }
            }
        return false;
        }
    };

// A Filter that matches:
//      1) FaceTree Node when the AABB around Segment overlaps Node box
//      2) FaceTree Leaf (containing a face*) when
//         a) the AABB around Segment overlaps a Leaf box, and
//         b) the AABB around Segment overlaps a facet (in face->FacetTree) box, and
//         c) the Segment intersects that facet box
struct IsForestIntersectingSegment
    {
    SGM::Result *m_pResult;
    SGM::Interval3D m_Interval;
    SGM::UnitVector3D m_Direction;
    double m_dLength;
    SGM::Ray3D m_Ray;

    IsForestIntersectingSegment() = delete;

    inline IsForestIntersectingSegment(SGM::Result &rResult, SGM::Point3D const & A, SGM::Point3D const & B)
        : m_pResult(&rResult),
          m_Interval(A,B),
          m_Direction(),
          m_dLength(MakeUnitVector3D(A,B,m_Direction)),
          m_Ray(A,m_Direction)
        { }

    inline bool operator()(SGM::BoxTree::Node const * node) const
        {
        return m_Interval.IntersectsBox(node->m_Bound);
        }

    inline bool operator()(SGM::BoxTree::Leaf const * leaf) const
        {
        if (m_Interval.IntersectsBox(leaf->m_Bound))
            {
            face* pFace = (face*)leaf->m_pObject;
            SGM::BoxTree const & FacetTree = pFace->GetFacetTree(*m_pResult);
            return FacetTree.Query(IsIntersectingBoxSegment(*m_pResult,m_Interval,m_Ray,m_dLength), SGM::BoxTree::FirstLeaf()).m_pObject != nullptr;
            }
        return false;
        }
    };


// True if the given Bound overlaps the box of any facet of faces in the given FaceTree.

inline bool AnyFacetIntersectsBox(SGM::Result &rResult,
                                  SGM::BoxTree const &FaceTree,
                                  SGM::Interval3D &Bound)
    {
    return FaceTree.Query(IsOverlappingFacetTree(rResult,Bound), SGM::BoxTree::FirstLeaf()).m_pObject != nullptr;
    }

inline bool AnyFacetIntersectsSegment(SGM::Result &rResult,
                                      SGM::BoxTree const &FaceTree,
                                      SGM::Point3D const &A,
                                      SGM::Point3D const &B)
    {
    return FaceTree.Query(IsForestIntersectingSegment(rResult,A,B), SGM::BoxTree::FirstLeaf()).m_pObject != nullptr;
    }

// For each point in Morton order, set true or false that the segment formed with the previous point may cross
// one or more faces.

bool PointCrossFacesLoop(SGM::Result &rResult,
                      const volume *pVolume,
                      size_t iBegin,
                      size_t iEnd,
                      const buffer<unsigned int> *p_aIndexOrdered,
                      const std::vector<SGM::Point3D> *p_aPoints,
                      std::vector<bool> *p_aPointCrosses)
    {
    const buffer<unsigned int> &aIndexOrdered = *p_aIndexOrdered;
    const std::vector<SGM::Point3D> &aPoints = *p_aPoints;
    std::vector<bool> &aPointCrosses = *p_aPointCrosses;
    SGM::BoxTree const &FaceTree=pVolume->GetFaceTree(rResult);

    aPointCrosses[iBegin] = true;

#if 1

    // use tree search of the morton segments O(log(N))

    MortonSegmentsTree<SGM::Interval3D,SGM::Point3D> SegmentTree(p_aPoints,p_aIndexOrdered,iBegin,iEnd);

    typedef MortonSegmentsTree<SGM::Interval3D,SGM::Point3D>::Node Node;
    typedef MortonSegmentsTree<SGM::Interval3D,SGM::Point3D>::Leaf Leaf;

    size_t nSegmentsChecked = 0;

//    void LevelOrderTraversal(Node * root)
//        {

        // level order traversal using queue
        Node* root = SegmentTree.m_Root;
        std::queue<Node *> q;
        q.push(root);
        while (!q.empty())
            {
            int n = q.size();
            while (n > 0)
                {
                Node * pNode = q.front();
                q.pop();

                // TODO: save the facets that are intersected, put into queue
                if (AnyFacetIntersectsBox(rResult,FaceTree,pNode->m_Interval))
                    {
                    if (pNode->m_bHasLeaves)
                        {
                        for (unsigned iChild = 0; iChild < pNode->m_nChildren; ++iChild)
                            {
                            Leaf* pLeaf = (Leaf*)pNode->m_aChildren[iChild];

                            // TODO: only look in the facets we saved above
                            if (AnyFacetIntersectsBox(rResult,FaceTree,pLeaf->m_Interval))
                                {
                                for (unsigned i = pLeaf->m_Begin+1; i < pLeaf->m_End; ++i)
                                    {
                                    SGM::Point3D const &NextPoint = aPoints[aIndexOrdered[i]];
                                    SGM::Point3D const &PreviousPoint = aPoints[aIndexOrdered[i - 1]];
                                    if (AnyFacetIntersectsSegment(rResult, FaceTree, PreviousPoint, NextPoint))
                                        {
                                        aPointCrosses[i] = true;
                                        }
                                    ++nSegmentsChecked;
                                    }
                                }
                            }
                        }
                    else
                        {
                        for (unsigned iChild = 0; iChild < pNode->m_nChildren; ++iChild)
                            {
                            Node* pChildNode = (Node*)pNode->m_aChildren[iChild];
                            q.push(pChildNode);
                            }
                        }
                    }
                n--;
                }
            }
//        }


    std::cout << "tree nSegmentsChecked = " << nSegmentsChecked << " out of " << iEnd-iBegin-1 << std::endl;

    // more expensive O(N) algorithm can be used to check the answer with the MortonSegmentsTree
    for (size_t i = iBegin+1; i < iEnd; ++i)
        {
        SGM::Point3D const &NextPoint = aPoints[aIndexOrdered[i]];
        SGM::Point3D const &PreviousPoint = aPoints[aIndexOrdered[i - 1]];
        if (AnyFacetIntersectsSegment(rResult, FaceTree, PreviousPoint, NextPoint))
            {
            if (!aPointCrosses[i])
                {
                std::cout << "aPointCrosses[" << i << "]" << " is not true" << std::endl;
                }
            }
        else
            {
            if (aPointCrosses[i])
                {
                std::cout << "aPointCrosses[" << i << "]" << " is not false" << std::endl;
                }
            }
        }

#else

    // loop over points in Morton order
    for (size_t i = iBegin+1; i < iEnd; ++i)
        {
        SGM::Point3D const &NextPoint = aPoints[aIndexOrdered[i]];
        SGM::Point3D const &PreviousPoint = aPoints[aIndexOrdered[i - 1]];
        SGM::Interval3D Box(PreviousPoint, NextPoint);
        if (AnyFacetIntersectsBox(rResult,FaceTree,Box))
        //if (AnyFacetIntersectsSegment(rResult, FaceTree, PreviousPoint, NextPoint))
            {
            aPointCrosses[i] = true;
            }
        }

#endif










    return true;
    }

bool PointsInVolumeLoop(SGM::Result &rResult,
                        const volume *pVolume,
                        double dTolerance,
                        const unsigned int *aVolumeShortestLengths,
                        const SGM::Point3D *pVolumeCentroid,
                        size_t iBegin,
                        size_t iEnd,
                        const buffer<unsigned int> *p_aIndexOrdered,
                        const std::vector<SGM::Point3D> *p_aPoints,
                        std::vector<bool> *p_aPointCrosses,
                        std::vector<bool> *p_aIsInside)
    {
    // these are passed as pointers in order to avoid copies when using multi-threaded std::future
    const SGM::Point3D &VolumeCentroid = *pVolumeCentroid;
    const buffer<unsigned int> &aIndexOrdered = *p_aIndexOrdered;
    const std::vector<SGM::Point3D> &aPoints = *p_aPoints;
    std::vector<bool> &aPointCrosses = *p_aPointCrosses;
    std::vector<bool> &aIsInside = *p_aIsInside;

    SGM::UnitVector3D Direction;
    SGM::Point3D FirstFacePoint;
    double dCost = 0.0;
    size_t nCountSinceNewRay = 0;
    size_t nHits = 0;
    bool bUsePrevious = false;

    size_t nPointsSolvedByCrossing = 0;
    size_t nPointsFindNextRay = 0;
    size_t nPointsCostZero = 0;
    size_t nPointsRayInVolume = 0;

    assert(aPointCrosses[iBegin]);

    // loop over points in Morton order
    for (size_t i = iBegin; i < iEnd; ++i)
        {
            // use the segment crossing pre-processing
        if (!aPointCrosses[i])
            {
            // we have our answer, it is the same as the previous point
            aIsInside[aIndexOrdered[i]] = aIsInside[aIndexOrdered[i-1]];
            dCost = 0.0;
            bUsePrevious = false;
            nHits = 0;
            ++nCountSinceNewRay;
            ++nPointsSolvedByCrossing;
            continue; // go to the next point
            }

        SGM::Point3D const &NextPoint = aPoints[aIndexOrdered[i]];
        RayFaceBoxIntersections NextRayIntersects;
        FindNextRayFaceBoxIntersections(rResult,
                                        pVolume,
                                        aVolumeShortestLengths,
                                        VolumeCentroid,
                                        NextPoint,
                                        bUsePrevious,
                                        dCost,
                                        Direction,
                                        nCountSinceNewRay,
                                        NextRayIntersects);
        ++nPointsFindNextRay;

        if (dCost == 0)
            {
            bUsePrevious = true;
            nHits = 0;
            ++nPointsCostZero;
            }
        else
            {
            aIsInside[aIndexOrdered[i]] = IsRayInVolume(rResult,
                                                        NextRayIntersects,
                                                        pVolume,
                                                        dTolerance,
                                                        bUsePrevious,
                                                        nHits,
                                                        FirstFacePoint);
            ++nPointsRayInVolume;

            if (bUsePrevious && nHits > 0)
                {
                Direction = FirstFacePoint - NextPoint;
                }
            }
        }

    std::cout << "PointsInVolumeLoop: " <<
                 iEnd - iBegin << " = " <<
                 nPointsSolvedByCrossing << " + (" <<
                 nPointsFindNextRay << " = " <<
                 nPointsCostZero << " + " <<
                 nPointsRayInVolume << ")" << std::endl;
    std::cout.flush();
    return iEnd > iBegin;
    }

std::vector<bool> PointsInVolume(SGM::Result                     &rResult,
                                 std::vector<SGM::Point3D> const &aPoints,
                                 volume                    const *pVolume,
                                 double                           dTolerance)
    {
    unsigned int aVolumeShortestLengths[3];
    SGM::Point3D VolumeCentroid;
    FindVolumeTreeLengths(rResult, pVolume, aVolumeShortestLengths, VolumeCentroid);

    // find an ordering of the points close together
    buffer<unsigned> aIndexOrdered = SGMInternal::OrderPointsMorton(aPoints);

    size_t nPoints = aPoints.size();
    std::vector<bool> aPointCrosses(nPoints,false);
    std::vector<bool> aIsInside(nPoints,false);

#ifdef SGM_MULTITHREADED
    rResult.GetThing()->SetConcurrentActive();
    unsigned nConcurrentThreads = std::thread::hardware_concurrency(); // assume hyperthreads are active and not useful
    nConcurrentThreads = std::max((unsigned) 4, nConcurrentThreads);
    SGM::ThreadPool threadPool(nConcurrentThreads);
    std::vector<std::future<bool>> futures;
    const unsigned NUM_CHUNKS = 8*nConcurrentThreads;
    const size_t CHUNK_SIZE = nPoints / NUM_CHUNKS;

    size_t nRemaining = nPoints % NUM_CHUNKS;
    size_t iBegin = 0;
    size_t iEnd = 0;
    for (unsigned iChunk = 0; iChunk < NUM_CHUNKS; ++iChunk)
        {
        iEnd += (nRemaining > 0) ? (CHUNK_SIZE + ((nRemaining--) != 0)) : CHUNK_SIZE;
        futures.emplace_back(threadPool.enqueue(std::bind(PointCrossFacesLoop,
                                                          rResult,
                                                          pVolume,
                                                          iBegin, iEnd,
                                                          &aIndexOrdered,
                                                          &aPoints,
                                                          &aPointCrosses)));
        iBegin = iEnd;
        }
    for (auto &&future: futures)
        {
        future.wait();
        future.get();
        }
    futures.clear();


    nRemaining = nPoints % NUM_CHUNKS;
    iBegin = 0;
    iEnd = 0;
    for (unsigned iChunk = 0; iChunk < NUM_CHUNKS; ++iChunk)
        {
        iEnd += (nRemaining > 0) ? (CHUNK_SIZE + ((nRemaining--) != 0)) : CHUNK_SIZE;
        futures.emplace_back(threadPool.enqueue(std::bind(PointsInVolumeLoop,
                                                          rResult,
                                                          pVolume,
                                                          dTolerance,
                                                          aVolumeShortestLengths,
                                                          &VolumeCentroid,
                                                          iBegin, iEnd,
                                                          &aIndexOrdered,
                                                          &aPoints,
                                                          &aPointCrosses,
                                                          &aIsInside)));
        iBegin = iEnd;
        }
    for (auto &&future: futures)
        {
        future.wait();
        future.get();
        }
    futures.clear();
    rResult.GetThing()->SetConcurrentInactive();
#else
    PointCrossFacesLoop(rResult,
                        pVolume,
                        0, nPoints,
                        &aIndexOrdered,
                        &aPoints,
                        &aPointCrosses);
    PointsInVolumeLoop(rResult,
                       pVolume,
                       dTolerance,
                       aVolumeShortestLengths,
                       &VolumeCentroid,
                       0, nPoints,
                       &aIndexOrdered,
                       &aPoints,
                       &aPointCrosses,
                       &aIsInside);
#endif
    return aIsInside;
    }


bool PointInVolume(SGM::Result        &rResult,
                   SGM::Point3D const &Point,
                   volume       const *pVolume,
                   double              dTolerance)
    {
    size_t nHits=0;
    bool bFound=true;
    size_t nCount=1;
    size_t nBadRays=0;
    SGM::UnitVector3D Axis(0,0,1);
    if(rResult.GetDebugFlag()==6)
        {
        std::vector<double> aData=rResult.GetDebugData();
        Axis=SGM::UnitVector3D(aData[0],aData[1],aData[2]);
        rResult.SetDebugFlag(0);
        }
    while(bFound)
        {
        std::vector<void const*> aHitFacesTree;
        bool bIsRayExpensive=true;
        size_t nCountRayExpensive = 0;
        while (bIsRayExpensive && nCountRayExpensive<=12)
            {
            aHitFacesTree.clear();
            bIsRayExpensive=IsRayExpensive(rResult,Point,Axis,pVolume,aHitFacesTree);
            if(nCountRayExpensive==12)  
                {
                break;
                }
            if (bIsRayExpensive)
                {
                Axis=SGM::UnitVector3D(cos(nCount),sin(nCount),cos(nCount+17));
                ++nCount;
                ++nCountRayExpensive;
                }
            }
        std::vector<face*> aHitsFaces;
        for (void const* pVoid : aHitFacesTree)
            {
            aHitsFaces.push_back((face*)pVoid);
            }
        bFound=false;
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        std::vector<entity *> aEntity;
        nHits=RayFireVolume(rResult,Point,Axis,pVolume,aHitsFaces,aPoints,aTypes,aEntity,dTolerance,false);
        if(rResult.GetDebugFlag()==7)
            {
            std::vector<double> aData;
            aData.push_back(Axis.X());
            aData.push_back(Axis.Y());
            aData.push_back(Axis.Z());
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
//                std::cout << std::setprecision(16);
//                std::cout << " Origin={" << std::setw(20) << Point.m_x << ',' << std::setw(20) << Point.m_y << ',' << std::setw(20) << Point.m_z << "} ";
//                std::cout << std::setprecision(16);
//                std::cout << " Axis={" << std::setw(20) << Axis.X() << ',' << std::setw(20) << Axis[1] << ',' << std::setw(20) << Axis[2] << "} ";

//                // print the first non-face
//                size_t nSize = aEntity.size();
//                for (size_t Index1 = 0; Index1 < nSize; ++Index1)
//                    {
//                    entity * entity = aEntity[Index1];
//                    if (entity->GetType() != SGM::FaceType)
//                        {
//                        std::cout << SGM::EntityTypeName(entity->GetType()) << " ID " << entity->GetID() << std::endl;
//                        break;
//                        }
//                    }
                ++nBadRays;
                if(5<nBadRays && aEntity[0]->GetType()==SGM::FaceType)
                    {
                    return PointInVolume(Point,aPoints[0],aEntity[0]);
                    }
                if(20<nBadRays)
                    {
                    return PointInVolume(Point,aPoints[0],aEntity[0]);
                    }
                Axis=SGM::UnitVector3D (cos(nCount),sin(nCount),cos(nCount+17));
                ++nCount;
                bFound=true;
                }
            }
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
                      bool                 bCheckScale)
{
    Signature signature = pFace->GetSignature(rResult);

    std::set<face *, EntityCompare> sFaces;
    FindFaces(rResult, rResult.GetThing(), sFaces);

    for (auto pEnt : sFaces)
    {
        if (pEnt != pFace)
        {
            Signature sigCheck = pEnt->GetSignature(rResult);
            if (signature.Matches(sigCheck, bCheckScale))
            {
                aSimilar.emplace_back(pEnt);
            }
        }
    }


}

} // End SGMInternal namespace
