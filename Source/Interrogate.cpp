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
                    double                    dTolerance,
                    std::vector<void const*> &aHitFaces)
    {
    SGM::BoxTree const &FaceTree = pVolume->GetFaceTree(rResult);
    SGM::Ray3D Ray(Origin, Axis);
    aHitFaces = FaceTree.FindIntersectsRay(Ray, dTolerance);

    // skip if debugging ray firing
    if (rResult.GetDebugFlag()==6)
        {
        return false;
        }

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

inline bool DoesRayMissFaceFacets(SGM::Result &rResult,face const* pFace, SGM::Ray3D const &Ray)
    {
    auto const & Tree = pFace->GetFacetTree(rResult);
    size_t count = Tree.CountIntersectsRayTight(Ray);
    return count == 0;
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

RayFaceBoxIntersections FindRayFacesCost(SGM::Result &rResult,
                                         volume const *pVolume,
                                         double dTolerance,
                                         SGM::Point3D const &Origin,
                                         SGM::UnitVector3D const &Direction)
    {
    SGM::BoxTree const &FaceTree = pVolume->GetFaceTree(rResult);
    std::vector<void const*> aHitFaces;
    RayFaceBoxIntersections RayFaceIntersection(Origin,Direction);
    aHitFaces = FaceTree.FindIntersectsRay(RayFaceIntersection.m_Ray, dTolerance);
    RayFaceIntersection.m_dCost = 0;
    RayFaceIntersection.m_aHitFaces.clear();
    RayFaceIntersection.m_aHitFaces.reserve(aHitFaces.size());
    for (void const* pVoid : aHitFaces)
        {
        face *pFace = (face *)pVoid;
        double dCost = CostOfFaceIntersection(pFace,RayFaceIntersection.m_Ray);
        if (dCost > 0)
            {
            RayFaceIntersection.m_dCost += dCost;
            RayFaceIntersection.m_aHitFaces.push_back(pFace);
            }
        }
    return std::move(RayFaceIntersection);
    }

// Use face facet boxtrees to filter out face missed by a ray.
// Return new cost of the given ray after faces are filtered out.

double RemoveMissedFacesFromRayIntersections(SGM::Result &rResult, RayFaceBoxIntersections &rayFaceIntersections)
    {
    SGM::Ray3D const &ray = rayFaceIntersections.m_Ray;
    std::vector<face*> & aHitFaces = rayFaceIntersections.m_aHitFaces;

    // note: size() of vector may be changed inside loop
    for (long int iFace = 0; iFace < (long int)aHitFaces.size(); ++iFace)
        {
        face *pFace = aHitFaces[iFace];
        if (DoesRayMissFaceFacets(rResult,pFace,ray))
            {
            // remove the face on this ray
            aHitFaces.erase(aHitFaces.begin() + iFace--); // reduces iterator by one
            if (aHitFaces.empty())
                {
                rayFaceIntersections.m_dCost = 0;
                return 0;
                }
            // remove cost of the face on this ray
            rayFaceIntersections.m_dCost -= CostOfFaceIntersection(pFace,ray);
            }
        }
    return rayFaceIntersections.m_dCost;
    }

RayFaceBoxIntersections FindCheapRay(SGM::Result                          &rResult,
                                     SGM::Point3D                   const &Point,
                                     volume                         const *pVolume,
                                     std::vector<SGM::UnitVector3D> const &GuessDirections,
                                     double                                dTolerance)
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
        aIntersections.push_back(FindRayFacesCost(rResult, pVolume, dTolerance, Point, Direction));
        if (aIntersections.back().m_dCost == 0)
            return aIntersections.back();
        }

    std::sort(aIntersections.begin(),aIntersections.end());

    // get better costs by removing faces whose facet trees do not intersect the rays
    for (size_t iRayFaceIntersections = nStartIndex; iRayFaceIntersections < aIntersections.size(); ++iRayFaceIntersections)
        {
        auto &rayFaceIntersection = aIntersections[iRayFaceIntersections];
        if (RemoveMissedFacesFromRayIntersections(rResult, rayFaceIntersection) < COST_THRESHOLD_1)
            return rayFaceIntersection;
        }
    nStartIndex += aIntersections.size();

    // try those guesses in the opposite direction
    for (size_t i = 0; i < NUM_GUESS_RAYS; ++i)
        {
        SGM::UnitVector3D Direction = GuessDirections[i];
        Direction.m_x = -Direction.m_x;
        Direction.m_y = -Direction.m_y;
        Direction.m_z = -Direction.m_z;
        aIntersections.push_back(FindRayFacesCost(rResult, pVolume, dTolerance, Point, Direction));
        if (aIntersections.back().m_dCost == COST_THRESHOLD_1)
            return aIntersections.back();
        }

    std::sort(aIntersections.begin()+nStartIndex,aIntersections.end());
    for (size_t iRayFaceIntersections = nStartIndex; iRayFaceIntersections < aIntersections.size(); ++iRayFaceIntersections)
        {
        auto &rayFaceIntersection = aIntersections[iRayFaceIntersections];
        if (RemoveMissedFacesFromRayIntersections(rResult, rayFaceIntersection) < COST_THRESHOLD_1)
            return rayFaceIntersection;
        }
    nStartIndex += aIntersections.size();

    // try harder with additional hard coded ray directions
    for (size_t i = 0; i < NUM_PRIMARY_RAYS; ++i)
        {
        double const *aDirection = aaDirections[i];
        SGM::UnitVector3D Direction;
        Direction.m_x = aDirection[0];
        Direction.m_y = aDirection[1];
        Direction.m_z = aDirection[2];
        aIntersections.push_back(FindRayFacesCost(rResult, pVolume, dTolerance, Point, Direction));
        auto &rayFaceIntersection = aIntersections.back();
        if (rayFaceIntersection.m_dCost < COST_THRESHOLD_1)
            {
            RemoveMissedFacesFromRayIntersections(rResult, rayFaceIntersection);
            return rayFaceIntersection;
            }
        }

    std::sort(aIntersections.begin()+nStartIndex,aIntersections.end());

    for (size_t iRayFaceIntersections = nStartIndex; iRayFaceIntersections < aIntersections.size(); ++iRayFaceIntersections)
        {
        auto &rayFaceIntersection = aIntersections[iRayFaceIntersections];
        if (RemoveMissedFacesFromRayIntersections(rResult, rayFaceIntersection) < COST_THRESHOLD_1)
            return rayFaceIntersection;
        }

    // Relax our requirements to a new threshold.
    // See Ii the new threshold is met by any existing rays.
    std::sort(aIntersections.begin(),aIntersections.end());
    if (aIntersections[0].m_dCost < COST_THRESHOLD_2)
        return aIntersections[0];

    nStartIndex += aIntersections.size();

    // add additional rays
    for (size_t i = NUM_PRIMARY_RAYS; i < NUM_TEST_RAYS; ++i)
        {
        double const *aDirection = aaDirections[i];
        SGM::UnitVector3D Direction;
        Direction.m_x = aDirection[0];
        Direction.m_y = aDirection[1];
        Direction.m_z = aDirection[2];
        aIntersections.push_back(FindRayFacesCost(rResult, pVolume, dTolerance, Point, Direction));
        auto &rayFaceIntersection = aIntersections.back();
        if (rayFaceIntersection.m_dCost < COST_THRESHOLD_2)
            {
            RemoveMissedFacesFromRayIntersections(rResult, rayFaceIntersection);
            return rayFaceIntersection;
            }
        }

    std::sort(aIntersections.begin()+nStartIndex, aIntersections.end());

    for (size_t iRayFaceIntersections = nStartIndex; iRayFaceIntersections < aIntersections.size(); ++iRayFaceIntersections)
        {
        auto & rayFaceIntersection = aIntersections[iRayFaceIntersections];
        if (RemoveMissedFacesFromRayIntersections(rResult, rayFaceIntersection) < COST_THRESHOLD_2)
            return rayFaceIntersection;
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
    double *pStart = &GuessDirections[0][0];
    std::fill(pStart,pStart+9, 0.0);

    // put a -1 or 1 in the directions of the Point is away from the Centroid
    // (in order of x,y,z of the the given shortest lengths
    for (unsigned i = 0; i < 3; ++i)
        {
        size_t iDirection = aShortestLengths[i];
        GuessDirections[i][iDirection] = std::signbit(CentroidDirection[iDirection]) ? -1.0 : 1.0;
        }
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

    // get info about the volume tree
    unsigned aShortestLengths[3];
    FindShortestLengths(pVolume->GetBox(rResult), aShortestLengths);
    SGM::Point3D Centroid = pVolume->GetFaceTree(rResult).FindCenterOfMass();
    std::vector<SGM::UnitVector3D> GuessDirections(3);

    // search for cheapest array for our first point
    SGM::Point3D const &FirstPoint = aPoints[aIndexOrdered[0]];
    FindGuessDirections(aShortestLengths,Centroid,FirstPoint,GuessDirections);
    RayFaceBoxIntersections FirstRayIntersects = FindCheapRay(rResult, FirstPoint, pVolume, GuessDirections, dTolerance);
    double dCost = FirstRayIntersects.m_dCost;
    SGM::UnitVector3D Direction = FirstRayIntersects.m_Ray.m_Direction;
    aRayFaceBoxIntersections.emplace_back(FirstRayIntersects);

    unsigned nCountSinceNewRay = 0;

    for (size_t i = 1; i < nPoints; ++i)
        {
        // for followings point
        SGM::Point3D const &NextPoint = aPoints[aIndexOrdered[i]];

        // reuse the last direction
        RayFaceBoxIntersections NextRayIntersects(FindRayFacesCost(rResult, pVolume, dTolerance, NextPoint, Direction));

        if (NextRayIntersects.m_dCost > 0)
            {
            SGM::Ray3D const &ray = NextRayIntersects.m_Ray;
            std::vector<face*> & aHitFaces = NextRayIntersects.m_aHitFaces;
            // careful, the size() will be changing
            for (long int iFace = 0; iFace < (long int)aHitFaces.size(); ++iFace)
                {
                face *pFace = aHitFaces[iFace];
                if (DoesRayMissFaceFacets(rResult,pFace,ray))
                    {
                    // remove the face and its contribution to cost on this ray
                    double dFaceCost = CostOfFaceIntersection(pFace,ray);
                    aHitFaces.erase(aHitFaces.begin() + iFace--);
                    NextRayIntersects.m_dCost -= dFaceCost;
                    if (NextRayIntersects.m_dCost == 0)
                        {
                        break;
                        }
                    }
                }
            }

        if (NextRayIntersects.m_dCost <= dCost ||                          // its as cheap as the previous ray
            (NextRayIntersects.m_dCost < 1000 && nCountSinceNewRay < 9)) // and not super expensive
            {
            // re-use the previous direction
            dCost =  NextRayIntersects.m_dCost;
            ++nCountSinceNewRay;
            }
        else
            {
            // cost has increased, find a better cheap ray by searching again
            nCountSinceNewRay = 0;
            FindGuessDirections(aShortestLengths,Centroid,NextPoint,GuessDirections);
            NextRayIntersects = FindCheapRay(rResult, NextPoint, pVolume, GuessDirections, dTolerance);
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
    SGM::UnitVector3D Direction(RayIntersections.m_Ray.m_Direction);
    std::vector<face*> aHitFaces = RayIntersections.m_aHitFaces;
    size_t nCount=1;
    size_t nBadRays=0;
    size_t nHits=0;
    bool bContinue=true;
    while(bContinue)
        {
        bContinue=false;
        std::vector<SGM::Point3D>          aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        std::vector<entity *>              aEntity;
        nHits=RayFireVolume(rResult,Point,Direction,pVolume,aPoints,aTypes,aEntity,dTolerance,false,&aHitFaces);
        if(nHits)
            {
            if(SGM::NearEqual(Point,aPoints[0],dTolerance))
                {
                return true;
                }
            if(!IsGoodRay(aTypes,aEntity))
                {
                ++nBadRays;
                if (nBadRays > 3)
                    {
                    //std::cout << "nBadRays = " << nBadRays << std::endl;
                    return PointInVolume(Point,aPoints[0],aEntity[0]);
                    }
                // look for a new ray
                size_t nNextMaxCount = nCount+4;
                RayFaceBoxIntersections TempRayIntersections;
                TempRayIntersections.m_dCost = std::numeric_limits<double>::max();
                while (TempRayIntersections.m_dCost > 1000 && nCount < nNextMaxCount)
                    {
                    Direction = SGM::UnitVector3D(cos(nCount), sin(nCount), cos(nCount + 17));
                    ++nCount;
                    TempRayIntersections = FindRayFacesCost(rResult, pVolume, dTolerance, Point, Direction);
                    }
                RemoveMissedFacesFromRayIntersections(rResult, TempRayIntersections);
//                if (nBadRays == 1)
//                    {
//                    std::cout << std::setprecision(16);
//                    std::cout << " Origin={" << std::setw(20) << Point.m_x << ',' << std::setw(20) << Point.m_y << ',' << std::setw(20) << Point.m_z << "} ";
//                    std::cout << " Axis={" << std::setw(20) << Direction.m_x << ',' << std::setw(20) << Direction.m_y << ',' << std::setw(20) << Direction.m_z << "} ";
//                    }
//                else if (nBadRays == 4)
//                    {
//                    std::cout << "BadRay " << std::setw(2) << nBadRays << ": << " << std::setw(2) << nCount << " cost = " << TempRayIntersections.m_dCost << std::endl;
//                    }
                aHitFaces.swap(TempRayIntersections.m_aHitFaces);
                bContinue=true;
                }
            }
        }
    //if (nBadRays > 0) std::cout << "nBadRays = " << nBadRays << std::endl;
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

//std::vector<bool> PointsInVolumeConcurrent(SGM::Result                     &rResult,
//                                           std::vector<SGM::Point3D> const &aPoints,
//                                           volume                    const *pVolume,
//                                           double                           dTolerance)
//    {
//    buffer<unsigned>                     aIndexOrdered;
//    std::vector<RayFaceBoxIntersections> aRayFaceBoxIntersections;
//    FindRaysForPointsConcurrent(rResult,aPoints,pVolume,dTolerance,aIndexOrdered,aRayFaceBoxIntersections);
//    size_t nPoints = aPoints.size();
//    std::vector<bool> aIsInside(nPoints,false);
//    // loop over rays in Z-order
//    for (size_t i = 0; i < nPoints; ++i)
//        {
//        RayFaceBoxIntersections const & RayIntersections = aRayFaceBoxIntersections[i];
//        if (RayIntersections.m_dCost != 0)
//            {
//            aIsInside[aIndexOrdered[i]] = IsRayInVolume(rResult,RayIntersections,pVolume,dTolerance);
//            }
//        }
//    return std::move(aIsInside);
//    }

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
        Axis.m_x=aData[0];
        Axis.m_y=aData[1];
        Axis.m_z=aData[2];
        }
    while(bFound)
        {
        std::vector<void const*> aHitFacesTree;
        bool bIsRayExpensive=true;
        size_t nCountRayExpensive = 0;
        while (bIsRayExpensive && nCountRayExpensive<=12)
            {
            aHitFacesTree.clear();
            bIsRayExpensive=IsRayExpensive(rResult,Point,Axis,pVolume,dTolerance,aHitFacesTree);
            if (bIsRayExpensive)
                {
                Axis=SGM::UnitVector3D(cos(nCount),sin(nCount),cos(nCount+17));
                ++nCount;
                ++nCountRayExpensive;
                }
            }
//        if(rResult.GetDebugFlag()==6)
//            {
//            rResult.SetDebugData({Axis.m_x, Axis.m_y, Axis.m_z});
//            }
        std::vector<face*> aHitsFaces;
        for (void const* pVoid : aHitFacesTree)
            {
            aHitsFaces.push_back((face*)pVoid);
            }
        bFound=false;
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        std::vector<entity *> aEntity;
        nHits=RayFireVolume(rResult,Point,Axis,pVolume,aPoints,aTypes,aEntity,dTolerance,false,&aHitsFaces);
//        if(rResult.GetDebugFlag()==6)
//            {
//            std::vector<double> aData;
//            aData.push_back(Axis.m_x);
//            aData.push_back(Axis.m_y);
//            aData.push_back(Axis.m_z);
//            rResult.SetDebugData(aData);
//            }
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
//                std::cout << " Axis={" << std::setw(20) << Axis.m_x << ',' << std::setw(20) << Axis.m_y << ',' << std::setw(20) << Axis.m_z << "} ";

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
                if (nBadRays > 4)
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
