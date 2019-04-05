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

namespace SGMInternal
{

bool IsGoodRay(std::vector<SGM::IntersectionType> aTypes,
               std::vector<entity *> aEntity)
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
        std::vector<SGM::BoxTree::BoundedItemType> aHitFaces;
        bool bIsRayExpensive=true;
        while (nCount<=12 && bIsRayExpensive)
            {
            aHitFaces.clear();
            bIsRayExpensive=IsRayExpensive(rResult,Point,Axis,pVolume,dTolerance,aHitFaces);
            if (bIsRayExpensive)
                {
                Axis=SGM::UnitVector3D(cos(nCount),sin(nCount),cos(nCount+17));
                ++nCount;
                }
            }

        bFound=false;
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        std::vector<entity *> aEntity;
        nHits=RayFireVolume(rResult,Point,Axis,pVolume,aPoints,aTypes,aEntity,dTolerance,false,&aHitFaces);
        size_t Index1;
        for(Index1=0;Index1<nHits;++Index1)
            {
            if(SGM::NearEqual(Point,aPoints[Index1],dTolerance))
                {
                return true;
                }
            if(!IsGoodRay(aTypes,aEntity))
                {
                Axis=SGM::UnitVector3D (cos(nCount),sin(nCount),cos(nCount+17));
                bFound=true;
                break;
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
