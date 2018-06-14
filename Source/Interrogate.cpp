#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMEntityClasses.h"
#include "SGMMathematics.h"

#include "Topology.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Interrogate.h"
#include "Intersectors.h"

namespace SGMInternal
{

bool PointInVolume(SGM::Result        &rResult,
                   SGM::Point3D const &Point,
                   volume       const *pVolume,
                   double              dTolerance)
    {
    size_t nHits=0;
    bool bFound=true;
    size_t nCount=1;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::UnitVector3D Axis(0,0,1);
    while(bFound)
        {
        bFound=false;
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        nHits=RayFireVolume(rResult,Point,Axis,pVolume,aPoints,aTypes,dTolerance);
        size_t Index1;
        for(Index1=0;Index1<nHits;++Index1)
            {
            if(aTypes[Index1]!=SGM::IntersectionType::PointType)
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
    std::set<volume *,EntityCompare>::const_iterator VolumeIter=sVolumes.begin();
    while(VolumeIter!=sVolumes.end())
        {
        volume *pVolume=*VolumeIter;
        if(PointInVolume(rResult,Point,pVolume,dTolerance))
            {
            return true;
            }
        ++VolumeIter;
        }
    return bAnswer;
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

} // End SGMInternal namespace
