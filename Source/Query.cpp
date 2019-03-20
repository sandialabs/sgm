#include "SGMInterrogate.h"
#include "SGMVector.h"
#include "SGMEntityClasses.h"

#include "Topology.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Query.h"
#include "Surface.h"
#include "Interrogate.h"

#include <cfloat>

namespace SGMInternal
{

void FindClosestPointOnEdge3D(SGM::Result        &,//rResult,
                              SGM::Point3D const &Point,
                              edge         const *pEdge,
                              SGM::Point3D       &ClosestPoint,
                              entity            *&pCloseEntity)
    {
    curve const *pCurve=pEdge->GetCurve();
    double t=pCurve->Inverse(Point,&ClosestPoint);
    pEdge->SnapToDomain(t,SGM_MIN_TOL);
    double dEdge=Point.DistanceSquared(ClosestPoint);
    if(pEdge->GetDomain().InInterval(t,SGM_ZERO)==false)
        {
        SGM::EntityType nCurveType=pCurve->GetCurveType();
        switch(nCurveType)
            {
            case SGM::LineType:
                {
                dEdge=DBL_MAX;
                break;
                }
            case SGM::CircleType:
                {
                dEdge=DBL_MAX;
                break;
                }
            default:
                {
                // Look for local mins
                dEdge=DBL_MAX;
                break;
                }
            }
        }
    double dStart=DBL_MAX,dEnd=DBL_MAX;
    if(pEdge->GetStart())
        {
        dStart=Point.DistanceSquared(pEdge->GetStart()->GetPoint());
        }
    if(pEdge->GetEnd())
        {
        dEnd=Point.DistanceSquared(pEdge->GetEnd()->GetPoint());
        }
    if(dStart<=dEnd && dStart<=dEdge)
        {
        ClosestPoint=pEdge->GetStart()->GetPoint();
        pCloseEntity=(entity *)pEdge->GetStart();
        }
    else if(dEnd<=dStart && dEnd<=dEdge)
        {
        ClosestPoint=pEdge->GetEnd()->GetPoint();
        pCloseEntity=(entity *)pEdge->GetEnd();
        }
    else
        {
        pCloseEntity=(entity *)pEdge;
        }
    }

void FindClosestPointOnFace(SGM::Result        &rResult,
                            SGM::Point3D const &Point,
                            face         const *pFace,
                            SGM::Point3D       &ClosestPoint,
                            entity            *&pCloseEntity)
    {
    surface const *pSurface=pFace->GetSurface();
    SGM::Point2D uv=pSurface->Inverse(Point,&ClosestPoint);
    double dDist=DBL_MAX;
    SGM::Point3D BoundaryPos;
    if(pFace->PointInFace(rResult,uv))
        {
        dDist=Point.DistanceSquared(ClosestPoint);
        pCloseEntity=(entity *)pFace;
        }
    std::set<edge *,EntityCompare> const &sEdges=pFace->GetEdges();
    for(auto pEdge : sEdges)
        {
        entity *pEnt;
        SGM::Point3D CPos;
        FindClosestPointOnEdge3D(rResult,Point,pEdge,CPos,pEnt);
        double dEdgeDist=CPos.DistanceSquared(Point);
        if(dEdgeDist<dDist)
            {
            dDist=dEdgeDist;
            pCloseEntity=pEnt;
            ClosestPoint=CPos;
            }
        }
    }

void FindClosestPointOnVolume(SGM::Result        &rResult,
                              SGM::Point3D const &Point,
                              volume       const *pVolume,
                              SGM::Point3D       &ClosestPoint,
                              entity            *&pCloseEntity,
                              bool                bBoundary)
    {
    std::set<face *,EntityCompare> const &sFaces=pVolume->GetFaces();
    SGM::Point3D TestPos(0,0,0);
    entity *TestEnt = nullptr;
    double dMinDist=std::numeric_limits<double>::max();
    for(auto pFace : sFaces)
        {
        FindClosestPointOnFace(rResult,Point,pFace,TestPos,TestEnt);
        double dDist=Point.DistanceSquared(TestPos);
        if(dDist<dMinDist)
            {
            dMinDist=dDist;
            ClosestPoint=TestPos;
            pCloseEntity=TestEnt;
            }
        }
    if(bBoundary==false)
        {
        if(PointInVolume(rResult,Point,pVolume))
            {
            if(SGM_MIN_TOL<dMinDist)
                {
                ClosestPoint=Point;
                pCloseEntity=(entity *)pVolume;
                }
            }
        }
    }

void FindClosestPointOnBody(SGM::Result        &rResult,
                            SGM::Point3D const &Point,
                            body         const *pBody,
                            SGM::Point3D       &ClosestPoint,
                            entity            *&pCloseEntity,
                            bool                bBoundary)
    {
    std::set<volume *,EntityCompare> const &sVolumes=pBody->GetVolumes();
    SGM::Point3D TestPos(0,0,0);
    entity *TestEnt = nullptr;
    double dMinDist=std::numeric_limits<double>::max();
    for(auto pVolume : sVolumes)
        {
        FindClosestPointOnVolume(rResult,Point,pVolume,TestPos,TestEnt,bBoundary);
        double dDist=Point.DistanceSquared(TestPos);
        if(dDist<dMinDist)
            {
            dMinDist=dDist;
            ClosestPoint=TestPos;
            pCloseEntity=TestEnt;
            }
        }
    }

void FindClosestPointOnThing(SGM::Result        &rResult,
                             SGM::Point3D const &Point,
                             thing        const *pThing,
                             SGM::Point3D       &ClosestPoint,
                             entity            *&pCloseEntity,
                             bool                bBoundary)
    {
    std::unordered_set<body *> sBodies=pThing->GetBodies(true);
    SGM::Point3D TestPos(0,0,0);
    entity *TestEnt = nullptr;
    double dMinDist=std::numeric_limits<double>::max();
    for(auto pBody : sBodies)
        {
        FindClosestPointOnBody(rResult,Point,pBody,TestPos,TestEnt,bBoundary);
        double dDist=Point.DistanceSquared(TestPos);
        if(dDist<dMinDist)
            {
            dMinDist=dDist;
            ClosestPoint=TestPos;
            pCloseEntity=TestEnt;
            }
        }
    }

void FindClosestPointOnEntity(SGM::Result        &rResult,
                              SGM::Point3D const &Point,
                              entity       const *pEntity,
                              SGM::Point3D       &ClosestPoint,
                              entity            *&pCloseEntity,
                              bool                bBoundary)
    {
    SGM::EntityType nTopologyType=pEntity->GetType();
    switch(nTopologyType)
        {
        case SGM::ThingType:
            {
            FindClosestPointOnThing(rResult,Point,(thing const *)pEntity,ClosestPoint,pCloseEntity,bBoundary);
            break;
            }
        case SGM::BodyType:
            {
            FindClosestPointOnBody(rResult,Point,(body const *)pEntity,ClosestPoint,pCloseEntity,bBoundary);
            break;
            }
        case SGM::VolumeType:
            {
            FindClosestPointOnVolume(rResult,Point,(volume const *)pEntity,ClosestPoint,pCloseEntity,bBoundary);
            break;
            }
        case SGM::FaceType:
            {
            FindClosestPointOnFace(rResult,Point,(face const *)pEntity,ClosestPoint,pCloseEntity);
            break;
            }
        case SGM::EdgeType:
            {
            FindClosestPointOnEdge3D(rResult,Point,(edge const *)pEntity,ClosestPoint,pCloseEntity);
            break;
            }
        case SGM::VertexType:
            {
            pCloseEntity=(entity *)pEntity;
            ClosestPoint=((vertex const *)pEntity)->GetPoint();
            break;
            }
        default:
            {
            throw;
            }
        }
    }

}