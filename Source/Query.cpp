#include "SGMInterrogate.h"
#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "Topology.h"
#include "EntityClasses.h"
#include "Curve.h"
#include "Query.h"
#include "Surface.h"
#include <cfloat>

namespace SGMInternal
{
void FindClosestPointOnEdge(SGM::Result        &,//rResult,
                            SGM::Point3D const &Point,
                            edge         const *pEdge,
                            SGM::Point3D       &ClosestPoint,
                            entity            *&pCloseEntity)
    {
    curve const *pCurve=pEdge->GetCurve();
    double t=pCurve->Inverse(Point,&ClosestPoint);
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
                // Look for local mins.
                throw;
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
    entity *pBoundary=nullptr;
    SGM::Point3D BoundaryPos;
    if(pFace->PointInFace(rResult,uv,&ClosestPoint,&pBoundary,&BoundaryPos))
        {
        dDist=Point.DistanceSquared(ClosestPoint);
        pCloseEntity=(entity *)pFace;
        }
    else
        {
        switch(pSurface->GetSurfaceType())
            {
            case SGM::EntityType::CylinderType:
                {
                break;
                }
            case SGM::EntityType::PlaneType:
                {
                break;
                }
            case SGM::EntityType::SphereType:
                {
                break;
                }
            default:
                {
                // Look for local mins.
                throw;
                }
            }
        }

    // Also check pBoundary.

    if(pBoundary)
        {
        double dBoundaryDist=Point.DistanceSquared(BoundaryPos);
        if(dBoundaryDist<dDist)
            {
            ClosestPoint=BoundaryPos;
            pCloseEntity=pBoundary;
            }
        }
    }

void FindClosestPointOnVolume(SGM::Result        &,//rResult,
                              SGM::Point3D const &,//Point,
                              volume       const *,//pVolume,
                              SGM::Point3D       &,//ClosestPoint,
                              entity            *&,//pCloseEntity,
                              bool                bBoundary)
    {
    if(bBoundary==false)
        {
        // Test for point in volume.
        throw;
        }
    else
        {
        
        }
    }

void FindClosestPointOnBody(SGM::Result        &,//rResult,
                            SGM::Point3D const &,//Point,
                            body         const *,//pBody,
                            SGM::Point3D       &,//ClosestPoint,
                            entity            *&,//pCloseEntity,
                            bool                bBoundary)
    {
    if(bBoundary==false)
        {
        // Test for point in volumes.
        throw;
        }
    else
        {
        
        }
    }

void FindClosestPointOnThing(SGM::Result        &,//rResult,
                             SGM::Point3D const &,//Point,
                             thing        const *,//pThing,
                             SGM::Point3D       &,//ClosestPoint,
                             entity            *&,//pCloseEntity,
                             bool                bBoundary)
    {
    if(bBoundary==false)
        {
        // Test for point in volumes.
        throw;
        }
    else
        {
        
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
            FindClosestPointOnEdge(rResult,Point,(edge const *)pEntity,ClosestPoint,pCloseEntity);
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