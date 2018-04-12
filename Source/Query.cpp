#include "SGMQuery.h"
#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "Topology.h"
#include "EntityClasses.h"
#include "Query.h"
#include <cfloat>

void FindClosestPointOnEdge(SGM::Result        &,//rResult,
                            SGM::Point3D const &Point,
                            edge         const *pEdge,
                            SGM::Point3D       &ClosestPoint,
                            entity            *&pCloseEntity)
    {
    curve const *pCurve=pEdge->GetCurve();
    double t=pCurve->Inverse(Point,&ClosestPoint);
    double dEdge=Point.DistanceSquared(ClosestPoint);
    if(pEdge->GetDomain().InInterval(t)==false)
        {
        SGM::EntityType nType=pCurve->GetCurveType();
        switch(nType)
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
    SGM::EntityType nType=pEntity->GetType();
    switch(nType)
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

void SGM::FindClosestPointOnEntity(SGM::Result        &rResult,
                                   SGM::Point3D const &Point,
                                   SGM::Entity  const &EntityID,
                                   SGM::Point3D       &ClosestPoint,
                                   SGM::Entity        &ClosestEntity,
                                   bool                bBoundary)
    {
    thing *pThing=rResult.GetThing();
    entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    entity *pCloseEntity;
    ::FindClosestPointOnEntity(rResult,Point,pEntity,ClosestPoint,pCloseEntity,bBoundary);
    ClosestEntity=Entity(pCloseEntity->GetID());
    }

size_t SGM::FindCloseEdges(SGM::Result            &rResult,
                           SGM::Point3D     const &Point,
                           SGM::Entity      const &EntityID,
                           double                  dMaxDistance,
                           std::vector<SGM::Edge> &aEdges)
    {
    thing *pThing=rResult.GetThing();
    entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    std::set<edge *> sEdges;
    FindEdges(rResult,pEntity,sEdges);
    double dTol=dMaxDistance*dMaxDistance;
    std::set<edge *>::iterator iter=sEdges.begin();
    while(iter!=sEdges.end())
        {
        SGM::Point3D ClosestPoint;
        entity *pCloseEntity;
        edge *pEdge=*iter;
        FindClosestPointOnEdge(rResult,Point,pEdge,ClosestPoint,pCloseEntity);
        if(Point.DistanceSquared(ClosestPoint)<dTol)
            {
            aEdges.push_back(SGM::Edge(pEdge->GetID()));
            }
        ++iter;
        }
    return aEdges.size();
    }

size_t SGM::FindCloseFaces(SGM::Result            &rResult,
                           SGM::Point3D     const &Point,
                           SGM::Entity      const &EntityID,
                           double                  dMaxDistance,
                           std::vector<SGM::Face> &aFaces)
    {
    thing *pThing=rResult.GetThing();
    entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    std::set<face *> sFaces;
    FindFaces(rResult,pEntity,sFaces);
    double dTol=dMaxDistance*dMaxDistance;
    std::set<face *>::iterator iter=sFaces.begin();
    while(iter!=sFaces.end())
        {
        SGM::Point3D ClosestPoint;
        entity *pCloseEntity;
        face *pFace=*iter;
        FindClosestPointOnFace(rResult,Point,pFace,ClosestPoint,pCloseEntity);
        if(Point.DistanceSquared(ClosestPoint)<dTol)
            {
            aFaces.push_back(SGM::Face(pFace->GetID()));
            }
        ++iter;
        }
    return aFaces.size();
    }
