#ifndef QUERY_H
#define QUERY_H

#include "SGMDataClasses.h"
#include "EntityClasses.h"
namespace SGMInternal
{
void FindClosestPointOnEdge(SGM::Result        &rResult,
                            SGM::Point3D const &Point,
                            edge         const *pEdge,
                            SGM::Point3D       &ClosestPoint,
                            entity            *&pCloseEntity);

void FindClosestPointOnEntity(SGM::Result        &rResult,
                              SGM::Point3D const &Point,
                              entity       const *pEntity,
                              SGM::Point3D       &ClosestPoint,
                              entity            *&pCloseEntity,
                              bool                bBoundary);

void FindClosestPointOnFace(SGM::Result        &rResult,
                            SGM::Point3D const &Point,
                            face         const *pFace,
                            SGM::Point3D       &ClosestPoint,
                            entity            *&pCloseEntity);
}
#endif // QUERY_H
