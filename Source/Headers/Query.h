#ifndef QUERY_H
#define QUERY_H

#include "SGMVector.h"
#include "EntityClasses.h"
namespace SGMInternal
{
void FindClosestPointOnEdge3D(SGM::Result        &rResult,
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

double CheckFacet(std::vector<SGM::Point3D>      const &aPoints3D,
                  std::vector<SGM::UnitVector3D> const &aNormals,
                  std::vector<unsigned>          const &aTriangles,
                  surface                        const *pSurface,
                  size_t                                nWhere);
}
#endif // QUERY_H
