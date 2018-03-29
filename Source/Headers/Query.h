#ifndef QUERY_H
#define QUERY_H

#include "SGMDataClasses.h"
#include "EntityClasses.h"

void FindClosestPointOnEdge(SGM::Result        &rResult,
                            SGM::Point3D const &Point,
                            edge         const *pEdge,
                            SGM::Point3D       &ClosestPoint,
                            entity            *&pCloseEntity);

#endif // QUERY_H