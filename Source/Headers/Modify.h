#ifndef MODIFY_H
#define MODIFY_H

#include "SGMVector.h"

#include "EntityClasses.h"

#include <set>
#include <map>

namespace SGMInternal
{

/*
void UniteBodies(SGM::Result &rResult,
                 body        *pKeepBody,
                 body        *pDeleteBody);
*/

void ReduceToVolumes(SGM::Result                      &rResult,
                     body                             *pBody,
                     std::set<volume *,EntityCompare> &sVolumes);

// pFace1 may be a nullptr and mHitMap(n) returns the edges or vertices
// hit by paramters on the curve on face n.

void TrimCurveWithFaces(SGM::Result               &rResult,
                        curve                     *pCurve,
                        face                const *pFace0,
                        face                const *pFace1, 
                        std::vector<edge *>       &aEdges,
                        std::vector<SGM::Point3D> &aPoints,
                        std::map<double,entity *> &mHitMap1,
                        std::map<double,entity *> &mHitMap2,
                        edge                const *pLimitEdge=nullptr); 

std::vector<face *> ImprintEdgeOnFace(SGM::Result &rResult,
                                      edge        *pEdge,
                                      face        *pFace);

// This version of ImprintEdgeOnFace assumes that the whole edge
// is on the face and the start and end entities are known.

std::vector<face *> ImprintEdgeOnFace(SGM::Result &rResult,
                                      edge        *pEdge,
                                      face        *pFace,
                                      entity      *pStartEntity,
                                      entity      *pEndEntity);

vertex *ImprintPoint(SGM::Result        &rResult,
                     SGM::Point3D const &Pos,
                     topology           *pTopology);

// Imprints the given point on given edge and returns a vertex at the point.

vertex *ImprintPointOnEdge(SGM::Result        &rResult,
                           SGM::Point3D const &Pos,
                           edge               *pEdge);
}

#endif