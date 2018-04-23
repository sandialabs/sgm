#ifndef TOPOLOGY_H
#define TOPOLOGY_H

#include "SGMDataClasses.h"
#include "EntityClasses.h"
#include <set>

// Find Functions

void FindBodies(SGM::Result      &rResult,
                entity     const *pEntity,
                std::set<body *> &sBodies);

void FindVolumes(SGM::Result        &rResult,
                 entity       const *pEntity,
                 std::set<volume *> &sVolumes);

void FindFaces(SGM::Result      &rResult,
               entity     const *pEntity,
               std::set<face *> &sFaces);

void FindEdges(SGM::Result      &rResult,
               entity     const *pEntity,
               std::set<edge *> &sEdges);

void FindVertices(SGM::Result        &rResult,
                  entity       const *pEntity,
                  std::set<vertex *> &sVertices);

void FindComplexes(SGM::Result         &rResult,
                   entity        const *pEntity,
                   std::set<complex *> &sComplexes);

void FindSurfaces(SGM::Result         &rResult,
                  entity        const *pEntity,
                  std::set<surface *> &sSurfaces);

void FindCurves(SGM::Result       &rResult,
                entity      const *pEntity,
                std::set<curve *> &sCurves);

// Ordering Functions

void OrderLoopEdges(SGM::Result                    &rResult,
                    face                     const *pFace,
                    std::set<edge *>         const &sEdges,
                    std::vector<edge *>            &aEdges,
                    std::vector<SGM::EdgeSideType> &aFlips);

#endif // TOPOLOGY_H