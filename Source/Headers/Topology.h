#ifndef TOPOLOGY_H
#define TOPOLOGY_H

#include "SGMDataClasses.h"
#include "EntityClasses.h"
#include <set>

// Find Functions

void FindBodies(SGM::Result      &rResult,
                entity     const *pEntity,
                std::set<body *> &sBodies,
                bool              bTopLevel);

void FindVolumes(SGM::Result        &rResult,
                 entity       const *pEntity,
                 std::set<volume *> &sVolumes,
                 bool                bTopLevel=false);

void FindFaces(SGM::Result      &rResult,
               entity     const *pEntity,
               std::set<face *> &sFaces,
                bool             bTopLevel=false);

void FindEdges(SGM::Result      &rResult,
               entity     const *pEntity,
               std::set<edge *> &sEdges,
               bool              bTopLevel=false);

void FindVertices(SGM::Result        &rResult,
                  entity       const *pEntity,
                  std::set<vertex *> &sVertices,
                  bool                bTopLevel=false);

void FindComplexes(SGM::Result         &rResult,
                   entity        const *pEntity,
                   std::set<complex *> &sComplexes,
                   bool                 bTopLevel=false);

void FindSurfaces(SGM::Result         &rResult,
                  entity        const *pEntity,
                  std::set<surface *> &sSurfaces,
                  bool                 bTopLevel=false);

void FindCurves(SGM::Result       &rResult,
                entity      const *pEntity,
                std::set<curve *> &sCurves,
                bool               bTopLevel=false);

// Ordering Functions

void OrderLoopEdges(SGM::Result                    &rResult,
                    face                     const *pFace,
                    std::set<edge *>         const &sEdges,
                    std::vector<edge *>            &aEdges,
                    std::vector<SGM::EdgeSideType> &aFlips);

#endif // TOPOLOGY_H
