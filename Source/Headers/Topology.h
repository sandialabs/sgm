#ifndef TOPOLOGY_H
#define TOPOLOGY_H

#include "SGMVector.h"
#include "EntityClasses.h"
#include <set>

namespace SGMInternal
{

//////////////////////////////////////////////////////////////////////////////
//
//  Find Functions
//
//////////////////////////////////////////////////////////////////////////////

void FindBodies(SGM::Result                    &rResult,
                entity                   const *pEntity,
                std::set<body *,EntityCompare> &sBodies,
                bool                            bTopLevel);

void FindVolumes(SGM::Result                      &rResult,
                 entity                     const *pEntity,
                 std::set<volume *,EntityCompare> &sVolumes,
                 bool                              bTopLevel=false);

void FindFaces(SGM::Result                    &rResult,
               entity                   const *pEntity,
               std::set<face *,EntityCompare> &sFaces,
                bool                           bTopLevel=false);

void FindEdges(SGM::Result                    &rResult,
               entity                   const *pEntity,
               std::set<edge *,EntityCompare> &sEdges,
               bool                            bTopLevel=false);

void FindWireEdges(SGM::Result                    &rResult,
                   entity                   const *pEntity,
                   std::set<edge *,EntityCompare> &sEdges);

void FindVertices(SGM::Result                      &rResult,
                  entity                     const *pEntity,
                  std::set<vertex *,EntityCompare> &sVertices,
                  bool                              bTopLevel=false);

void FindComplexes(SGM::Result                       &rResult,
                   entity                      const *pEntity,
                   std::set<complex *,EntityCompare> &sComplexes,
                   bool                               bTopLevel=false);

void FindSurfaces(SGM::Result                       &rResult,
                  entity                      const *pEntity,
                  std::set<surface *,EntityCompare> &sSurfaces,
                  bool                               bTopLevel=false);

void FindCurves(SGM::Result                     &rResult,
                entity                    const *pEntity,
                std::set<curve *,EntityCompare> &sCurves,
                bool                             bTopLevel=false);

// Returns the number of edges that are adjacent to both pVertex1, and
// pVertex2.  If pFace is not nullptr, then only edges on pFace are 
// returned.

size_t FindCommonEdgesFromVertices(SGM::Result         &rResult,
                                   vertex        const *pVertex1,       // Input
                                   vertex        const *pVertex2,       // Input
                                   std::vector<edge *> &aEdges,         // Ouput
                                   face          const *pFace=nullptr); // Optional input

size_t FindCommonEdgesFromFaces(SGM::Result         &rResult,
                                face          const *pFace1,   
                                face          const *pFace2,   
                                std::vector<edge *> &aEdges);

size_t FindEdgesOnFaceAtVertex(SGM::Result         &rResult,
                               vertex        const *pVertex,
                               face          const *pFace,
                               std::vector<edge *> &aEdges);

// Returns all faces including the given one that share a vertex or
// edge with the given face.

size_t FindAdjacentFaces(SGM::Result                    &rResult,
                         face                     const *pFace,
                         std::set<face *,EntityCompare> &sFaces);

// Returns all the faces in the same cell as the one sided face pFace.

void FindFacesOfCell(SGM::Result                    &rResult,
                     face                           *pFace,
                     std::set<face *,EntityCompare> &sFaces);

//////////////////////////////////////////////////////////////////////////////
//
//  Ordering Functions
//
//////////////////////////////////////////////////////////////////////////////

void OrderLoopEdges(SGM::Result                          &rResult,
                    face                           const *pFace,
                    std::set<edge *,EntityCompare> const &sEdges,
                    std::vector<edge *>                  &aEdges,
                    std::vector<SGM::EdgeSideType>       &aFlips);

size_t OrderEdgesAboutVertexOnFace(SGM::Result         &rResult,
                                   vertex        const *pVertex,
                                   face          const *pFace,
                                   std::vector<edge *> &aEdges);

//////////////////////////////////////////////////////////////////////////////
//
//  Modify Functions
//
//////////////////////////////////////////////////////////////////////////////

void ImprintVerticesOnClosedEdges(SGM::Result &rResult);

void RemoveFace(SGM::Result &rResult,
                face        *pFace);

void MergeOutSeams(SGM::Result &rResult,
                   entity      *pEntity);

body *UnhookFaces(SGM::Result         &rResult,
                  std::vector<face *> &aFaces);

//////////////////////////////////////////////////////////////////////////////
//
//  Other functions put here for now.
//
//////////////////////////////////////////////////////////////////////////////

void FindAttributes(SGM::Result                         &rResult,
                    entity                        const *pEntity,
                    std::set<attribute *,EntityCompare> &sAttributes,
                    bool                                 bTopLevel=false);

}
#endif // TOPOLOGY_H
