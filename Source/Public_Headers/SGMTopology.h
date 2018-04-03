#ifndef SGM_TOPOLOGY_CLASSES_H
#define SGM_TOPOLOGY_CLASSES_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"

#include <vector>
#include <set>

namespace SGM
    {
    SGM::Thing FindThing(SGM::Result       &rResult,
                         SGM::Entity const &EntityID);

    size_t FindBodies(SGM::Result         &rResult,
                      SGM::Entity   const &EntityID,
                      std::set<SGM::Body> &aBodies);

    size_t FindComplexes(SGM::Result            &rResult,
                         SGM::Entity      const &EntityID,
                         std::set<SGM::Complex> &aComplexes);

    size_t FindVolumes(SGM::Result           &rResult,
                       SGM::Entity     const &EntityID,
                       std::set<SGM::Volume> &aVolumes);

    size_t FindFaces(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<SGM::Face> &aFaces);

    size_t FindEdges(SGM::Result         &rResult,
                     SGM::Entity   const &EntityID,
                     std::set<SGM::Edge> &aEdges);
    
    size_t FindVertices(SGM::Result           &rResult,
                        SGM::Entity     const &EntityID,
                        std::set<SGM::Vertex> &aVertices);

    } // End of SGM namespace

#endif // SGM_TOPOLOGY_CLASSES_H