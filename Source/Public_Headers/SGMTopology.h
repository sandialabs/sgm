#ifndef SGM_TOPOLOGY_CLASSES_H
#define SGM_TOPOLOGY_CLASSES_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"

#include <vector>
#include <set>

namespace SGM
    {
    // To create a thing just use the following code;
    // SGM::Thing MyThing;
    
    void FindBodies(SGM::Result         &rResult,
                    SGM::Entity   const &EntityID,
                    std::set<SGM::Body> &sBodies);

    void FindComplexes(SGM::Result            &rResult,
                       SGM::Entity      const &EntityID,
                       std::set<SGM::Complex> &sComplexes);

    void FindVolumes(SGM::Result           &rResult,
                     SGM::Entity     const &EntityID,
                     std::set<SGM::Volume> &sVolumes);

    void FindFaces(SGM::Result         &rResult,
                   SGM::Entity   const &EntityID,
                   std::set<SGM::Face> &sFaces);

    void FindEdges(SGM::Result         &rResult,
                   SGM::Entity   const &EntityID,
                   std::set<SGM::Edge> &sEdges);
    
    void FindVertices(SGM::Result           &rResult,
                      SGM::Entity     const &EntityID,
                      std::set<SGM::Vertex> &sVertices);

    } // End of SGM namespace

#endif // SGM_TOPOLOGY_CLASSES_H