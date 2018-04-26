#ifndef SGM_TOPOLOGY_CLASSES_H
#define SGM_TOPOLOGY_CLASSES_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"

#include <vector>
#include <set>

#include "sgm_export.h"

namespace SGM
    {
    // To create a thing just use the following code;
    // SGM::Thing MyThing;
    
    SGM_EXPORT void FindBodies(SGM::Result         &rResult,
                               SGM::Entity   const &EntityID,
                               std::set<SGM::Body> &sBodies);

    SGM_EXPORT void FindComplexes(SGM::Result            &rResult,
                                  SGM::Entity      const &EntityID,
                                  std::set<SGM::Complex> &sComplexes);

    SGM_EXPORT void FindVolumes(SGM::Result           &rResult,
                                SGM::Entity     const &EntityID,
                                std::set<SGM::Volume> &sVolumes);

    SGM_EXPORT void FindFaces(SGM::Result         &rResult,
                              SGM::Entity   const &EntityID,
                              std::set<SGM::Face> &sFaces);

    SGM_EXPORT void FindEdges(SGM::Result         &rResult,
                              SGM::Entity   const &EntityID,
                              std::set<SGM::Edge> &sEdges);
    
    SGM_EXPORT void FindVertices(SGM::Result           &rResult,
                                 SGM::Entity     const &EntityID,
                                 std::set<SGM::Vertex> &sVertices);

    } // End of SGM namespace

#endif // SGM_TOPOLOGY_CLASSES_H