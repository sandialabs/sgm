#ifndef SGM_TOPOLOGY_CLASSES_H
#define SGM_TOPOLOGY_CLASSES_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"

#include <vector>
#include <set>

#include "sgm_export.h"

namespace SGM
    {

//////////////////////////////////////////////////////////////////////////////
//
//  General Topology Traversal Functions.
//
//////////////////////////////////////////////////////////////////////////////

    SGM_EXPORT void FindBodies(SGM::Result         &rResult,
                               SGM::Entity   const &EntityID,
                               std::set<SGM::Body> &sBodies,
                               bool                 bTopLevel=false);

    SGM_EXPORT void FindComplexes(SGM::Result            &rResult,
                                  SGM::Entity      const &EntityID,
                                  std::set<SGM::Complex> &sComplexes,
                                  bool                    bTopLevel=false);

    SGM_EXPORT void FindVolumes(SGM::Result           &rResult,
                                SGM::Entity     const &EntityID,
                                std::set<SGM::Volume> &sVolumes,
                                bool                   bTopLevel=false);

    SGM_EXPORT void FindFaces(SGM::Result         &rResult,
                              SGM::Entity   const &EntityID,
                              std::set<SGM::Face> &sFaces,
                              bool                 bTopLevel=false);

    SGM_EXPORT void FindSurfaces(SGM::Result            &rResult,
                                 SGM::Entity      const &EntityID,
                                 std::set<SGM::Surface> &sSurfaces,
                                 bool                    bTopLevel=false);

    SGM_EXPORT void FindEdges(SGM::Result         &rResult,
                              SGM::Entity   const &EntityID,
                              std::set<SGM::Edge> &sEdges,
                              bool                 bTopLevel=false);
    
    SGM_EXPORT void FindCurves(SGM::Result         &rResult,
                              SGM::Entity    const &EntityID,
                              std::set<SGM::Curve> &sCurves,
                              bool                 bTopLevel=false);

    SGM_EXPORT void FindVertices(SGM::Result           &rResult,
                                 SGM::Entity     const &EntityID,
                                 std::set<SGM::Vertex> &sVertices,
                                 bool                   bTopLevel=false);

//////////////////////////////////////////////////////////////////////////////
//
//  Specific Topology Traversal Functions.
//
//////////////////////////////////////////////////////////////////////////////

    SGM_EXPORT SGM::Surface GetSurfaceOfFace(SGM::Result     &rResult,
                                             SGM::Face const &FaceID);

//////////////////////////////////////////////////////////////////////////////
//
//  Topology Modification Functions.
//
//////////////////////////////////////////////////////////////////////////////

    SGM_EXPORT void ImprintVerticesOnClosedEdges(SGM::Result &rResult);

    } // End of SGM namespace

#endif // SGM_TOPOLOGY_CLASSES_H
