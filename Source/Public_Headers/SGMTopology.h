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

    SGM_EXPORT std::vector<SGM::Entity> FindTopLevelEntities(SGM::Result &rResult);

    SGM_EXPORT void FindBodies(SGM::Result         &rResult,
                               SGM::Entity   const &EntityID,
                               std::set<SGM::Body> &sBodies,
                               bool                 bTopLevel=false);

    SGM_EXPORT SGM::Body FindBody(SGM::Result       &rResult,
                                  SGM::Entity const &EntityID);

    SGM_EXPORT void FindComplexes(SGM::Result            &rResult,
                                  SGM::Entity      const &EntityID,
                                  std::set<SGM::Complex> &sComplexes,
                                  bool                    bTopLevel=false);

    SGM_EXPORT void FindVolumes(SGM::Result           &rResult,
                                SGM::Entity     const &EntityID,
                                std::set<SGM::Volume> &sVolumes,
                                bool                   bTopLevel=false);

    SGM_EXPORT SGM::Volume FindVolume(SGM::Result       &rResult,
                                      SGM::Entity const &EntityID);

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

    SGM_EXPORT void FindWireEdges(SGM::Result         &rResult,
                                  SGM::Entity   const &EntityID,
                                  std::set<SGM::Edge> &sEdges);
    
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
//  General Topology Data Functions.
//
//////////////////////////////////////////////////////////////////////////////

    SGM_EXPORT bool IsSheetBody(SGM::Result     &rResult,
                                SGM::Body const &BodyID);

    SGM_EXPORT bool IsWireBody(SGM::Result     &rResult,
                               SGM::Body const &BodyID);

    SGM_EXPORT SGM::Surface GetSurfaceOfFace(SGM::Result     &rResult,
                                             SGM::Face const &FaceID);

    SGM_EXPORT SGM::Curve GetCurveOfEdge(SGM::Result     &rResult,
                                         SGM::Edge const &EdgeID);

    SGM_EXPORT SGM::Interval1D const &GetDomainOfEdge(SGM::Result     &rResult,
                                                      SGM::Edge const &EdgeID);

    SGM_EXPORT SGM::Point3D const &GetPointOfVertex(SGM::Result       &rResult,
                                                    SGM::Vertex const &VertexID);

    SGM_EXPORT std::vector<SGM::Point3D> const &GetPointsOfBody(SGM::Result     &rResult,
                                                                SGM::Body const &BodyID);

    SGM_EXPORT int GetSidesOfFace(SGM::Result     &rResult,
                                  SGM::Face const &FaceID);

    SGM_EXPORT bool IsFaceFlipped(SGM::Result     &rResult,
                                  SGM::Face const &FaceID);

    SGM_EXPORT double GetToleranceOfEdge(SGM::Result     &rResult,
                                         SGM::Edge const &EdgeID);

//////////////////////////////////////////////////////////////////////////////
//
//  Other Topology Functions.
//
//////////////////////////////////////////////////////////////////////////////

    // Returns all faces including the given one that share a vertex or
    // edge with the given face.

    SGM_EXPORT size_t FindAdjacentFaces(SGM::Result            &rResult,
                                        SGM::Face        const &FaceID,
                                        std::vector<SGM::Face> &aFaces);

    SGM_EXPORT size_t FindCommonEdgesFromFaces(SGM::Result            &rResult,
                                               SGM::Face        const &FaceID1,   
                                               SGM::Face        const &FaceID2,   
                                               std::vector<SGM::Edge> &aEdges);

    } // End of SGM namespace

#endif // SGM_TOPOLOGY_CLASSES_H
