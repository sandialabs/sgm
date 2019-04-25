#ifndef SGM_MODIFY_CLASSES_H
#define SGM_MODIFY_CLASSES_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"

#include <vector>
#include <set>

#include "sgm_export.h"

namespace SGM
{

// Unite the two given bodies

SGM_EXPORT void UniteBodies(SGM::Result &rResult,
                            SGM::Body   &KeepBodyID,
                            SGM::Body   &DeletedBodyID);

// Subtract the deleted body from the returned body.

SGM_EXPORT void SubtractBodies(SGM::Result &rResult,
                               SGM::Body   &KeepBodyID,
                               SGM::Body   &DeletedBodyID);

// Intersect the deleted body from the returned body.

SGM_EXPORT void IntersectBodies(SGM::Result &rResult,
                                SGM::Body   &KeepBodyID,
                                SGM::Body   &DeletedBodyID);

// Imprints the deleted body onto the returned body.

SGM_EXPORT void ImprintBodies(SGM::Result &rResult,
                              SGM::Body   &KeepBodyID,
                              SGM::Body   &DeletedBodyID);

// Strip bodies from volumes.

SGM_EXPORT void ReduceToVolumes(SGM::Result           &rResult,
                                SGM::Body             &BodyID,
                                std::set<SGM::Volume> &sVolumes);

// Returns a vector of edges that consist of the given curve trimmed by
// the given face.

SGM_EXPORT void TrimCurveWithFace(SGM::Result               &rResult,
                                  SGM::Curve                &CurveID,
                                  SGM::Face           const &FaceID,
                                  std::vector<SGM::Edge>    &aEdges);

// Impints the given edge on the given face and returns a vector of the
// resulting faces.  EdgeID is deleted and FaceID is changed.

SGM_EXPORT std::vector<SGM::Face> ImprintEdgeOnFace(SGM::Result &rResult,
                                                    SGM::Edge   &EdgeID,
                                                    SGM::Face   &FaceID);

// Imprint point on the given Topology.  For performance sake the lowest
// known level of topology should be given such as a face or edge.

SGM_EXPORT SGM::Vertex ImprintPoint(SGM::Result        &rResult,
                                    SGM::Point3D const &Pos,
                                    SGM::Topology      &TopologyID);

SGM_EXPORT void Merge(SGM::Result &rResult,
                      SGM::Entity &EntityID);

// Adds vertices at the seam of closed edges.

SGM_EXPORT void ImprintVerticesOnClosedEdges(SGM::Result &rResult);

// Replaces the surface of FaceID with SurfaceID.

SGM_EXPORT void TweakFace(SGM::Result  &rResult,
                          SGM::Face    &FaceID,
                          SGM::Surface &SurfaceID);

class SGM_EXPORT RepairOptions
    {
    public:

        RepairOptions():
            m_bReparamNURBs(true),
            m_bRemoveSlivers(true),
            m_bMerge(true),
            m_bAutoMatch(false),
            m_bExtremeMerge(false),
            m_bRotateCircles(true),
            m_bMakeAllEdgesHaveVertices(false)
            {}

        bool m_bReparamNURBs;
        bool m_bRemoveSlivers;
        bool m_bMerge;
        bool m_bAutoMatch;
        bool m_bExtremeMerge;
        bool m_bRotateCircles;
        bool m_bMakeAllEdgesHaveVertices;
    };

SGM_EXPORT void Repair(SGM::Result            &rResult,
                       std::vector<SGM::Body> &aBodies,
                       SGM::RepairOptions     *pOptions=nullptr);

SGM_EXPORT size_t FindOverLappingEdges(SGM::Result                  &rResult,
                                       std::vector<SGM::Body> const &aBodies,
                                       std::vector<SGM::Edge>       &aEdges);

} // End of SGM namespace

#endif
