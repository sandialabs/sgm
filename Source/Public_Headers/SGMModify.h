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

// Returns a vector of edges that consist of the given curve trimmed by
// the given face, along with a vector of isolated points where the
// curve intersects the face.

SGM_EXPORT void TrimCurveWithFace(SGM::Result               &rResult,
                                  SGM::Curve                &CurveID,
                                  SGM::Face           const &FaceID,
                                  std::vector<SGM::Edge>    &aEdges,
                                  std::vector<SGM::Point3D> &aPoints);

// Impints the given edge on the given face and returns a vector of the
// resulting faces.  EdgeID is deleted and FaceID is changed.

SGM_EXPORT std::vector<SGM::Face> ImprintEdgeOnFace(SGM::Result &rResult,
                                                    SGM::Edge   &EdgeID,
                                                    SGM::Face   &FaceID);

// Imprint point on the given Topology.  For performance sake the lowest
// known level of topology should be given such as a face or edge.

SGM_EXPORT SGM::Vertex ImprintPoint(SGM::Result        &rResult,
                                    SGM::Point3D const &Pos,
                                    SGM::Topology      &Topology);

// Adds vertices at the seam of closed edges.

SGM_EXPORT void ImprintVerticesOnClosedEdges(SGM::Result &rResult);

// Unhook the given faces and return them in a different body.
/*
SGM_EXPORT SGM::Body UnhookFaces(SGM::Result            &rResult,
                                 std::vector<SGM::Face> &aFaces);
*/

} // End of SGM namespace

#endif