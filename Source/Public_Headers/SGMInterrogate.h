#ifndef SGM_QUERY_H
#define SGM_QUERY_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMResult.h"
#include "SGMMathematics.h"

#include <vector>
#include <set>

#include "sgm_export.h"

namespace SGM
    {
    // Closest point query functions

    // FindCloseEdges finds the edges of EntityID that are within dMaxDistance
    // of the given Point.  The function returns the number of edges that were
    // found and added to aEdges.

    SGM_EXPORT size_t FindCloseEdges(SGM::Result            &rResult,
                                     SGM::Point3D     const &Point,
                                     SGM::Entity      const &EntityID,
                                     double                  dMaxDistance,
                                     std::vector<SGM::Edge> &aEdges);

    // FindCloseFaces finds the faces of EntityID that are within dMaxDistance
    // of the given Point.  The function returns the number of faces that were
    // found and added to aFaces.

    SGM_EXPORT size_t FindCloseFaces(SGM::Result            &rResult,
                                     SGM::Point3D     const &Point,
                                     SGM::Entity      const &EntityID,
                                     double                  dMaxDistance,
                                     std::vector<SGM::Face> &aFaces);

    // FindClosestPointOnEntity returns both the closest point on EntityID
    // to the given Point, and the lowest level ClosetEntity that Point is
    // on.  For example, if EntityID is a body, then ClosestEntity will be
    // either a body, face, edge, or vertex.  In the event that the point
    // is closest to an isolated point in the body then the body will be 
    // returned as the ClosestEntity.  If bBoundary is true then, only
    // points on the boundary are returned and point in body is not called.

    SGM_EXPORT void FindClosestPointOnEntity(SGM::Result        &rResult,
                                             SGM::Point3D const &Point,
                                             SGM::Entity  const &EntityID,
                                             SGM::Point3D       &ClosestPoint,
                                             SGM::Entity        &ClosestEntity,
                                             bool                bBoundary=true);

    // FindClosetPointBetweenEntities is the same as FindClosestPointOnEntity
    // other than the two entities are used.

    SGM_EXPORT void FindClosetPointBetweenEntities(SGM::Result       &rResult,
                                                   SGM::Entity const &Entity1,
                                                   SGM::Entity const &Entity2,
                                                   SGM::Point3D      &ClosestPoint1,
                                                   SGM::Point3D      &ClosestPoint2,
                                                   SGM::Entity       &ClosestEntity1,
                                                   SGM::Entity       &ClosestEntity2);

    // Point containment functions.

    SGM_EXPORT bool PointInEntity(SGM::Result        &rResult,
                                  SGM::Point3D const &Point,
                                  SGM::Entity  const &EntityID,
                                  double              dTolerance=SGM_MIN_TOL);

    } // End of SGM namespace

#endif // SGM_QUERY_H