#ifndef SGM_INTERSECTOR_H
#define SGM_INTERSECTOR_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"

namespace SGM
    {
    size_t RayFire(SGM::Result               &rResult,
                   SGM::Point3D        const &Origin,
                   SGM::UnitVector3D   const &Axis,
                   SGM::Entity         const &EntityID,
                   std::vector<SGM::Point3D> &aPoints);

    size_t IntersectCurves(SGM::Result               &rResult,
                           SGM::Curve          const &CurveID1,
                           SGM::Curve          const &CurveID2,
                           std::vector<SGM::Point3D> &aPoints,
                           SGM::Edge           const *pEdge1=nullptr,
                           SGM::Edge           const *pEdge2=nullptr);

    size_t IntersectCurveAndSurface(SGM::Result               &rResult,
                                    SGM::Curve          const &CurveID,
                                    SGM::Surface        const &SurfaceID,
                                    std::vector<SGM::Point3D> &aPoints,
                                    SGM::Edge           const *pEdge=nullptr,
                                    SGM::Face           const *pFace=nullptr);

    size_t IntersectSegment(SGM::Result               &rResult,
                            SGM::Segment3D      const &Segment,
                            SGM::Entity         const &EntityID,
                            std::vector<SGM::Point3D> &aPoints,
                            std::vector<SGM::Entity>  &aEntity);

    size_t IntersectRectangleWithEdges(SGM::Result               &rResult,
                                       SGM::Interval3D     const &rRectangle,
                                       SGM::Entity         const &EntityID,
                                       std::vector<SGM::Point3D> &aPoints,
                                       std::vector<SGM::Entity>  &aEntity);

    } // End of SGM namespace

#endif // SGM_INTERSECTOR_H