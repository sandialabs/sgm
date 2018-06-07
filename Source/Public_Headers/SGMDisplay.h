#ifndef SGM_DISPLAY_CLASSES_H
#define SGM_DISPLAY_CLASSES_H

#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMResult.h"

#include <vector>
#include <set>

#include "sgm_export.h"

namespace SGM
    {
    SGM_EXPORT std::vector<SGM::Point3D> const &GetEdgePoints(SGM::Result     &rResult,
                                                              SGM::Edge const &EdgeID);

    SGM_EXPORT std::vector<SGM::Point3D> const &GetFacePoints3D(SGM::Result     &rResult,
                                                                SGM::Face const &FaceID);

    SGM_EXPORT std::vector<SGM::Point2D> const &GetFacePoints2D(SGM::Result     &rResult,
                                                                SGM::Face const &FaceID);

    SGM_EXPORT std::vector<size_t> const &GetFaceTriangles(SGM::Result     &rResult,
                                                           SGM::Face const &FaceID);

    SGM_EXPORT size_t FindTriStrips(SGM::Result                       &rResult,
                                    SGM::Face                   const &FaceID,
                                    std::vector<std::vector<size_t> > &aaStrips);

    SGM_EXPORT std::vector<SGM::UnitVector3D> const &GetFaceNormals(SGM::Result     &rResult,
                                                                    SGM::Face const &FaceID);

    SGM_EXPORT SGM::Point3D const &GetPointOfVertex(SGM::Result       &rResult,
                                                    SGM::Vertex const &VertexID);

    SGM_EXPORT std::vector<SGM::Point3D> const &GetComplexPoints(SGM::Result        &rResult,
                                                                 SGM::Complex const &ComplexID);

    SGM_EXPORT std::vector<size_t> const &GetComplexSegments(SGM::Result        &rResult,
                                                             SGM::Complex const &ComplexID);

    SGM_EXPORT std::vector<size_t> const &GetComplexTriangles(SGM::Result        &rResult,
                                                              SGM::Complex const &ComplexID);

    } // End of SGM namespace

#endif // SGM_DISPLAY_CLASSES_H