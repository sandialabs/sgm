#ifndef SGM_DISPLAY_CLASSES_H
#define SGM_DISPLAY_CLASSES_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"

#include <vector>
#include <set>

namespace SGM
    {
    std::vector<SGM::Point3D> const &GetEdgePoints(SGM::Result     &rResult,
                                                   SGM::Edge const &EdgeID);

    std::vector<SGM::Point3D> const &GetFacePoints(SGM::Result     &rResult,
                                                   SGM::Face const &FaceID);

    std::vector<size_t> const &GetFaceTriangles(SGM::Result     &rResult,
                                                SGM::Face const &FaceID);

    std::vector<SGM::UnitVector3D> const &GetFaceNormals(SGM::Result     &rResult,
                                                         SGM::Face const &FaceID);

    SGM::Point3D const &GetPointOfVertex(SGM::Result       &rResult,
                                         SGM::Vertex const &VertexID);

    std::vector<SGM::Point3D> const &GetComplexPoints(SGM::Result        &rResult,
                                                      SGM::Complex const &ComplexID);

    std::vector<size_t> const &GetComplexSegments(SGM::Result        &rResult,
                                                  SGM::Complex const &ComplexID);

    std::vector<size_t> const &GetComplexTriangles(SGM::Result        &rResult,
                                                   SGM::Complex const &ComplexID);

    } // End of SGM namespace

#endif // SGM_DISPLAY_CLASSES_H