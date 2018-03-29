#ifndef SGM_COMPLEX_H
#define SGM_COMPLEX_H

#include "SGMEntityClasses.h"
#include "SGMDataClasses.h"

namespace SGM
    {
    // Creation functions

    SGM::Complex CreatePoints(SGM::Result                     &rResult,
                              std::vector<SGM::Point3D> const &aPoints);

    SGM::Complex CreateSegments(SGM::Result                    &rResult,
                                std::vector<SGM::Point3D> const &aPoints,
                                std::vector<size_t>       const &aSegments);

    SGM::Complex CreateTriangles(SGM::Result                    &rResult,
                                 std::vector<SGM::Point3D> const &aPoints,
                                 std::vector<size_t>       const &aTriangles);

    SGM::Complex CreateSlice(SGM::Result             &rResult,
                             SGM::Complex      const &ComplexID,
                             SGM::Point3D      const &Point,
                             SGM::UnitVector3D const &Normal,
                             bool                     bLocal);

    SGM::Complex CreatePolygon(SGM::Result                     &rResult,
                               std::vector<SGM::Point3D> const &aPoints,
                               bool                             bFilled);

    SGM::Complex CreateRectangle(SGM::Result        &rResult,
                                 SGM::Point2D const &Pos0,
                                 SGM::Point2D const &Pos1,
                                 bool                bFilled);

    // Find functions

    size_t FindComponets(SGM::Result               &rResult,
                         SGM::Complex        const &ComplexID,
                         std::vector<SGM::Complex> &aComponents);

    size_t FindBoundary(SGM::Result               &rResult,
                        SGM::Complex        const &ComplexID,
                        std::vector<SGM::Complex> &aBoundary);

    size_t FindGenus(SGM::Result        &rResult,
                     SGM::Complex const &ComplexID);

    // Spliting functions

    size_t SplitWithPlane(SGM::Result               &rResult,
                          SGM::Complex        const &ComplexID,
                          SGM::Point3D        const &Point,
                          SGM::UnitVector3D   const &Normal,
                          std::vector<SGM::Complex> &aComponents);

    size_t SplitWithSlices(SGM::Result                     &rResult,
                           SGM::Complex              const &ComplexID,
                           std::vector<SGM::Complex> const &aSlices,
                           std::vector<SGM::Complex>       &aComponents);

    size_t SplitWithComplex(SGM::Result               &rResult,
                            SGM::Complex        const &ComplexID,
                            SGM::Complex        const &SliceID,
                            std::vector<SGM::Complex> &aComponents);

    } // End of SGM namespace

#endif // SGM_COMPLEX_H