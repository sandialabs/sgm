#ifndef SGM_COMPLEX_H
#define SGM_COMPLEX_H

#include "SGMEntityClasses.h"
#include "SGMDataClasses.h"

#include "sgm_export.h"

namespace SGM
    {
    // Creation functions

    SGM::Complex SGM_EXPORT CreatePoints(SGM::Result                     &rResult,
                                         std::vector<SGM::Point3D> const &aPoints);

    SGM::Complex SGM_EXPORT CreateSegments(SGM::Result                    &rResult,
                                           std::vector<SGM::Point3D> const &aPoints,
                                           std::vector<size_t>       const &aSegments);

    SGM::Complex SGM_EXPORT CreateTriangles(SGM::Result                    &rResult,
                                            std::vector<SGM::Point3D> const &aPoints,
                                            std::vector<size_t>       const &aTriangles);

    SGM::Complex SGM_EXPORT CreateSlice(SGM::Result             &rResult,
                                        SGM::Complex      const &ComplexID,
                                        SGM::Point3D      const &Point,
                                        SGM::UnitVector3D const &Normal,
                                        bool                     bLocal);

    SGM::Complex SGM_EXPORT CreatePolygon(SGM::Result                     &rResult,
                                          std::vector<SGM::Point3D> const &aPoints,
                                          bool                             bFilled);

    SGM::Complex SGM_EXPORT CreateRectangle(SGM::Result        &rResult,
                                            SGM::Point2D const &Pos0,
                                            SGM::Point2D const &Pos1,
                                            bool                bFilled);

    // Find functions

    size_t SGM_EXPORT FindComponents(SGM::Result               &rResult,
                                     SGM::Complex const        &ComplexID,
                                     std::vector<SGM::Complex> &aComponents);

    size_t SGM_EXPORT FindBoundary(SGM::Result               &rResult,
                                   SGM::Complex        const &ComplexID,
                                   std::vector<SGM::Complex> &aBoundary);

    size_t SGM_EXPORT FindGenus(SGM::Result        &rResult,
                                SGM::Complex const &ComplexID);

    // Splitting functions

    size_t SGM_EXPORT SplitWithPlane(SGM::Result               &rResult,
                                     SGM::Complex        const &ComplexID,
                                     SGM::Point3D        const &Point,
                                     SGM::UnitVector3D   const &Normal,
                                     std::vector<SGM::Complex> &aComponents);

    // Split with slices will split the given two-dimensional complex
    // with the given one-dimensional planar complexes.

    size_t SGM_EXPORT SplitWithSlices(SGM::Result                     &rResult,
                                      SGM::Complex              const &ComplexID,
                                      std::vector<SGM::Complex> const &aSlices,
                                      std::vector<SGM::Complex>       &aComponents);

    // Split with complex will split the given two-dimensional complex
    // with the given one dimensional sub-complex.  That is to say that
    // the vertices and edges of SliceID must be vertices and edges of
    // ComplexID.

    size_t SGM_EXPORT SplitWithComplex(SGM::Result               &rResult,
                                       SGM::Complex        const &ComplexID,
                                       SGM::Complex        const &SliceID,
                                       std::vector<SGM::Complex> &aComponents);

    } // End of SGM namespace

#endif // SGM_COMPLEX_H