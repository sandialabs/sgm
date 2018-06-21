#ifndef SGM_COMPLEX_H
#define SGM_COMPLEX_H

#include "SGMEntityClasses.h"
#include "SGMVector.h"
#include "SGMResult.h"

#include "sgm_export.h"

namespace SGM
    {
    // Creation functions

    SGM_EXPORT SGM::Complex CreatePoints(SGM::Result                     &rResult,
                                         std::vector<SGM::Point3D> const &aPoints);

    SGM_EXPORT SGM::Complex CreateSegments(SGM::Result                    &rResult,
                                           std::vector<SGM::Point3D> const &aPoints,
                                           std::vector<unsigned int> const &aSegments);

    SGM_EXPORT SGM::Complex CreateTriangles(SGM::Result                    &rResult,
                                            std::vector<SGM::Point3D> const &aPoints,
                                            std::vector<unsigned int> const &aTriangles);

    SGM_EXPORT SGM::Complex CreateSlice(SGM::Result             &rResult,
                                        SGM::Complex      const &ComplexID,
                                        SGM::Point3D      const &Point,
                                        SGM::UnitVector3D const &Normal,
                                        bool                     bLocal);

    SGM_EXPORT SGM::Complex CreatePolygon(SGM::Result                     &rResult,
                                          std::vector<SGM::Point3D> const &aPoints,
                                          bool                             bFilled);

    SGM_EXPORT SGM::Complex CreateRectangle(SGM::Result        &rResult,
                                            SGM::Point2D const &Pos0,
                                            SGM::Point2D const &Pos1,
                                            bool                bFilled);

    // Find functions

    SGM_EXPORT size_t FindComponents(SGM::Result               &rResult,
                                     SGM::Complex const        &ComplexID,
                                     std::vector<SGM::Complex> &aComponents);

    SGM_EXPORT size_t FindBoundary(SGM::Result               &rResult,
                                   SGM::Complex        const &ComplexID,
                                   std::vector<SGM::Complex> &aBoundary);

    SGM_EXPORT size_t FindGenus(SGM::Result        &rResult,
                                SGM::Complex const &ComplexID);

    // Splitting functions

    SGM_EXPORT size_t SplitWithPlane(SGM::Result               &rResult,
                                     SGM::Complex        const &ComplexID,
                                     SGM::Point3D        const &Point,
                                     SGM::UnitVector3D   const &Normal,
                                     std::vector<SGM::Complex> &aComponents);

    // Split with slices will split the given two-dimensional complex
    // with the given one-dimensional planar complexes.

    SGM_EXPORT size_t SplitWithSlices(SGM::Result                     &rResult,
                                      SGM::Complex              const &ComplexID,
                                      std::vector<SGM::Complex> const &aSlices,
                                      std::vector<SGM::Complex>       &aComponents);

    // Split with complex will split the given two-dimensional complex
    // with the given one dimensional sub-complex.  That is to say that
    // the vertices and edges of SliceID must be vertices and edges of
    // ComplexID.

    SGM_EXPORT size_t SplitWithComplex(SGM::Result               &rResult,
                                       SGM::Complex        const &ComplexID,
                                       SGM::Complex        const &SliceID,
                                       std::vector<SGM::Complex> &aComponents);

    } // End of SGM namespace

#endif // SGM_COMPLEX_H