#ifndef INTERSECTOR_H
#define INTERSECTOR_H

#include "SGMDataClasses.h"
#include "EntityClasses.h"

size_t IntersectLineAndCurve(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             curve                        const *pCurve,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndLine(SGM::Point3D                 const &Origin,
                            SGM::UnitVector3D            const &Axis,
                            SGM::Interval1D              const &Domain,
                            line                         const *pLine,
                            double                              dTolerance,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndCircle(SGM::Point3D                  const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               circle                       const *pCircle,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndSurface(SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               surface                      const *pSurface,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndCylinder(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                SGM::Interval1D              const &Domain,
                                cylinder                     const *pCylinder,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndPlane(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             plane                        const *pPlane,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndSphere(SGM::Point3D                  const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               sphere                       const *pSphere,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes);

size_t IntersectLineAndTorus(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             torus                        const *pTorus,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes);

// Returns the local minimum distance pair of points, from the given t1, and t2,
// on the the given curves as Pos1, and Pos1.  In addition, the corresponding
// parameters of the two points are returned as t1, and t2.  The functions 
// returns the distance between to two points.

double FindLocalMin(curve  const *pCurve1,    // Input
                    curve  const *pCurve2,    // Input
                    double       &t1,         // Input and output
                    double       &t2,         // Input and output
                    SGM::Point3D &Pos1,       // Output
                    SGM::Point3D &Pos2);      // Output

#endif // INTERSECTOR_H