#ifndef SGM_FACET_TO_BREP_H
#define SGM_FACET_TO_BREP_H

#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include <vector>

// Given a vector of points FindConic returns a matching conic curve.
// If no single conic curve fits the given points within the given tolerance,
// then false is returned.  Two sheet hyperolas and double lines are not 
// returned.
// 
// The function returns  a line, circle, ellipse, parabola, or a hyperbola.  
// If more than one curve type can fix the given five points then the first
// curve type on the above list is returned.  

bool FindConic(SGM::Result                     &rResult,
               std::vector<SGM::Point3D> const &aPoints,  // Requires 5 points.
               SGM::Curve                      &ConicID,
               double                           dTolerance);

#endif // SGM_FACET_TO_BREP_H