#ifndef SGM_CONSTANTS_H
#define SGM_CONSTANTS_H

// Special Numerical Values

#define SGM_PI      3.1415926535897932384626433832795
#define SGM_TWO_PI  6.283185307179586476925286766559
#define SGM_HALF_PI 1.570796326794896619231321691639

// SGM Tolerances

// Used as an upper bound for the distance to all topology.  As a general
// rule all topology should have distance to the origin well within this bound.

#define SGM_MAX     1E+12

// Used as a lower error bound for all numerical values.  As a general
// rule all algorithms attempt to converge to error values lower than this
// value even though more error may be acceptable.

#define SGM_ZERO    1E-12

// Used as a lower bound on the size and parameters of all non-zero dimensional
// topology and geometry. That is to say that edges, sphere radii, the difference
// between the major and minor axes of an ellipse, the half angle of a cone...
// should all be larger than this value.

#define SGM_MIN_TOL 1E-6

// Used to determine the upper bound of acceptable error of a polynomial
// approximation. The actual upper bound is this number times the length or
// width of the geometry.

#define SGM_FIT     1E-3


#endif //SGM_CONSTANTS_H
