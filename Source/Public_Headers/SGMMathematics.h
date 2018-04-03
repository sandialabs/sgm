#ifndef SGM_MATHEMATICS_H
#define SGM_MATHEMATICS_H

#include "SGMDataClasses.h"
#include <vector>

#define SGM_PI      3.1415926535897932384626433832795
#define SGM_TWO_PI  6.283185307179586476925286766559
#define SGM_HALF_PI 1.570796326794896619231321691639
#define SGM_MAX     1E+12
#define SGM_ZERO    1E-12

namespace SGM
    {
    /////////////////////////////////////////////////////////////////////////  
    //
    //  Point vector functions
    //
    /////////////////////////////////////////////////////////////////////////

    SGM::Interval2D FindBoundingBox2D(std::vector<SGM::Point2D> const &aPoints);

    SGM::Interval3D FindBoundingBox3D(std::vector<SGM::Point3D> const &aPoints);

    bool FindLeastSquarePlane(std::vector<SGM::Point3D> const &aPoints,
                              SGM::Point3D                    &Origin,
                              SGM::UnitVector3D               &XVec,
                              SGM::UnitVector3D               &YVec,
                              SGM::UnitVector3D               &ZVec);

    bool FindLeastSquareLine3D(std::vector<SGM::Point3D> const &aPoints,
                               SGM::Point3D                    &Origin,
                               SGM::UnitVector3D               &Axis);

    SGM::Point3D FindCenterOfMass3D(std::vector<SGM::Point3D> const &aPoints);

    SGM::Point3D FindCenterOfMass2D(std::vector<SGM::Point2D> const &aPoints);

    /////////////////////////////////////////////////////////////////////////
    //
    //  Polygon functions
    //
    //  All polygons are given as a vector of Point2Ds that form a closed
    //  cycle.  The end point is assumed to be the start point and should
    //  not be added to the vector. It is assumed that polygons run 
    //  counter clockwise so that the inside of the polygon is on the left.
    //
    /////////////////////////////////////////////////////////////////////////

    //  Cycles that loop counter clockwise return positive area.

    double PolygonArea(std::vector<SGM::Point2D> const &aPolygon);

    size_t FindConcavePoints(std::vector<SGM::Point2D> const &aPolygon,
                             std::vector<size_t>             &aConcavePoints);

    bool PointInPolygon(SGM::Point2D              const &Pos,
                        std::vector<SGM::Point2D> const &aPolygon);

    // Returns a vetor of triangles, indexed into aPoints for the given 
    // polygons.  The first polygon is assumed to be the outside polygon and
    // couter clockwise.  The following polygons are assumed to be inside the
    // first one, disjoint, clockwise.

    void TriangulatePolygon(SGM::Result                             &rResult,
                            std::vector<SGM::Point2D>         const &aPoints,
                            std::vector<std::vector<size_t> > const &aaPolygons,
                            std::vector<size_t>                     &aTriangles,
                            std::vector<size_t>                     &aAdjacences);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Triangles functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns true if D is inside the triangle (A,B,C)

    bool InTriangle(SGM::Point2D const &A,
                    SGM::Point2D const &B,
                    SGM::Point2D const &C,
                    SGM::Point2D const &D);
    
    // Returns true if D is inside the Circumcircle of the triangle (A,B,C).
    // it is assumed that A, B, and C are in counter clockwise order.

    bool InCircumcircle(SGM::Point2D const &A,
                        SGM::Point2D const &B,
                        SGM::Point2D const &C,
                        SGM::Point2D const &D);

    // Given triangles in the form <a0,b0,c0,a1,b1,c1,...>
    // FindAdjacences2D return a vector of the form <Tab0,Tbc0,Tca0,Tab1,Tbc1,Tca1,...>
    // such that Tab0 is the index of the start of the triangle in aTriangles
    // that is adjacent to the frist triangle along the edge ab.
    // If more than one triangle is adjacent to the frist triangle along the same
    // edge for example T0, T1, T2. Then T0 will point to T1, T1 will point to T2
    // and T2 will point to T0.  If an edge does not have a triangle that is
    // adjacent to it then the vector aAdjacency will have the value SIZE_MAX
    // for that edges.

    size_t FindAdjacences2D(std::vector<size_t> const &aTriangles,
                            std::vector<size_t>       &aAdjacency);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Algebra functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns x and y given
    // a1*x+b1*y=c1 and a2*x+b2*y=c2.
    // Returns false if a unique answer does not exist.

    bool CramersRule(double a1,double b1,double c1,
                     double a2,double b2,double c2,
                     double &x,double &y);

    // Returns the determinate of the below matrix
    //
    //  | dA, dB, dC |
    //  | dC, dD, dE |
    //  | dG, dH, dI |

    double Determinate(double dA,double dB,double dC,
                       double dD,double dE,double dF,
                       double dG,double dH,double dI);

    // Given several linear equations in the form
    //
    // a0*x+b0*y+c0*z+...=s0
    // a1*x+b1*y+c1*z+...=s1
    // a2*x+b2*y+c2*z+...=s2
    // ...
    // Where each term in aMatrix is a vector of the form
    // <an,bn,cn,...,sn, with the same number of unknowns as equations,
    // then  LinearSolve will return (x,y,z,...) as the back terms
    // of each term in aMatrix.  If the given matrix is singular, then
    // the function will return false.

    bool LinearSolve(std::vector<std::vector<double> > &aMatrix);

    // Snapps x to -1 or 1 if x is outside the interval [-1,1] so that
    // acos will not return an error.

    double SAFEacos(double x);

    size_t Linear(double a,double b,
                  std::vector<double> aRoots);

    size_t Quadratic(double a,double b,double c,
                     std::vector<double> &aRoots);

    size_t Cubic(double a,double b,double c,double d,
                 std::vector<double> &aRoots);

    size_t Quartic(double a,double b,double c,double d,double e,
                   std::vector<double> &aRoots);

    } // End of SGM namespace

#endif // SGM_MATHEMATICS_H