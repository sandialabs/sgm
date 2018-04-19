#ifndef SGM_MATHEMATICS_H
#define SGM_MATHEMATICS_H

#include "SGMDataClasses.h"
#include <vector>

#define SGM_PI      3.1415926535897932384626433832795
#define SGM_TWO_PI  6.283185307179586476925286766559
#define SGM_HALF_PI 1.570796326794896619231321691639
#define SGM_MAX     1E+12
#define SGM_ZERO    1E-12
#define SGM_MIN_TOL 1E-6

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

    SGM::Point2D FindCenterOfMass2D(std::vector<SGM::Point2D> const &aPoints);
    
    SGM::Point3D FindCenterOfMass3D(std::vector<SGM::Point3D> const &aPoints);
    
    // Returns the cumulative cord lengths between the given vector of points.
    // If bNormalize=true, then the lengths are scales to go from zero to one.

    void FindLengths3D(std::vector<SGM::Point3D> const &aPoints,
                       std::vector<double>             &aLengths,
                       bool                             bNormalize=false);

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

    // Returns a vector of triangles, indexed into aPoints for the given
    // polygons.  The first polygon is assumed to be the outside polygon and
    // counter clockwise.  The following polygons are assumed to be inside the
    // first one, disjoint, clockwise.

    void TriangulatePolygon(SGM::Result                             &rResult,
                            std::vector<SGM::Point2D>         const &aPoints,
                            std::vector<std::vector<size_t> > const &aaPolygons,
                            std::vector<size_t>                     &aTriangles,
                            std::vector<size_t>                     &aAdjacencies);

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
    
    // Given triangles in the form <a0,b0,c0,a1,b1,c1,...>
    // FindAdjacences2D return a vector of the form <Tab0,Tbc0,Tca0,Tab1,Tbc1,Tca1,...>
    // such that Tab0 is the index of the start of the triangle in aTriangles
    // that is adjacent to the first triangle along the edge ab.
    // If more than one triangle is adjacent to the first triangle along the same
    // edge for example T0, T1, T2. Then T0 will point to T1, T1 will point to T2
    // and T2 will point to T0.  If an edge does not have a triangle that is
    // adjacent to it then the vector aAdjacency will have the value SIZE_MAX
    // for that edges.

    size_t FindAdjacences2D(std::vector<size_t> const &aTriangles,
                            std::vector<size_t>       &aAdjacency);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Circle functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns false if the given three points are co-linear.  Otherwise the
    // center, normal and radius of a circle that contains the three points
    // is returned.
    
    bool FindCircle(SGM::Point3D const &Pos0,
                    SGM::Point3D const &Pos1,
                    SGM::Point3D const &Pos2,
                    SGM::Point3D       &Center,
                    SGM::UnitVector3D  &Normal,
                    double             &dRadius);
    
    // Returns true if D is inside the Circumcircle of the triangle (A,B,C).
    // it is assumed that A, B, and C are in counter clockwise order.

    bool InCircumcircle(SGM::Point2D const &A,
                        SGM::Point2D const &B,
                        SGM::Point2D const &C,
                        SGM::Point2D const &D);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Linear Algebra Functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns x and y given
    // a1*x+b1*y=c1 and a2*x+b2*y=c2.
    // Returns false if a unique answer does not exist.

    bool CramersRule(double a1,double b1,double c1,
                     double a2,double b2,double c2,
                     double &x,double &y);

    // Gaussian elimination with partial pivoting is used.
    // Given two or more linear equations in the form
    //
    // a0*x+b0*y+c0*z+...=s0
    // a1*x+b1*y+c1*z+...=s1
    // a2*x+b2*y+c2*z+...=s2
    // ...
    // Where each term in aaMatrix is a vector of the form
    // <an,bn,cn,...,sn, with the same number of unknowns as equations,
    // then  LinearSolve will return (x,y,z,...) as the back terms
    // of each term in aaMatrix.  The function as order O(equations^3), 
    // where n is the number of rows.  If the given matrix is singular,
    // then the function will return false.

    bool LinearSolve(std::vector<std::vector<double> > &aaMatrix);

    // Given a banded matrix compressed as follows;
    // 
    // a0*x+b0*y+ 0  +  0 +...=s0      ( 0,a0,b0,s0)
    // a1*x+b1*y+c1*z+  0 +...=s1  =>  (a1,b1,c1,s1)
    //  0  +b2*y+c2*z+d2*w+...=s2      (b2,c2,d2,s2)
    //  0  + 0  +c2*z+d2*w+...=s2      ...
    // ...                             (wn,xn, 0,sn)
    //
    // The values (a0,b0,c0,...) are returned in the last column.
    // If the given matrix is singular, then the function will return false.
    // This function runs in linear time O(equations*bandwidth^2) assuming the 
    // bandwidth is relatively small to the number of equations.

    bool BandedSolve(std::vector<std::vector<double> > &aaMatrix);

    // Returns the determinate of a 2 by 2 matrix[row][column].

    double Determinate2D(double const aaMatrix[2][2]);

    // Returns the determinate of a 3 by 3 matrix[row][column].

    double Determinate3D(double const aaMatrix[3][3]);

    // Returns the trace of a two by two matrix[row][column].

    double Trace2D(double const aaMatrix[2][2]);

    // Returns the trace of a three by three matrix[row][column].

    double Trace3D(double const aaMatrix[3][3]);

    // Returns the product of two two by two matrices, A*B=C

    void FindProduct2D(double const aaMatrix1[2][2], // A
                       double const aMatrix2[2][2],  // B
                       double       aAnswer[2][2]);  // C

    // Returns the product of two three by three matrices, A*B=C

    void FindProduct3D(double const aaMatrix1[3][3], // A
                       double const aMatrix2[3][3],  // B
                       double       aAnswer[3][3]);  // C

    // Returns the characteristic polynomial of a two by two 
    // matrix[row][column] in the form a*x^2+b*x+c.

    void CharacteristicPolynomial2D(double const aaMatrix[2][2],
                                    double a,double b,double c);

    // Returns the characteristic polynomial of a three by three 
    // matrix[row][column] in the form a*x^3+b*x^2+c*x+d.

    void CharacteristicPolynomial3D(double const aaMatrix[3][3],
                                    double a,double b,double c,double d);

    // Returns true if the given matrix is a diagonal matrix.

    bool IsDiagonal2D(double const aaMatrix[2][2]);

    // Returns true if the given matrix is a diagonal matrix.

    bool IsDiagonal3D(double const aaMatrix[3][3]);

    // Returns the Eigen vectors and values of a two by two
    // matrix[row][column].

    size_t FindEigenVectors2D(double              const aaMatrix[2][2],
                              std::vector<double>       &aValues,
                              std::vector<UnitVector2D> &aVectors);

    // Returns the Eigen vectors and values of a three by three
    // matrix[row][column].

    size_t FindEigenVectors3D(double              const aaMatrix[3][3],
                              std::vector<double>       &aValues,
                              std::vector<UnitVector3D> &aVectors);
    
    ///////////////////////////////////////////////////////////////////////////
    //
    //  Polynomials Functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Return the real roots of a linear, quadratic, cubic, and quartic 
    // equation.The returned roots are ordered from smallest to largest.  The  
    // coefficients are ordered from largest degree to smallest.  The function
    // returns the number of real roots.

    size_t Linear(double a,double b,
                  std::vector<double> aRoots);

    size_t Quadratic(double a,double b,double c,
                     std::vector<double> &aRoots);

    size_t Cubic(double a,double b,double c,double d,
                 std::vector<double> &aRoots);

    size_t Quartic(double a,double b,double c,double d,double e,
                   std::vector<double> &aRoots);

    // Given a vector of N points in the XY-plane PolynomialFit returns
    // the coefficients of a degree N-1 polynomial that passes through the
    // given points.  For examples if four points are given, then a degree
    // three polynomial with coefficients (a,b,c,d) of the form 
    // a*x^3+b*x^2+c*x+d=y will be returned.  If any two points have the
    // same x coordinate, then false will be returned.

    bool PolynomialFit(std::vector<SGM::Point2D> aPoints,
                       std::vector<double>       aCoefficients);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Trigonometry functions
    //
    ///////////////////////////////////////////////////////////////////////////
    
    // Snaps x to -1 or 1 if x is outside the interval [-1,1] so that
    // acos will not return an error.

    double SAFEacos(double x);

    // Returns zero if both y and x are zero so that atan2 will not return 
    // an error.

    double SAFEatan2(double y,double x);
    
    ///////////////////////////////////////////////////////////////////////////
    //
    //  Calculus functions
    //
    ///////////////////////////////////////////////////////////////////////////
    
    // Returns a numeric first derivative of f(x), with error of order h^5,   
    // given, f(x-2h), f(x-h), f(x+h), f(x+2h), and h.  The function is
    // templated two work with {double, Point2D, Point3D, or Point4D} as
    // the Pos type, and {double, Vector2D, Vector3D, or Vector4D} as the
    // Vec type.

    template<class Pos, class Vec>
    Vec FirstDerivative(Pos const &fxm2h,
                        Pos const &fxmh,
                        Pos const &fxph,
                        Pos const &fxp2h,
                        double     h)
        {
        return (-Vec(fxp2h)+8*Vec(fxph)-8*Vec(fxmh)+Vec(fxm2h))/(12*h);
        }

    // Returns a numeric second derivative of f(x), with error of order h^4,   
    // given, f(x-2h), f(x-h), f(x), f(x+h), f(x+2h), and h.  The function is
    // templated two work with {double, Point2D, Point3D, or Point4D} as
    // the Pos type, and {double, Vector2D, Vector3D, or Vector4D} as the
    // Vec type.

    template<class Pos, class Vec>
    Vec SecondDerivative(Pos const &fxm2h,
                         Pos const &fxmh,
                         Pos const &fx,
                         Pos const &fxph,
                         Pos const &fxp2h,
                         double     h)
        {
        return (-Vec(fxp2h)+16*Vec(fxph)-30*Vec(fx)+16*Vec(fxmh)-Vec(fxm2h))/(12*h*h);
        }

    // Returns the numerical partial derivatives of f(x,y) with respect to x, y,
    // x twice, y twice, and the cross partial with respect to x and y.
    // The error of the returned derivatives are of order dx^5, dy^5,
    // dx^4, max(dx^5,dy^5), dy^4 respectively. aMatrix should contain
    // the following values of f(x,y).
    //
    // aMatrix[0][0]=f(x-dx*2,y-dy*2);
    // aMatrix[0][1]=f(x-dx*2,y-dy);
    // aMatrix[0][2]=f(x-dx*2,y);
    // aMatrix[0][3]=f(x-dx*2,y+dy);
    // aMatrix[0][4]=f(x-dx*2,y+dy*2);
    //
    // aMatrix[1][0]=f(x-dx,y-dy*2);
    // aMatrix[1][1]=f(x-dx,y-dy);
    // aMatrix[1][2]=f(x-dx,y);
    // aMatrix[1][3]=f(x-dx,y+dy);
    // aMatrix[1][4]=f(x-dx,y+dy*2);
    //
    // aMatrix[2][0]=f(x,y-dy*2);
    // aMatrix[2][1]=f(x,y-dy);
    // aMatrix[2][2]=f(x,y);
    // aMatrix[2][3]=f(x,y+dy);
    // aMatrix[2][4]=f(x,y+dy*2);
    //
    // aMatrix[3][0]=f(x+dx,y-dy*2);
    // aMatrix[3][1]=f(x+dx,y-dy);
    // aMatrix[3][2]=f(x+dx,y);
    // aMatrix[3][3]=f(x+dx,y+dy);
    // aMatrix[3][4]=f(x+dx,y+dy*2);
    //
    // aMatrix[4][0]=f(x+dx*2,y-dy*2);
    // aMatrix[4][1]=f(x+dx*2,y-dy);
    // aMatrix[4][2]=f(x+dx*2,y);
    // aMatrix[4][3]=f(x+dx*2,y+dy);
    // aMatrix[4][4]=f(x+dx*2,y+dy*2);

    template<class Pos, class Vec>
    void PartialDerivatives(Pos const aMatrix[5][5],
                            double    dx,
                            double    dy,
                            Vec      &dDFX,
                            Vec      &dDFY,
                            Vec      &dDFXX,
                            Vec      &dDFXY,
                            Vec      &dDFYY)
        {
        dDFX=SGM::FirstDerivative<Pos,Vec>(aMatrix[0][2],aMatrix[1][2],aMatrix[3][2],aMatrix[4][2],dx);
        dDFY=SGM::FirstDerivative<Pos,Vec>(aMatrix[2][0],aMatrix[2][1],aMatrix[2][3],aMatrix[2][4],dy);
        dDFXX=SGM::SecondDerivative<Pos,Vec>(aMatrix[0][2],aMatrix[1][2],aMatrix[2][2],aMatrix[3][2],aMatrix[4][2],dx);
        Vec dfx0=SGM::FirstDerivative<Pos,Vec>(aMatrix[0][0],aMatrix[1][0],aMatrix[3][0],aMatrix[4][0],dx);
        Vec dfx1=SGM::FirstDerivative<Pos,Vec>(aMatrix[0][1],aMatrix[1][1],aMatrix[3][1],aMatrix[4][1],dx);
        Vec dfx3=SGM::FirstDerivative<Pos,Vec>(aMatrix[0][3],aMatrix[1][3],aMatrix[3][3],aMatrix[4][3],dx);
        Vec dfx4=SGM::FirstDerivative<Pos,Vec>(aMatrix[0][4],aMatrix[1][4],aMatrix[3][4],aMatrix[4][4],dx);
        dDFXY=SGM::FirstDerivative<Pos,Vec>(Pos(dfx0),Pos(dfx1),Pos(dfx3),Pos(dfx4),dy);
        dDFYY=SGM::SecondDerivative<Pos,Vec>(aMatrix[2][0],aMatrix[2][1],aMatrix[2][2],aMatrix[2][3],aMatrix[2][4],dy);
        }
    
    } // End of SGM namespace

#endif // SGM_MATHEMATICS_H