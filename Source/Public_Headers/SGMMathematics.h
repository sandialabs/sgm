#ifndef SGM_MATHEMATICS_H
#define SGM_MATHEMATICS_H

#include <vector>
#include <set>
#include <map>

#include "sgm_export.h"

#include "SGMConstants.h"
#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMResult.h"
#include "SGMSegment.h"

namespace SGM
    {
    /////////////////////////////////////////////////////////////////////////  
    //
    //  Point vector functions
    //
    /////////////////////////////////////////////////////////////////////////

    SGM_EXPORT bool FindLeastSquarePlane(std::vector<Point3D> const &aPoints,
                                         Point3D                    &Origin,
                                         UnitVector3D               &XVec,
                                         UnitVector3D               &YVec,
                                         UnitVector3D               &ZVec,
                                         std::vector<double>        *aEigenValues=nullptr);

    // Returns the min and max X, Y, and Z value with Origin at (0,0,0).

    SGM_EXPORT Interval3D FindOrientedBox(std::vector<Point3D> const &aPoints,
                                          Point3D              const &Origin,
                                          UnitVector3D         const &XVec,
                                          UnitVector3D         const &YVec,
                                          UnitVector3D         const &ZVec);

    // FindLeastSquareLine3D returns the origin and axis of the lease square line
    // through the given points.  The function returns false if a line cannot be found.
    // In addition, the returned axis goes in the direction of the given vector of points.

    SGM_EXPORT bool FindLeastSquareLine3D(std::vector<Point3D> const &aPoints,
                                          Point3D                    &Origin,
                                          UnitVector3D               &Axis);

    // ProjectPointsToPlane will project the given vector of 3D points 
    // to the given plane resulting in a vector of 2D points.  The function
    // returns the maximum distance that the given points are from the plane.

    SGM_EXPORT double ProjectPointsToPlane(std::vector<Point3D> const &aPoints3D,
                                           Point3D              const &Origin,
                                           UnitVector3D         const &XVec,
                                           UnitVector3D         const &YVec,
                                           UnitVector3D         const &ZVec,
                                           std::vector<Point2D>       &aPoints2D);

    SGM_EXPORT bool ArePointsCoplanar(std::vector<Point3D> const &aPoints3D,
                                      double                      dTolerance,
                                      Point3D                    *Origin=nullptr,
                                      UnitVector3D               *Normal=nullptr);

    SGM_EXPORT Point2D FindCenterOfMass2D(std::vector<Point2D> const &aPoints);
    
    SGM_EXPORT Point3D FindCenterOfMass3D(std::vector<Point3D> const &aPoints);
    
    // Returns the cumulative cord lengths between the given vector of points.
    // If bNormalize=true, then the lengths are scales to go from zero to one.

    SGM_EXPORT void FindLengths3D(std::vector<Point3D> const &aPoints,
                                  std::vector<double>        &aLengths,
                                  bool                        bNormalize=false);

    // DoPointsMatch returns true if aPoints1 and aPoints2 contain the same points
    // within the given tolerance, and it returns a map that maps vector aPoints1,
    // to vector aPoints2.

    SGM_EXPORT bool DoPointsMatch(std::vector<Point3D>          const &aPoints1,
                                  std::vector<Point3D>          const &aPoints2,
                                  std::map<unsigned int,unsigned int> &mMatchMap,
                                  double                              dTolerance);

    // Returns the distance from Pos to the closest point in aPoints along 
    // with the nWhere, which is the index into aPoints of the closest point to Pos.

    SGM_EXPORT double DistanceToPoints(std::vector<Point3D> const &aPoints,
                                       Point3D              const &Pos,
                                       size_t                     &nWhere);

    SGM_EXPORT void RemoveDuplicates1D(std::vector<double> &aPoints,
                                       double               dTolerance);

    SGM_EXPORT void RemoveDuplicates2D(std::vector<Point2D> &aPoints,
                                       double                dTolerance);

    // Optionaly a box may be given that the points must be in also.

    SGM_EXPORT void RemoveDuplicates3D(std::vector<Point3D> &aPoints,
                                       double                dTolerance,
                                       Interval3D     const *Box=nullptr);

    // Returns the Hausdarff distance between the two point sets.

    SGM_EXPORT double HausdorffDistance(std::vector<Point3D> const &aPoints1,
                                        std::vector<Point3D> const &aPoints2);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Circle functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns false if the given three points are co-linear.  Otherwise the
    // center, normal and radius of a circle that contains the three points
    // is returned.
    
    SGM_EXPORT bool FindCircle(Point3D const &Pos0,
                               Point3D const &Pos1,
                               Point3D const &Pos2,
                               Point3D       &Center,
                               UnitVector3D  &Normal,
                               double        &dRadius);
    
    // Returns true if D is inside the Circumcircle of the triangle (A,B,C).
    // it is assumed that A, B, and C are in counter clockwise order. 
    // The returned value of dDeterminate can be used to tell if we are in a close
    // call situation, in which case a value close to zero will be returned.

    SGM_EXPORT bool InCircumcircle(Point2D const &A,
                                   Point2D const &B,
                                   Point2D const &C,
                                   Point2D const &D,
                                   double        &dDeterminate);
    
    SGM_EXPORT bool FindLeastSqaureCircle3D(std::vector<Point3D> const &aPoints,
                                            SGM::Point3D               &Center,
                                            SGM::UnitVector3D          &Normal,
                                            double                     &dRadius);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Number Theory Functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns the greatest common divisor of nA and nB or zero if nA and
    // nB are not both positive.

    SGM_EXPORT size_t GreatestCommonDivisor(size_t nA,
                                            size_t nB);
    
    // Returns true if nA and nB are positive and relatively prime.

    SGM_EXPORT bool RelativelyPrime(size_t nA,
                                    size_t nB);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Order theory Functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Given a partial order, definded by a set of ordered pairs of indices,
    // such that a<b if and only if <a,b> is in the set, find the maximal
    // elements.  An element is m is maximal if there does not exist an
    // element p, not equal to m, such that m<p.
    
    SGM_EXPORT size_t FindMaximalElements(std::set<std::pair<size_t,size_t> > const &sPartialOrder,
                                          std::vector<size_t>                       &aMaximalElements);

    // Given a partial order, definded by a set of ordered pairs of indices,
    // such that a<b if and only if <a,b> is in the set, find the decendents of
    // a given nParent element.  An element a is a decendent of b if a<b.

    SGM_EXPORT size_t FindDescendants(std::set<std::pair<size_t, size_t> > const &sPartialOrder,
                                      size_t nParent,
                                      std::vector<size_t> &aDescendants);

    // Given a partial order, definded by a set of ordered pairs of indices,
    // such that a<b if and only if <a,b> is in the set, find the decendents of
    // a given group of parent elements.  An element a is a decendent of b if a<b.

    SGM_EXPORT size_t FindDecendentsOfGroup(std::set<std::pair<size_t,size_t> > const &sPartialOrder,
                                            std::vector<size_t>                 const &aParents,
                                            std::vector<size_t>                       &aDecendents);

    // Given a partial order, definded by a set of ordered pairs of indices,
    // such that a<b if and only if <a,b> is in the set, find the childern of
    // a given nParent element.  An element c is a child of p if c<p and
    // there does not exist an element e such that a<e and e<b.

    SGM_EXPORT size_t FindChildren(std::set<std::pair<size_t, size_t> > const &sPartialOrder,
                                   size_t nParent,
                                   std::vector<size_t> &aChildren);

    // Returns all the decendents by their generation from a given nParent.

    SGM_EXPORT size_t FindGenerations(std::set<std::pair<size_t,size_t> > const &sPartialOrder,
                                      size_t                                     nParent,
                                      std::vector<std::vector<size_t> >         &aaGenerations);

    // Subsets a partial order to the only contain the given elements.

    SGM_EXPORT void SubsetPartialOrder(std::vector<size_t> const &aKeep,
                                       std::set<std::pair<size_t, size_t> > &sPartialOrder);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Linear Algebra Functions
    //
    ///////////////////////////////////////////////////////////////////////////

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

    SGM_EXPORT bool LinearSolve(std::vector<std::vector<double> > &aaMatrix);

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

    SGM_EXPORT bool BandedSolve(std::vector<std::vector<double> > &aaMatrix);

    // Returns the determinate of a 2 by 2 matrix[row][column].

    SGM_EXPORT double Determinate2D(double const aaMatrix[2][2]);

    // Returns the determinate of a 3 by 3 matrix[row][column].

    SGM_EXPORT double Determinate3D(double const aaMatrix[3][3]);

    // Returns the trace of a two by two matrix[row][column].

    SGM_EXPORT double Trace2D(double const aaMatrix[2][2]);

    // Returns the trace of a three by three matrix[row][column].

    SGM_EXPORT double Trace3D(double const aaMatrix[3][3]);

    // Returns the product of two two by two matrices, A*B=C

    SGM_EXPORT void FindProduct2D(double const aaMatrix1[2][2],  // A
                                  double const aaMatrix2[2][2],  // B
                                  double       aaAnswer[2][2]);  // C

    // Returns the product of two three by three matrices, A*B=C

    SGM_EXPORT void FindProduct3D(double const aaMatrix1[3][3],  // A
                                  double const aaMatrix2[3][3],  // B
                                  double       aaAnswer[3][3]);  // C

    // Returns the characteristic polynomial of a two by two 
    // matrix[row][column] in the form a*x^2+b*x+c.

    SGM_EXPORT void CharacteristicPolynomial2D(double const aaMatrix[2][2],
                                               double &a,double &b,double &c);

    // Returns the characteristic polynomial of a three by three 
    // matrix[row][column] in the form a*x^3+b*x^2+c*x+d.

    SGM_EXPORT void CharacteristicPolynomial3D(double const aaMatrix[3][3],
                                               double &a,double &b,double &c,double &d);

    // Returns true if the given matrix is a diagonal matrix.

    SGM_EXPORT bool IsDiagonal2D(double const aaMatrix[2][2]);

    // Returns true if the given matrix is a diagonal matrix.

    SGM_EXPORT bool IsDiagonal3D(double const aaMatrix[3][3]);

    // Returns the Eigen vectors and values of a two by two
    // matrix[row][column].

    SGM_EXPORT size_t FindEigenVectors2D(double              const aaMatrix[2][2],
                                         std::vector<double>       &aValues,
                                         std::vector<UnitVector2D> &aVectors);

    // Returns the Eigen vectors and values of a three by three
    // matrix[row][column].

    SGM_EXPORT size_t FindEigenVectors3D(double              const aaMatrix[3][3],
                                         std::vector<double>       &aValues,
                                         std::vector<UnitVector3D> &aVectors);

    // Returns a point in the given coordinate system of the given point Pos.
    // If X, Y, and Z are known to be mutually orthogonal, then bOrthogonal should 
    // be set to true.  Either way it is assumed that the three vectors are 
    // linearly independent.

    SGM_EXPORT Point3D FindLocalCoordinates(Point3D      const &Origin,
                                            UnitVector3D const &X,
                                            UnitVector3D const &Y,
                                            UnitVector3D const &Z,
                                            Point3D      const &Pos,
                                            bool                bOrthogonal);
    
    ///////////////////////////////////////////////////////////////////////////
    //
    //  Polynomial Functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Return the real roots of a linear, quadratic, cubic, and quartic 
    // equation.The returned roots are ordered from smallest to largest.  The  
    // coefficients are ordered from largest degree to smallest.  The function
    // returns the number of real roots.

    SGM_EXPORT size_t Linear(double a,double b,
                             std::vector<double> &aRoots);

    SGM_EXPORT size_t Quadratic(double a,double b,double c,
                                std::vector<double> &aRoots);

    SGM_EXPORT size_t Cubic(double a,double b,double c,double d,
                            std::vector<double> &aRoots);

    // Quartic returns the roots of f(x)=a*x^4+b*x^3+c*x^2+d*x+c.
    // dTolerance is used to find double roots and keep from
    // returning two roots if abs(f(x)) is less than dTolerance
    // at a root of the derivative.

    SGM_EXPORT size_t Quartic(double a,double b,double c,double d,double e,
                              std::vector<double> &aRoots,
                              double dTolerance);

    // Given three points in the xy plane with different x values the
    // coefficients of a quadratic of the form dA*x^2+dB*x+dC=y are returned.

    SGM_EXPORT bool FindQuadratic(SGM::Point2D const &xy1,
                                  SGM::Point2D const &xy2,
                                  SGM::Point2D const &xy3,
                                  double             &dA,
                                  double             &dB,
                                  double             &dC);

    // Given the coefficients of a polynomial in the form a*x^n+b*x^(n-1)+...
    // i.e. largest power of x first, the function LagrangeBound returns a bound
    // B such that all the roots of the polynomial are between -B and B.

    SGM_EXPORT double LagrangeBound(std::vector<double> aCoefficients);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Trigonometry functions
    //
    ///////////////////////////////////////////////////////////////////////////
    
    // Snaps x to -1 or 1 if x is outside the interval [-1,1] so that
    // acos will not return an error.

    SGM_EXPORT double SAFEacos(double x);

    // Snaps x to -1 or 1 if x is outside the interval [-1,1] so that
    // asin will not return an error.

    SGM_EXPORT double SAFEasin(double x);

    // Returns zero if both y and x are zero so that atan2 will not return 
    // an error.

    SGM_EXPORT double SAFEatan2(double y,double x);
    
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
        dDFX=FirstDerivative<Pos,Vec>(aMatrix[0][2],aMatrix[1][2],aMatrix[3][2],aMatrix[4][2],dx);
        dDFY=FirstDerivative<Pos,Vec>(aMatrix[2][0],aMatrix[2][1],aMatrix[2][3],aMatrix[2][4],dy);
        dDFXX=SecondDerivative<Pos,Vec>(aMatrix[0][2],aMatrix[1][2],aMatrix[2][2],aMatrix[3][2],aMatrix[4][2],dx);
        Vec dfx0=FirstDerivative<Pos,Vec>(aMatrix[0][0],aMatrix[1][0],aMatrix[3][0],aMatrix[4][0],dx);
        Vec dfx1=FirstDerivative<Pos,Vec>(aMatrix[0][1],aMatrix[1][1],aMatrix[3][1],aMatrix[4][1],dx);
        Vec dfx3=FirstDerivative<Pos,Vec>(aMatrix[0][3],aMatrix[1][3],aMatrix[3][3],aMatrix[4][3],dx);
        Vec dfx4=FirstDerivative<Pos,Vec>(aMatrix[0][4],aMatrix[1][4],aMatrix[3][4],aMatrix[4][4],dx);
        dDFXY=FirstDerivative<Pos,Vec>(Pos(dfx0),Pos(dfx1),Pos(dfx3),Pos(dfx4),dy);
        dDFYY=SecondDerivative<Pos,Vec>(aMatrix[2][0],aMatrix[2][1],aMatrix[2][2],aMatrix[2][3],aMatrix[2][4],dy);
        }

    } // End of SGM namespace

#endif // SGM_MATHEMATICS_H