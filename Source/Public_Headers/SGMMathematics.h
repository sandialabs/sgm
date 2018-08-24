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

namespace SGM
    {
    /////////////////////////////////////////////////////////////////////////  
    //
    //  Point vector functions
    //
    /////////////////////////////////////////////////////////////////////////

    SGM_EXPORT bool FindLeastSquarePlane(std::vector<SGM::Point3D> const &aPoints,
                                         SGM::Point3D                    &Origin,
                                         SGM::UnitVector3D               &XVec,
                                         SGM::UnitVector3D               &YVec,
                                         SGM::UnitVector3D               &ZVec);

    SGM_EXPORT bool FindLeastSquareLine3D(std::vector<SGM::Point3D> const &aPoints,
                                          SGM::Point3D                    &Origin,
                                          SGM::UnitVector3D               &Axis);

    // ProjectPointsToPlane will project the given vector of 3D points 
    // to the given plane resulting in a vector of 2D points.  The function
    // returns the maximum distance that the given points are from the plane.

    SGM_EXPORT double ProjectPointsToPlane(std::vector<SGM::Point3D> const &aPoints3D,
                                           SGM::Point3D              const &Origin,
                                           SGM::UnitVector3D         const &XVec,
                                           SGM::UnitVector3D         const &YVec,
                                           SGM::UnitVector3D         const &ZVec,
                                           std::vector<SGM::Point2D>       &aPoints2D);

    SGM_EXPORT bool ArePointsCoplanar(std::vector<SGM::Point3D> const &aPoints3D,
                                      double                           dTolerance,
                                      SGM::Point3D                    *Origin = nullptr,
                                      SGM::UnitVector3D               *Normal = nullptr);

    SGM_EXPORT Point2D FindCenterOfMass2D(std::vector<SGM::Point2D> const &aPoints);
    
    SGM_EXPORT Point3D FindCenterOfMass3D(std::vector<SGM::Point3D> const &aPoints);
    
    // Returns the cumulative cord lengths between the given vector of points.
    // If bNormalize=true, then the lengths are scales to go from zero to one.

    SGM_EXPORT void FindLengths3D(std::vector<SGM::Point3D> const &aPoints,
                                  std::vector<double>             &aLengths,
                                  bool                             bNormalize=false);

    // DoPointsMatch returns true if aPoints1 and aPoints2 contain the same points
    // within the given tolerance, and it returns a map that maps vector aPoints1,
    // to vector aPoints2.

    SGM_EXPORT bool DoPointsMatch(std::vector<SGM::Point3D>     const &aPoints1,
                                  std::vector<SGM::Point3D>     const &aPoints2,
                                  std::map<unsigned int,unsigned int> &mMatchMap,
                                  double                              dTolerance);

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

    // Counter clockwise polygons return positive areas.  Clockwise polygons
    // return negative areas.

    SGM_EXPORT double PolygonArea(std::vector<SGM::Point2D> const &aPolygon);

    SGM_EXPORT size_t FindConcavePoints(std::vector<SGM::Point2D> const &aPolygon,
                                        std::vector<size_t>             &aConcavePoints);

    // If the given point is only the polygon then the returned answer may
    // be either true or false.

    SGM_EXPORT bool PointInPolygon(SGM::Point2D              const &Pos,
                                   std::vector<SGM::Point2D> const &aPolygon);

    // Returns a vector of triangles, indexed into aPoints for the given
    // polygons.  The first polygon is assumed to be the outside polygon and
    // counter clockwise.  The following polygons are assumed to be inside the
    // first one, disjoint, clockwise.  The function returns false is aaPolygons
    // or aPoints is empty with an error of ResultTypeInsufficientData.
    // If the clockwise polygons are not contained inside a counter clockwise
    // polygon then false is returned with an error of 

    SGM_EXPORT bool TriangulatePolygon(SGM::Result                                   &rResult,
                                       std::vector<SGM::Point2D>               const &aPoints,
                                       std::vector<std::vector<unsigned int> > const &aaPolygons,
                                       std::vector<unsigned int>                     &aTriangles,
                                       std::vector<unsigned int>                     &aAdjacencies);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Triangles functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns true if D is inside the triangle (A,B,C)

    SGM_EXPORT bool InTriangle(SGM::Point2D const &A,
                               SGM::Point2D const &B,
                               SGM::Point2D const &C,
                               SGM::Point2D const &D);

    // Returns true if D is inside the angle formed by A as the angle vertex
    // and B and C as its sides.  If D is found first, before C when going
    // counter clockwise from B to D around C, or if D is on the ray AB or AC,
    // then then true is returned. 

    SGM_EXPORT bool InAngle(SGM::Point2D const &A,
                            SGM::Point2D const &B,
                            SGM::Point2D const &C,
                            SGM::Point2D const &D);
    
    // Given triangles in the form <a0,b0,c0,a1,b1,c1,...>
    // FindAdjacences2D return a vector of the form <Tab0,Tbc0,Tca0,Tab1,Tbc1,Tca1,...>
    // such that Tab0 is the index of the start of the triangle in aTriangles
    // that is adjacent to the first triangle along the edge ab.
    // If more than one triangle is adjacent to the first triangle along the same
    // edge for example T0, T1, T2, then T0 will point to T1, T1 will point to T2
    // and T2 will point to T0.  If an edge does not have a triangle that is
    // adjacent to it then the vector aAdjacency will have the value 
    // std::numeric_limits<unsigned int>::max() for that edges.

    SGM_EXPORT size_t FindAdjacences2D(std::vector<unsigned int> const &aTriangles,
                                       std::vector<unsigned int>       &aAdjacences);

    // Given segments in the form <a0,b0,a1,b1,b2,c2,...>
    // FindAdjacences1D returns a verctor of the form <Sa0,Sb0,Sa1,Sb1,...>
    // such that Sa0 is the index of the start of the segment in aSegments
    // that is adjacent to the first segment at the point a.
    // If more than one segment is adjacent to the first segment a the seam
    // point, then all ajecent segments are given in no order in a cycle of one
    // pointing to the next and eventually bact to the first one.  If no segment
    // is adjecent then std::numeric_limits<unsigned int>::max() is returned for 
    // the point.

    SGM_EXPORT size_t FindAdjacences1D(std::vector<unsigned int> const &aSegments,
                                       std::vector<unsigned int>       &aAdjacences);

    // Returns the length of the longest edges of the given triangles. 

    SGM_EXPORT double FindMaxEdgeLength(std::vector<SGM::Point3D> const &aPoints,
                                        std::vector<unsigned int> const &aTriangles);

    // Returns the boundary edges of the given triangles.  Boundary edges are
    // edges that belong to only one triangle. The direction of the triangle's edge
    // is returned in the set.

    SGM_EXPORT void FindBoundaryEdges(std::vector<unsigned int>                 const &aTriangles,
                                      std::set<std::pair<unsigned int,unsigned int> > &sBoundaryEdges);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Circle functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns false if the given three points are co-linear.  Otherwise the
    // center, normal and radius of a circle that contains the three points
    // is returned.
    
    SGM_EXPORT bool FindCircle(SGM::Point3D const &Pos0,
                               SGM::Point3D const &Pos1,
                               SGM::Point3D const &Pos2,
                               SGM::Point3D       &Center,
                               SGM::UnitVector3D  &Normal,
                               double             &dRadius);
    
    // Returns true if D is inside the Circumcircle of the triangle (A,B,C).
    // it is assumed that A, B, and C are in counter clockwise order. 
    // The returned value of dDet can be used to tell if we are in a close
    // call situation, in which case a value close to zero will be returned.

    SGM_EXPORT bool InCircumcircle(SGM::Point2D const &A,
                                   SGM::Point2D const &B,
                                   SGM::Point2D const &C,
                                   SGM::Point2D const &D,
                                   double             &dDet);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Number Theory Functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns the greatest common divisor of nA and nB or zero if nA and
    // nB are not both positive.

    size_t GreatestCommonDivisor(size_t nA,
                                 size_t nB);
    
    // Returns true if nA and nB are positive and relatively prime.

    bool RelativelyPrime(size_t nA,
                         size_t nB);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Order theory Functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Given a partial order, definded by a set of ordered pairs of indices,
    // such that a<=b if and only if <a,b> is in the set, find the maximal
    // elements.  An element is m is maximal if there does not exist an
    // element p, not equal to m, such that m<p.
    
    size_t FindMaximalElements(std::set<std::pair<size_t,size_t> > const &sPartialOrder,
                               std::vector<size_t>                       &aMaximalElements);

    // Given a partial order, definded by a set of ordered pairs of indices,
    // such that a<=b if and only if <a,b> is in the set, find the decendents of
    // a given nParent element.  An element a is a decendent of b if a<b.

    size_t FindDecendents(std::set<std::pair<size_t,size_t> > const &sPartialOrder,
                          size_t                                     nParent,
                          std::vector<size_t>                       &aDecendents);

    // Given a partial order, definded by a set of ordered pairs of indices,
    // such that a<=b if and only if <a,b> is in the set, find the decendents of
    // a given group of parent elements.  An element a is a decendent of b if a<b.

    size_t FindDecendentsOfGroup(std::set<std::pair<size_t,size_t> > const &sPartialOrder,
                                 std::vector<size_t>                 const &aParents,
                                 std::vector<size_t>                       &aDecendents);

    // Given a partial order, definded by a set of ordered pairs of indices,
    // such that a<=b if and only if <a,b> is in the set, find the childern of
    // a given nParent element.  An element c is a child of p if c<p and
    // there does not exist an element e such that a<e and e<b.

    size_t FindChildern(std::set<std::pair<size_t,size_t> > const &sPartialOrder,
                        size_t                                     nParent,
                        std::vector<size_t>                       &aChildern);

    // Returns all the decendents by their generation from a given nParent.

    size_t FindGenerations(std::set<std::pair<size_t,size_t> > const &sPartialOrder,
                           size_t                                     nParent,
                           std::vector<std::vector<size_t> >         &aaGenerations);

    // Subsets a partial order to the only contain the given elements.

    void SubsetPartailOrder(std::vector<size_t>           const &aKeep,
                            std::set<std::pair<size_t,size_t> > &sPartialOrder);

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

    SGM_EXPORT void FindProduct2D(double const aaMatrix1[2][2], // A
                                  double const aMatrix2[2][2],  // B
                                  double       aAnswer[2][2]);  // C

    // Returns the product of two three by three matrices, A*B=C

    SGM_EXPORT void FindProduct3D(double const aaMatrix1[3][3], // A
                                  double const aMatrix2[3][3],  // B
                                  double       aAnswer[3][3]);  // C

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
    
    ///////////////////////////////////////////////////////////////////////////
    //
    //  Polynomials Functions
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

    // Given a vector of N points in the XY-plane PolynomialFit returns
    // the coefficients of a degree N-1 polynomial that passes through the
    // given points.  For examples if four points are given, then a degree
    // three polynomial with coefficients (a,b,c,d) of the form 
    // a*x^3+b*x^2+c*x+d=y will be returned.  If any two points have the
    // same x coordinate, then false will be returned.

    SGM_EXPORT bool PolynomialFit(std::vector<SGM::Point2D> aPoints,
                                  std::vector<double>       aCoefficients);

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

    // Returns the definite integral of the given function, f, from a to b.  
    // The integration is numerically performed using Romberg integration.
    // The void * passed into f is optional data that may be used be the 
    // function to return a value for x.

    double Integrate1D(double f(double x,void const *pData),
                       SGM::Interval1D        const &Domain,
                       void                   const *pData=nullptr,
                       double                        dTolerance=SGM_ZERO);

    // Returns the definite integral of the given function, f, over the given domain.  
    // The integration is numerically performed using Romberg integration.
    // The void * passed into f is optional data that may be used be the 
    // function to return a value for uv.

    double Integrate2D(double f(SGM::Point2D const &uv,void const *pData),
                       SGM::Interval2D                      const &Domain,
                       void                                 const *pData=nullptr,
                       double                                      dTolerance=SGM_ZERO);

    // Returns the definite integral of the given function, f, over the given triangle ABC. 
    // The integration is numerically performed using Romberg integration.
    // The void * passed into f is optional data that may be used be the 
    // function to return a value for uv.

    double IntegrateTriangle(double f(SGM::Point2D const &uv,void const *pData),
                             SGM::Point2D                         const &PosA,
                             SGM::Point2D                         const &PosB,
                             SGM::Point2D                         const &PosC,
                             void                                 const *pData=nullptr,
                             double                                      dTolerance=SGM_ZERO);
    
    } // End of SGM namespace

#endif // SGM_MATHEMATICS_H