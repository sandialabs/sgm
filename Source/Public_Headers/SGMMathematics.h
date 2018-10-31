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
#include "SGMEntityClasses.h"

namespace SGM
    {
    /////////////////////////////////////////////////////////////////////////  
    //
    //  Point vector functions
    //
    /////////////////////////////////////////////////////////////////////////

    SGM_EXPORT bool FindLeastSquarePlane(std::vector<SGM::Point3D> const &aPoints,
                                         Point3D                         &Origin,
                                         UnitVector3D                    &XVec,
                                         UnitVector3D                    &YVec,
                                         UnitVector3D                    &ZVec);

    // Returns the min and max X, Y, and Z value with Origin at (0,0,0).

    SGM_EXPORT SGM::Interval3D FindOrientedBox(std::vector<SGM::Point3D> const &aPoints,
                                               Point3D                   const &Origin,
                                               UnitVector3D              const &XVec,
                                               UnitVector3D              const &YVec,
                                               UnitVector3D              const &ZVec);

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
                                      Point3D                    *Origin = nullptr,
                                      UnitVector3D               *Normal = nullptr);

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

    SGM_EXPORT double DistanceToPoints(std::vector<SGM::Point3D> const &aPoints,
                                       SGM::Point3D              const &Pos);

    SGM_EXPORT void RemoveDuplicates2D(std::vector<SGM::Point2D> &aPoints,
                                       double                     dTolerance);

    // Optionaly a box may be given that the points must be in also.

    SGM_EXPORT void RemoveDuplicates3D(std::vector<SGM::Point3D> &aPoints,
                                       double                     dTolerance,
                                       SGM::Interval3D     const *Box=nullptr);

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

    SGM_EXPORT double PolygonArea(std::vector<Point2D> const &aPolygon);

    // Returns the indices of of all the points that form concave angles.

    SGM_EXPORT size_t FindConcavePoints(std::vector<Point2D> const &aPolygon,
                                        std::vector<size_t>        &aConcavePoints);

    // If the given point is on the polygon, then the returned answer may
    // be either true or false.

    SGM_EXPORT bool PointInPolygon(Point2D              const &Pos,
                                   std::vector<Point2D> const &aPolygon);

    // Returns the distances to the given polygon from the given point. 
    // In other words the smallest distance to one of the defining line 
    // segments of the polygon.

    SGM_EXPORT double DistanceToPolygon(Point2D              const &Pos,
                                        std::vector<Point2D> const &aPolygon);

    // Returns true if the given point is inside the first polygon and outside the others,
    // If the given point is on one of the the polygon, then the returned answer may
    // be either true or false.

    SGM_EXPORT bool PointInPolygonGroup(Point2D                                 const &Pos,
                                        std::vector<Point2D>                    const &aPoints2D,
                                        std::vector<std::vector<unsigned int> > const &aaPolygons);

    // Coverts a polygon from a vector of indices to a vector of points.

    std::vector<SGM::Point2D> PointFormPolygon(std::vector<Point2D>      const &aPoints2D,
                                               std::vector<unsigned int> const &aPolygon);

    // Merges the indices of close points, within dTolerance, in aPolygon.

    std::vector<unsigned int> MergePolygon(std::vector<Point2D>      const &aPoints2D,
                                           std::vector<unsigned int> const &aPolygon,
                                           double                           dTolerance);

    // Returns the length of the smallest edge of the given polygon.

    SGM_EXPORT double SmallestPolygonEdge(std::vector<Point2D> const &aPolygon);

    // Returns a vector of triangles, indexed into aPoints for the given
    // polygons.  The first polygon is assumed to be the outside polygon and
    // counter clockwise.  The following polygons are assumed to be inside the
    // first one, disjoint, clockwise.  The function returns false is aaPolygons
    // or aPoints is empty with an error of ResultTypeInsufficientData.
    // If the clockwise polygons are not contained inside a counter clockwise
    // polygon then false is returned.  In addition, the aAjacencies are returned
    // since they are found in the process.  If the polygons are known to not
    // self-intersect, then bSelfIntersect may be set to false and the function
    // will run faster.

    SGM_EXPORT bool TriangulatePolygonWithHoles(Result                                        &rResult,
                                                std::vector<Point2D>                    const &aPoints2D,
                                                std::vector<std::vector<unsigned int> > const &aaPolygons,
                                                std::vector<unsigned int>                     &aTriangles,
                                                std::vector<unsigned int>                     &aAdjacencies,
                                                bool                                           bSelfIntersecting=true);

    // Same as TriangulatePolygonWithHoles but works on only one polygon.  If the polygon 
    // is known to not self-intersect, then bSelfIntersect may be set to false and the 
    // function will run faster.

    SGM_EXPORT bool TriangulatePolygon(Result                          &rResult,
                                       std::vector<Point2D>      const &aPoints2D,
                                       std::vector<unsigned int> const &aPolygon,
                                       std::vector<unsigned int>       &aTriangles,
                                       bool                             bSelfIntersecting=true);

    // Returns true and a polygon of the form (a,b,c,d,...) if the given segments
    // of the form (a0,b0,a1,b1,a2,b2,...) form a cylic graph.

    SGM_EXPORT bool FindPolygon(std::vector<unsigned int> const &aSegments,
                                std::vector<unsigned int>       &aPolygon);

    // Given a vector of points along, with a vector of polygons, on the given points, divide the polygons into
    // nested groups of polygons where each first polygon is an outside polygon and the following polygons are
    // inside the first polygon in the group.  If an inside polygon is not inside an outside polygon, then
    // an empty polygon is returned for the first or outside polygon of the group.

    SGM_EXPORT void GroupPolygons(std::vector<std::vector<unsigned int> >         const &aaPolygons,
                                  std::vector<Point2D>                            const &aPoints2D,
                                  std::vector<std::vector<std::vector<unsigned int> > > &aaaPolygons);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Triangle functions
    //
    ///////////////////////////////////////////////////////////////////////////

    // Returns true if D is inside the triangle (A,B,C)

    SGM_EXPORT bool InTriangle(Point2D const &A,
                               Point2D const &B,
                               Point2D const &C,
                               Point2D const &D);

    // Returns true if D is inside the angle formed by A as the angle vertex
    // and B and C as its sides.  If D is found first, before C when going
    // counter clockwise from B to D around A, or if D is on the ray AB or AC,
    // then then true is returned. 

    SGM_EXPORT bool InAngle(Point2D const &A,
                            Point2D const &B,
                            Point2D const &C,
                            Point2D const &D);

    SGM_EXPORT Point2D CenterOfMass(Point2D const &A,
                                    Point2D const &B,
                                    Point2D const &C);

    SGM_EXPORT Point3D CenterOfMass(Point3D const &A,
                                    Point3D const &B,
                                    Point3D const &C);

    // Returns the area of the triangle ABC as positive if ABC are counter 
    // clockwise else it returns a negative area of triangle ABC.

    SGM_EXPORT double SignedArea(Point2D const &A,
                                 Point2D const &B,
                                 Point2D const &C);
    
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

    // Returns the length of the longest edge of the given triangles. 

    SGM_EXPORT double FindMaxEdgeLength3D(std::vector<Point3D>      const &aPoints3D,
                                          std::vector<unsigned int> const &aTriangles);

    SGM_EXPORT double FindMaxEdgeLength2D(std::vector<Point2D>      const &aPoints2D,
                                          std::vector<unsigned int> const &aTriangles);

    // Returns the length of the shortest edge of the given triangles. 

    SGM_EXPORT double FindMinEdgeLength3D(std::vector<Point3D>      const &aPoints3D,
                                          std::vector<unsigned int> const &aTriangles);

    SGM_EXPORT double FindMinEdgeLength2D(std::vector<Point2D>      const &aPoints2D,
                                          std::vector<unsigned int> const &aTriangles);

    // Returns the boundary edges of the given triangles.  Boundary edges are
    // edges that belong to only one triangle. The direction of the triangle's edge
    // is returned in the set.

    SGM_EXPORT void FindBoundaryEdges(std::vector<unsigned int>                 const &aTriangles,
                                      std::set<std::pair<unsigned int,unsigned int> > &sBoundaryEdges);

    // Creates a vector of points and triangles for a grid of u and v values.

    SGM_EXPORT void CreateTrianglesFromGrid(std::vector<double> const &aUValues,
                                            std::vector<double> const &aVValues,
                                            std::vector<Point2D>      &aPoints2D,
                                            std::vector<unsigned int> &aTriangles);

    // Inserts a polygon into the given triangles and remove triangles that are outside
    // the polygon, where the polygon is assumed to go counter clockwise.  The indices 
    // of the inserted polygon points is returned in aPolygonIndices.  In addition, the
    // function will update a vector of 3D points and normals if the starting ones are
    // passed to the function along with their surface.  Moreover, an optional vector
    // flags may be given that tell is a point is to be imprinted or not, with false
    // meaning to skip the imprinting of the point.

    SGM_EXPORT bool InsertPolygon(SGM::Result                &rResult,
                                  std::vector<Point2D> const &aPolygon,
                                  std::vector<Point2D>       &aPoints2D,
                                  std::vector<unsigned int>  &aTriangles,
                                  std::vector<unsigned int>  &aPolygonIndices,
                                  SGM::Surface               *pSurfaceID=nullptr,
                                  std::vector<Point3D>       *pPoints3D=nullptr,
                                  std::vector<UnitVector3D>  *pNormals=nullptr,
                                  std::vector<bool>          *pImprintFlag=nullptr);

    // Returns true if the intersection of triangle {A,B,C} and the given segment
    // consists of more than one point.

    SGM_EXPORT bool SegmentCrossesTriangle(Segment2D const &Seg,
                                           Point2D   const &A,
                                           Point2D   const &B,
                                           Point2D   const &C);

    // Given a vector of triangle indices of the form (a0,b0,c0,a1,b1,c1,...) find 
    // the boundary as a vector of segment indices of the form (a0,b0,a1,b1,a2,b2,...).
    // In addition, the indices of the interior points are also returned.

    SGM_EXPORT void FindBoundary(std::vector<unsigned int> const &aTriangles,
                                 std::vector<unsigned int>       &aBoundary,
                                 std::set<unsigned int>          &sInterior);

    // Returns the number of connect components of the given line segments as returned
    // from the function FindBoundary.

    SGM_EXPORT size_t FindComponents1D(std::vector<unsigned int> const &aSegments);

    // Removes the given point index from the given triangles.  Note that the point is 
    // left in the vector aPoints2D but removed from aTriangles. Returns false is the 
    // point cannot be removed.  The function also returns the indices of the triangles 
    // that were removed or changed, and the point indices of the triangels that were replaced.

    SGM_EXPORT bool RemovePointFromTriangles(SGM::Result               &rResult,
                                             unsigned int               nRemoveIndex,
                                             std::vector<Point2D>      &aPoints2D,
                                             std::vector<unsigned int> &aTriangles,
                                             std::vector<unsigned int> &aRemovedOrChanged,
                                             std::vector<unsigned int> &aReplacedTriangles);

    // Given a vector of triangles on a given vector of points along with a vector of polygons
    // that have been imprinted into the triangles RemoveOutsideTriangles removes the triangles
    // that are outside of the given polygons.  The function returns false if the given polygons 
    // are not well nested.  In addition, aPoints2D, and optionaly pPoints3D and pNormals are
    // reduced to only the used points.  Moreover, if dMinDist is not zero, then interior points
    // that are within dMinDist of the boundary are also removed. 

    SGM_EXPORT void RemoveOutsideTriangles(SGM::Result                                   &rResult,
                                           std::vector<std::vector<unsigned int> > const &aaPolygons,
                                           std::vector<Point2D>                          &aPoints2D,
                                           std::vector<unsigned int>                     &aTriangles,
                                           double                                         dMinDist=0.0,
                                           std::vector<Point3D>                          *pPoints3D=nullptr,
                                           std::vector<UnitVector3D>                     *pNormals=nullptr);

    // Resets the indices in aTriangles and reduces the vector aPoints2D to only contain the 
    // points used the the given triangles.  Optionaly pPoints3D and pNormals are also reduced.

    SGM_EXPORT void ReduceToUsedPoints(std::vector<Point2D>      &aPoints2D,
                                       std::vector<unsigned int> &aTriangles,
                                       std::vector<Point3D>      *pPoints3D,
                                       std::vector<UnitVector3D> *pNormals);

    // Returns true if the given triangles are edge connected.

    SGM_EXPORT bool AreEdgeConnected(std::vector<unsigned int> const &aTriangles);

    // Merges the triangles based on two triangles are made to point to the same points
    // if the vertices are within dTolerance to each other.

    SGM_EXPORT void MergeTriangles3D(std::vector<Point3D> const &aPoints3D,
                                     std::vector<unsigned int>  &aTriangles,
                                     double                     dTolerance);

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

    SGM_EXPORT bool PolynomialFit(std::vector<Point2D> aPoints,
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

    // Returns the definite integral of the given function, f, from a to b.  
    // The integration is numerically performed using Romberg integration.
    // The void * passed into f is optional data that may be used be the 
    // function to return a value for x.

    double Integrate1D(double f(double x,void const *pData),
                       Interval1D        const &Domain,
                       void                   const *pData=nullptr,
                       double                        dTolerance=SGM_ZERO);

    // Returns the definite integral of the given function, f, over the given domain.  
    // The integration is numerically performed using Romberg integration.
    // The void * passed into f is optional data that may be used be the 
    // function to return a value for uv.

    double Integrate2D(double f(Point2D const &uv,void const *pData),
                       Interval2D                      const &Domain,
                       void                                 const *pData=nullptr,
                       double                                      dTolerance=SGM_ZERO);

    // Returns the definite integral of the given function, f, over the given triangle ABC. 
    // The integration is numerically performed using Romberg integration.
    // The void * passed into f is optional data that may be used be the 
    // function to return a value for uv.

    double IntegrateTriangle(double f(Point2D const &uv,void const *pData),
                             Point2D                         const &PosA,
                             Point2D                         const &PosB,
                             Point2D                         const &PosC,
                             void                                 const *pData=nullptr,
                             double                                      dTolerance=SGM_ZERO);
    
    ///////////////////////////////////////////////////////////////////////////
    //
    //  Polyhedra
    //
    ///////////////////////////////////////////////////////////////////////////

    // Creaete the triangles and vertices of a icosahedron, with the given 
    // center and circumscribed radius.  In addition a vertex of the icosahedron
    // is oriented to point in the ZAxis from the center, and one of the vertices 
    // that is adjacent to the ZAxis vertex is oriented to lie on the seam of
    // the circumscribed sphere with its seam in the XAxis direction from the 
    // center.  If nRefine is positive then the triangles are sphericaly refined.

    SGM_EXPORT void CreateIcosahedron(double                     dCircumscribedRadius,
                                      SGM::Point3D        const &Center,
                                      SGM::UnitVector3D   const &ZAxis,
                                      SGM::UnitVector3D   const &XAxis,
                                      std::vector<SGM::Point3D> &aPoints3D,
                                      std::vector<unsigned int> &aTriangles,
                                      int                        nRefine=0);

    // Creaete the triangles and vertices of a octahedron, with the given 
    // center and circumscribed radius.  In addition a vertex of the octahedron
    // is oriented to point in the ZAxis from the center, and one of the vertices 
    // that is adjacent to the ZAxis vertex is oriented to lie on the seam of
    // the circumscribed sphere with its seam in the XAxis direction from the 
    // center.   If nRefine is positive then the triangles are sphericaly refined.

    SGM_EXPORT void CreateOctahedron(double                     dCircumscribedRadius,
                                     SGM::Point3D        const &Center,
                                     SGM::UnitVector3D   const &ZAxis,
                                     SGM::UnitVector3D   const &XAxis,
                                     std::vector<SGM::Point3D> &aPoints3D,
                                     std::vector<unsigned int> &aTriangles,
                                     int                        nRefine=0);
    } // End of SGM namespace

#endif // SGM_MATHEMATICS_H