#ifndef SGM_TRIANGLE_H
#define SGM_TRIANGLE_H

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

///////////////////////////////////////////////////////////////////////////
//
//  Triangle functions
//
///////////////////////////////////////////////////////////////////////////

// Cached triangle data for fast 2D point in 2D triangle tests.

class TriangleData2D
    {
    public:

    TriangleData2D(Point2D const &A,
                   Point2D const &B,
                   Point2D const &C);

    bool InTriangle(Point2D const &P) const;

    private:

    Point2D m_dA;
    Point2D m_dB;
    Point2D m_dC;

    double m_dU_CA;
    double m_dV_CA;
    double m_dU_BA;
    double m_dV_BA;
    double m_dD;
    };

// Returns true if P is inside the triangle (A,B,C)

inline bool InTriangle(Point2D const &A,
                       Point2D const &B,
                       Point2D const &C,
                       Point2D const &P);

inline Point2D CenterOfMass(Point2D const &A,
                            Point2D const &B,
                            Point2D const &C);

inline Point3D CenterOfMass(Point3D const &A,
                            Point3D const &B,
                            Point3D const &C);

// Returns the area of the triangle ABC as positive if ABC are counter
// clockwise else it returns a negative area of triangle ABC.

inline double SignedArea(Point2D const &A,
                         Point2D const &B,
                         Point2D const &C);

// Returns true if D is inside the angle formed by A as the angle vertex
// and B and C as its sides.  If D is found first, before C when going
// counter clockwise from B to D around A, or if D is on the ray AB or AC,
// then true is returned.

SGM_EXPORT bool InAngle(Point2D const &A,
                        Point2D const &B,
                        Point2D const &C,
                        Point2D const &D);

// Given triangles in the form <a0,b0,c0,a1,b1,c1,...>
// FindAdjacencies return a vector of the form <Tab0,Tbc0,Tca0,Tab1,Tbc1,Tca1,...>
// such that Tab0 is the index of the start of the triangle in aTriangles
// that is adjacent to the first triangle along the edge ab.
// If more than one triangle is adjacent to the first triangle along the same
// edge for example T0, T1, T2, then T0 will point to T1, T1 will point to T2
// and T2 will point to T0.  If an edge does not have a triangle that is
// adjacent to it then the vector aAdjacency will have the value
// std::numeric_limits<unsigned int>::max() for that edges.

SGM_EXPORT size_t FindAdjacencies2D(std::vector<unsigned int> const &aTriangles,
                                    std::vector<unsigned int> &aAdjacences);

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
                                   std::vector<unsigned int> &aAdjacences);

// Returns the length of the longest edge of the given triangles.

SGM_EXPORT double FindMaxEdgeLength3D(std::vector<Point3D> const &aPoints3D,
                                      std::vector<unsigned int> const &aTriangles);

SGM_EXPORT double FindMaxEdgeLength2D(std::vector<Point2D> const &aPoints2D,
                                      std::vector<unsigned int> const &aTriangles);

// Returns the length of the shortest edge of the given triangles.

SGM_EXPORT double FindMinEdgeLength3D(std::vector<Point3D> const &aPoints3D,
                                      std::vector<unsigned int> const &aTriangles);

SGM_EXPORT double FindMinEdgeLength2D(std::vector<Point2D> const &aPoints2D,
                                      std::vector<unsigned int> const &aTriangles);

// Creates a vector of points and triangles for a grid of u and v values.
// Optionaly the distance to the closest point, for each point in the grid, is returned.

SGM_EXPORT void CreateTrianglesFromGrid(std::vector<double> const &aUValues,
                                        std::vector<double> const &aVValues,
                                        std::vector<Point2D> &aPoints2D,
                                        std::vector<unsigned int> &aTriangles,
                                        std::vector<double> *aDistances = nullptr);

// Returns true if the intersection of triangle {A,B,C} and the given segment
// consists of more than one point.

SGM_EXPORT bool SegmentCrossesTriangle(Segment2D const &Seg,
                                       Point2D const &A,
                                       Point2D const &B,
                                       Point2D const &C);

// Given a vector of triangle indices of the form (a0,b0,c0,a1,b1,c1,...) find
// the boundary as a vector of segment indices of the form (a0,b0,a1,b1,a2,b2,...).
// In addition, the indices of the interior points are also returned.

SGM_EXPORT void FindBoundary(std::vector<unsigned int> const &aTriangles,
                             std::vector<unsigned int> &aBoundary,
                             std::set<unsigned int> &sInterior);

// Returns the number of connect components of the given line segments as returned
// from the function FindBoundary.

SGM_EXPORT size_t FindComponents1D(std::vector<unsigned int> const &aSegments);

// Removes the given point index from the given triangles.  Note that the point is
// left in the vector aPoints2D but removed from aTriangles. Returns false is the
// point cannot be removed.  The function also returns the indices of the triangles
// that were removed or changed, and the point indices of the triangels that were replaced.

SGM_EXPORT bool RemovePointFromTriangles(Result &rResult,
                                         unsigned int nRemoveIndex,
                                         std::vector<Point2D> &aPoints2D,
                                         std::vector<unsigned int> &aTriangles,
                                         std::vector<unsigned int> &aRemovedOrChanged,
                                         std::vector<unsigned int> &aReplacedTriangles);

// Given a vector of triangles on a given vector of points along with a vector of polygons
// that have been imprinted into the triangles RemoveOutsideTriangles removes the triangles
// that are outside of the given polygons.  The function returns false if the given polygons
// are not well nested.  In addition, aPoints2D, and optionaly pPoints3D and pNormals are
// reduced to only the used points.  Moreover, if dMinDist is not zero, then interior points
// that are within dMinDist of the boundary are also removed.

SGM_EXPORT void RemoveOutsideTriangles(Result &rResult,
                                       std::vector<std::vector<unsigned int> > const &aaPolygons,
                                       std::vector<Point2D> &aPoints2D,
                                       std::vector<unsigned int> &aTriangles,
                                       double dMinDist = 0.0,
                                       std::vector<Point3D> *pPoints3D = nullptr,
                                       std::vector<UnitVector3D> *pNormals = nullptr);

// Resets the indices in aTriangles and reduces the vector aPoints2D to only contain the
// points used the the given triangles.  Optionaly pPoints3D and pNormals are also reduced.

SGM_EXPORT void ReduceToUsedPoints(std::vector<Point2D> &aPoints2D,
                                   std::vector<unsigned int> &aTriangles,
                                   std::vector<Point3D> *pPoints3D,
                                   std::vector<UnitVector3D> *pNormals);

// Returns true if the given triangles are edge connected.

SGM_EXPORT bool AreEdgeConnected(std::vector<unsigned int> const &aTriangles);

// Merges the triangles based on two triangles are made to point to the same points
// if the vertices are within dTolerance to each other.

SGM_EXPORT void MergeTriangles3D(std::vector<Point3D> const &aPoints3D,
                                 std::vector<unsigned int> &aTriangles,
                                 double dTolerance);

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
                                  Point3D        const &Center,
                                  UnitVector3D   const &ZAxis,
                                  UnitVector3D   const &XAxis,
                                  std::vector<Point3D> &aPoints3D,
                                  std::vector<unsigned int> &aTriangles,
                                  int                        nRefine=0);

// Creaete the triangles and vertices of a octahedron, with the given
// center and circumscribed radius.  In addition a vertex of the octahedron
// is oriented to point in the ZAxis from the center, and one of the vertices
// that is adjacent to the ZAxis vertex is oriented to lie on the seam of
// the circumscribed sphere with its seam in the XAxis direction from the
// center.   If nRefine is positive then the triangles are sphericaly refined.

SGM_EXPORT void CreateOctahedron(double                     dCircumscribedRadius,
                                 Point3D        const &Center,
                                 UnitVector3D   const &ZAxis,
                                 UnitVector3D   const &XAxis,
                                 std::vector<Point3D> &aPoints3D,
                                 std::vector<unsigned int> &aTriangles,
                                 int                        nRefine=0);


} // namespace SGM

#include "Inline/SGMTriangle.inl" // inline implementations

#endif //SGM_TRIANGLE_H
