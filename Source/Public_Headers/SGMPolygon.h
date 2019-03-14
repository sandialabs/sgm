#ifndef SGM_POLYGON_H
#define SGM_POLYGON_H

#include <vector>

#include "sgm_export.h"
#include "SGMResult.h"
#include "SGMVector.h"

namespace SGM {

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

//SGM_EXPORT size_t FindConcavePoints(std::vector<Point2D> const &aPolygon,
//                                    std::vector<size_t>        &aConcavePoints);

// If the given point is on the polygon, then the returned answer may
// be either true or false.

SGM_EXPORT bool PointInPolygon(Point2D              const &Pos,
                               std::vector<Point2D> const &aPolygon);

// Returns true if the given point is inside the first polygon and outside the others,
// If the given point is on one of the the polygon, then the returned answer may
// be either true or false.  So that the function can run fast on lots of points for
// the same polygons the signed areas of the polygons is also required.

SGM_EXPORT bool PointInPolygonGroup(Point2D                                 const &Pos,
                                    std::vector<Point2D>                    const &aPoints2D,
                                    std::vector<std::vector<unsigned int> > const &aaPolygons,
                                    std::vector<double>                     const &aAreas);

// Coverts a polygon from a vector of indices to a vector of points.

SGM_EXPORT std::vector<Point2D> PointsFromPolygon(std::vector<Point2D>      const &aPoints2D,
                                                  std::vector<unsigned int> const &aPolygon);

// Merges the indices of close points, within dTolerance, in aPolygon.

SGM_EXPORT std::vector<unsigned int> MergePolygon(std::vector<Point2D>      const &aPoints2D,
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

// Given a vector of points along, with a vector of polygons, on the given points, divide the polygons into
// nested groups of polygons where each first polygon is an outside polygon and the following polygons are
// inside the first polygon in the group.  If an inside polygon is not inside an outside polygon, then
// an empty polygon is returned for the first or outside polygon of the group.

SGM_EXPORT void GroupPolygons(std::vector<std::vector<unsigned int> >         const &aaPolygons,
                              std::vector<Point2D>                            const &aPoints2D,
                              std::vector<std::vector<std::vector<unsigned int> > > &aaaPolygons);

// Returns true and a polygon of the form (a,b,c,d,...) if the given segments
// of the form (a0,b0,a1,b1,a2,b2,...) form a cylic graph.

SGM_EXPORT bool FindPolygon(std::vector<unsigned int> const &aSegments,
                            std::vector<unsigned int>       &aPolygon);

// Inserts a polygon into the given triangles and remove triangles that are outside
// the polygon, where the polygon is assumed to go counter clockwise.  The indices
// of the inserted polygon points is returned in aPolygonIndices.  In addition, the
// function will update a vector of 3D points and normals if the starting ones are
// passed to the function along with their surface.  Moreover, an optional vector of
// flags may be given that tell if a point is to be imprinted or not, with false
// meaning to skip the imprinting of the point.

SGM_EXPORT bool InsertPolygon(Result                     &rResult,
                              std::vector<Point2D> const &aPolygon,
                              std::vector<Point2D>       &aPoints2D,
                              std::vector<unsigned int>  &aTriangles,
                              std::vector<unsigned int>  &aPolygonIndices,
                              Surface                    *pSurfaceID = nullptr,
                              std::vector<Point3D>       *pPoints3D = nullptr,
                              std::vector<UnitVector3D>  *pNormals = nullptr,
                              std::vector<bool>          *pImprintFlag = nullptr);

}

#endif //SGM_POLYGON_H
