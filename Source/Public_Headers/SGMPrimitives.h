#ifndef SGM_PRIMITIVES_H
#define SGM_PRIMITIVES_H

#include "SGMEntityClasses.h"
#include "SGMVector.h"
#include "SGMResult.h"

#include <set>
#include <vector>

namespace SGMInternal
{
class thing;
}

namespace SGM
    {
///////////////////////////////////////////////////////////////////////////////
//
//  By creating a SGM::Result one starts the SGM modeler.
//
///////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGMInternal::thing *CreateThing();

SGM_EXPORT void DeleteThing(SGMInternal::thing *pThing);  // if you create a thing, you are responsible for deleting it

///////////////////////////////////////////////////////////////////////////////
//
//  Three Dimensional Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGM::Body CreateBlock(SGM::Result        &rResult,
                                 SGM::Point3D const &Point1,
                                 SGM::Point3D const &Point2);

SGM_EXPORT SGM::Body CreateSphere(SGM::Result        &rResult,
                                  SGM::Point3D const &Center,
                                  double              dRadius);

SGM_EXPORT SGM::Body CreateCylinder(SGM::Result        &rResult,
                                    SGM::Point3D const &BottomCenter,
                                    SGM::Point3D const &TopCenter,
                                    double              dRadius);

SGM_EXPORT SGM::Body CreateCone(SGM::Result        &rResult,
                                SGM::Point3D const &BottomCenter,
                                SGM::Point3D const &TopCenter,
                                double              dBottomRadius,
                                double              dTopRadius,
                                bool                bSheetBody=false);

// If bApple is set to true and dMajorRadius<dMinorRadius, then
// an apple torus will be made, else a lemon torus will be made.
// If dMajorRadius==dMinorRadius, then a pinched torus is made.

SGM_EXPORT SGM::Body CreateTorus(SGM::Result             &rResult,
                                 SGM::Point3D      const &Center,
                                 SGM::UnitVector3D const &Axis,
                                 double                   dMinorRadius,
                                 double                   dMajorRadius,
                                 bool                     bApple=true);

SGM_EXPORT SGM::Body CreateRevolve(SGM::Result             &rResult,
                                   SGM::Point3D      const &Origin,
                                   SGM::UnitVector3D const &Axis,
                                   SGM::Curve              &IDCurve);

///////////////////////////////////////////////////////////////////////////////
//
//  Two Dimensional Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGM::Body CreateDisk(SGM::Result             &rResult,
                                SGM::Point3D      const &Center,
                                SGM::UnitVector3D const &Normal,
                                double                   dRadius);

// If edges are not given then the whole surface is used to make a sheet body.

SGM_EXPORT SGM::Body CreateSheetBody(SGM::Result                    &rResult,
                                     SGM::Surface                   &SurfaceID,
                                     std::vector<SGM::Edge>         &aEdges,
                                     std::vector<SGM::EdgeSideType> &aTypes);

SGM_EXPORT SGM::Face CreateFaceFromSurface(SGM::Result                    &rResult,
                                           SGM::Surface                   &SurfaceID,
                                           std::vector<SGM::Edge>         &aEdges,
                                           std::vector<SGM::EdgeSideType> &aTypes);

SGM_EXPORT SGM::Body CoverPlanarWire(SGM::Result &rResult,
                                     SGM::Body   &PlanarWireID);

///////////////////////////////////////////////////////////////////////////////
//
//  One Dimensional Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGM::Edge CreateEdge(SGM::Result           &rResult,
                                SGM::Curve            &CurveID,
                                SGM::Interval1D const *pDomain=nullptr);

SGM_EXPORT SGM::Edge CreateEdge(SGM::Result &rResult,
                                SGM::Curve  &CurveID,
                                SGM::Vertex &Start,
                                SGM::Vertex &End);

SGM_EXPORT SGM::Edge CreateLinearEdge(SGM::Result        &rResult,
                                      SGM::Point3D const &StartPos,
                                      SGM::Point3D const &EndPos);

SGM_EXPORT SGM::Body CreateWireBody(SGM::Result               &rResult,
                                    std::set<SGM::Edge> const &sEdges);

// Closed PolyLines should have the first point duplicated at the end of the vector.
// Line segments may intersect but only at their end points.

SGM_EXPORT SGM::Body CreatePolyLine(SGM::Result                     &rResult,
                                    std::vector<SGM::Point3D> const &aPoints);

SGM_EXPORT SGM::Body CreatePointBody(SGM::Result                  &rResult,
                                     std::set<SGM::Point3D> const &sPoints);

///////////////////////////////////////////////////////////////////////////////
//
//  Simplicial Complex Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM_EXPORT SGM::Complex CreateComplex(SGM::Result                     &rResult,
                                      std::vector<SGM::Point3D> const &aPoints,
                                      std::vector<unsigned int> const &aSegments,
                                      std::vector<unsigned int> const &aTriangles);


    } // End of SGM namespace

#endif // SGM_PRIMITIVES_H
