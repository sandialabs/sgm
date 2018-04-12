#ifndef SGM_PRIMITIVES_H
#define SGM_PRIMITIVES_H

#include "SGMEntityClasses.h"
#include "SGMDataClasses.h"
#include <set>
#include <vector>

namespace SGM
    {
///////////////////////////////////////////////////////////////////////////////
//
//  By creating a SGM::Result one starts the SGM modeler.
//
///////////////////////////////////////////////////////////////////////////////

SGM::Result CreateResult();

///////////////////////////////////////////////////////////////////////////////
//
//  Three Dimensional Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM::Body CreateBlock(SGM::Result        &rResult,
                      SGM::Point3D const &Point1,
                      SGM::Point3D const &Point2);

SGM::Body CreateSphere(SGM::Result        &rResult,
                       SGM::Point3D const &Center,
                       double              dRadius);

SGM::Body CreateCylinder(SGM::Result        &rResult,
                         SGM::Point3D const &BottomCenter,
                         SGM::Point3D const &TopCenter,
                         double              dRadius);

SGM::Body CreateCone(SGM::Result        &rResult,
                     SGM::Point3D const &BottomCenter,
                     SGM::Point3D const &TopCenter,
                     double              dBottomRadius,
                     double              dTopRadius);

// If bApple is set to true and dMajorRadius<dMinorRadius, then
// an apple torus will be made, else a lemon torus will be made.
// If dMajorRadius==dMinorRadius, then a pinched torus is made.

SGM::Body CreateTorus(SGM::Result             &rResult,
                      SGM::Point3D      const &Center,
                      SGM::UnitVector3D const &Axis,
                      double                   dMajorRadius,
                      double                   dMinorRadius,
                      bool                     bApple=true);

///////////////////////////////////////////////////////////////////////////////
//
//  Two Dimensional Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM::Body CreateSheetBody(SGM::Result               &rResult,
                          SGM::Surface              &SurfaceID,
                          std::set<SGM::Edge> const &sEdges);

SGM::Body CoverPlanarWire(SGM::Result &rResult,
                          SGM::Body   &PlanarWireID);

///////////////////////////////////////////////////////////////////////////////
//
//  One Dimensional Creation Functions
//
///////////////////////////////////////////////////////////////////////////////

SGM::Edge CreateEdge(SGM::Result  &rResult,
                     SGM::Curve   &CurveID,
                     SGM::Point3D &Start,
                     SGM::Point3D &End);

SGM::Body CreateWireBody(SGM::Result               &rResult,
                         std::set<SGM::Edge> const &sEdges);

// Closed PolyLines should have the first point duplicated at the end of the vector.
// Line segments may intersect but only at their end points.

SGM::Body CreatePolyLine(SGM::Result                     &rResult,
                         std::vector<SGM::Point3D> const &aPoints);

    } // End of SGM namespace

#endif // SGM_PRIMITIVES_H