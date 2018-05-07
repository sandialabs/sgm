#ifndef SGM_PRIMITIVES_H
#define SGM_PRIMITIVES_H

#include "SGMEntityClasses.h"
#include "SGMDataClasses.h"
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
                                double              dTopRadius);

// If bApple is set to true and dMajorRadius<dMinorRadius, then
// an apple torus will be made, else a lemon torus will be made.
// If dMajorRadius==dMinorRadius, then a pinched torus is made.

SGM_EXPORT SGM::Body CreateTorus(SGM::Result             &rResult,
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

SGM_EXPORT SGM::Body CreateSheetBody(SGM::Result               &rResult,
                                     SGM::Surface              &SurfaceID,
                                     std::set<SGM::Edge> const &sEdges);

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

SGM_EXPORT SGM::Body CreateWireBody(SGM::Result               &rResult,
                                    std::set<SGM::Edge> const &sEdges);

// Closed PolyLines should have the first point duplicated at the end of the vector.
// Line segments may intersect but only at their end points.

SGM_EXPORT SGM::Body CreatePolyLine(SGM::Result                     &rResult,
                                    std::vector<SGM::Point3D> const &aPoints);

    } // End of SGM namespace

#endif // SGM_PRIMITIVES_H
