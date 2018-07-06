#include <limits>
#include <string>
#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMPrimitives.h"
#include "SGMGeometry.h"
#include "SGMIntersector.h"

TEST(intersection_check, intersect_ellipse_and_plane)
{
    SGMInternal::thing *pThing=SGM::CreateThing();
    SGM::Result rResult(pThing);

    SGM::Point3D Center(1,2,3);
    SGM::UnitVector3D XAxis(1,0,0);
    SGM::UnitVector3D YAxis(0,1,0);
    double dXRadius = 5;
    double dYRadius = 7;
    SGM::Curve EllipseCurve = SGM::CreateEllipse(rResult, Center, XAxis, YAxis, dXRadius, dYRadius);

    // coincident plane
    SGM::Point3D PlaneOrigin(1,2,3);
    SGM::UnitVector3D PlaneNormal(0,0,1);

    double dTolerance = SGM_MIN_TOL;
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::IntersectionType> aTypes;
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    SGM::Point3D ExpectedPoint = Center + 5*(XAxis);
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint, aPoints[0], dTolerance));
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint, aPoints[1], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::CoincidentType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::CoincidentType);

    // plane rotated about the y axis of the ellipse
    PlaneNormal = SGM::UnitVector3D(1,0,2);
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    SGM::Point3D ExpectedPoint1 = Center - dYRadius*(YAxis);
    SGM::Point3D ExpectedPoint2 = Center + dYRadius*(YAxis);
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint1, aPoints[0], dTolerance));
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint2, aPoints[1], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);

    // ellipse rotated about z
    XAxis = SGM::UnitVector3D(1,2,0);
    YAxis = SGM::UnitVector3D(2,-1,0);
    SGM::Curve EllipseCurve2 = SGM::CreateEllipse(rResult, Center, XAxis, YAxis, dXRadius, dYRadius);

    // plane rotated about x axis of the ellipse
    PlaneNormal = SGM::UnitVector3D(2,-1,1);
    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 2);
    EXPECT_EQ(aTypes.size(), 2);
    ExpectedPoint1 = Center + dXRadius*(XAxis);
    ExpectedPoint2 = Center - dXRadius*(XAxis);
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint1, aPoints[0], dTolerance));
    EXPECT_TRUE(SGM::NearEqual(ExpectedPoint2, aPoints[1], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::PointType);
    EXPECT_EQ(aTypes[1], SGM::IntersectionType::PointType);

    // plane tangent to the ellipse
    SGM::Point3D PosOnEllipse;
    SGM::Vector3D Tangent;
    double dUvalue = SGM_PI/5.0;
    SGM::EvaluateCurve(rResult, EllipseCurve2, dUvalue, &PosOnEllipse, &Tangent);

    SGM::UnitVector3D EllipseNormal = XAxis*YAxis;
    PlaneOrigin = PosOnEllipse + EllipseNormal*11.0;
    PlaneNormal = Tangent*EllipseNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 1);
    EXPECT_EQ(aTypes.size(), 1);
    EXPECT_TRUE(SGM::NearEqual(PosOnEllipse, aPoints[0], dTolerance));
    EXPECT_EQ(aTypes[0], SGM::IntersectionType::TangentType);

    // plane misses ellipse by slightly more than tolerance
    PlaneOrigin = PlaneOrigin + (1.0e5)*PlaneNormal;

    aPoints.clear();
    aTypes.clear();
    SGM::IntersectCurveAndPlane(rResult, EllipseCurve2, PlaneOrigin, PlaneNormal, aPoints, aTypes, dTolerance);

    EXPECT_EQ(aPoints.size(), 0);

    SGM::DeleteThing(pThing);
}

//size_t IntersectCurveAndPlane(SGM::Result                        &rResult,
//                              curve                        const *pCurve,
//                              SGM::Point3D                 const &PlaneOrigin,
//                              SGM::UnitVector3D            const &PlaneNorm,
//                              std::vector<SGM::Point3D>          &aPoints,
//                              std::vector<SGM::IntersectionType> &aTypes,
//                              double                              dTolerance);
