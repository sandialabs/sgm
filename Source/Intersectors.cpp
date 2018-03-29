#include "Intersectors.h"

double FindLocalMin(curve  const *pCurve1,    // Input
                    curve  const *pCurve2,    // Input
                    double       &t1,         // Input and output
                    double       &t2,         // Input and output
                    SGM::Point3D &Pos1,       // Output
                    SGM::Point3D &Pos2)       // Output
    {
    pCurve1->Evaluate(t1,&Pos1);
    pCurve2->Evaluate(t2,&Pos2);
    double dLast=DBL_MAX;
    double dDist2=Pos1.DistanceSquared(Pos2);
    size_t nCount=0;
    while(1E-24<fabs(dLast-dDist2) && nCount<100)
        {
        dLast=dDist2;
        t1=pCurve1->Inverse(Pos2,&Pos1,&t1);
        t2=pCurve2->Inverse(Pos1,&Pos2,&t2);
        dDist2=Pos1.DistanceSquared(Pos2);
        ++nCount;
        }
    return sqrt(dDist2);
    }

size_t IntersectLineAndCylinder(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                SGM::Interval1D              const &Domain,
                                cylinder                     const *pCylinder,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes)
    {
    // Empty, coincident, one tangent point, or two points.

    Origin;
    Axis;
    Domain;
    pCylinder;
    dTolerance;
    aPoints;
    aTypes;

    return 0;
    }

size_t IntersectLineAndPlane(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             plane                        const *pPlane,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes)
    {
    // Empty, coincident, or one point.

    SGM::Point3D Pos0=Origin+Axis*Domain.m_dMin;
    SGM::Point3D Pos1=Origin+Axis*Domain.m_dMin;
    SGM::Point3D PlaneOrigin=pPlane->m_Origin;
    SGM::UnitVector3D PlaneNorm=pPlane->m_ZAxis;
    double dDist0=(Pos0-PlaneOrigin)%PlaneNorm;
    double dDist1=(Pos1-PlaneOrigin)%PlaneNorm;
    if(fabs(dDist0)<dTolerance && fabs(dDist1)<dTolerance)
        {
        aPoints.push_back(Pos0);
        aPoints.push_back(Pos1);
        aTypes.push_back(SGM::IntersectionType::CoincidentType);
        aTypes.push_back(SGM::IntersectionType::CoincidentType);
        return 2;
        }
    if(dDist0*dDist1<0)
        {
        double dFraction;
        if(dDist0<0)
            {
            dFraction=dDist0/(dDist0-dDist1);
            }
        else
            {
            dFraction=dDist0/(dDist1-dDist0);
            }
        aPoints.push_back(SGM::MidPoint(Pos0,Pos1,dFraction));
        aTypes.push_back(SGM::IntersectionType::PointType);
        return 1;
        }
    else
        {
        if(fabs(dDist0)<dTolerance)
            {
            aPoints.push_back(Pos0);
            aTypes.push_back(SGM::IntersectionType::PointType);
            return 1;
            }
        else if(fabs(dDist1)<dTolerance)
            {
            aPoints.push_back(Pos1);
            aTypes.push_back(SGM::IntersectionType::PointType);
            return 1;
            }
        return 0;
        }
    }

size_t IntersectLineAndSphere(SGM::Point3D                  const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               sphere                       const *pSphere,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes)
    {
    // Empty, one tangent point or two points.

    Origin;
    Axis;
    Domain;
    pSphere;
    dTolerance;
    aPoints;
    aTypes;

    return 0;
    }

size_t IntersectLineAndTorus(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             torus                        const *pTorus,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes)
    {
    // Empty, one tangent point, two points, two tangents points, two points and one tangent point, four points.

    Origin;
    Axis;
    Domain;
    pTorus;
    dTolerance;
    aPoints;
    aTypes;

    return 0;
    }


size_t IntersectLineAndSurface(SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               surface                      const *pSurface,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes)
    {
    switch(pSurface->GetSurfaceType())
        {
        case SGM::EntityType::CylinderType:
            {
            return IntersectLineAndCylinder(Origin,Axis,Domain,(cylinder *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::PlaneType:
            {
            return IntersectLineAndPlane(Origin,Axis,Domain,(plane *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::SphereType:
            {
            return IntersectLineAndSphere(Origin,Axis,Domain,(sphere *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::TorusType:
            {
            return IntersectLineAndTorus(Origin,Axis,Domain,(torus *)pSurface,dTolerance,aPoints,aTypes);
            }
        default:
            {
            throw;
            }
        }
    }