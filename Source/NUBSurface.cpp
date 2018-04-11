#include "SGMDataClasses.h"
#include "EntityClasses.h"

NUBsurface::NUBsurface(SGM::Result                                   &rResult,
                       std::vector<std::vector<SGM::Point3D> > const &aControlPoints,
                       std::vector<double>                     const &aUKnots,
                       std::vector<double>                     const &aVKnots):
    surface(rResult,SGM::NUBSurfaceType),m_aaControlPoints(aControlPoints),m_aUKnots(aUKnots),m_aVKnots(aVKnots)
    {
    }

std::vector<SGM::Point3D> const &NUBsurface::GetSeedPoints() const
    {
    return m_aSeedPoints;
    }

std::vector<SGM::Point2D> const &NUBsurface::GetSeedParams() const
    {
    return m_aSeedParams;
    }