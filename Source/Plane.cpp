#include "SGMEnums.h"
#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Surface.h"
namespace SGMInternal
{
plane::plane(SGM::Result             &rResult,
             SGM::Point3D      const &Origin,
             SGM::UnitVector3D const &XAxis,
             SGM::UnitVector3D const &YAxis,
             SGM::UnitVector3D const &ZAxis,
             double                   dScale):
    surface(rResult,SGM::PlaneType),m_Origin(Origin),m_XAxis(XAxis),
    m_YAxis(YAxis),m_ZAxis(ZAxis),m_dScale(dScale)
    {
    m_Domain.m_UDomain.m_dMin=-SGM_MAX;
    m_Domain.m_UDomain.m_dMax=SGM_MAX;
    m_Domain.m_VDomain.m_dMin=-SGM_MAX;
    m_Domain.m_VDomain.m_dMax=SGM_MAX;
    m_bClosedU=false;
    m_bClosedV=false;
    }

plane::plane(SGM::Result        &rResult,
             SGM::Point3D const &Origin,
             SGM::Point3D const &XPos,
             SGM::Point3D const &YPos):
    surface(rResult,SGM::PlaneType),m_Origin(Origin),
    m_XAxis(XPos-Origin),m_YAxis(YPos-Origin),m_ZAxis(m_XAxis*m_YAxis),m_dScale(1.0)
    {
    m_Domain.m_UDomain.m_dMin=-SGM_MAX;
    m_Domain.m_UDomain.m_dMax=SGM_MAX;
    m_Domain.m_VDomain.m_dMin=-SGM_MAX;
    m_Domain.m_VDomain.m_dMax=SGM_MAX;
    m_bClosedU=false;
    m_bClosedV=false;
    }
}