#include "SGMEnums.h"
#include "SGMVector.h"
#include "SGMEntityClasses.h"
#include "SGMMathematics.h"
#include "SGMTransform.h"

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

plane::plane(SGM::Result &rResult,
             plane const *pPlane):
    surface(rResult,SGM::PlaneType),m_Origin(pPlane->m_Origin),
    m_XAxis(pPlane->m_XAxis),m_YAxis(pPlane->m_YAxis),m_ZAxis(pPlane->m_ZAxis),m_dScale(pPlane->m_dScale)
    {
    m_Domain=pPlane->m_Domain;
    }

plane *plane::Clone(SGM::Result &rResult) const
    {
    plane *pAnswer = new plane(rResult, this);
    pAnswer->m_sFaces=m_sFaces;
    pAnswer->m_sOwners=m_sOwners;
    pAnswer->m_sAttributes=m_sAttributes;
    pAnswer->m_Box=m_Box;
    return pAnswer;
    }

void plane::Transform(SGM::Transform3D const &Trans)
    {
    m_Origin=Trans*m_Origin;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_ZAxis=Trans*m_ZAxis;
    m_dScale*=Trans.Scale();
    }

curve *plane::UParamLine(SGM::Result &, double) const
    { return nullptr; } // no curve

curve *plane::VParamLine(SGM::Result &, double) const
    { return nullptr; }


}
