#include "SGMVector.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Surface.h"

namespace SGMInternal
{
sphere::sphere(SGM::Result             &rResult,
               SGM::Point3D      const &Center,
               double                   dRadius,
               SGM::UnitVector3D const *XAxis,
               SGM::UnitVector3D const *YAxis):
    surface(rResult,SGM::SphereType),m_Center(Center),m_dRadius(dRadius)
    {
    m_bClosedU=true;
    m_bSingularHighV=true;
    m_bSingularLowV=true;
    m_Domain.m_UDomain.m_dMin=0;
    m_Domain.m_UDomain.m_dMax=SGM_TWO_PI;
    m_Domain.m_VDomain.m_dMin=-SGM_HALF_PI;
    m_Domain.m_VDomain.m_dMax=SGM_HALF_PI;

    if(XAxis && YAxis)
        {
        m_XAxis=*XAxis;
        m_YAxis=*YAxis;
        m_ZAxis=m_XAxis*m_YAxis;
        }
    else
        {
        m_XAxis=SGM::UnitVector3D(1.0,0.0,0.0);
        m_YAxis=SGM::UnitVector3D(0.0,1.0,0.0);
        m_ZAxis=SGM::UnitVector3D(0.0,0.0,1.0);
        }
    }
}
