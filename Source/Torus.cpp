#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"

torus::torus(SGM::Result             &rResult,
             SGM::Point3D      const &Center,
             SGM::UnitVector3D const &ZAxis,
             double                   dMinorRadius,
             double                   dMajorRadius,
             bool                     bApple,
             SGM::UnitVector3D const *XAxis):
    surface(rResult,SGM::SphereType),m_Center(Center),m_ZAxis(ZAxis),
    m_dMinorRadius(dMinorRadius),m_dMajorRadius(dMajorRadius)
    {
    if(XAxis)
        {
        m_XAxis=*XAxis;
        }
    else
        {
        m_XAxis=ZAxis.Orthogonal();
        }

    m_YAxis=m_ZAxis*m_XAxis;
    m_bClosedU=true;
    m_Domain.m_UDomain.m_dMin=0.0;
    m_Domain.m_UDomain.m_dMax=SGM_TWO_PI;

    if(m_dMajorRadius+SGM_ZERO<m_dMinorRadius)
        {
        double dT=SGM::SAFEacos(m_dMajorRadius/m_dMinorRadius);
        if(bApple) // Apple Torus
            {
            m_Domain.m_VDomain.m_dMin=dT-SGM_PI;
            m_Domain.m_VDomain.m_dMax=SGM_PI-dT;
            }
        else // Limon Torus
            {
            m_Domain.m_VDomain.m_dMin=SGM_PI-dT;
            m_Domain.m_VDomain.m_dMax=SGM_PI+dT;
            }
        }
    else if(fabs(m_dMajorRadius-m_dMinorRadius)<SGM_ZERO) // Pinched Torus
        {
        m_bClosedV=true;
        m_Domain.m_VDomain.m_dMin=-SGM_PI;
        m_Domain.m_VDomain.m_dMax=SGM_PI;
        m_bSingularLowV=true;
        m_bSingularHighV=true;
        }
    else // Normal Torus
        {
        m_bClosedV=true;
        m_Domain.m_VDomain.m_dMin=0.0;
        m_Domain.m_VDomain.m_dMax=SGM_TWO_PI;
        }
    }