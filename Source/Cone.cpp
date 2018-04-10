#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"

cone::cone(SGM::Result             &rResult,
           SGM::Point3D      const &Center,
           SGM::UnitVector3D const &ZAxis,
           double                   dRadius,
           double                   dHalfAngle,
           SGM::UnitVector3D const *XAxis):
    surface(rResult,SGM::ConeType),m_Origin(Center),m_ZAxis(ZAxis)
    {
    m_dSinHalfAngle=sin(dHalfAngle);
    m_dCosHalfAngle=cos(dHalfAngle);

    m_Domain.m_UDomain.m_dMin=0.0;
    m_Domain.m_UDomain.m_dMax=SGM_TWO_PI;
    m_Domain.m_VDomain.m_dMin=-SGM_MAX;
    m_Domain.m_VDomain.m_dMax=1.0/m_dSinHalfAngle;

    m_bClosedU=true;
    m_bClosedV=false;
    m_bSingularHighV=true;
    m_dRadius=dRadius;

    if(XAxis)
        {
        m_XAxis=*XAxis;
        }
    else
        {
        m_XAxis=m_ZAxis.Orthogonal();
        }
    m_YAxis=m_ZAxis*m_XAxis;
    }
   