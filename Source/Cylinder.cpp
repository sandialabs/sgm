#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMEntityClasses.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Mathematics.h"

namespace SGMInternal
{
cylinder::cylinder(SGM::Result             &rResult,
                   SGM::Point3D      const &Bottom,
                   SGM::Point3D      const &Top,
                   double                   dRadius,
                   SGM::UnitVector3D const *XAxis):
    surface(rResult,SGM::CylinderType),m_Origin(SGM::MidPoint(Bottom,Top))
    {
    m_Domain.m_UDomain.m_dMin=0.0;
    m_Domain.m_UDomain.m_dMax=SGM_TWO_PI;
    m_Domain.m_VDomain.m_dMin=-SGM_MAX;
    m_Domain.m_VDomain.m_dMax=SGM_MAX;
    m_bClosedU=true;
    m_bClosedV=false;

    m_dRadius=dRadius;
    m_ZAxis=Top-Bottom;
    if(XAxis)
        {
        m_XAxis=*XAxis;
        }
    else
        {
        m_XAxis=m_ZAxis.Orthogonal();
        }
    m_YAxis=Snap(m_ZAxis*m_XAxis);
    }
}