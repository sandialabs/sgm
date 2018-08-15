#include "SGMTransform.h"

#include "Curve.h"
#include "Mathematics.h"
#include "Surface.h"

namespace SGMInternal {
    cylinder::cylinder(SGM::Result &rResult,
                       SGM::Point3D const &Bottom,
                       SGM::Point3D const &Top,
                       double dRadius,
                       SGM::UnitVector3D const *XAxis) :
            surface(rResult, SGM::CylinderType), m_Origin(SGM::MidPoint(Bottom, Top))
    {
        m_Domain.m_UDomain.m_dMin = 0.0;
        m_Domain.m_UDomain.m_dMax = SGM_TWO_PI;
        m_Domain.m_VDomain.m_dMin = -SGM_MAX;
        m_Domain.m_VDomain.m_dMax = SGM_MAX;
        m_bClosedU = true;
        m_bClosedV = false;

        m_dRadius = dRadius;
        m_ZAxis = Top - Bottom;
        if (XAxis)
            {
            m_XAxis = *XAxis;
            }
        else
            {
            m_XAxis = m_ZAxis.Orthogonal();
            }
        m_YAxis = Snap(m_ZAxis * m_XAxis);
    }

    void cylinder::Transform(SGM::Transform3D const &Trans)
    {
        m_Origin = Trans * m_Origin;
        m_XAxis = Trans * m_XAxis;
        m_YAxis = Trans * m_YAxis;
        m_ZAxis = Trans * m_ZAxis;
        m_dRadius *= Trans.Scale();
    }

    curve *cylinder::UParamLine(SGM::Result &rResult, double dU) const
    {
        double dRadius=m_dRadius;
        SGM::Point2D uv(dU,0);
        SGM::Point3D Pos;
        SGM::Vector3D Vec;
        Evaluate(uv,&Pos,nullptr,&Vec);
        return new line(rResult,Pos,Vec,dRadius);
    }

    curve *cylinder::VParamLine(SGM::Result &, double) const
    { return nullptr; }


}