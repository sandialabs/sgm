#include "SGMVector.h"
#include "SGMTransform.h"
#include "EntityClasses.h"
#include "Curve.h"

namespace SGMInternal
{

parabola::parabola(SGM::Result &rResult,
                   SGM::Point3D const &Center,
                   SGM::UnitVector3D const &XAxis,
                   SGM::UnitVector3D const &YAxis,
                   double dA) :
        curve(rResult, SGM::ParabolaType),
        m_Center(Center),
        m_XAxis(XAxis),
        m_YAxis(YAxis),
        m_Normal(XAxis * YAxis),
        m_dA(dA)
    {
    m_Domain.m_dMin = -SGM_MAX;
    m_Domain.m_dMax = SGM_MAX;
    }

parabola::parabola(SGM::Result &rResult, parabola const &other):
            curve(rResult, other),
            m_Center(other.m_Center),
            m_XAxis(other.m_XAxis),
            m_YAxis(other.m_YAxis),
            m_Normal(other.m_Normal),
            m_dA(other.m_dA)
    {}

parabola *parabola::Clone(SGM::Result &rResult) const
    { return new parabola(rResult, *this); }

void parabola::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    ParabolaEvaluate(m_Center, m_XAxis, m_YAxis, m_dA, t, Pos, D1, D2);
    }

bool parabola::IsSame(curve const *pOther,double dTolerance) const
    {
    if(pOther->GetCurveType()!=m_CurveType)
        {
        return false;
        }
    parabola const *pCurve2=(parabola const *)pOther;
    if(SGM::NearEqual(m_Center,pCurve2->m_Center,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_XAxis,pCurve2->m_XAxis,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_YAxis,pCurve2->m_YAxis,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_Normal,pCurve2->m_Normal,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_dA,pCurve2->m_dA,dTolerance,false)==false)
        {
        return false;
        }
    return true;
    }

double parabola::Inverse(SGM::Point3D const &Pos,
                      SGM::Point3D       *ClosePos,
                      double       const *) const
    {
    return ParabolaInverse(m_Center, m_XAxis, m_YAxis, m_dA, Pos, ClosePos, nullptr);
    }

void parabola::Transform(SGM::Result            &,//rResult,
                         SGM::Transform3D const &Trans)
    {
    m_Center=Trans*m_Center;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_Normal=Trans*m_Normal;
    if(double dScale=Trans.Scale())
        m_dA*=dScale;
    }

}