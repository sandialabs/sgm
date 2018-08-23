#include "SGMVector.h"
#include "SGMTransform.h"

#include "Curve.h"

namespace SGMInternal
{
hyperbola::hyperbola(SGM::Result             &rResult,
                     SGM::Point3D      const &Center,
                     SGM::UnitVector3D const &XAxis,
                     SGM::UnitVector3D const &YAxis,
                     double                   dA,
                     double                   dB):  
    curve(rResult,SGM::HyperbolaType),m_Center(Center),m_XAxis(XAxis),m_YAxis(YAxis),
    m_Normal(XAxis*YAxis),m_dA(dA),m_dB(dB)
    {
    m_Domain.m_dMin=-SGM_MAX;
    m_Domain.m_dMax=SGM_MAX;
    }

hyperbola::hyperbola(SGM::Result &rResult, hyperbola const &other):
        curve(rResult, other),
        m_Center(other.m_Center),
        m_XAxis(other.m_XAxis),
        m_YAxis(other.m_YAxis),
        m_Normal(other.m_Normal),
        m_dA(other.m_dA),
        m_dB(other.m_dB)
    {}

hyperbola *hyperbola::Clone(SGM::Result &rResult) const
    { return new hyperbola(rResult, *this); }

void hyperbola::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    // f(t)=a*sqrt(1+t^2/b^2)

    double dB2=m_dB*m_dB;
    double dR=1.0+t*t/(dB2);
    double dS=sqrt(dR);
    if(Pos)
        {
        double y=m_dA*dS;
        Pos->m_x=m_Center.m_x+m_XAxis.m_x*t+m_YAxis.m_x*y;
        Pos->m_y=m_Center.m_y+m_XAxis.m_y*t+m_YAxis.m_y*y;
        Pos->m_z=m_Center.m_z+m_XAxis.m_z*t+m_YAxis.m_z*y;
        }
    if(D1)
        {
        double dy=m_dA*t/(dB2*dS);
        D1->m_x=m_XAxis.m_x+m_YAxis.m_x*dy;
        D1->m_y=m_XAxis.m_y+m_YAxis.m_y*dy;
        D1->m_z=m_XAxis.m_z+m_YAxis.m_z*dy;
        }
    if(D2)
        {
        double ddy=-m_dA*t*t/(dB2*dB2*dR*dS);
        D2->m_x=m_YAxis.m_x*ddy;
        D2->m_y=m_YAxis.m_y*ddy;
        D2->m_z=m_YAxis.m_z*ddy;
        }
    }

double hyperbola::Inverse(SGM::Point3D const &Pos,
                          SGM::Point3D       *ClosePos,
                          double       const *) const
    {
    SGM::Point3D const &Center=m_Center;
    SGM::UnitVector3D const &XVec=m_XAxis;
    SGM::Vector3D Vec=Pos-Center;
    double dParam=XVec%Vec;
    double dAnswer=NewtonsMethod(dParam,Pos);
    if(ClosePos)
        {
        Evaluate(dAnswer,ClosePos);
        }
    return dAnswer;
    }

void hyperbola::Transform(SGM::Transform3D const &Trans)
    {
    // f(t)=a*sqrt(1+t^2/b^2)
    m_Center=Trans*m_Center;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_Normal=Trans*m_Normal;
    if (double dScale=Trans.Scale())
        {
        m_dA*=dScale;
        m_dB*=dScale;
        }
    }



}