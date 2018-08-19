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

parabola::parabola(SGM::Result &rResult, parabola const *other):
            curve(rResult,other),
            m_Center(other->m_Center),
            m_XAxis(other->m_XAxis),
            m_YAxis(other->m_YAxis),
            m_Normal(other->m_Normal),
            m_dA(other->m_dA)
    {}

parabola *parabola::Clone(SGM::Result &rResult) const
    { return new parabola(rResult,this); }

void parabola::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    double y=m_dA*t*t;

    if(Pos)
        {
        Pos->m_x=m_Center.m_x+m_XAxis.m_x*t+m_YAxis.m_x*y;
        Pos->m_y=m_Center.m_y+m_XAxis.m_y*t+m_YAxis.m_y*y;
        Pos->m_z=m_Center.m_z+m_XAxis.m_z*t+m_YAxis.m_z*y;
        }
    if(D1)
        {
        double dy=2.0*m_dA*t;
        D1->m_x=m_XAxis.m_x+m_YAxis.m_x*dy;
        D1->m_y=m_XAxis.m_y+m_YAxis.m_y*dy;
        D1->m_z=m_XAxis.m_z+m_YAxis.m_z*dy;
        }
    if(D2)
        {
        double ddy=2.0*m_dA;
        D2->m_x=m_YAxis.m_x*ddy;
        D2->m_y=m_YAxis.m_y*ddy;
        D2->m_z=m_YAxis.m_z*ddy;
        }
    }

double parabola::Inverse(SGM::Point3D const &Pos,
                      SGM::Point3D       *ClosePos,
                      double       const *) const
    {
    SGM::Vector3D Vec=Pos-m_Center;
    double Px=m_XAxis%Vec;
    double Py=m_YAxis%Vec;
    double a=4*m_dA*m_dA;
    double b=0.0;
    double c=2.0-4.0*m_dA*Py;
    double d=-2.0*Px;
    std::vector<double> aRoots;
    size_t nRoots=SGM::Cubic(a,b,c,d,aRoots);
    double dAnswer=0.0;
    double dMin=std::numeric_limits<double>::max();
    size_t Index1;
    for(Index1=0;Index1<nRoots;++Index1)
        {
        double t=aRoots[Index1];
        SGM::Point3D CPos;
        Evaluate(t,&CPos);
        double dDist=CPos.DistanceSquared(Pos);
        if(dDist<dMin)
            {
            dMin=dDist;
            dAnswer=t;
            if(ClosePos)
                {
                *ClosePos=CPos;
                }
            }
        }
    return dAnswer;
    }

void parabola::Transform(SGM::Transform3D const &Trans)
    {
    m_Center=Trans*m_Center;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_Normal=Trans*m_Normal;
    if(double dScale=Trans.Scale())
        m_dA*=dScale;
    }


}