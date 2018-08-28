#include "SGMVector.h"
#include "SGMTransform.h"
#include "Curve.h"

namespace SGMInternal
{
PointCurve::PointCurve(SGM::Result           &rResult,
                       SGM::Point3D    const &Pos,
                       SGM::Interval1D const *pDomain):
    curve(rResult,SGM::PointCurveType),m_Pos(Pos)
    {
    if(pDomain)
        {
        m_Domain=*pDomain;
        }
    else
        {
        m_Domain.m_dMin=0.0;
        m_Domain.m_dMax=0.0;
        }
    }
    
PointCurve::PointCurve(SGM::Result &rResult, const PointCurve &other) :
        curve(rResult, other),
        m_Pos(other.m_Pos)
{}

PointCurve * PointCurve::Clone(SGM::Result &rResult) const
{ return new PointCurve(rResult, *this); }

void PointCurve::Evaluate(double,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    if(Pos)
        {
        *Pos=m_Pos;
        }
    if(D1)
        {
        *D1=SGM::Vector3D(0.0,0.0,0.0);
        }
    if(D2)
        {
        *D2=SGM::Vector3D(0.0,0.0,0.0);
        }
    }

double PointCurve::Inverse(SGM::Point3D const &,
                           SGM::Point3D       *ClosePos,
                           double       const *pGuess) const
    {
    SGM::Point3D const &Origin=m_Pos;
    if(ClosePos)
        {
        *ClosePos=Origin;
        }
    if(pGuess)
        {
        double dParam=*pGuess;
        if(dParam<GetDomain().m_dMin)
            {
            return GetDomain().m_dMin;
            }
        if(dParam>GetDomain().m_dMax)
            {
            return GetDomain().m_dMax;
            }
        return dParam;
        }
    return GetDomain().m_dMin;
    }

void PointCurve::Transform(SGM::Transform3D const &Trans)
    {
    m_Pos=Trans*m_Pos;
    }

}