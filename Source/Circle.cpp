#include "SGMMathematics.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Curve.h"

namespace SGMInternal
{

circle::circle(SGM::Result &rResult,
               SGM::Point3D const &Center,
               SGM::UnitVector3D const &Normal,
               double dRadius,
               SGM::UnitVector3D const *pXAxis,
               SGM::Interval1D const *pDomain) :
        curve(rResult, SGM::CircleType),
        m_Center(Center),
        m_Normal(Normal),
        m_dRadius(dRadius)
    {
    if (pXAxis)
        {
        m_XAxis = *pXAxis;
        }
    else
        {
        m_XAxis = Normal.Orthogonal();
        }
    m_YAxis = Normal * m_XAxis;
    if (pDomain)
        {
        m_Domain = *pDomain;
        }
    else
        {
        m_Domain.m_dMin = 0;
        m_Domain.m_dMax = SGM_TWO_PI;
        }

    if (SGM::NearEqual(m_Domain.Length(), SGM_TWO_PI, SGM_MIN_TOL, false))
        {
        m_bClosed = true;
        }
    }

circle::circle(SGM::Result &rResult, circle const &other) :
        curve(rResult, other),
        m_Center(other.m_Center),
        m_Normal(other.m_Normal),
        m_XAxis(other.m_XAxis),
        m_YAxis(other.m_YAxis),
        m_dRadius(other.m_dRadius)
    {}

circle *circle::Clone(SGM::Result &rResult) const
    { return new circle(rResult,*this); }

void circle::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    double dCos=cos(t);
    double dSin=sin(t);

    if(Pos)
        {
        Pos->m_x=m_Center[0]+(m_XAxis[0]*dCos+m_YAxis[0]*dSin)*m_dRadius;
        Pos->m_y=m_Center[1]+(m_XAxis[1]*dCos+m_YAxis[1]*dSin)*m_dRadius;
        Pos->m_z=m_Center[2]+(m_XAxis[2]*dCos+m_YAxis[2]*dSin)*m_dRadius;
        }
    if(D1)
        {
        D1->m_x=(m_YAxis[0]*dCos-m_XAxis[0]*dSin)*m_dRadius;
        D1->m_y=(m_YAxis[1]*dCos-m_XAxis[1]*dSin)*m_dRadius;
        D1->m_z=(m_YAxis[2]*dCos-m_XAxis[2]*dSin)*m_dRadius;
        }
    if(D2)
        {
        D2->m_x=(-m_XAxis[0]*dCos-m_YAxis[0]*dSin)*m_dRadius;
        D2->m_y=(-m_XAxis[1]*dCos-m_YAxis[1]*dSin)*m_dRadius;
        D2->m_z=(-m_XAxis[2]*dCos-m_YAxis[2]*dSin)*m_dRadius;
        }
    }

double circle::Inverse(SGM::Point3D const &Pos,
                      SGM::Point3D       *ClosePos,
                      double       const *pGuess) const
    {
    SGM::Point3D const &Center=GetCenter();
    SGM::UnitVector3D const &XAxis=GetXAxis();
    SGM::UnitVector3D const &YAxis=GetYAxis();
    double dRadius=GetRadius();

    double dSpokeX=Pos.m_x-Center.m_x;
    double dSpokeY=Pos.m_y-Center.m_y;
    double dSpokeZ=Pos.m_z-Center.m_z;

    double dx=dSpokeX*XAxis[0]+dSpokeY*XAxis[1]+dSpokeZ*XAxis[2];
    double dy=dSpokeX*YAxis[0]+dSpokeY*YAxis[1]+dSpokeZ*YAxis[2];
    double t=std::atan2(dy,dx);

    while(t<m_Domain.m_dMin)
        {
        t+=SGM_TWO_PI;
        }
    while(m_Domain.m_dMax<t)
        {
        t-=SGM_TWO_PI;
        }
    if(pGuess)
        {
        if (SGM::NearEqual(t, m_Domain.m_dMin, SGM_MIN_TOL, false) &&
            SGM::NearEqual(*pGuess, m_Domain.m_dMax, SGM_MIN_TOL, false))
            {
            t = *pGuess;
            }
        else if (SGM::NearEqual(t, m_Domain.m_dMax, SGM_MIN_TOL, false) &&
                 SGM::NearEqual(*pGuess, m_Domain.m_dMin, SGM_MIN_TOL, false))
            {
            t=*pGuess;
            }
        }

    if(ClosePos)
        {
        double dCos=cos(t);
        double dSin=sin(t);

        ClosePos->m_x=Center.m_x+(XAxis[0]*dCos+YAxis[0]*dSin)*dRadius;
        ClosePos->m_y=Center.m_y+(XAxis[1]*dCos+YAxis[1]*dSin)*dRadius;
        ClosePos->m_z=Center.m_z+(XAxis[2]*dCos+YAxis[2]*dSin)*dRadius;
        }
    return t;
    }

void circle::Transform(SGM::Result            &,//rResult,
                       SGM::Transform3D const &Trans)
    {
    m_Center=Trans*m_Center;
    m_Normal=Trans*m_Normal;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    if(double dScale=Trans.Scale())
        m_dRadius*=dScale;
    }

bool circle::IsSame(curve const *pOther,double dTolerance) const
    {
    if(pOther->GetCurveType()!=m_CurveType)
        {
        return false;
        }
    auto pCurve2=(circle const *)pOther;
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
    if(SGM::NearEqual(m_dRadius,pCurve2->m_dRadius,dTolerance,false)==false)
        {
        return false;
        }
    return true;
    }

double circle::FindLength(SGM::Interval1D const &Domain,double ) const //dTolerance) const
    {
    return Domain.Length()*m_dRadius;
    }

}
