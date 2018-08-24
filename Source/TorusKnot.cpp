#include "SGMVector.h"
#include "SGMTransform.h"
#include "EntityClasses.h"
#include "Curve.h"

namespace SGMInternal
{
TorusKnot::TorusKnot(SGM::Result             &rResult,
                     SGM::Point3D      const &Center,
                     SGM::UnitVector3D const &XAxis,
                     SGM::UnitVector3D const &YAxis,
                     double                   dMinorRadius,
                     double                   dMajorRadius,
                     size_t                   nA,
                     size_t                   nB):
    curve(rResult,SGM::TorusKnotCurveType),m_Center(Center),m_XAxis(XAxis),
    m_YAxis(YAxis),m_Normal(XAxis*YAxis),m_dMinorRadius(dMinorRadius),
    m_dMajorRadius(dMajorRadius),m_nA(nA),m_nB(nB)
    {                                   
    m_Domain.m_dMin=0.0;           
    m_Domain.m_dMax=SGM_TWO_PI;    
    m_bClosed=true;
    }          

TorusKnot::TorusKnot(SGM::Result &rResult, TorusKnot const &other):
        curve(rResult, other),
        m_Center(other.m_Center),
        m_XAxis(other.m_XAxis),
        m_YAxis(other.m_YAxis),
        m_Normal(m_XAxis*m_YAxis),
        m_dMinorRadius(other.m_dMinorRadius),
        m_dMajorRadius(other.m_dMajorRadius),
        m_nA(other.m_nA),
        m_nB(other.m_nB)
    {}

TorusKnot *TorusKnot::Clone(SGM::Result &rResult) const
    {
        return new TorusKnot(rResult, *this);
    }

void TorusKnot::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    double dCosA=cos(m_nA*t);
    double dSinA=sin(m_nA*t);
    double dCosB=m_dMinorRadius*cos(m_nB*t);
    double dSinB=m_dMinorRadius*sin(m_nB*t);

    if(Pos)
        {
        Pos->m_x=m_Center.m_x+(m_XAxis.m_x*dCosA+m_YAxis.m_x*dSinA)*(m_dMajorRadius+dCosB)-m_Normal.m_x*dSinB;
        Pos->m_y=m_Center.m_y+(m_XAxis.m_y*dCosA+m_YAxis.m_y*dSinA)*(m_dMajorRadius+dCosB)-m_Normal.m_y*dSinB;
        Pos->m_z=m_Center.m_z+(m_XAxis.m_z*dCosA+m_YAxis.m_z*dSinA)*(m_dMajorRadius+dCosB)-m_Normal.m_z*dSinB;
        }
    if(D1)
        {
        double dCosAnA=dCosA*m_nA;
        double dSinAnA=dSinA*m_nA;
        double dCosBnB=dCosB*m_nB;
        double dSinBnB=dSinB*m_nB;
        D1->m_x=(m_YAxis.m_x*dCosAnA-m_XAxis.m_x*dSinAnA)*(m_dMajorRadius+dCosB)-(m_XAxis.m_x*dCosA+m_YAxis.m_x*dSinA)*dSinBnB-m_Normal.m_x*dCosBnB;
        D1->m_y=(m_YAxis.m_y*dCosAnA-m_XAxis.m_y*dSinAnA)*(m_dMajorRadius+dCosB)-(m_XAxis.m_y*dCosA+m_YAxis.m_y*dSinA)*dSinBnB-m_Normal.m_y*dCosBnB;
        D1->m_z=(m_YAxis.m_z*dCosAnA-m_XAxis.m_z*dSinAnA)*(m_dMajorRadius+dCosB)-(m_XAxis.m_z*dCosA+m_YAxis.m_z*dSinA)*dSinBnB-m_Normal.m_z*dCosBnB;
        }
    if(D2)
        {
        double dCosAnA=dCosA*m_nA;
        double dSinAnA=dSinA*m_nA;
        double dCosBnB=dCosB*m_nB;
        double dSinBnB=dSinB*m_nB;
        double dCosAnAnA=dCosAnA*m_nA;
        double dSinAnAnA=dSinAnA*m_nA;
        double dCosBnBnB=dCosBnB*m_nB;
        double dSinBnBnB=dSinBnB*m_nB;
        D2->m_x=(-m_YAxis.m_x*dSinAnAnA-m_XAxis.m_x*dCosAnAnA)*(m_dMajorRadius+dCosB)-(m_YAxis.m_x*dCosAnA-m_XAxis.m_x*dSinAnA)*dSinBnB-
                (m_XAxis.m_x*dCosA+m_YAxis.m_x*dSinA)*dCosBnBnB+(m_XAxis.m_x*dSinAnA-m_YAxis.m_x*dCosAnA)*dSinBnB+m_Normal.m_x*dSinBnBnB;
        D2->m_y=(-m_YAxis.m_y*dSinAnAnA-m_XAxis.m_y*dCosAnAnA)*(m_dMajorRadius+dCosB)-(m_YAxis.m_y*dCosAnA-m_XAxis.m_y*dSinAnA)*dSinBnB-
                (m_XAxis.m_y*dCosA+m_YAxis.m_y*dSinA)*dCosBnBnB+(m_XAxis.m_y*dSinAnA-m_YAxis.m_y*dCosAnA)*dSinBnB+m_Normal.m_y*dSinBnBnB;
        D2->m_z=(-m_YAxis.m_z*dSinAnAnA-m_XAxis.m_z*dCosAnAnA)*(m_dMajorRadius+dCosB)-(m_YAxis.m_z*dCosAnA-m_XAxis.m_z*dSinAnA)*dSinBnB-
                (m_XAxis.m_z*dCosA+m_YAxis.m_z*dSinA)*dCosBnBnB+(m_XAxis.m_z*dSinAnA-m_YAxis.m_z*dCosAnA)*dSinBnB+m_Normal.m_z*dSinBnBnB;
        }
    }

double TorusKnot::Inverse(SGM::Point3D const &Pos,
                          SGM::Point3D       *ClosePos,
                          double       const *pGuess) const
    {
    double dBestT=0;
    if(pGuess)
        {
        dBestT=*pGuess;
        }
    else
        {
        SGM::UnitVector3D const &ZAxis  =m_Normal;

        // Find the u value.

        double x=Pos.m_x-m_Center.m_x;
        double y=Pos.m_y-m_Center.m_y;
        double z=Pos.m_z-m_Center.m_z;

        double dUx=x*m_XAxis.m_x+y*m_XAxis.m_y+z*m_XAxis.m_z;
        double dUy=x*m_YAxis.m_x+y*m_YAxis.m_y+z*m_YAxis.m_z;
        double du=SGM::SAFEatan2(dUy,dUx);

        size_t Index1;
        std::vector<double> aStarts;
        double t=du/m_nA;
        aStarts.push_back(t);
        for(Index1=0;Index1<m_nA;++Index1)
            {
            t+=SGM_TWO_PI;
            aStarts.push_back(t);
            }
        t=du/m_nB;
        aStarts.push_back(t);
        for(Index1=0;Index1<m_nB;++Index1)
            {
            t+=SGM_TWO_PI;
            aStarts.push_back(t);
            }

        double dBestDist=std::numeric_limits<double>::max();
        size_t nStarts=aStarts.size();
        for(Index1=0;Index1<nStarts;++Index1)
            {
            double dT=aStarts[Index1];
            double dX=m_dMajorRadius*cos(m_nA*dT);
            double dY=m_dMajorRadius*sin(m_nA*dT);
            double dZ=-m_dMinorRadius*sin(m_nB*dT);
            SGM::Point3D TPos=m_Center+m_XAxis*dX+m_YAxis*dY+ZAxis*dZ;
            double dDist=TPos.DistanceSquared(Pos);
            if(dDist<dBestDist)
                {
                dBestDist=dDist;
                dBestT=dT;
                }
            }
        }
    double dAnswer=NewtonsMethod(dBestT,Pos);
    if(dBestT<-SGM_ZERO)
        {
        dBestT+=SGM_TWO_PI;
        }
    if(SGM_TWO_PI+SGM_ZERO<dBestT)
        {
        dBestT-=SGM_TWO_PI;
        }

    if(ClosePos)
        {
        Evaluate(dAnswer,ClosePos);
        }
    return dAnswer;
    }

void TorusKnot::Transform(SGM::Transform3D const &Trans)
    {
    m_Center=Trans*m_Center;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_Normal=Trans*m_Normal;
    if(double dScale=Trans.Scale())
        {
        m_dMinorRadius*=dScale;
        m_dMajorRadius*=dScale;
        }
    }

}                                       
                                        
                                        