#include "SGMVector.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Curve.h"
#include "Faceter.h"

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
        Pos->m_x=m_Center.m_x+(m_XAxis[0]*dCosA+m_YAxis[0]*dSinA)*(m_dMajorRadius+dCosB)+m_Normal[0]*dSinB;
        Pos->m_y=m_Center.m_y+(m_XAxis[1]*dCosA+m_YAxis[1]*dSinA)*(m_dMajorRadius+dCosB)+m_Normal[1]*dSinB;
        Pos->m_z=m_Center.m_z+(m_XAxis[2]*dCosA+m_YAxis[2]*dSinA)*(m_dMajorRadius+dCosB)+m_Normal[2]*dSinB;
        }
    if(D1)
        {
        double dCosAnA=dCosA*m_nA;
        double dSinAnA=dSinA*m_nA;
        double dCosBnB=dCosB*m_nB;
        double dSinBnB=dSinB*m_nB;
        D1->m_x=(m_YAxis[0]*dCosAnA-m_XAxis[0]*dSinAnA)*(m_dMajorRadius+dCosB)-(m_XAxis[0]*dCosA+m_YAxis[0]*dSinA)*dSinBnB+m_Normal[0]*dCosBnB;
        D1->m_y=(m_YAxis[1]*dCosAnA-m_XAxis[1]*dSinAnA)*(m_dMajorRadius+dCosB)-(m_XAxis[1]*dCosA+m_YAxis[1]*dSinA)*dSinBnB+m_Normal[1]*dCosBnB;
        D1->m_z=(m_YAxis[2]*dCosAnA-m_XAxis[2]*dSinAnA)*(m_dMajorRadius+dCosB)-(m_XAxis[2]*dCosA+m_YAxis[2]*dSinA)*dSinBnB+m_Normal[2]*dCosBnB;
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
        D2->m_x=(-m_YAxis[0]*dSinAnAnA-m_XAxis[0]*dCosAnAnA)*(m_dMajorRadius+dCosB)-(m_YAxis[0]*dCosAnA-m_XAxis[0]*dSinAnA)*dSinBnB-
                (m_XAxis[0]*dCosA+m_YAxis[0]*dSinA)*dCosBnBnB+(m_XAxis[0]*dSinAnA-m_YAxis[0]*dCosAnA)*dSinBnB-m_Normal[0]*dSinBnBnB;
        D2->m_y=(-m_YAxis[1]*dSinAnAnA-m_XAxis[1]*dCosAnAnA)*(m_dMajorRadius+dCosB)-(m_YAxis[1]*dCosAnA-m_XAxis[1]*dSinAnA)*dSinBnB-
                (m_XAxis[1]*dCosA+m_YAxis[1]*dSinA)*dCosBnBnB+(m_XAxis[1]*dSinAnA-m_YAxis[1]*dCosAnA)*dSinBnB-m_Normal[1]*dSinBnBnB;
        D2->m_z=(-m_YAxis[2]*dSinAnAnA-m_XAxis[2]*dCosAnAnA)*(m_dMajorRadius+dCosB)-(m_YAxis[2]*dCosAnA-m_XAxis[2]*dSinAnA)*dSinBnB-
                (m_XAxis[2]*dCosA+m_YAxis[2]*dSinA)*dCosBnBnB+(m_XAxis[2]*dSinAnA-m_YAxis[2]*dCosAnA)*dSinBnB-m_Normal[2]*dSinBnBnB;
        }
    }

std::vector<SGM::Point3D> const &TorusKnot::GetSeedPoints() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedPoints;
    }

std::vector<double> const &TorusKnot::GetSeedParams() const
    {
    if(m_aSeedPoints.empty())
        {
        FacetOptions Options;
        Options.m_dEdgeAngleTol=SEED_POINT_EDGE_ANGLE_TOL;
        FacetCurve(this,m_Domain,Options,m_aSeedPoints,m_aSeedParams);
        }
    return m_aSeedParams;
    }

double TorusKnot::Inverse(SGM::Point3D const &Pos,
                          SGM::Point3D       *ClosePos,
                          double       const *pGuess) const
    {
    double dParam=0;
    if(pGuess)
        {
        dParam=*pGuess;
        }
    else
        {
        std::vector<SGM::Point3D> const &aPoints=GetSeedPoints();
        std::vector<double> const &aParams=GetSeedParams();
        double dMin=std::numeric_limits<double>::max();
        size_t Index1;
        size_t nPoints=aPoints.size();
        for(Index1=0;Index1<nPoints;++Index1)
            {
            SGM::Point3D const &TestPos=aPoints[Index1];
            double dDist=TestPos.DistanceSquared(Pos);
            if(dDist<dMin)
                {
                dMin=dDist;
                dParam=aParams[Index1];
                }
            }
        }
    double dAnswer=NewtonsMethod(dParam,Pos);
    if(ClosePos)
        {
        Evaluate(dAnswer,ClosePos);
        }
    return dAnswer;
    }

void TorusKnot::Transform(SGM::Result            &,//rResult,
                          SGM::Transform3D const &Trans)
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

bool TorusKnot::IsSame(curve const *pOther,double dTolerance) const
    {
    if(pOther->GetCurveType()!=m_CurveType)
        {
        return false;
        }
    auto pTorusKnot2=(TorusKnot const *)pOther;
    if(m_nA!=pTorusKnot2->m_nA)
        {
        return false;
        }
    if(m_nB!=pTorusKnot2->m_nB)
        {
        return false;
        }
    if(SGM::NearEqual(m_dMajorRadius,pTorusKnot2->m_dMajorRadius,dTolerance,false)==false)
        {
        return false;
        }
    else if(SGM::NearEqual(m_dMinorRadius,pTorusKnot2->m_dMinorRadius,dTolerance,false)==false)
        {
        return false;
        }
    else if(SGM::NearEqual(m_Normal,pTorusKnot2->m_Normal,dTolerance)==false)
        {
        return false;
        }
    else if(SGM::NearEqual(m_XAxis,pTorusKnot2->m_XAxis,dTolerance)==false)
        {
        return false;
        }
    else if(SGM::NearEqual(m_YAxis,pTorusKnot2->m_YAxis,dTolerance)==false)
        {
        return false;
        }
    else if(SGM::NearEqual(m_Center,pTorusKnot2->m_Center,dTolerance)==false)
        {
        return false;
        }
    return true;
    }
}                                       
                                        
                                        