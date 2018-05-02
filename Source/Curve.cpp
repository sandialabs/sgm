#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Topology.h"
#include "Curve.h"
#include <limits>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cfloat>

namespace SGM { namespace Impl {

curve::curve(SGM::Result &rResult,SGM::EntityType nType):
    entity(rResult,SGM::EntityType::CurveType),m_CurveType(nType) 
    {
    m_bClosed=false;
    }

void curve::AddEdge(edge *pEdge) 
    {
    m_sEdges.insert(pEdge);
    }

void curve::RemoveEdge(edge *pEdge) 
    {
    m_sEdges.erase(pEdge);
    }

curve *curve::MakeCopy(SGM::Result &rResult) const
    {
    switch(m_CurveType)
        {
        case SGM::LineType:
            {
            return new line(rResult,(line const *)this);
            }
        case SGM::CircleType:
            {
            return new circle(rResult,(circle const *)this);
            }
        default:
            {
            throw;
            }
        }
    }

double NewtonsMethod(curve        const *pCurve,
                     double              dStart,
                     SGM::Point3D const &Pos)
    {
    SGM::Point3D Origin;
    SGM::Vector3D Vec;
    pCurve->Evaluate(dStart,&Origin,&Vec);
    double dt=((Pos-Origin)%Vec)/Vec.MagnitudeSquared();
    dStart+=dt;
    while(SGM_ZERO<dt || dt<-SGM_ZERO)
        {
        pCurve->Evaluate(dStart,&Origin,&Vec);
        dt=((Pos-Origin)%Vec)/Vec.MagnitudeSquared();
        dStart+=dt;
        }
    return dStart;
    }

double curve::Inverse(SGM::Point3D const &Pos,
                      SGM::Point3D       *ClosePos,
                      double       const *pGuess) const
    {
    switch(m_CurveType)
        {
        case SGM::LineType:
            {
            line const *pLine=(line const *)this;
            SGM::Point3D const &Origin=pLine->GetOrigin();
            SGM::UnitVector3D const &Axis=pLine->GetAxis();
            double dScale=pLine->GetScale();
            double t=((Pos.m_x-Origin.m_x)*Axis.m_x+(Pos.m_y-Origin.m_y)*Axis.m_y+(Pos.m_z-Origin.m_z)*Axis.m_z)/dScale;

            if(ClosePos)
                {
                ClosePos->m_x=Origin.m_x+Axis.m_x*t*dScale;
                ClosePos->m_y=Origin.m_y+Axis.m_y*t*dScale;
                ClosePos->m_z=Origin.m_z+Axis.m_z*t*dScale;
                }
            return t;
            }
        case SGM::CircleType:
            {
            circle const *pCircle=(circle const *)this;
            SGM::Point3D const &Center=pCircle->GetCenter();
            SGM::UnitVector3D const &XAxis=pCircle->GetXAxis();
            SGM::UnitVector3D const &YAxis=pCircle->GetYAxis();
            double dRadius=pCircle->GetRadius();

            double dSpokeX=Pos.m_x-Center.m_x;
            double dSpokeY=Pos.m_y-Center.m_y;
            double dSpokeZ=Pos.m_z-Center.m_z;

            double dx=dSpokeX*XAxis.m_x+dSpokeY*XAxis.m_y+dSpokeZ*XAxis.m_z;
            double dy=dSpokeX*YAxis.m_x+dSpokeY*YAxis.m_y+dSpokeZ*YAxis.m_z;
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
                if( SGM::NearEqual(t,m_Domain.m_dMin,SGM_MIN_TOL,false)==true &&
                    SGM::NearEqual(*pGuess,m_Domain.m_dMax,SGM_MIN_TOL,false)==true)
                    {
                    t=*pGuess;
                    }
                else if(SGM::NearEqual(t,m_Domain.m_dMax,SGM_MIN_TOL,false)==true &&
                        SGM::NearEqual(*pGuess,m_Domain.m_dMin,SGM_MIN_TOL,false)==true)
                    {
                    t=*pGuess;
                    }
                }
            
            if(ClosePos)
                {
                double dCos=cos(t);
                double dSin=sin(t);

                ClosePos->m_x=Center.m_x+(XAxis.m_x*dCos+YAxis.m_x*dSin)*dRadius;
                ClosePos->m_y=Center.m_y+(XAxis.m_y*dCos+YAxis.m_y*dSin)*dRadius;
                ClosePos->m_z=Center.m_z+(XAxis.m_z*dCos+YAxis.m_z*dSin)*dRadius;
                }
            return t;
            }
        case SGM::PointCurveType:
            {
            PointCurve const *pPointCurve=(PointCurve const *)this;
            SGM::Point3D const &Origin=pPointCurve->m_Pos;
            if(ClosePos)
                {
                *ClosePos=Origin;
                }
            if(pGuess)
                {
                double dParam=*pGuess;
                if(dParam<pPointCurve->GetDomain().m_dMin)
                    {
                    return pPointCurve->GetDomain().m_dMin;
                    }
                if(dParam>pPointCurve->GetDomain().m_dMax)
                    {
                    return pPointCurve->GetDomain().m_dMax;
                    }
                return dParam;
                }
            return pPointCurve->GetDomain().m_dMin;
            }
        case SGM::EllipseType:
            {
            ellipse const *pEllipse=(ellipse const *)this;
            SGM::Point3D const &Center=pEllipse->m_Center;
            SGM::UnitVector3D const &XVec=pEllipse->m_XAxis;
            SGM::UnitVector3D const &YVec=pEllipse->m_YAxis;
            SGM::Vector3D Vec=Pos-Center;
            double dx=XVec%Vec;
            double dy=YVec%Vec;
            double dParam=SGM::SAFEatan2(dy,dx);
            double dAnswer=NewtonsMethod(this,dParam,Pos);
            if(ClosePos)
                {
                Evaluate(dAnswer,ClosePos);
                }
            if(pGuess)
                {
                if( SGM::NearEqual(dAnswer,m_Domain.m_dMin,SGM_MIN_TOL,false)==true &&
                    SGM::NearEqual(*pGuess,m_Domain.m_dMax,SGM_MIN_TOL,false)==true)
                    {
                    dAnswer=*pGuess;
                    }
                else if(SGM::NearEqual(dAnswer,m_Domain.m_dMax,SGM_MIN_TOL,false)==true &&
                        SGM::NearEqual(*pGuess,m_Domain.m_dMin,SGM_MIN_TOL,false)==true)
                    {
                    dAnswer=*pGuess;
                    }
                }
            return dAnswer;
            }
        case SGM::HyperbolaType:
            {
            hyperbola const *pHyperbola=(hyperbola const *)this;
            SGM::Point3D const &Center=pHyperbola->m_Center;
            SGM::UnitVector3D const &XVec=pHyperbola->m_XAxis;
            SGM::Vector3D Vec=Pos-Center;
            double dParam=XVec%Vec;
            double dAnswer=NewtonsMethod(this,dParam,Pos);
            if(ClosePos)
                {
                Evaluate(dAnswer,ClosePos);
                }
            return dAnswer;
            }
        case SGM::ParabolaType:
            {
            parabola const *pParabola=(parabola const *)this;
            SGM::Point3D const &Center=pParabola->m_Center;
            SGM::UnitVector3D const &XVec=pParabola->m_XAxis;
            SGM::UnitVector3D const &YVec=pParabola->m_YAxis;
            double dA=pParabola->m_dA;
            SGM::Vector3D Vec=Pos-Center;
            double Px=XVec%Vec;
            double Py=YVec%Vec;
            double a=4*dA*dA;
            double b=0.0;
            double c=2.0-4.0*dA*Py;
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
        case SGM::NUBCurveType:
            {
            double dParam=0;
            if(pGuess)
                {
                dParam=*pGuess;
                }
            else
                {
                NUBcurve const *pNUB=(NUBcurve const *)this;
                std::vector<SGM::Point3D> const &aPoints=pNUB->NUBcurve::GetSeedPoints();
                std::vector<double> const &aParams=pNUB->NUBcurve::GetSeedParams();
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
            double dAnswer=NewtonsMethod(this,dParam,Pos);
            if(ClosePos)
                {
                Evaluate(dAnswer,ClosePos);
                }
            return dAnswer;
            }
        case SGM::NURBCurveType:
            {
            double dParam=0;
            if(pGuess)
                {
                dParam=*pGuess;
                }
            else
                {
                NURBcurve const *pNURB=(NURBcurve const *)this;
                std::vector<SGM::Point3D> const &aPoints=pNURB->NURBcurve::GetSeedPoints();
                std::vector<double> const &aParams=pNURB->NURBcurve::GetSeedParams();
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
            double dAnswer=NewtonsMethod(this,dParam,Pos);
            if(ClosePos)
                {
                Evaluate(dAnswer,ClosePos);
                }
            return dAnswer;
            }
        default:
            {
            throw;
            }
        }
    }

void curve::Transform(SGM::Transform3D const &Trans)
    {
    switch(m_CurveType)
        {
        case SGM::LineType:
            {
            line *pLine=(line *)this;
            SGM::Point3D &Origin=pLine->m_Origin;
            SGM::UnitVector3D &Axis=pLine->m_Axis;
            double dScale=pLine->m_dScale;

            Origin=Trans*Origin;
            Axis=Trans*Axis;
            dScale=dScale*Trans.Scale(Axis);

            break;
            }
        case SGM::CircleType:
            {
            circle *pCircle=(circle *)this;
            SGM::Point3D &Center=pCircle->m_Center;
            SGM::UnitVector3D &XAxis=pCircle->m_XAxis;
            SGM::UnitVector3D &YAxis=pCircle->m_YAxis;
            SGM::UnitVector3D &ZAxis=pCircle->m_Normal;
            double &dRadius=pCircle->m_dRadius;

            if(double dScale=Trans.Scale())
                {
                Center=Trans*Center;
                XAxis=Trans*XAxis;
                YAxis=Trans*YAxis;
                ZAxis=Trans*ZAxis;
                dRadius*=dScale;
                }

            break;
            }
        case SGM::ParabolaType:
            {
            parabola *pParabola=(parabola *)this;
            SGM::Point3D &Center=pParabola->m_Center;
            SGM::UnitVector3D &XAxis=pParabola->m_XAxis;
            SGM::UnitVector3D &YAxis=pParabola->m_YAxis;
            double &dA=pParabola->m_dA;

            if(double dScale=Trans.Scale())
                {
                Center=Trans*Center;
                XAxis=Trans*XAxis;
                YAxis=Trans*YAxis;
                dA*=dScale;
                }

            break;
            }
        case SGM::HyperbolaType:
            {
            // f(t)=a*sqrt(1+t^2/b^2)

            hyperbola *pHyperbola=(hyperbola *)this;
            SGM::Point3D &Center=pHyperbola->m_Center;
            SGM::UnitVector3D &XAxis=pHyperbola->m_XAxis;
            SGM::UnitVector3D &YAxis=pHyperbola->m_YAxis;
            double &dA=pHyperbola->m_dA;
            double &dB=pHyperbola->m_dB;

            if(double dScale=Trans.Scale())
                {
                Center=Trans*Center;
                XAxis=Trans*XAxis;
                YAxis=Trans*YAxis;
                dA*=dScale;
                dB*=dScale;
                }

            break;
            }
        case SGM::EllipseType:
            {
            ellipse *pEllipse=(ellipse *)this;
            SGM::Point3D &Center=pEllipse->m_Center;
            SGM::UnitVector3D &XAxis=pEllipse->m_XAxis;
            SGM::UnitVector3D &YAxis=pEllipse->m_YAxis;
            double &dA=pEllipse->m_dA;
            double &dB=pEllipse->m_dB;

            if(double dScale=Trans.Scale())
                {
                Center=Trans*Center;
                XAxis=Trans*XAxis;
                YAxis=Trans*YAxis;
                dA*=dScale;
                dB*=dScale;
                }

            break;
            }
        case SGM::PointCurveType:
            {
            PointCurve *pPointCurve=(PointCurve *)this;
            SGM::Point3D &Center=pPointCurve->m_Pos;
            Center=Trans*Center;
            break;
            }
        case SGM::NUBCurveType:
            {
            NUBcurve *pNUB=(NUBcurve *)this;
            std::vector<SGM::Point3D> &aControlPoints=pNUB->m_aControlPoints;
            size_t nControlPoints=aControlPoints.size();
            size_t Index1;
            for(Index1=0;Index1<nControlPoints;++Index1)
                {
                aControlPoints[Index1]=Trans*aControlPoints[Index1];
                }
            break;
            }
        case SGM::NURBCurveType:
            {
            NURBcurve *pNURB=(NURBcurve *)this;
            std::vector<SGM::Point4D> &aControlPoints=pNURB->m_aControlPoints;
            size_t nControlPoints=aControlPoints.size();
            size_t Index1;
            for(Index1=0;Index1<nControlPoints;++Index1)
                {
                SGM::Point4D Pos=aControlPoints[Index1];
                SGM::Point3D Pos3D(Pos.m_x,Pos.m_y,Pos.m_z);
                Pos3D=Trans*Pos3D;
                aControlPoints[Index1]=SGM::Point4D(Pos3D.m_x,Pos3D.m_y,Pos3D.m_z,Pos.m_w);
                }
            break;
            }
        default:
            {
            throw;
            }
        }
    }

SGM::Vector3D curve::Curvature(double t) const
    {
    SGM::Vector3D dt,ddt;
    Evaluate(t,nullptr,&dt,&ddt);
    double dSpeed=dt%dt;
    return ((dt*ddt)*dt)/(dSpeed*dSpeed);
    }

void curve::Evaluate(double t,SGM::Point3D *Pos,SGM::Vector3D *D1,SGM::Vector3D *D2) const
    {
    switch(m_CurveType)
        {
        case SGM::LineType:
            {
            line const *pLine=(line const *)this;
            SGM::Point3D const &Origin=pLine->GetOrigin();
            SGM::UnitVector3D const &Axis=pLine->GetAxis();
            double dScale=pLine->GetScale();

            if(Pos)
                {
                Pos->m_x=Origin.m_x+Axis.m_x*t*dScale;
                Pos->m_y=Origin.m_y+Axis.m_y*t*dScale;
                Pos->m_z=Origin.m_z+Axis.m_z*t*dScale;
                }
            if(D1)
                {
                D1->m_x=Axis.m_x*dScale;
                D1->m_y=Axis.m_y*dScale;
                D1->m_z=Axis.m_z*dScale;
                }
            if(D2)
                {
                D2->m_x=0;
                D2->m_y=0;
                D2->m_z=0;
                }
            break;
            }
        case SGM::CircleType:
            {
            circle const *pCircle=(circle const *)this;
            SGM::Point3D const &Center=pCircle->GetCenter();
            SGM::UnitVector3D const &XAxis=pCircle->GetXAxis();
            SGM::UnitVector3D const &YAxis=pCircle->GetYAxis();
            double dRadius=pCircle->GetRadius();

            double dCos=cos(t);
            double dSin=sin(t);

            if(Pos)
                {
                Pos->m_x=Center.m_x+(XAxis.m_x*dCos+YAxis.m_x*dSin)*dRadius;
                Pos->m_y=Center.m_y+(XAxis.m_y*dCos+YAxis.m_y*dSin)*dRadius;
                Pos->m_z=Center.m_z+(XAxis.m_z*dCos+YAxis.m_z*dSin)*dRadius;
                }
            if(D1)
                {
                D1->m_x=(YAxis.m_x*dCos-XAxis.m_x*dSin)*dRadius;
                D1->m_y=(YAxis.m_y*dCos-XAxis.m_y*dSin)*dRadius;
                D1->m_z=(YAxis.m_z*dCos-XAxis.m_z*dSin)*dRadius;
                }
            if(D2)
                {
                D2->m_x=(-XAxis.m_x*dCos-YAxis.m_x*dSin)*dRadius;
                D2->m_y=(-XAxis.m_y*dCos-YAxis.m_y*dSin)*dRadius;
                D2->m_z=(-XAxis.m_z*dCos-YAxis.m_z*dSin)*dRadius;
                }
            break;
            }
        case SGM::ParabolaType:
            {
            parabola const *pParabola=(parabola const *)this;
            SGM::Point3D const &Center=pParabola->m_Center;
            SGM::UnitVector3D const &XAxis=pParabola->m_XAxis;
            SGM::UnitVector3D const &YAxis=pParabola->m_YAxis;
            double dA=pParabola->m_dA;

            double y=dA*t*t;

            if(Pos)
                {
                Pos->m_x=Center.m_x+XAxis.m_x*t+YAxis.m_x*y;
                Pos->m_y=Center.m_y+XAxis.m_y*t+YAxis.m_y*y;
                Pos->m_z=Center.m_z+XAxis.m_z*t+YAxis.m_z*y;
                }
            if(D1)
                {
                double dy=2.0*dA*t;
                D1->m_x=XAxis.m_x+YAxis.m_x*dy;
                D1->m_y=XAxis.m_y+YAxis.m_y*dy;
                D1->m_z=XAxis.m_z+YAxis.m_z*dy;
                }
            if(D2)
                {
                double ddy=2.0*dA;
                D2->m_x=YAxis.m_x*ddy;
                D2->m_y=YAxis.m_y*ddy;
                D2->m_z=YAxis.m_z*ddy;
                }
            break;
            }
        case SGM::HyperbolaType:
            {
            // f(t)=a*sqrt(1+t^2/b^2)

            hyperbola const *pHyperbola=(hyperbola const *)this;
            SGM::Point3D const &Center=pHyperbola->m_Center;
            SGM::UnitVector3D const &XAxis=pHyperbola->m_XAxis;
            SGM::UnitVector3D const &YAxis=pHyperbola->m_YAxis;
            double dA=pHyperbola->m_dA;
            double dB=pHyperbola->m_dB;

            double dB2=dB*dB;
            double dR=1.0+t*t/(dB2);
            double dS=sqrt(dR);
            if(Pos)
                {
                double y=dA*dS;
                Pos->m_x=Center.m_x+XAxis.m_x*t+YAxis.m_x*y;
                Pos->m_y=Center.m_y+XAxis.m_y*t+YAxis.m_y*y;
                Pos->m_z=Center.m_z+XAxis.m_z*t+YAxis.m_z*y;
                }
            if(D1)
                {
                double dy=dA*t/(dB2*dS);
                D1->m_x=XAxis.m_x+YAxis.m_x*dy;
                D1->m_y=XAxis.m_y+YAxis.m_y*dy;
                D1->m_z=XAxis.m_z+YAxis.m_z*dy;
                }
            if(D2)
                {
                double ddy=-dA*t*t/(dB2*dB2*dR*dS);
                D2->m_x=YAxis.m_x*ddy;
                D2->m_y=YAxis.m_y*ddy;
                D2->m_z=YAxis.m_z*ddy;
                }
            break;
            }
        case SGM::EllipseType:
            {
            ellipse const *pEllipse=(ellipse const *)this;
            SGM::Point3D const &Center=pEllipse->m_Center;
            SGM::UnitVector3D const &XAxis=pEllipse->m_XAxis;
            SGM::UnitVector3D const &YAxis=pEllipse->m_YAxis;
            double dA=pEllipse->m_dA;
            double dB=pEllipse->m_dB;

            double dCos=cos(t);
            double dSin=sin(t);
            double dCosA=dCos*dA;
            double dSinB=dSin*dB;

            if(Pos)
                {
                Pos->m_x=Center.m_x+XAxis.m_x*dCosA+YAxis.m_x*dSinB;
                Pos->m_y=Center.m_y+XAxis.m_y*dCosA+YAxis.m_y*dSinB;
                Pos->m_z=Center.m_z+XAxis.m_z*dCosA+YAxis.m_z*dSinB;
                }
            if(D1)
                {
                double dCosB=dCos*dB;
                double dSinA=dSin*dA;
                D1->m_x=YAxis.m_x*dCosB-XAxis.m_x*dSinA;
                D1->m_y=YAxis.m_y*dCosB-XAxis.m_y*dSinA;
                D1->m_z=YAxis.m_z*dCosB-XAxis.m_z*dSinA;
                }
            if(D2)
                {
                D2->m_x=-XAxis.m_x*dCosA-YAxis.m_x*dSinB;
                D2->m_y=-XAxis.m_y*dCosA-YAxis.m_y*dSinB;
                D2->m_z=-XAxis.m_z*dCosA-YAxis.m_z*dSinB;
                }
            break;
            }
        case SGM::PointCurveType:
            {
            PointCurve const *pPointCurve=(PointCurve const *)this;
            SGM::Point3D const &Center=pPointCurve->m_Pos;
            if(Pos)
                {
                *Pos=Center;
                }
            if(D1)
                {
                *D1=SGM::Vector3D(0.0,0.0,0.0);
                }
            if(D2)
                {
                *D2=SGM::Vector3D(0.0,0.0,0.0);
                }
            break;
            }
        case SGM::NUBCurveType:
            {
            // From "The NURBS Book", page 82, Algorithm A3.1.

            NUBcurve const *pNUB=(NUBcurve const *)this;
            std::vector<double> const &aKnots=pNUB->m_aKnots;
            std::vector<SGM::Point3D> const &aControlPoints=pNUB->m_aControlPoints;
            size_t nDegree=pNUB->GetDegree();
            size_t nSpanIndex=FindSpanIndex(m_Domain,nDegree,t,aKnots);
            size_t nDerivatives=0;
            if(D1) nDerivatives=1;
            if(D2) nDerivatives=2;
            double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
            double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
            size_t Index1;
            for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
                {
                aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
                }
            FindBasisFunctions(nSpanIndex,t,nDegree,nDerivatives,&aKnots[0],aaBasisFunctions);
            if(Pos)
                {
                double *aBasis=aaBasisFunctions[0];
                double x=0,y=0,z=0;
                for(Index1=0;Index1<=nDegree;++Index1)
                    {
                    x+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_x;
                    y+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_y;
                    z+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_z;
                    }
                Pos->m_x=x;
                Pos->m_y=y;
                Pos->m_z=z;
                }
            if(D1)
                {
                double *aBasis=aaBasisFunctions[1];
                double x=0,y=0,z=0;
                for(Index1=0;Index1<=nDegree;++Index1)
                    {
                    x+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_x;
                    y+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_y;
                    z+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_z;
                    }
                D1->m_x=x;
                D1->m_y=y;
                D1->m_z=z;
                }
            if(D2)
                {
                double *aBasis=aaBasisFunctions[2];
                double x=0,y=0,z=0;
                for(Index1=0;Index1<=nDegree;++Index1)
                    {
                    x+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_x;
                    y+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_y;
                    z+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_z;
                    }
                D2->m_x=x;
                D2->m_y=y;
                D2->m_z=z;
                }
            break;
            }
        case SGM::NURBCurveType:
            {
            // From "The NURBS Book", page 124-127, Algorithm A4.1 and A4.2.

            NURBcurve const *pNURB=(NURBcurve const *)this;
            std::vector<double> const &aKnots=pNURB->m_aKnots;
            std::vector<SGM::Point4D> const &aControlPoints=pNURB->m_aControlPoints;
            size_t nDegree=pNURB->GetDegree();
            size_t nSpanIndex=FindSpanIndex(m_Domain,nDegree,t,aKnots);
            size_t nDerivatives=0;
            if(D1) nDerivatives=1;
            if(D2) nDerivatives=2;
            double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
            double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
            size_t Index1;
            for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
                {
                aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
                }
            FindBasisFunctions(nSpanIndex,t,nDegree,nDerivatives,&aKnots[0],aaBasisFunctions);
            if(Pos)
                {
                double *aBasis=aaBasisFunctions[0];
                double x=0,y=0,z=0,w=0;
                for(Index1=0;Index1<=nDegree;++Index1)
                    {
                    x+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_x;
                    y+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_y;
                    z+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_z;
                    w+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_w;
                    }
                Pos->m_x=x/w;
                Pos->m_y=y/w;
                Pos->m_z=z/w;
                }
            if(D1)
                {
                double *aBasis=aaBasisFunctions[1];
                double x=0,y=0,z=0,w=0;
                for(Index1=0;Index1<=nDegree;++Index1)
                    {
                    x+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_x;
                    y+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_y;
                    z+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_z;
                    w+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_w;
                    }
                D1->m_x=x/w;
                D1->m_y=y/w;
                D1->m_z=z/w;
                }
            if(D2)
                {
                double *aBasis=aaBasisFunctions[2];
                double x=0,y=0,z=0,w=0;
                for(Index1=0;Index1<=nDegree;++Index1)
                    {
                    x+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_x;
                    y+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_y;
                    z+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_z;
                    w+=aBasis[Index1]*aControlPoints[nSpanIndex-nDegree+Index1].m_w;
                    }
                D2->m_x=x/w;
                D2->m_y=y/w;
                D2->m_z=z/w;
                }
            break;
            }
        default:
            {
            throw;
            }
        }
    }

}}
