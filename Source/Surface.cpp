#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"
#include "Topology.h"
#include <cmath>
#include <algorithm>
#include <limits>

surface::surface(SGM::Result &rResult,SGM::EntityType nType):
    entity(rResult,SGM::EntityType::SurfaceType),m_SurfaceType(nType)
    {
    m_bClosedU=false;
    m_bClosedV=false;
    m_bSingularLowU=false;
    m_bSingularHighU=false;
    m_bSingularLowV=false;
    m_bSingularHighV=false;
    }

void surface::AddFace(face *pFace) 
    {
    void* other = (void*)this;
    if(other)
        {
        m_sFaces.insert(pFace);
        }
    }

void surface::RemoveFace(face *pFace) 
    {
    void* other = (void*)this;
    if(other)
        {
        m_sFaces.erase(pFace);
        }
    }

curve *surface::UParamLine(SGM::Result &rResult,
                           double       dU) const
    {
    switch(m_SurfaceType)
        {
        case SGM::PlaneType:
            {
            break;
            }
        case SGM::ConeType:
            {
            break;
            }
        case SGM::CylinderType:
            {
            cylinder const *pCylinder=(cylinder const *)this;
            double dRadius=pCylinder->m_dRadius;
            SGM::Point2D uv(dU,0);
            SGM::Point3D Pos;
            SGM::Vector3D Vec;
            Evaluate(uv,&Pos,nullptr,&Vec);
            return new line(rResult,Pos,Vec,dRadius);
            }
        case SGM::SphereType:
            {
            sphere const *pSphere=(sphere const *)this;
            SGM::Point2D uv(dU,0);
            SGM::Point3D Pos;
            Evaluate(uv,&Pos);
            SGM::Point3D const &Center=pSphere->m_Center;
            SGM::UnitVector3D const &ZAxis=pSphere->m_ZAxis;
            double dRadius=pSphere->m_dRadius;
            SGM::UnitVector3D XAxis=Pos-Center;
            SGM::UnitVector3D Normal=XAxis*ZAxis;
            SGM::Interval2D const &Domain=pSphere->GetDomain();
            return new circle(rResult,Center,Normal,dRadius,&XAxis,&Domain.m_VDomain);
            break;
            }
        case SGM::TorusType:
            {
            torus const *pTorus=(torus const *)this;
            SGM::Interval2D const &Domain=pTorus->GetDomain();
            SGM::Point2D uv0(dU,Domain.m_VDomain.MidPoint(0.0));
            SGM::Point2D uv1(dU,Domain.m_VDomain.MidPoint(0.3));
            SGM::Point2D uv2(dU,Domain.m_VDomain.MidPoint(0.7));
            SGM::Point3D Pos0,Pos1,Pos2;
            Evaluate(uv0,&Pos0);
            Evaluate(uv1,&Pos1);
            Evaluate(uv2,&Pos2);
            SGM::Point3D Center;
            SGM::UnitVector3D Normal;
            double dRadius;
            SGM::FindCircle(Pos0,Pos1,Pos2,Center,Normal,dRadius);
            SGM::UnitVector3D XAxis=Pos0-Center;
            return new circle(rResult,Center,Normal,dRadius,&XAxis);
            break;
            }
        case SGM::NUBSurfaceType:
            {
            NUBsurface const *pNUBSurface=(NUBsurface const *)this;
            if(pNUBSurface->m_bSingularHighU && SGM::NearEqual(dU,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false))
                {
                SGM::Point3D Pos;
                pNUBSurface->Evaluate(SGM::Point2D(dU,m_Domain.m_VDomain.m_dMin),&Pos);
                return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
                }
            else if(pNUBSurface->m_bSingularLowU && SGM::NearEqual(dU,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false))
                {
                SGM::Point3D Pos;
                pNUBSurface->Evaluate(SGM::Point2D(dU,m_Domain.m_VDomain.m_dMin),&Pos);
                return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
                }
            else
                {
                std::vector<double> const &aUKnots=pNUBSurface->GetUKnots();
                std::vector<std::vector<SGM::Point3D> > const &aaControlPoints=pNUBSurface->GetControlPoints();
                size_t nUDegree=pNUBSurface->GetUDegree();
                size_t nSpanIndex=FindSpanIndex(m_Domain.m_UDomain,nUDegree,dU,aUKnots);

                double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
                double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
                size_t Index1,Index2;
                for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
                    {
                    aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
                    }
                FindBasisFunctions(nSpanIndex,dU,nUDegree,0,&aUKnots[0],aaBasisFunctions);

                std::vector<SGM::Point3D> aControlPoints;
                size_t nControlPoints=aaControlPoints[0].size();
                aControlPoints.assign(nControlPoints,SGM::Point3D(0,0,0));
                for(Index1=0;Index1<nControlPoints;++Index1)
                    {
                    for(Index2=0;Index2<=nUDegree;++Index2)
                        {
                        aControlPoints[Index1]+=aaBasisFunctions[0][Index2]*
                            SGM::Vector3D(aaControlPoints[nSpanIndex-nUDegree+Index2][Index1]);
                        }
                    }
                return new NUBcurve(rResult,aControlPoints,aUKnots);
                }
            break;
            }
        default:
            {
            throw;
            }
        }
    return nullptr;
    }

curve *surface::VParamLine(SGM::Result &rResult,
                           double       dV) const
    {
    switch(m_SurfaceType)
        {
        case SGM::PlaneType:
            {
            break;
            }
        case SGM::ConeType:
            {
            break;
            }
        case SGM::CylinderType:
            {
            break;
            }
        case SGM::SphereType:
            {
            break;
            }
        case SGM::TorusType:
            {
            torus const *pTorus=(torus const *)this;
            SGM::Interval2D const &Domain=pTorus->GetDomain();
            SGM::Point2D uv0(Domain.m_UDomain.MidPoint(0.0),dV);
            SGM::Point2D uv1(Domain.m_UDomain.MidPoint(0.3),dV);
            SGM::Point2D uv2(Domain.m_UDomain.MidPoint(0.7),dV);
            SGM::Point3D Pos0,Pos1,Pos2;
            Evaluate(uv0,&Pos0);
            Evaluate(uv1,&Pos1);
            Evaluate(uv2,&Pos2);
            SGM::Point3D Center;
            SGM::UnitVector3D Normal;
            double dRadius;
            SGM::FindCircle(Pos0,Pos1,Pos2,Center,Normal,dRadius);
            SGM::UnitVector3D XAxis=Pos0-Center;
            return new circle(rResult,Center,Normal,dRadius,&XAxis);
            break;
            }
        case SGM::NUBSurfaceType:
            {
            NUBsurface const *pNUBSurface=(NUBsurface const *)this;
            if(pNUBSurface->m_bSingularHighV && SGM::NearEqual(m_Domain.m_VDomain.m_dMax,dV,SGM_MIN_TOL,false))
                {
                SGM::Point3D Pos;
                pNUBSurface->Evaluate(SGM::Point2D(dV,m_Domain.m_VDomain.m_dMin),&Pos);
                return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
                }
            else if(pNUBSurface->m_bSingularLowV && SGM::NearEqual(m_Domain.m_VDomain.m_dMin,dV,SGM_MIN_TOL,false))
                {
                SGM::Point3D Pos;
                pNUBSurface->Evaluate(SGM::Point2D(dV,m_Domain.m_VDomain.m_dMin),&Pos);
                return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
                }
            else
                {
                std::vector<double> const &aVKnots=pNUBSurface->GetVKnots();
                std::vector<std::vector<SGM::Point3D> > const &aaControlPoints=pNUBSurface->GetControlPoints();
                size_t nVDegree=pNUBSurface->GetVDegree();
                size_t nSpanIndex=FindSpanIndex(m_Domain.m_VDomain,nVDegree,dV,aVKnots);

                double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
                double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
                size_t Index1,Index2;
                for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
                    {
                    aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
                    }
                FindBasisFunctions(nSpanIndex,dV,nVDegree,0,&aVKnots[0],aaBasisFunctions);

                std::vector<SGM::Point3D> aControlPoints;
                size_t nControlPoints=aaControlPoints.size();
                aControlPoints.assign(nControlPoints,SGM::Point3D(0,0,0));
                for(Index1=0;Index1<nControlPoints;++Index1)
                    {
                    for(Index2=0;Index2<=nVDegree;++Index2)
                        {
                        aControlPoints[Index1]+=aaBasisFunctions[0][Index2]*
                            SGM::Vector3D(aaControlPoints[Index1][nSpanIndex-nVDegree+Index2]);
                        }
                    }
                return new NUBcurve(rResult,aControlPoints,aVKnots);
                }
            break;
            }
        default:
            {
            throw;
            }
        }
    return nullptr;
    }

void surface::Evaluate(SGM::Point2D const &uv,
                       SGM::Point3D       *Pos,
                       SGM::Vector3D      *Du,
                       SGM::Vector3D      *Dv,
                       SGM::UnitVector3D  *Norm,
                       SGM::Vector3D      *Duu,
                       SGM::Vector3D      *Duv,
                       SGM::Vector3D      *Dvv) const

    {
    switch(m_SurfaceType)
        {
        case SGM::PlaneType:
            {
            plane const *pPlane=(plane *)this;

            SGM::Point3D      const &Origin=pPlane->m_Origin;
            SGM::UnitVector3D const &XAxis =pPlane->m_XAxis;
            SGM::UnitVector3D const &YAxis =pPlane->m_YAxis;
            SGM::UnitVector3D const &ZAxis =pPlane->m_ZAxis;
            double                   dScale=pPlane->m_dScale;

            if(Pos)
                {
                Pos->m_x=Origin.m_x+(XAxis.m_x*uv.m_u+YAxis.m_x*uv.m_v)*dScale;
                Pos->m_y=Origin.m_y+(XAxis.m_y*uv.m_u+YAxis.m_y*uv.m_v)*dScale;
                Pos->m_z=Origin.m_z+(XAxis.m_z*uv.m_u+YAxis.m_z*uv.m_v)*dScale;
                }
            if(Du)
                {
                Du->m_x=XAxis.m_x*dScale;
                Du->m_y=XAxis.m_y*dScale;
                Du->m_z=XAxis.m_z*dScale;
                }
            if(Dv)
                {
                Dv->m_x=YAxis.m_x*dScale;
                Dv->m_y=YAxis.m_y*dScale;
                Dv->m_z=YAxis.m_z*dScale;
                }
            if(Norm)
                {
                *Norm=ZAxis;
                }
            if(Duu)
                {
                Duu->m_x=0;
                Duu->m_y=0;
                Duu->m_z=0;
                }
            if(Duv)
                {
                Duv->m_x=0;
                Duv->m_y=0;
                Duv->m_z=0;
                }
            if(Dvv)
                {
                Dvv->m_x=0;
                Dvv->m_y=0;
                Dvv->m_z=0;
                }
            break;
            }
        case SGM::CylinderType:
            {
            cylinder const *pCylinder=(cylinder *)this;

            SGM::Point3D      const &Center =pCylinder->m_Origin;
            SGM::UnitVector3D const &XAxis  =pCylinder->m_XAxis;
            SGM::UnitVector3D const &YAxis  =pCylinder->m_YAxis;
            SGM::UnitVector3D const &ZAxis  =pCylinder->m_ZAxis;
            double                   dRadius=pCylinder->m_dRadius;

            double dCos=cos(uv.m_u);
            double dSin=sin(uv.m_u);

            if(Pos)
                {
                Pos->m_x=Center.m_x+(XAxis.m_x*dCos+YAxis.m_x*dSin+ZAxis.m_x*uv.m_v)*dRadius;
                Pos->m_y=Center.m_y+(XAxis.m_y*dCos+YAxis.m_y*dSin+ZAxis.m_y*uv.m_v)*dRadius;
                Pos->m_z=Center.m_z+(XAxis.m_z*dCos+YAxis.m_z*dSin+ZAxis.m_z*uv.m_v)*dRadius;
                }
            if(Du)
                {
                Du->m_x=(YAxis.m_x*dCos-XAxis.m_x*dSin)*dRadius;
                Du->m_y=(YAxis.m_y*dCos-XAxis.m_y*dSin)*dRadius;
                Du->m_z=(YAxis.m_z*dCos-XAxis.m_z*dSin)*dRadius;
                }
            if(Dv)
                {
                Dv->m_x=ZAxis.m_x*dRadius;
                Dv->m_y=ZAxis.m_y*dRadius;
                Dv->m_z=ZAxis.m_z*dRadius;
                }
            if(Norm)
                {
                Norm->m_x=XAxis.m_x*dCos+YAxis.m_x*dSin;
                Norm->m_y=XAxis.m_y*dCos+YAxis.m_y*dSin;
                Norm->m_z=XAxis.m_z*dCos+YAxis.m_z*dSin;
                }
            if(Duu)
                {
                Duu->m_x=(-XAxis.m_x*dCos-YAxis.m_x*dSin)*dRadius;
                Duu->m_y=(-XAxis.m_y*dCos-YAxis.m_y*dSin)*dRadius;
                Duu->m_z=(-XAxis.m_z*dCos-YAxis.m_z*dSin)*dRadius;
                }
            if(Duv)
                {
                Duv->m_x=0;
                Duv->m_y=0;
                Duv->m_z=0;
                }
            if(Dvv)
                {
                Dvv->m_x=0;
                Dvv->m_y=0;
                Dvv->m_z=0;
                }
            break;
            }
        case SGM::SphereType:
            {
            sphere const *pSphere=(sphere *)this;

            SGM::Point3D      const &Center =pSphere->m_Center;
            SGM::UnitVector3D const &XAxis  =pSphere->m_XAxis;
            SGM::UnitVector3D const &YAxis  =pSphere->m_YAxis;
            SGM::UnitVector3D const &ZAxis  =pSphere->m_ZAxis;
            double                   dRadius=pSphere->m_dRadius;

            double dCosU=cos(uv.m_u);
            double dSinU=sin(uv.m_u);
            double dCosV=cos(uv.m_v);
            double dSinV=sin(uv.m_v);

            if(Pos)
                {
                Pos->m_x=Center.m_x+((XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dCosV+ZAxis.m_x*dSinV)*dRadius;
                Pos->m_y=Center.m_y+((XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dCosV+ZAxis.m_y*dSinV)*dRadius;
                Pos->m_z=Center.m_z+((XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dCosV+ZAxis.m_z*dSinV)*dRadius;
                }
            if(Du)
                {
                Du->m_x=(YAxis.m_x*dCosU-XAxis.m_x*dSinU)*dCosV*dRadius;
                Du->m_y=(YAxis.m_y*dCosU-XAxis.m_y*dSinU)*dCosV*dRadius;
                Du->m_z=(YAxis.m_z*dCosU-XAxis.m_z*dSinU)*dCosV*dRadius;
                }
            if(Dv)
                {
                Dv->m_x=(ZAxis.m_x*dCosV-(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dSinV)*dRadius;
                Dv->m_y=(ZAxis.m_y*dCosV-(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dSinV)*dRadius;
                Dv->m_z=(ZAxis.m_z*dCosV-(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dSinV)*dRadius;
                }
            if(Norm)
                {
                Norm->m_x=(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dCosV+ZAxis.m_x*dSinV;
                Norm->m_y=(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dCosV+ZAxis.m_y*dSinV;
                Norm->m_z=(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dCosV+ZAxis.m_z*dSinV;
                }
            if(Duu)
                {
                Duu->m_x=(-YAxis.m_x*dSinU-XAxis.m_x*dCosU)*dCosV*dRadius;
                Duu->m_y=(-YAxis.m_y*dSinU-XAxis.m_y*dCosU)*dCosV*dRadius;
                Duu->m_z=(-YAxis.m_z*dSinU-XAxis.m_z*dCosU)*dCosV*dRadius;
                }
            if(Duv)
                {
                Duv->m_x=(XAxis.m_x*dSinU-YAxis.m_x*dCosU)*dSinV*dRadius;
                Duv->m_y=(XAxis.m_y*dSinU-YAxis.m_y*dCosU)*dSinV*dRadius;
                Duv->m_z=(XAxis.m_z*dSinU-YAxis.m_z*dCosU)*dSinV*dRadius;
                }
            if(Dvv)
                {
                Dvv->m_x=(-ZAxis.m_x*dSinV-(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dCosV)*dRadius;
                Dvv->m_y=(-ZAxis.m_y*dSinV-(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dCosV)*dRadius;
                Dvv->m_z=(-ZAxis.m_z*dSinV-(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dCosV)*dRadius;
                }
            break;
            }
        case SGM::TorusType:
            {
            torus const *pTorus=(torus *)this;

            SGM::Point3D      const &Center =pTorus->m_Center;
            SGM::UnitVector3D const &XAxis  =pTorus->m_XAxis;
            SGM::UnitVector3D const &YAxis  =pTorus->m_YAxis;
            SGM::UnitVector3D const &ZAxis  =pTorus->m_ZAxis;
            double                   dMinorRadius=pTorus->m_dMinorRadius;
            double                   dMajorRadius=pTorus->m_dMajorRadius;

            double dCosU=cos(uv.m_u);
            double dSinU=sin(uv.m_u);
            double dCosV=cos(uv.m_v);
            double dSinV=sin(uv.m_v);

            double dMCV=dMajorRadius+dCosV*dMinorRadius;
            if(Pos)
                {
                double dSM=dSinV*dMinorRadius;
                Pos->m_x=Center.m_x+(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dMCV+ZAxis.m_x*dSM;
                Pos->m_y=Center.m_y+(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dMCV+ZAxis.m_y*dSM;
                Pos->m_z=Center.m_z+(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dMCV+ZAxis.m_z*dSM;
                }
            if(Du)
                {
                Du->m_x=(YAxis.m_x*dCosU-XAxis.m_x*dSinU)*dMCV;
                Du->m_y=(YAxis.m_y*dCosU-XAxis.m_y*dSinU)*dMCV;
                Du->m_z=(YAxis.m_z*dCosU-XAxis.m_z*dSinU)*dMCV;
                }
            if(Dv)
                {
                double dVdMCV=-dSinV*dMinorRadius;
                double dVdSM=dCosV*dMinorRadius;
                Dv->m_x=(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dVdMCV+ZAxis.m_x*dVdSM;
                Dv->m_y=(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dVdMCV+ZAxis.m_y*dVdSM;
                Dv->m_z=(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dVdMCV+ZAxis.m_z*dVdSM;
                }
            if(Norm)
                {
                double dCM=dCosV*dMinorRadius;
                double dSM=dSinV*dMinorRadius;
                Norm->m_x=(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dCM+ZAxis.m_x*dSM;
                Norm->m_y=(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dCM+ZAxis.m_y*dSM;
                Norm->m_z=(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dCM+ZAxis.m_z*dSM;
                double dScale=1.0/sqrt(Norm->m_x*Norm->m_x+Norm->m_y*Norm->m_y+Norm->m_z*Norm->m_z);
                Norm->m_x*=dScale;
                Norm->m_y*=dScale;
                Norm->m_z*=dScale;
                }
            if(Duu)
                {
                Duu->m_x=(-YAxis.m_x*dSinU-XAxis.m_x*dCosU)*dMCV;
                Duu->m_y=(-YAxis.m_y*dSinU-XAxis.m_y*dCosU)*dMCV;
                Duu->m_z=(-YAxis.m_z*dSinU-XAxis.m_z*dCosU)*dMCV;
                }
            if(Duv)
                {
                double dVdMCV=-dSinV*dMinorRadius;
                Duv->m_x=(YAxis.m_x*dCosU-XAxis.m_x*dSinU)*dVdMCV;
                Duv->m_y=(YAxis.m_y*dCosU-XAxis.m_y*dSinU)*dVdMCV;
                Duv->m_z=(YAxis.m_z*dCosU-XAxis.m_z*dSinU)*dVdMCV;
                }
            if(Dvv)
                {
                double ddVdMCV=-dCosV*dMinorRadius;
                double ddVdSM=-dSinV*dMinorRadius;
                Dvv->m_x=(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*ddVdMCV+ZAxis.m_x*ddVdSM;
                Dvv->m_y=(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*ddVdMCV+ZAxis.m_y*ddVdSM;
                Dvv->m_z=(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*ddVdMCV+ZAxis.m_z*ddVdSM;
                }
            break;
            }
        case SGM::ConeType:
            {
            cone const *pCone=(cone *)this;

            SGM::Point3D      const &Center=pCone->m_Origin;
            SGM::UnitVector3D const &XAxis =pCone->m_XAxis;
            SGM::UnitVector3D const &YAxis =pCone->m_YAxis;
            SGM::UnitVector3D const &ZAxis =pCone->m_ZAxis;
            double            dSinHalfAngle=pCone->m_dSinHalfAngle;
            double            dCosHalfAngle=pCone->m_dCosHalfAngle;
            double                  dRadius=pCone->m_dRadius;

            double dVScale=dRadius*(1.0-uv.m_v*dSinHalfAngle);
            double dCosU=cos(uv.m_u);
            double dSinU=sin(uv.m_u);
            double dZScale=uv.m_v*dRadius*dCosHalfAngle;
                
            if(Pos)
                {
                Pos->m_x=Center.m_x+(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dVScale+ZAxis.m_x*dZScale;
                Pos->m_y=Center.m_y+(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dVScale+ZAxis.m_y*dZScale;
                Pos->m_z=Center.m_z+(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dVScale+ZAxis.m_z*dZScale;
                }
            if(Du)
                {
                Du->m_x=(YAxis.m_x*dCosU-XAxis.m_x*dSinU)*dVScale;
                Du->m_y=(YAxis.m_y*dCosU-XAxis.m_y*dSinU)*dVScale;
                Du->m_z=(YAxis.m_z*dCosU-XAxis.m_z*dSinU)*dVScale;
                }
            if(Dv)
                {
                double dDVScale=-dRadius*dSinHalfAngle;
                double dDZScale=dRadius*dCosHalfAngle;
                Dv->m_x=(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dDVScale+ZAxis.m_x*dDZScale;
                Dv->m_y=(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dDVScale+ZAxis.m_y*dDZScale;
                Dv->m_z=(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dDVScale+ZAxis.m_z*dDZScale;
                }
            if(Norm)
                {
                Norm->m_x=(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dCosHalfAngle+ZAxis.m_x*dSinHalfAngle;
                Norm->m_y=(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dCosHalfAngle+ZAxis.m_y*dSinHalfAngle;
                Norm->m_z=(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dCosHalfAngle+ZAxis.m_z*dSinHalfAngle;
                }
            if(Duu)
                {
                Duu->m_x=(-YAxis.m_x*dSinU-XAxis.m_x*dCosU)*dVScale;
                Duu->m_y=(-YAxis.m_y*dSinU-XAxis.m_y*dCosU)*dVScale;
                Duu->m_z=(-YAxis.m_z*dSinU-XAxis.m_z*dCosU)*dVScale;
                }
            if(Duv)
                {
                double dVScaleD=-dRadius*dSinHalfAngle;
                Duv->m_x=(YAxis.m_x*dCosU-XAxis.m_x*dSinU)*dVScaleD;
                Duv->m_y=(YAxis.m_y*dCosU-XAxis.m_y*dSinU)*dVScaleD;
                Duv->m_z=(YAxis.m_z*dCosU-XAxis.m_z*dSinU)*dVScaleD;
                }
            if(Dvv)
                {
                Dvv->m_x=0;
                Dvv->m_y=0;
                Dvv->m_z=0;
                }
            break;
            }
        case SGM::NUBSurfaceType:
            {
            // From "The NURBs Book" Algorithm A3.6.

            NUBsurface const *pNUB=(NUBsurface const *)this;
            std::vector<std::vector<SGM::Point3D> > const &aControlPoints=pNUB->m_aaControlPoints;
            
            std::vector<double> const &aUKnots=pNUB->m_aUKnots;
            size_t nUDegree=pNUB->GetUDegree();
            size_t nUSpanIndex=FindSpanIndex(m_Domain.m_UDomain,nUDegree,uv.m_u,aUKnots);

            std::vector<double> const &aVKnots=pNUB->m_aVKnots;
            size_t nVDegree=pNUB->GetVDegree();
            size_t nVSpanIndex=FindSpanIndex(m_Domain.m_VDomain,nVDegree,uv.m_v,aVKnots);

            size_t nUDerivatives=0;
            if(Du || Norm || Duv) nUDerivatives=1;
            if(Duu) nUDerivatives=2;

            size_t nVDerivatives=0;
            if(Dv || Norm || Duv) nVDerivatives=1;
            if(Dvv) nVDerivatives=2;

            size_t Index1,Index2,Index3;

            double aUMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
            double *aaUBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
            for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
                {
                aaUBasisFunctions[Index1]=aUMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
                }
            FindBasisFunctions(nUSpanIndex,uv.m_u,nUDegree,nUDerivatives,&aUKnots[0],aaUBasisFunctions);

            double aVMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
            double *aaVBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
            for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
                {
                aaVBasisFunctions[Index1]=aVMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
                }
            FindBasisFunctions(nVSpanIndex,uv.m_v,nVDegree,nVDerivatives,&aVKnots[0],aaVBasisFunctions);

            SGM::Point3D temp[SMG_MAX_NURB_DEGREE_PLUS_ONE];
            SGM::Point3D SKL[3][3];
            for(Index1=0;Index1<=nUDerivatives;++Index1)
                {
                for(Index2=0;Index2<=nVDegree;++Index2)
                    {
                    temp[Index2]=SGM::Point3D(0.0,0.0,0.0);
                    for(Index3=0;Index3<=nUDegree;++Index3)
                        {
                        double dFactor=aaUBasisFunctions[Index1][Index3];
                        SGM::Point3D const &ControlPos=aControlPoints[nUSpanIndex-nUDegree+Index3]
                                                                     [nVSpanIndex-nVDegree+Index2];
                        temp[Index2].m_x+=dFactor*ControlPos.m_x;
                        temp[Index2].m_y+=dFactor*ControlPos.m_y;
                        temp[Index2].m_z+=dFactor*ControlPos.m_z;
                        }
                    }

                for(Index2=0;Index2<=nVDerivatives;++Index2)
                    {
                    SKL[Index1][Index2].m_x=0.0;
                    SKL[Index1][Index2].m_y=0.0;
                    SKL[Index1][Index2].m_z=0.0;
                    for(Index3=0;Index3<=nVDegree;++Index3)
                        {
                        SKL[Index1][Index2].m_x+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_x;
                        SKL[Index1][Index2].m_y+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_y;
                        SKL[Index1][Index2].m_z+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_z;
                        }
                    }
                }

            // Fill in the answers.

            if(Pos)
                {
                *Pos=SKL[0][0];
                }
            if(Du)
                {
                *Du=SGM::Vector3D(SKL[1][0]);
                }
            if(Dv)
                {
                *Dv=SGM::Vector3D(SKL[0][1]);
                }
            if(Norm)
                {
                *Norm=SGM::Vector3D(SKL[1][0])*SGM::Vector3D(SKL[0][1]);
                }
            if(Duu)
                {
                *Duu=SGM::Vector3D(SKL[2][0]);
                }
            if(Duv)
                {
                *Duv=SGM::Vector3D(SKL[1][1]);
                }
            if(Dvv)
                {
                *Dvv=SGM::Vector3D(SKL[0][2]);
                }

            break;
            }
        default:
            {
            throw;
            }
        }
    }

SGM::Point2D NewtonsMethod(surface      const *pSurface,
                           SGM::Point2D const &StartUV,
                           SGM::Point3D const &Pos)
    {
    SGM::Point3D SurfPos;
    SGM::Vector3D DU,DV;
    SGM::UnitVector3D Norm;
    SGM::Point2D Answer=StartUV;
    double DeltaU=std::numeric_limits<double>::max();
    double DeltaV=std::numeric_limits<double>::max();
    while(SGM_ZERO<DeltaU || DeltaU<-SGM_ZERO ||
          SGM_ZERO<DeltaV || DeltaV<-SGM_ZERO)
        {
        pSurface->Evaluate(Answer,&SurfPos,&DU,&DV,&Norm);
        SGM::Point3D ProjectPos=Pos-Norm*((Pos-SurfPos)%Norm);
        SGM::Vector3D S=ProjectPos-SurfPos;
        DeltaU=(S%DU)/DU.MagnitudeSquared();
        DeltaV=(S%DV)/DV.MagnitudeSquared();
        Answer.m_u+=DeltaU;
        Answer.m_v+=DeltaV;
        }
    return Answer;
    }

void surface::Curvature(SGM::Point2D const &uv,
                        SGM::UnitVector3D  &Vec1,
                        SGM::UnitVector3D  &Vec2,
                        double             &k1,
                        double             &k2) const
    {
    // Find the Eigen vectors and values of the second fundamental form
    // 
    //  | L M |  L = Duu % Norm, M = Duv % Norm, N = Dvv % Norm.
    //  | M N |

    SGM::UnitVector3D Norm;
    SGM::Vector3D Du,Dv,Duu,Duv,Dvv;
    Evaluate(uv,nullptr,&Du,&Dv,&Norm,&Duu,&Duv,&Dvv);
    double L=Duu%Norm;
    double M=Duv%Norm;
    double N=Dvv%Norm;
    const double aaMatrix[2][2] =
        {
        L, M,
        M, N
        };
    std::vector<double> aValues;
    std::vector<SGM::UnitVector2D> aVectors;
    size_t nValues=SGM::FindEigenVectors2D(&aaMatrix[0],aValues,aVectors);
    if(nValues==2)
        {
        SGM::UnitVector3D UDu=Du,UDv=Dv;
        k1=aValues[0];
        k2=aValues[1];
        Vec1=aVectors[0].m_u*UDu+aVectors[0].m_v*UDv;
        Vec2=aVectors[1].m_u*UDu+aVectors[1].m_v*UDv;
        }
    }

SGM::Point2D surface::Inverse(SGM::Point3D const &Pos,
                              SGM::Point3D       *ClosePos,
                              SGM::Point2D const *pGuess) const
    {
    SGM::Point2D uv;
    switch (m_SurfaceType)
        {
        case SGM::PlaneType:
            {
            plane const *pPlane=(plane const *)this;

            SGM::Point3D      const &Origin=pPlane->m_Origin;
            SGM::UnitVector3D const &XAxis =pPlane->m_XAxis;
            SGM::UnitVector3D const &YAxis =pPlane->m_YAxis;
            double dScale=pPlane->m_dScale;
            double dx=Pos.m_x-Origin.m_x;
            double dy=Pos.m_y-Origin.m_y;
            double dz=Pos.m_z-Origin.m_z;
            uv.m_u=(dx*XAxis.m_x+dy*XAxis.m_y+dz*XAxis.m_z)/dScale;
            uv.m_v=(dx*YAxis.m_x+dy*YAxis.m_y+dz*YAxis.m_z)/dScale;

            if(ClosePos)
                {
                ClosePos->m_x=Origin.m_x+(XAxis.m_x*uv.m_u+YAxis.m_x*uv.m_v)*dScale;
                ClosePos->m_y=Origin.m_y+(XAxis.m_y*uv.m_u+YAxis.m_y*uv.m_v)*dScale;
                ClosePos->m_z=Origin.m_z+(XAxis.m_z*uv.m_u+YAxis.m_z*uv.m_v)*dScale;
                }
            break;
            }
        case SGM::CylinderType:
            {
            cylinder const *pCylinder=(cylinder const *)this;

            SGM::Point3D      const &Center =pCylinder->m_Origin;
            SGM::UnitVector3D const &XAxis  =pCylinder->m_XAxis;
            SGM::UnitVector3D const &YAxis  =pCylinder->m_YAxis;
            SGM::UnitVector3D const &ZAxis  =pCylinder->m_ZAxis;
            double                   dRadius=pCylinder->m_dRadius;

            double x=Pos.m_x-Center.m_x;
            double y=Pos.m_y-Center.m_y;
            double z=Pos.m_z-Center.m_z;

            double dx=x*XAxis.m_x+y*XAxis.m_y+z*XAxis.m_z;
            double dy=x*YAxis.m_x+y*YAxis.m_y+z*YAxis.m_z;
            uv.m_u=SGM::SAFEatan2(dy,dx);
            uv.m_v=(x*ZAxis.m_x+y*ZAxis.m_y+z*ZAxis.m_z)/dRadius;

            if(uv.m_u<m_Domain.m_UDomain.m_dMin)
                {
                uv.m_u+=SGM_TWO_PI;
                }
            if(pGuess)
                {
                throw;
                }
            
            if(ClosePos)
                {
                double dCos=cos(uv.m_u);
                double dSin=sin(uv.m_u);
                ClosePos->m_x=Center.m_x+(XAxis.m_x*dCos+YAxis.m_x*dSin+ZAxis.m_x*uv.m_v)*dRadius;
                ClosePos->m_y=Center.m_y+(XAxis.m_y*dCos+YAxis.m_y*dSin+ZAxis.m_y*uv.m_v)*dRadius;
                ClosePos->m_z=Center.m_z+(XAxis.m_z*dCos+YAxis.m_z*dSin+ZAxis.m_z*uv.m_v)*dRadius;
                }
            break;
            }
        case SGM::ConeType:
            {
            cone const *pCone=(cone const *)this;

            SGM::Point3D      const &Center =pCone->m_Origin;
            SGM::UnitVector3D const &XAxis  =pCone->m_XAxis;
            SGM::UnitVector3D const &YAxis  =pCone->m_YAxis;
            SGM::UnitVector3D const &ZAxis  =pCone->m_ZAxis;
            double             dSinHalfAngle=pCone->m_dSinHalfAngle;
            double             dCosHalfAngle=pCone->m_dCosHalfAngle;
            double             dRadius      =pCone->m_dRadius;

            double x=Pos.m_x-Center.m_x;
            double y=Pos.m_y-Center.m_y;
            double z=Pos.m_z-Center.m_z;

            double dx=x*XAxis.m_x+y*XAxis.m_y+z*XAxis.m_z;
            double dy=x*YAxis.m_x+y*YAxis.m_y+z*YAxis.m_z;
            uv.m_u=SGM::SAFEatan2(dy,dx);

            double dCosU=cos(uv.m_u);
            double dSinU=sin(uv.m_u);
            
            double Sx=XAxis.m_x*dCosU+YAxis.m_x*dSinU;
            double Sy=XAxis.m_y*dCosU+YAxis.m_y*dSinU;
            double Sz=XAxis.m_z*dCosU+YAxis.m_z*dSinU;

            double Lx=ZAxis.m_x*dCosHalfAngle-Sx*dSinHalfAngle;
            double Ly=ZAxis.m_y*dCosHalfAngle-Sy*dSinHalfAngle;
            double Lz=ZAxis.m_z*dCosHalfAngle-Sz*dSinHalfAngle;

            double dSx=Pos.m_x-Center.m_x-Sx*dRadius;
            double dSy=Pos.m_y-Center.m_y-Sy*dRadius;
            double dSz=Pos.m_z-Center.m_z-Sz*dRadius;
            
            uv.m_v=(Lx*dSx+Ly*dSy+Lz*dSz)/dRadius;

            double dMaxV=1.0/dSinHalfAngle;
            if(dMaxV<uv.m_v)
                {
                uv.m_v=dMaxV;
                }

            if(uv.m_u<m_Domain.m_UDomain.m_dMin)
                {
                uv.m_u+=SGM_TWO_PI;
                }
            if(pGuess)
                {
                throw;
                }
            
            if(ClosePos)
                {
                double dVScale=dRadius*(1.0-uv.m_v*dSinHalfAngle);
                double dZScale=uv.m_v*dRadius*dCosHalfAngle;
                
                ClosePos->m_x=Center.m_x+(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*dVScale+ZAxis.m_x*dZScale;
                ClosePos->m_y=Center.m_y+(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*dVScale+ZAxis.m_y*dZScale;
                ClosePos->m_z=Center.m_z+(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dVScale+ZAxis.m_z*dZScale;
                }
            break;
            }
        case SGM::SphereType:
            {
            sphere const *pSphere=(sphere const *)this;

            SGM::Point3D      const &Center =pSphere->m_Center;
            SGM::UnitVector3D const &XAxis  =pSphere->m_XAxis;
            SGM::UnitVector3D const &YAxis  =pSphere->m_YAxis;
            SGM::UnitVector3D const &ZAxis  =pSphere->m_ZAxis;
            double                   dRadius=pSphere->m_dRadius;

            double x=Pos.m_x-Center.m_x;
            double y=Pos.m_y-Center.m_y;
            double z=Pos.m_z-Center.m_z;

            double dUx=x*XAxis.m_x+y*XAxis.m_y+z*XAxis.m_z;
            double dUy=x*YAxis.m_x+y*YAxis.m_y+z*YAxis.m_z;
            uv.m_u=SGM::SAFEatan2(dUy,dUx);
            SGM::UnitVector3D Spoke=(Pos-ZAxis*((Pos-Center)%ZAxis))-Center;
            double dVx=x*Spoke.m_x+y*Spoke.m_y+z*Spoke.m_z;
            double dVy=x*ZAxis.m_x+y*ZAxis.m_y+z*ZAxis.m_z;
            uv.m_v=SGM::SAFEatan2(dVy,dVx);

            if(uv.m_u<m_Domain.m_UDomain.m_dMin)
                {
                uv.m_u+=SGM_TWO_PI;
                }
            if(pGuess)
                {
                throw;
                }
            
            if(ClosePos)
                {
                SGM::UnitVector3D Ray=Pos-Center;
                *ClosePos=Center+Ray*dRadius;
                }
            break;
            }
        case SGM::TorusType:
            {
            torus const *pTorus=(torus const *)this;

            SGM::Point3D      const &Center =pTorus->m_Center;
            SGM::UnitVector3D const &XAxis  =pTorus->m_XAxis;
            SGM::UnitVector3D const &YAxis  =pTorus->m_YAxis;
            SGM::UnitVector3D const &ZAxis  =pTorus->m_ZAxis;
            double dMajorRadius=pTorus->m_dMajorRadius;

            double x=Pos.m_x-Center.m_x;
            double y=Pos.m_y-Center.m_y;
            double z=Pos.m_z-Center.m_z;

            double dUx=x*XAxis.m_x+y*XAxis.m_y+z*XAxis.m_z;
            double dUy=x*YAxis.m_x+y*YAxis.m_y+z*YAxis.m_z;
            uv.m_u=SGM::SAFEatan2(dUy,dUx);

            SGM::UnitVector3D Spoke=(Pos-ZAxis*((Pos-Center)%ZAxis))-Center;

            double cx=Pos.m_x-Center.m_x-Spoke.m_x*dMajorRadius;
            double cy=Pos.m_y-Center.m_y-Spoke.m_y*dMajorRadius;
            double cz=Pos.m_z-Center.m_z-Spoke.m_z*dMajorRadius;

            double dVx=cx*Spoke.m_x+cy*Spoke.m_y+cz*Spoke.m_z;
            double dVy=cx*ZAxis.m_x+cy*ZAxis.m_y+cz*ZAxis.m_z;
            uv.m_v=SGM::SAFEatan2(dVy,dVx);

            if(pTorus->GetKind()==SGM::TorusKindType::LemonType)
                {
                uv.m_v-=SGM_PI;
                }
            if(uv.m_v<m_Domain.m_VDomain.m_dMin)
                {
                uv.m_v+=SGM_TWO_PI;
                }
            if(uv.m_u<m_Domain.m_UDomain.m_dMin)
                {
                uv.m_u+=SGM_TWO_PI;
                }
            if(pGuess)
                {
                throw;
                }

            if(ClosePos)
                {
                pTorus->Evaluate(uv,ClosePos);
                }

            break;
            }
        case SGM::NUBSurfaceType:
            {
            NUBsurface const *pNUBSurface=(NUBsurface const *)this;

            SGM::Point2D StartUV(0.0,0.0);
            if(pGuess)
                {
                StartUV=*pGuess;
                }
            else
                {
                std::vector<SGM::Point3D> const &aSeedPoints=pNUBSurface->GetSeedPoints();
                std::vector<SGM::Point2D> const &aSeedParams=pNUBSurface->GetSeedParams();
                size_t nSeedPoints=aSeedPoints.size();
                size_t Index1;
                double dMin=std::numeric_limits<double>::max();
                for(Index1=0;Index1<nSeedPoints;++Index1)
                    {
                    double dDist=aSeedPoints[Index1].DistanceSquared(Pos);
                    if(dDist<dMin)
                        {
                        dMin=dDist;
                        StartUV=aSeedParams[Index1];
                        }
                    }
                }

            uv=NewtonsMethod(this,StartUV,Pos);
            if(ClosePos)
                {
                Evaluate(uv,ClosePos);
                }
            if(pGuess)
                {
                throw;
                }

            break;
            }
        default:
            {
            throw;
            }
        }
    return uv;
    }
