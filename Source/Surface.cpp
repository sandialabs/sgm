#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include "Topology.h"
#include <cmath>

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
    if(this)
        {
        m_sFaces.insert(pFace);
        }
    }

void surface::RemoveFace(face *pFace) 
    {
    if(this)
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
        case SGM::CylinderType:
            {
            cylinder const *pCylinder=(cylinder const *)this;
            double dRadius=pCylinder->m_dRadius;
            SGM::Point2D uv(dU,0);
            SGM::Point3D Pos;
            SGM::Vector3D Vec;
            Evaluate(uv,&Pos,NULL,&Vec);
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
            SGM::Interval1D Domain=pSphere->GetDomain().m_VDomain;
            SGM::UnitVector3D Normal=XAxis*ZAxis;
            return new circle(rResult,Center,Normal,dRadius,&XAxis,&Domain);
            break;
            }
        case SGM::TorusType:
            {
            break;
            }
        default:
            {
            throw;
            }
        }
    return NULL;
    }

curve *surface::VParamLine(SGM::Result &,//rResult,
                           double                  ) const //dV) const
    {
    switch(m_SurfaceType)
        {
        case SGM::PlaneType:
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
            break;
            }
        default:
            {
            throw;
            }
        }
    return NULL;
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
                Pos->m_z=Center.m_z+(XAxis.m_z*dCos+YAxis.m_z*dSin+ZAxis.m_x*uv.m_v)*dRadius;
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
                Dv->m_z=ZAxis.m_x*dRadius;
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
                Pos->m_z=Center.m_x+((XAxis.m_z*dCosU+YAxis.m_z*dSinU)*dCosV+ZAxis.m_z*dSinV)*dRadius;
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
                double dVdMCV=-dSinV*dMajorRadius;
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
                }
            if(Duu)
                {
                Duu->m_x=(-YAxis.m_x*dSinU-XAxis.m_x*dCosU)*dMCV;
                Duu->m_y=(-YAxis.m_y*dSinU-XAxis.m_y*dCosU)*dMCV;
                Duu->m_z=(-YAxis.m_z*dSinU-XAxis.m_z*dCosU)*dMCV;
                }
            if(Duv)
                {
                double dVdMCV=-dSinV*dMajorRadius;
                Duv->m_x=(YAxis.m_x*dCosU-XAxis.m_x*dSinU)*dVdMCV;
                Duv->m_y=(YAxis.m_y*dCosU-XAxis.m_y*dSinU)*dVdMCV;
                Duv->m_z=(YAxis.m_z*dCosU-XAxis.m_z*dSinU)*dVdMCV;
                }
            if(Dvv)
                {
                double ddVdMCV=-dCosV*dMajorRadius;
                double ddVdSM=-dSinV*dMinorRadius;
                Dvv->m_x=(XAxis.m_x*dCosU+YAxis.m_x*dSinU)*ddVdMCV+ZAxis.m_x*ddVdSM;
                Dvv->m_y=(XAxis.m_y*dCosU+YAxis.m_y*dSinU)*ddVdMCV+ZAxis.m_y*ddVdSM;
                Dvv->m_z=(XAxis.m_z*dCosU+YAxis.m_z*dSinU)*ddVdMCV+ZAxis.m_z*ddVdSM;
                }
            break;
            }
        default:
            {
            throw;
            }
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
            plane const *pPlane=(plane *)this;

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
            cylinder const *pCylinder=(cylinder *)this;

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
            uv.m_u=atan2(dy,dx);
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
                ClosePos->m_z=Center.m_z+(XAxis.m_z*dCos+YAxis.m_z*dSin+ZAxis.m_x*uv.m_v)*dRadius;
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
            double                   dRadius=fabs(pSphere->m_dRadius);

            double x=Pos.m_x-Center.m_x;
            double y=Pos.m_y-Center.m_y;
            double z=Pos.m_z-Center.m_z;

            double dUx=x*XAxis.m_x+y*XAxis.m_y+z*XAxis.m_z;
            double dUy=x*YAxis.m_x+y*YAxis.m_y+z*YAxis.m_z;
            uv.m_u=atan2(dUy,dUx);
            SGM::UnitVector3D Spoke=(Pos-ZAxis*((Pos-Center)%ZAxis))-Center;
            double dVx=x*Spoke.m_x+y*Spoke.m_y+z*Spoke.m_z;
            double dVy=x*ZAxis.m_x+y*ZAxis.m_y+z*ZAxis.m_z;
            uv.m_v=atan2(dVy,dVx);

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
            break;
            }
        default:
            {
            throw;
            }
        }
    return uv;
    }
