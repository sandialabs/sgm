#include <cmath>
#include <algorithm>
#include <limits>

#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"
#include "Topology.h"

namespace SGMInternal
{
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

surface *surface::MakeCopy(SGM::Result &rResult) const
    {
    surface *pAnswer=NULL;
    switch(m_SurfaceType)
        {
        case SGM::PlaneType:
            {
            pAnswer = new plane(rResult,(plane const *)this);
            break;
            }
        default:
            {
            throw;
            }
        }
    pAnswer->m_sFaces=m_sFaces;
    pAnswer->m_sOwners=m_sOwners;
    pAnswer->m_sAttributes=m_sAttributes;
    pAnswer->m_Box=m_Box;
    return pAnswer;
    }

void surface::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
    {
    
    // Run though all the pointers and change them if they are in the map.
    
    std::set<face *,EntityCompare> m_sFixedFaces;
    for(auto pFace : m_sFaces)
        {
        auto MapValue=mEntityMap.find(pFace);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedFaces.insert((face *)MapValue->second);
            }
        else
            {
            m_sFixedFaces.insert(pFace);
            }
        }
    m_sFaces=m_sFixedFaces;

    std::set<attribute *,EntityCompare> m_sFixedAttributes;
    for(auto pAttribute : m_sAttributes)
        {
        auto MapValue=mEntityMap.find(pAttribute);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedAttributes.insert((attribute *)MapValue->second);
            }
        else
            {
            m_sFixedAttributes.insert(pAttribute);
            }
        }
    m_sAttributes=m_sFixedAttributes;

    std::set<entity *,EntityCompare> m_sFixedOwners;
    for(auto pEntity : m_sOwners)
        {
        auto MapValue=mEntityMap.find(pEntity);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedOwners.insert((attribute *)MapValue->second);
            }
        else
            {
            m_sFixedOwners.insert(pEntity);
            }
        }
    m_sOwners=m_sFixedOwners;}

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

double AreaIntegrand(SGM::Point2D const &uv,void const *pData)
    {
    surface const *pSurface=(surface const *)pData;
    SGM::Vector3D VecU,VecV;
    pSurface->Evaluate(uv,nullptr,&VecU,&VecV);
    return (VecU*VecV).Magnitude();
    }

double surface::FindAreaOfParametricTriangle(SGM::Result        &,//rResult,
                                             SGM::Point2D const &PosA,
                                             SGM::Point2D const &PosB,
                                             SGM::Point2D const &PosC) const
    {
    return SGM::IntegrateTriangle(SGMInternal::AreaIntegrand,PosA,PosB,PosC,this);
    }

void surface::Transform(SGM::Transform3D const &Trans)
    {
    switch(m_SurfaceType)
        {
        case SGM::PlaneType:
            {
            plane *pPlane=(plane *)this;
            SGM::Point3D &Origin=pPlane->m_Origin;
            SGM::UnitVector3D &XAxis=pPlane->m_XAxis;
            SGM::UnitVector3D &YAxis=pPlane->m_YAxis;
            SGM::UnitVector3D &ZAxis=pPlane->m_ZAxis;
            double dPlaneScale=pPlane->m_dScale;

            Origin=Trans*Origin;
            XAxis=Trans*XAxis;
            YAxis=Trans*YAxis;
            ZAxis=Trans*ZAxis;
            if(double dScale=Trans.Scale())
                {
                dPlaneScale*=dScale;
                }

            break;
            }
        case SGM::CylinderType:
            {
            cylinder *pCylinder=(cylinder *)this;
            SGM::Point3D &Origin=pCylinder->m_Origin;
            SGM::UnitVector3D &XAxis=pCylinder->m_XAxis;
            SGM::UnitVector3D &YAxis=pCylinder->m_YAxis;
            SGM::UnitVector3D &ZAxis=pCylinder->m_ZAxis;
            double &dRadius=pCylinder->m_dRadius;

            Origin=Trans*Origin;
            XAxis=Trans*XAxis;
            YAxis=Trans*YAxis;
            ZAxis=Trans*ZAxis;
            if(double dScale=Trans.Scale())
                {
                dRadius*=dScale;
                }

            break;
            }
        case SGM::ConeType:
            {
            cone *pCone=(cone *)this;
            SGM::Point3D &Origin=pCone->m_Origin;
            SGM::UnitVector3D &XAxis=pCone->m_XAxis;
            SGM::UnitVector3D &YAxis=pCone->m_YAxis;
            SGM::UnitVector3D &ZAxis=pCone->m_ZAxis;
            double &dRadius=pCone->m_dRadius;

            Origin=Trans*Origin;
            XAxis=Trans*XAxis;
            YAxis=Trans*YAxis;
            ZAxis=Trans*ZAxis;
            if(double dScale=Trans.Scale())
                {
                dRadius*=dScale;
                }
            break;
            }
        case SGM::SphereType:
            {
            sphere *pSphere=(sphere *)this;
            SGM::Point3D &Center=pSphere->m_Center;
            SGM::UnitVector3D &XAxis=pSphere->m_XAxis;
            SGM::UnitVector3D &YAxis=pSphere->m_YAxis;
            SGM::UnitVector3D &ZAxis=pSphere->m_ZAxis;
            double &dRadius=pSphere->m_dRadius;

            Center=Trans*Center;
            XAxis=Trans*XAxis;
            YAxis=Trans*YAxis;
            ZAxis=Trans*ZAxis;
            if(double dScale=Trans.Scale())
                {
                dRadius*=dScale;
                }
            break;
            }
        case SGM::TorusType:
            {
            torus *pTorus=(torus *)this;
            SGM::Point3D &Center=pTorus->m_Center;
            SGM::UnitVector3D &XAxis=pTorus->m_XAxis;
            SGM::UnitVector3D &YAxis=pTorus->m_YAxis;
            SGM::UnitVector3D &ZAxis=pTorus->m_ZAxis;
            double &dMinorRadius=pTorus->m_dMinorRadius;
            double &dMajorRadius=pTorus->m_dMajorRadius;

            Center=Trans*Center;
            XAxis=Trans*XAxis;
            YAxis=Trans*YAxis;
            ZAxis=Trans*ZAxis;
            if(double dScale=Trans.Scale())
                {
                dMinorRadius*=dScale;
                dMajorRadius*=dScale;
                }
            break;
            }
        case SGM::NUBSurfaceType:
            {
            NUBsurface *pNUBSurface=(NUBsurface *)this;
            std::vector<std::vector<SGM::Point3D> > aaPoints=pNUBSurface->m_aaControlPoints;
            size_t nSize1=aaPoints.size();
            size_t nSize2=aaPoints[0].size();
            size_t Index1,Index2;
            for(Index1=0;Index1<nSize1;++Index1)
                {
                for(Index2=0;Index2<nSize2;++Index2)
                    {
                    aaPoints[Index1][Index2]=Trans*aaPoints[Index1][Index2];
                    }
                }
            break;
            }
        case SGM::NURBSurfaceType:
            {
            NURBsurface *pNURBSurface=(NURBsurface *)this;
            std::vector<std::vector<SGM::Point4D> > aaPoints=pNURBSurface->m_aaControlPoints;
            size_t nSize1=aaPoints.size();
            size_t nSize2=aaPoints[0].size();
            size_t Index1,Index2;
            for(Index1=0;Index1<nSize1;++Index1)
                {
                for(Index2=0;Index2<nSize2;++Index2)
                    {
                    SGM::Point4D Pos=aaPoints[Index1][Index2];
                    SGM::Point3D Pos3D(Pos.m_x,Pos.m_y,Pos.m_z);
                    Pos3D=Trans*Pos3D;
                    aaPoints[Index1][Index2]=SGM::Point4D(Pos3D.m_x,Pos3D.m_y,Pos3D.m_z,Pos.m_w);
                    }
                }
            break;
            }
        case SGM::RevolveType:
            {
            revolve *pRevolve=(revolve *)this;
            SGM::Point3D &Origin=pRevolve->m_Origin;
            SGM::UnitVector3D &XAxis=pRevolve->m_XAxis;
            SGM::UnitVector3D &YAxis=pRevolve->m_YAxis;
            SGM::UnitVector3D &ZAxis=pRevolve->m_ZAxis;
            curve *pCurve=pRevolve->m_pCurve;

            Origin=Trans*Origin;
            XAxis=Trans*XAxis;
            YAxis=Trans*YAxis;
            ZAxis=Trans*ZAxis;

            if(pCurve->GetEdges().empty() && pCurve->GetOwners().size()==1)
                {
                pCurve->Transform(Trans);
                }
            else
                {
                // Make a copy and tranform the copy.
                throw;
                }

            break;
            }
        
        case SGM::ExtrudeType:
            {
            extrude *pExtrude=(extrude *)this;
            SGM::UnitVector3D &vAxis=pExtrude->m_vAxis;
            vAxis=Trans*vAxis;
            curve *pCurve=pExtrude->m_pCurve;

            if(pCurve->GetEdges().empty() && pCurve->GetOwners().size()==1)
                {
                pCurve->Transform(Trans);
                }
            else
                {
                // Make a copy and transform the copy.
                throw;
                }

            break;
            }
        default:
            {
            throw;
            break;
            }
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
            cone const *pCone=(cone const *)this;
            SGM::Point3D Apex=pCone->FindApex();
            SGM::Point3D UZero;
            SGM::Point2D uv(dU,0.0);
            pCone->Evaluate(uv,&UZero);
            return new line(rResult,Apex,UZero-Apex,pCone->m_dRadius);
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
                curve *pCurve=new NUBcurve(rResult,aControlPoints,pNUBSurface->GetVKnots());
                return pCurve;
                }
            }
        case SGM::NURBSurfaceType:
            {
            NURBsurface const *pNURBSurface=(NURBsurface const *)this;
            if(pNURBSurface->m_bSingularHighU && SGM::NearEqual(dU,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false))
                {
                SGM::Point3D Pos;
                pNURBSurface->Evaluate(SGM::Point2D(dU,m_Domain.m_VDomain.m_dMin),&Pos);
                return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
                }
            else if(pNURBSurface->m_bSingularLowU && SGM::NearEqual(dU,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false))
                {
                SGM::Point3D Pos;
                pNURBSurface->Evaluate(SGM::Point2D(dU,m_Domain.m_VDomain.m_dMin),&Pos);
                return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
                }
            else
                {
                std::vector<double> const &aUKnots=pNURBSurface->GetUKnots();
                std::vector<std::vector<SGM::Point4D> > const &aaControlPoints=pNURBSurface->GetControlPoints();
                size_t nUDegree=pNURBSurface->GetUDegree();
                size_t nSpanIndex=FindSpanIndex(m_Domain.m_UDomain,nUDegree,dU,aUKnots);

                double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
                double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
                size_t Index1,Index2;
                for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
                    {
                    aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
                    }
                FindBasisFunctions(nSpanIndex,dU,nUDegree,0,&aUKnots[0],aaBasisFunctions);

                std::vector<SGM::Point4D> aControlPoints;
                size_t nControlPoints=aaControlPoints[0].size();
                aControlPoints.assign(nControlPoints,SGM::Point4D(0.0,0.0,0.0,0.0));
                for(Index1=0;Index1<nControlPoints;++Index1)
                    {
                    for(Index2=0;Index2<=nUDegree;++Index2)
                        {
                        SGM::Point4D const &XYZW=aaControlPoints[nSpanIndex-nUDegree+Index2][Index1];
                        double dBasisWeight=XYZW.m_w*aaBasisFunctions[0][Index2];
                        aControlPoints[Index1]+=SGM::Vector4D(XYZW.m_x*dBasisWeight,XYZW.m_y*dBasisWeight,XYZW.m_z*dBasisWeight,dBasisWeight);
                        }
                    SGM::Point4D const &Pos=aControlPoints[Index1];
                    double dRWeight=1.0/Pos.m_w;
                    aControlPoints[Index1]=SGM::Point4D(Pos.m_x*dRWeight,Pos.m_y*dRWeight,Pos.m_z*dRWeight,Pos.m_w);
                    }
                curve *pParamCurve=new NURBcurve(rResult,aControlPoints,pNURBSurface->GetVKnots());
                return pParamCurve;
                }
            break;
            }
        case SGM::RevolveType:
            {
            revolve const *pRevolve=(revolve const *)this;
            curve const *pCurve=pRevolve->m_pCurve;
            curve *pParam=(curve *)pCurve->Copy(rResult);
            return pParam;
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
                size_t nUDegree=pNUBSurface->GetUDegree();
                size_t nSpanIndex=FindSpanIndex(m_Domain.m_VDomain,nUDegree,dV,aVKnots);

                double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
                double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
                size_t Index1,Index2;
                for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
                    {
                    aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
                    }
                FindBasisFunctions(nSpanIndex,dV,nUDegree,0,&aVKnots[0],aaBasisFunctions);

                std::vector<SGM::Point3D> aControlPoints;
                size_t nControlPoints=aaControlPoints.size();
                aControlPoints.assign(nControlPoints,SGM::Point3D(0,0,0));
                for(Index1=0;Index1<nControlPoints;++Index1)
                    {
                    for(Index2=0;Index2<=nUDegree;++Index2)
                        {
                        aControlPoints[Index1]+=aaBasisFunctions[0][Index2]*
                            SGM::Vector3D(aaControlPoints[Index1][nSpanIndex-nUDegree+Index2]);
                        }
                    }
                return new NUBcurve(rResult,aControlPoints,pNUBSurface->GetUKnots());
                }
            break;
            }
        case SGM::NURBSurfaceType:
            {
            NURBsurface const *pNURBSurface=(NURBsurface const *)this;
            if(pNURBSurface->m_bSingularHighV && SGM::NearEqual(m_Domain.m_VDomain.m_dMax,dV,SGM_MIN_TOL,false))
                {
                SGM::Point3D Pos;
                pNURBSurface->Evaluate(SGM::Point2D(dV,m_Domain.m_VDomain.m_dMin),&Pos);
                return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
                }
            else if(pNURBSurface->m_bSingularLowV && SGM::NearEqual(m_Domain.m_VDomain.m_dMin,dV,SGM_MIN_TOL,false))
                {
                SGM::Point3D Pos;
                pNURBSurface->Evaluate(SGM::Point2D(dV,m_Domain.m_VDomain.m_dMin),&Pos);
                return new PointCurve(rResult,Pos,&m_Domain.m_UDomain);
                }
            else
                {
                std::vector<double> const &aVKnots=pNURBSurface->GetVKnots();
                std::vector<std::vector<SGM::Point4D> > const &aaControlPoints=pNURBSurface->GetControlPoints();
                size_t nVDegree=pNURBSurface->GetVDegree();
                size_t nSpanIndex=FindSpanIndex(m_Domain.m_VDomain,nVDegree,dV,aVKnots);

                double aMemory[SMG_MAX_NURB_DEGREE_PLUS_ONE_SQUARED];
                double *aaBasisFunctions[SMG_MAX_NURB_DEGREE_PLUS_ONE];
                size_t Index1,Index2;
                for(Index1=0;Index1<SMG_MAX_NURB_DEGREE_PLUS_ONE;++Index1)
                    {
                    aaBasisFunctions[Index1]=aMemory+Index1*SMG_MAX_NURB_DEGREE_PLUS_ONE;
                    }
                FindBasisFunctions(nSpanIndex,dV,nVDegree,0,&aVKnots[0],aaBasisFunctions);

                std::vector<SGM::Point4D> aControlPoints;
                size_t nControlPoints=aaControlPoints.size();
                aControlPoints.assign(nControlPoints,SGM::Point4D(0.0,0.0,0.0,0.0));
                for(Index1=0;Index1<nControlPoints;++Index1)
                    {
                    for(Index2=0;Index2<=nVDegree;++Index2)
                        {
                        SGM::Point4D const &XYZW=aaControlPoints[Index1][nSpanIndex-nVDegree+Index2];
                        double dBasisWeight=XYZW.m_w*aaBasisFunctions[0][Index2];
                        aControlPoints[Index1]+=SGM::Vector4D(XYZW.m_x*dBasisWeight,XYZW.m_y*dBasisWeight,XYZW.m_z*dBasisWeight,dBasisWeight);
                        }
                    SGM::Point4D const &Pos=aControlPoints[Index1];
                    double dRWeight=1.0/Pos.m_w;
                    aControlPoints[Index1]=SGM::Point4D(Pos.m_x*dRWeight,Pos.m_y*dRWeight,Pos.m_z*dRWeight,Pos.m_w);
                    }

                return new NURBcurve(rResult,aControlPoints,pNURBSurface->GetUKnots());
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

            double du=uv.m_u;
            if(pTorus->m_nKind==SGM::TorusKindType::LemonType)
                {
                du+=SGM_PI;
                }

            double dCosU=cos(du);
            double dSinU=sin(du);
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
            std::vector<std::vector<SGM::Point3D> > const &aaControlPoints=pNUB->m_aaControlPoints;
            
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
                        SGM::Point3D const &ControlPos=aaControlPoints[nUSpanIndex-nUDegree+Index3]
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
        case SGM::NURBSurfaceType:
            {
            // From "The NURBs Book" Algorithm A3.6.

            NURBsurface const *pNURB=(NURBsurface const *)this;
            std::vector<std::vector<SGM::Point4D> > const &aaControlPoints=pNURB->m_aaControlPoints;
            
            std::vector<double> const &aUKnots=pNURB->m_aUKnots;
            size_t nUDegree=pNURB->GetUDegree();
            size_t nUSpanIndex=FindSpanIndex(m_Domain.m_UDomain,nUDegree,uv.m_u,aUKnots);

            std::vector<double> const &aVKnots=pNURB->m_aVKnots;
            size_t nVDegree=pNURB->GetVDegree();
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

            SGM::Point4D temp[SMG_MAX_NURB_DEGREE_PLUS_ONE];
            SGM::Point4D SKL[3][3];
            for(Index1=0;Index1<=nUDerivatives;++Index1)
                {
                for(Index2=0;Index2<=nVDegree;++Index2)
                    {
                    temp[Index2]=SGM::Point4D(0.0,0.0,0.0,0.0);
                    for(Index3=0;Index3<=nUDegree;++Index3)
                        {
                        SGM::Point4D const &ControlPos=aaControlPoints[nUSpanIndex-nUDegree+Index3]
                                                                      [nVSpanIndex-nVDegree+Index2];
                        double dBasis=ControlPos.m_w*aaUBasisFunctions[Index1][Index3];
                        temp[Index2].m_x+=dBasis*ControlPos.m_x;
                        temp[Index2].m_y+=dBasis*ControlPos.m_y;
                        temp[Index2].m_z+=dBasis*ControlPos.m_z;
                        temp[Index2].m_w+=dBasis;
                        }
                    }

                for(Index2=0;Index2<=nVDerivatives;++Index2)
                    {
                    SKL[Index1][Index2].m_x=0.0;
                    SKL[Index1][Index2].m_y=0.0;
                    SKL[Index1][Index2].m_z=0.0;
                    SKL[Index1][Index2].m_w=0.0;
                    for(Index3=0;Index3<=nVDegree;++Index3)
                        {
                        SKL[Index1][Index2].m_x+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_x;
                        SKL[Index1][Index2].m_y+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_y;
                        SKL[Index1][Index2].m_z+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_z;
                        SKL[Index1][Index2].m_w+=aaVBasisFunctions[Index2][Index3]*temp[Index3].m_w;
                        }
                    }
                }
            
            // Convert to three-dimensional points.

            SGM::Vector3D values[3][3],Aders[3][3];
            double wders[3][3];

            Aders[0][0].m_x=SKL[0][0].m_x;
            Aders[0][0].m_y=SKL[0][0].m_y;
            Aders[0][0].m_z=SKL[0][0].m_z;
            wders[0][0]=SKL[0][0].m_w;

            int nMaxDerivatives=0;
            if(0<nUDerivatives || 0<nVDerivatives)
                {
                ++nMaxDerivatives;

                Aders[1][0].m_x=SKL[1][0].m_x;
                Aders[1][0].m_y=SKL[1][0].m_y;
                Aders[1][0].m_z=SKL[1][0].m_z;
                wders[1][0]=SKL[1][0].m_w;

                Aders[0][1].m_x=SKL[0][1].m_x;
                Aders[0][1].m_y=SKL[0][1].m_y;
                Aders[0][1].m_z=SKL[0][1].m_z;
                wders[0][1]=SKL[0][1].m_w;

                if(1<nUDerivatives || 1<nVDerivatives)
                    {
                    ++nMaxDerivatives;

                    Aders[2][0].m_x=SKL[2][0].m_x;
                    Aders[2][0].m_y=SKL[2][0].m_y;
                    Aders[2][0].m_z=SKL[2][0].m_z;
                    wders[2][0]=SKL[2][0].m_w;

                    Aders[1][1].m_x=SKL[1][1].m_x;
                    Aders[1][1].m_y=SKL[1][1].m_y;
                    Aders[1][1].m_z=SKL[1][1].m_z;
                    wders[1][1]=SKL[1][1].m_w;

                    Aders[0][2].m_x=SKL[0][2].m_x;
                    Aders[0][2].m_y=SKL[0][2].m_y;
                    Aders[0][2].m_z=SKL[0][2].m_z;
                    wders[0][2]=SKL[0][2].m_w;
                    }
                }
    
            // Algorithm A4.4 from page 137-138 of the "NURB" book.

            int i,j,k,s;
            for(k=0;k<=nMaxDerivatives;++k)
                {
                for(s=0;s<=nMaxDerivatives-k;++s)
                    {
                    SGM::Vector3D v(Aders[k][s].m_x,Aders[k][s].m_y,Aders[k][s].m_z);
                    for(j=1;j<=s;++j)
                        {
                        if(s==2 && j==1)
                            {
                            v=v-2*wders[0][j]*values[k][s-j];
                            }
                        else
                            {
                            v=v-wders[0][j]*values[k][s-j];
                            }
                        }
                    for(i=1;i<=k;++i)
                        {
                        if(k==2 && i==1)
                            {
                            v=v-2*wders[i][0]*values[k-i][s];
                            }
                        else
                            {
                            v=v-wders[i][0]*values[k-i][s];
                            }
                        SGM::Vector3D v2(0.0,0.0,0.0);
                        for(j=1;j<=s;++j)
                            {
                            if(s==2 && j==1)
                                {
                                v2=v2+2*wders[i][j]*values[k-i][s-j];
                                }
                            else
                                {
                                v2=v2+wders[i][j]*values[k-i][s-j];
                                }
                            }
                        if(k==2 && i==1)
                            {
                            v=v-2*v2;
                            }
                        else
                            {
                            v=v-v2;
                            }
                        }
                    double denom=1.0/wders[0][0];
                    values[k][s].m_x=v.m_x*denom;
                    values[k][s].m_y=v.m_y*denom;
                    values[k][s].m_z=v.m_z*denom;
                    }
                }

            // Fill in the answers.

            if(Pos)
                {
                *Pos=SGM::Point3D(values[0][0]);
                }
            if(Du)
                {
                *Du=values[1][0];
                }
            if(Dv)
                {
                *Dv=values[0][1];
                }
            if(Norm)
                {
                *Norm=values[1][0]*values[0][1];
                }
            if(Duu)
                {
                *Duu=values[2][0];
                }
            if(Duv)
                {
                *Duv=values[1][1];
                }
            if(Dvv)
                {
                *Dvv=values[0][2];
                }

            break;
            }
        case SGM::RevolveType:
            {
            revolve const *pRevolve=(revolve *)this;

            SGM::Point3D      const &Origin =pRevolve->m_Origin;
            SGM::UnitVector3D const &XAxis  =pRevolve->m_XAxis;
            SGM::UnitVector3D const &YAxis  =pRevolve->m_YAxis;
            SGM::UnitVector3D const &ZAxis  =pRevolve->m_ZAxis;

            SGM::Point3D   CurvePos;
            SGM::Vector3D  DvCurve;
            SGM::Vector3D *pDvCurve = &DvCurve;
            SGM::Vector3D  DvvCurve;
            SGM::Vector3D *pDvvCurve = &DvvCurve;
            SGM::Vector3D  dvAxisPos(0,0,0);
            SGM::Vector3D  dvvAxisPos(0,0,0);
            SGM::Vector3D  DuLocal(0,0,0);
            SGM::Vector3D  DvLocal(0,0,0);
            double A1_half = 0.0;
            double dvRadius = 0.0;

            if (nullptr == Dv && nullptr == Duv && nullptr == Dvv && nullptr == Norm)
                pDvCurve = nullptr;
            if (nullptr == Dvv)
                pDvvCurve = nullptr;

            // evaluate curve, find curve point projected to axis, and find radius
            pRevolve->m_pCurve->Evaluate(uv.m_v, &CurvePos, pDvCurve, pDvvCurve);
            SGM::Point3D AxisPos = Origin + ((CurvePos - Origin) % ZAxis) * ZAxis;
            SGM::Vector3D vRadius = CurvePos - AxisPos;
            double dRadius = vRadius.Magnitude();

            if (pDvCurve)
            {
                dvAxisPos = (DvCurve % ZAxis) * ZAxis;
                A1_half = ((vRadius.m_x) * (DvCurve.m_x - dvAxisPos.m_x) +
                          (vRadius.m_y) * (DvCurve.m_y - dvAxisPos.m_y) +
                          (vRadius.m_z) * (DvCurve.m_z - dvAxisPos.m_z) );
                dvRadius = A1_half / dRadius;
            }

            // find radius and derivatives with respect to v
            double dCos=cos(uv.m_u);
            double dSin=sin(uv.m_u);
            double dRSin=dRadius * dSin;
            double dRCos=dRadius * dCos;

            if (nullptr != Pos)
                *Pos = AxisPos + dRCos * XAxis + dRSin * YAxis;

            if (nullptr != Du || nullptr != Norm )
                DuLocal = (dRCos * YAxis) - (dRSin * XAxis);

            if (nullptr != Dv || nullptr != Norm )
                DvLocal = dvAxisPos + (dvRadius * dCos * XAxis) + (dvRadius * dSin * YAxis);

            if (nullptr != Du)
                *Du = DuLocal;

            if (nullptr != Dv)
                *Dv = DvLocal;

            if (nullptr != Norm)
                *Norm = DuLocal * DvLocal;

            if (nullptr != Duu)
                *Duu = (-1) * ( (dRSin * YAxis) + (dRCos * XAxis) );

            if (nullptr != Duv)
                *Duv = (dvRadius * dCos * YAxis) - (dvRadius * dSin * XAxis);

            if (nullptr != Dvv)
            {
                dvvAxisPos = (DvvCurve % ZAxis) * ZAxis;
                double A2_half = (((DvCurve.m_x - dvAxisPos.m_x) * (DvCurve.m_x - dvAxisPos.m_x) + (vRadius.m_x) * (DvvCurve.m_x - dvvAxisPos.m_x)) +
                                  ((DvCurve.m_y - dvAxisPos.m_y) * (DvCurve.m_y - dvAxisPos.m_y) + (vRadius.m_y) * (DvvCurve.m_y - dvvAxisPos.m_y)) +
                                  ((DvCurve.m_z - dvAxisPos.m_z) * (DvCurve.m_z - dvAxisPos.m_z) + (vRadius.m_z) * (DvvCurve.m_z - dvvAxisPos.m_z)));
                double dvvRadius = ((-1 * A1_half * A1_half) / (dRadius * dRadius * dRadius)) + ((A2_half) / (dRadius));
                *Dvv = dvvAxisPos + (dvvRadius * dCos * XAxis) + (dvvRadius * dSin * YAxis);
            }

            break;
            }
        case SGM::ExtrudeType:
            {
            extrude const *pExtrude=(extrude *)this;

            SGM::UnitVector3D const &vAxis=pExtrude->m_vAxis;

            SGM::Point3D   CurvePos;
            SGM::Vector3D  DuCurve;
            SGM::Vector3D *pDuCurve = &DuCurve;
            SGM::Vector3D  DuuCurve;
            SGM::Vector3D *pDuuCurve = &DuuCurve;

            if (nullptr == Dv && nullptr == Duv && nullptr == Duu && nullptr == Norm)
                pDuCurve = nullptr;
            if (nullptr == Duu)
                pDuuCurve = nullptr;

            // Evaluate the defining curve.

            pExtrude->m_pCurve->Evaluate(uv.m_u, &CurvePos, pDuCurve, pDuuCurve);

            // Fill in the answers.

            if(Pos)
                {
                *Pos=CurvePos+vAxis*uv.m_v;
                }
            if(Du)
                {
                *Du=*pDuCurve;
                }
            if(Dv)
                {
                *Dv=vAxis;
                }
            if(Norm)
                {
                *Norm=(*pDuCurve)*vAxis;
                }
            if(Duu)
                {
                *Duu=*pDuuCurve;
                }
            if(Duv)
                {
                *Duv=SGM::Vector3D(0,0,0);
                }
            if(Dvv)
                {
                *Dvv=SGM::Vector3D(0,0,0);
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
    size_t nCount=0;
    while(nCount<100 &&
       (SGM_ZERO<DeltaU || DeltaU<-SGM_ZERO ||
        SGM_ZERO<DeltaV || DeltaV<-SGM_ZERO))
        {
        pSurface->Evaluate(Answer,&SurfPos,&DU,&DV,&Norm);
        double dDot=(Pos-SurfPos)%Norm;
        SGM::Point3D ProjectPos=Pos-Norm*dDot;
        SGM::Vector3D S=ProjectPos-SurfPos;
        DeltaU=(S%DU)/DU.MagnitudeSquared();
        DeltaV=(S%DV)/DV.MagnitudeSquared();
        Answer.m_u+=DeltaU;
        Answer.m_v+=DeltaV;
        pSurface->SnapToDomain(Answer);
        if(fabs(dDot)<SGM_ZERO)
            {
            break;
            }
        ++nCount;
        }
    return Answer;
    }

bool surface::IsSingularity(SGM::Point2D const &uv,double dTolerance) const
    {
    if(m_bSingularHighU)
        {
        if(SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMax,dTolerance,false))
            {
            return true;
            }
        }
    if(m_bSingularHighV)
        {
        if(SGM::NearEqual(uv.m_v,m_Domain.m_VDomain.m_dMax,dTolerance,false))
            {
            return true;
            }
        }
    if(m_bSingularLowU)
        {
        if(SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMin,dTolerance,false))
            {
            return true;
            }
        }
    if(m_bSingularLowV)
        {
        if(SGM::NearEqual(uv.m_v,m_Domain.m_VDomain.m_dMin,dTolerance,false))
            {
            return true;
            }
        }
    return false;
    }

bool surface::IsSame(surface const *pOther,double dTolerance) const
    {
    if(pOther->m_SurfaceType!=m_SurfaceType)
        {
        return false;
        }
    switch(m_SurfaceType) // Note that at this point we know that this and pOther are the same type.
        {
        case SGM::EntityType::SphereType:
            {
            bool bAnswer=true;
            sphere const *pSphere1=(sphere const *)this;
            sphere const *pSphere2=(sphere const *)pOther;
            if(SGM::NearEqual(pSphere1->m_dRadius,pSphere2->m_dRadius,dTolerance,false)==false)
                {
                bAnswer=false;
                }
            else if(SGM::NearEqual(pSphere1->m_Center,pSphere2->m_Center,dTolerance)==false)
                {
                bAnswer=false;
                }
            return bAnswer;
            }
        case SGM::EntityType::CylinderType:
            {
            bool bAnswer=true;
            cylinder const *pCylinder1=(cylinder const *)this;
            cylinder const *pCylinder2=(cylinder const *)pOther;
            if(SGM::NearEqual(pCylinder1->m_dRadius,pCylinder2->m_dRadius,dTolerance,false)==false)
                {
                bAnswer=false;
                }
            else if(SGM::NearEqual(fabs(pCylinder1->m_ZAxis%pCylinder2->m_ZAxis),1.0,dTolerance,false)==false)
                {
                bAnswer=false;
                }
            else 
                {
                SGM::Point3D const &Pos1=pCylinder1->m_Origin;
                SGM::Point3D const &Pos2=pCylinder2->m_Origin;
                SGM::UnitVector3D const &Axis1=pCylinder1->m_ZAxis;
                if(SGM::NearEqual(Pos2.Distance(Pos1+Axis1*((Pos2-Pos1)%Axis1)),0.0,dTolerance,false)==false)
                    {
                    bAnswer=false;
                    }
                }
            return bAnswer;
            }
        case SGM::EntityType::ConeType:
            {
            bool bAnswer=true;
            cone const *pCone1=(cone const *)this;
            cone const *pCone2=(cone const *)pOther;
            if(SGM::NearEqual(pCone1->m_dCosHalfAngle,pCone2->m_dCosHalfAngle,dTolerance,false)==false)
                {
                bAnswer=false;
                }
            else if(SGM::NearEqual(pCone1->m_ZAxis%pCone2->m_ZAxis,1.0,dTolerance,false)==false)
                {
                bAnswer=false;
                }
            else 
                {
                SGM::Point3D const &Pos1=pCone1->FindApex();
                SGM::Point3D const &Pos2=pCone2->FindApex();
                if(SGM::NearEqual(Pos1.Distance(Pos2),0.0,dTolerance,false)==false)
                    {
                    bAnswer=false;
                    }
                }
            return bAnswer;
            }
        case SGM::EntityType::TorusType:
            {
            bool bAnswer=true;
            torus const *pTorus1=(torus const *)this;
            torus const *pTorus2=(torus const *)pOther;
            if(SGM::NearEqual(pTorus1->m_dMajorRadius,pTorus2->m_dMajorRadius,dTolerance,false)==false)
                {
                bAnswer=false;
                }
            else if(SGM::NearEqual(pTorus1->m_dMinorRadius,pTorus2->m_dMinorRadius,dTolerance,false)==false)
                {
                bAnswer=false;
                }
            else if(SGM::NearEqual(fabs(pTorus1->m_ZAxis%pTorus2->m_ZAxis),1.0,dTolerance,false)==false)
                {
                bAnswer=false;
                }
            else if(SGM::NearEqual(pTorus1->m_Center,pTorus2->m_Center,dTolerance)==false)
                {
                bAnswer=false;
                }
            return bAnswer;
            }
        default:
            {
            return false;
            }
        }
    }

void surface::PrincipleCurvature(SGM::Point2D const &uv,
                                 SGM::UnitVector3D  &Vec1,
                                 SGM::UnitVector3D  &Vec2,
                                 double             &k1,
                                 double             &k2) const
    {
    switch(m_SurfaceType)
        {
        case SGM::EntityType::PlaneType:
            {
            plane const *pPlane=(plane const *)this;
            k1=0;
            k2=0;
            Vec1=pPlane->m_XAxis;
            Vec2=pPlane->m_YAxis;
            break;
            }
        case SGM::EntityType::SphereType:
            {
            sphere const *pSphere=(sphere const *)this;
            k1=1.0/pSphere->m_dRadius;
            k2=1.0/pSphere->m_dRadius;
            SGM::Vector3D dU,dV;
            pSphere->Evaluate(uv,nullptr,&dU,&dV);
            Vec1=dU;
            Vec2=dV;
            break;
            }
        case SGM::EntityType::CylinderType:
            {
            cylinder const *pCylinder=(cylinder const *)this;
            k1=1.0/pCylinder->m_dRadius;
            k2=0.0;
            SGM::Vector3D dU,dV;
            pCylinder->Evaluate(uv,nullptr,&dU,&dV);
            Vec1=dU;
            Vec2=dV;
            break;
            }
        case SGM::EntityType::ConeType:
            {
            cone const *pCone=(cone const *)this;
            SGM::Vector3D dU,dV;
            SGM::Point3D Pos;
            pCone->Evaluate(uv,&Pos,&dU,&dV);
            double dDist=Pos.Distance(pCone->m_Origin+(pCone->m_ZAxis)*((Pos-pCone->m_Origin)%(pCone->m_ZAxis)));
            if(SGM_ZERO<dDist)
                {
                k1=1.0/dDist;
                }
            else
                {
                k1=SGM_MAX;
                }
            k2=0.0;
            Vec1=dU;
            Vec2=dV;
            break;
            }
        default:
            {
            // Find the eigen vectors and values of the second fundamental form.
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
        }
    }

double surface::DirectionalCurvature(SGM::Point2D      const &uv,
                                     SGM::UnitVector3D const &Direction) const
    {
    SGM::UnitVector3D Vec1,Vec2,Norm;
    double k1,k2;
    PrincipleCurvature(uv,Vec1,Vec2,k1,k2);
    Norm=Vec1*Vec2;
    double dt=Direction.Angle(Vec1,Norm);
    double dCos=cos(dt);
    double dSin=sin(dt);
    return k1*dCos*dCos+k2*dSin*dSin;
    }

int surface::UContinuity() const
    {
    switch(m_SurfaceType)
        {
        case SGM::NUBSurfaceType:
            {
            NUBsurface const *pNUB=(NUBsurface const*)this;
            return pNUB->UContinuity();
            }
        case SGM::NURBSurfaceType:
            {
            NURBsurface const *pNURB=(NURBsurface const*)this;
            return pNURB->UContinuity();
            }
        default:
            {
            return std::numeric_limits<int>::max();
            }
        }
    }

int surface::VContinuity() const
    {
    switch(m_SurfaceType)
        {
        case SGM::NUBSurfaceType:
            {
            NUBsurface const *pNUB=(NUBsurface const*)this;
            return pNUB->VContinuity();
            }
        case SGM::NURBSurfaceType:
            {
            NURBsurface const *pNURB=(NURBsurface const*)this;
            return pNURB->VContinuity();
            }
        default:
            {
            return std::numeric_limits<int>::max();
            }
        }
    }

void surface::SnapToDomain(SGM::Point2D &uv) const
    {
    if(m_bClosedU)
        {
        while(uv.m_u<m_Domain.m_UDomain.m_dMin)
            {
            uv.m_u+=m_Domain.m_UDomain.Length();
            }
        while(m_Domain.m_UDomain.m_dMax<uv.m_u)
            {
            uv.m_u-=m_Domain.m_UDomain.Length();
            }
        }
    else
        {
        if(uv.m_u<m_Domain.m_UDomain.m_dMin)
            {
            uv.m_u=m_Domain.m_UDomain.m_dMin;
            }
        if(m_Domain.m_UDomain.m_dMax<uv.m_u)
            {
            uv.m_u=m_Domain.m_UDomain.m_dMax;
            }
        }
    if(m_bClosedV)
        {
        while(uv.m_v<m_Domain.m_VDomain.m_dMin)
            {
            uv.m_v+=m_Domain.m_VDomain.Length();
            }
        while(m_Domain.m_VDomain.m_dMax<uv.m_v)
            {
            uv.m_v-=m_Domain.m_VDomain.Length();
            }
        }
    else
        {
        if(uv.m_v<m_Domain.m_VDomain.m_dMin)
            {
            uv.m_v=m_Domain.m_VDomain.m_dMin;
            }
        if(m_Domain.m_VDomain.m_dMax<uv.m_v)
            {
            uv.m_v=m_Domain.m_VDomain.m_dMax;
            }
        }
    }

SGM::UnitVector2D surface::FindSurfaceDirection(SGM::Point2D        &uv,
                                                SGM::Vector3D const &Vec) const
    {
    SGM::Vector3D Du,Dv;
    Evaluate(uv,nullptr,&Du,&Dv);
    SGM::UnitVector3D UDu=Du,VDv=Dv;
    double du=UDu%Vec;
    double dv=VDv%Vec;
    return SGM::UnitVector2D(du,dv);
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

            while(uv.m_u<m_Domain.m_UDomain.m_dMin)
                {
                uv.m_u+=SGM_TWO_PI;
                }
            while(m_Domain.m_UDomain.m_dMax<uv.m_u)
                {
                uv.m_u-=SGM_TWO_PI;
                }

            if(pGuess)
                {
                // Check for points on the axis, and on the seam.

               if(m_Domain.m_UDomain.InInterval(uv.m_u,SGM_ZERO)==false)
                    {
                    if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false) &&
                        SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false))
                        {
                        uv.m_u=m_Domain.m_UDomain.m_dMax;
                        }
                    else if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false) &&
                             SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false))
                        {
                        uv.m_u=m_Domain.m_UDomain.m_dMin;
                        }
                    }
                else if(SGM::NearEqual(fabs(SGM::UnitVector3D(Pos-Center)%ZAxis),1.0,SGM_MIN_TOL,false))
                    {
                    uv.m_u=pGuess->m_u;
                    }
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
            double dz=x*ZAxis.m_x+y*ZAxis.m_y+z*ZAxis.m_z;
            uv.m_u=SGM::SAFEatan2(dy,dx);
            uv.m_v=dz/(dCosHalfAngle*dRadius);

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
                // Check for points on the axis, and on the seam.

               if(m_Domain.m_UDomain.InInterval(uv.m_u,SGM_ZERO)==false)
                    {
                    if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false) &&
                        SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false))
                        {
                        uv.m_u=m_Domain.m_UDomain.m_dMax;
                        }
                    else if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false) &&
                             SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false))
                        {
                        uv.m_u=m_Domain.m_UDomain.m_dMin;
                        }
                    }
                else if(SGM::NearEqual(fabs(SGM::UnitVector3D(Pos-Center)%ZAxis),1.0,SGM_MIN_TOL,false))
                    {
                    uv.m_u=pGuess->m_u;
                    }
                }
            
            if(ClosePos)
                {
                double dVScale=dRadius*(1.0-uv.m_v*dSinHalfAngle);
                double dZScale=uv.m_v*dRadius*dCosHalfAngle;
                
                double dCosU=cos(uv.m_u);
                double dSinU=sin(uv.m_u);
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
            SGM::Vector3D VSpoke=(Pos-ZAxis*((Pos-Center)%ZAxis))-Center;
            if(VSpoke.Magnitude()<SGM_ZERO)
                {
                if((Pos-Center)%ZAxis<0)
                    {
                    // South Pole
                    uv.m_v=-SGM_HALF_PI;
                    }
                else
                    {
                    // North Pole
                    uv.m_v=SGM_HALF_PI;
                    }
                }
            else
                {
                SGM::UnitVector3D Spoke=VSpoke;
                double dVx=x*Spoke.m_x+y*Spoke.m_y+z*Spoke.m_z;
                double dVy=x*ZAxis.m_x+y*ZAxis.m_y+z*ZAxis.m_z;
                uv.m_v=SGM::SAFEatan2(dVy,dVx);
                }

            if(uv.m_u<m_Domain.m_UDomain.m_dMin)
                {
                uv.m_u+=SGM_TWO_PI;
                }
            if(pGuess)
                {
                // Check for points on the north-south pole axis, at the 
                // center, and on the seam.

                if(SGM::NearEqual(Pos,Center,SGM_MIN_TOL))
                    {
                    uv=*pGuess;
                    }
                else if(m_Domain.m_UDomain.InInterval(uv.m_u,SGM_ZERO)==false)
                    {
                    if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false) &&
                        SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false))
                        {
                        uv.m_u=m_Domain.m_UDomain.m_dMax;
                        }
                    else if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false) &&
                             SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false))
                        {
                        uv.m_u=m_Domain.m_UDomain.m_dMin;
                        }
                    }
                else if(SGM::NearEqual(fabs(SGM::UnitVector3D(Pos-Center)%ZAxis),1.0,SGM_MIN_TOL,false))
                    {
                    uv.m_u=pGuess->m_u;
                    }
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

            // Find the u value.

            double x=Pos.m_x-Center.m_x;
            double y=Pos.m_y-Center.m_y;
            double z=Pos.m_z-Center.m_z;

            double dUx=x*XAxis.m_x+y*XAxis.m_y+z*XAxis.m_z;
            double dUy=x*YAxis.m_x+y*YAxis.m_y+z*YAxis.m_z;
            uv.m_u=SGM::SAFEatan2(dUy,dUx);

            // Find the v value.

            SGM::UnitVector3D Spoke=(Pos-ZAxis*((Pos-Center)%ZAxis))-Center;
            if(pTorus->GetKind()==SGM::TorusKindType::LemonType)
                {
                Spoke.Negate();
                }
            double cx=Pos.m_x-Center.m_x-Spoke.m_x*dMajorRadius;
            double cy=Pos.m_y-Center.m_y-Spoke.m_y*dMajorRadius;
            double cz=Pos.m_z-Center.m_z-Spoke.m_z*dMajorRadius;
            double dVx=cx*Spoke.m_x+cy*Spoke.m_y+cz*Spoke.m_z;
            double dVy=cx*ZAxis.m_x+cy*ZAxis.m_y+cz*ZAxis.m_z;
            uv.m_v=SGM::SAFEatan2(dVy,dVx);
            
            // Adjust to the domain.
            
            while(uv.m_v<m_Domain.m_VDomain.m_dMin)
                {
                uv.m_v+=SGM_TWO_PI;
                }
            while(uv.m_u<m_Domain.m_UDomain.m_dMin)
                {
                uv.m_u+=SGM_TWO_PI;
                }
            while(uv.m_v>m_Domain.m_VDomain.m_dMax)
                {
                uv.m_v-=SGM_TWO_PI;
                }
            while(uv.m_u>m_Domain.m_UDomain.m_dMax)
                {
                uv.m_u-=SGM_TWO_PI;
                }
            
            // fix apples and lemons

            if( pTorus->GetKind()==SGM::TorusKindType::AppleType ||
                pTorus->GetKind()==SGM::TorusKindType::LemonType)
                {
                if(m_Domain.m_VDomain.InInterval(uv.m_v,SGM_ZERO)==false)
                    {
                    SGM::Point3D NorthPole,SouthPole;
                    SGM::Point2D uv1(uv.m_u,m_Domain.m_VDomain.m_dMax);
                    pTorus->Evaluate(uv1,&NorthPole);
                    SGM::Point2D uv2(uv.m_u,m_Domain.m_VDomain.m_dMin);
                    pTorus->Evaluate(uv2,&SouthPole);
                    double dNorth=NorthPole.DistanceSquared(Pos);
                    double dSouth=SouthPole.DistanceSquared(Pos);
                    if(dNorth<dSouth)
                        {
                        uv.m_v=m_Domain.m_VDomain.m_dMax;
                        }
                    else
                        {
                        uv.m_v=m_Domain.m_VDomain.m_dMin;
                        }
                    }
                }

            if(pGuess)
                {
                // Check for points on the axis, and on the seam.

               if(m_Domain.m_UDomain.InInterval(uv.m_u,SGM_ZERO)==false)
                    {
                    if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false) &&
                        SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false))
                        {
                        uv.m_u=m_Domain.m_UDomain.m_dMax;
                        }
                    else if( SGM::NearEqual(pGuess->m_u,m_Domain.m_UDomain.m_dMin,SGM_MIN_TOL,false) &&
                             SGM::NearEqual(uv.m_u,m_Domain.m_UDomain.m_dMax,SGM_MIN_TOL,false))
                        {
                        uv.m_u=m_Domain.m_UDomain.m_dMin;
                        }
                    }
               if( pTorus->m_nKind!=SGM::TorusKindType::LemonType &&
                   pTorus->m_nKind!=SGM::TorusKindType::AppleType &&
                   m_Domain.m_VDomain.InInterval(uv.m_v,SGM_ZERO)==false)
                    {
                    if( SGM::NearEqual(pGuess->m_v,m_Domain.m_VDomain.m_dMax,SGM_MIN_TOL,false) &&
                        SGM::NearEqual(uv.m_v,m_Domain.m_VDomain.m_dMin,SGM_MIN_TOL,false))
                        {
                        uv.m_v=m_Domain.m_VDomain.m_dMax;
                        }
                    else if( SGM::NearEqual(pGuess->m_v,m_Domain.m_VDomain.m_dMin,SGM_MIN_TOL,false) &&
                             SGM::NearEqual(uv.m_v,m_Domain.m_VDomain.m_dMax,SGM_MIN_TOL,false))
                        {
                        uv.m_v=m_Domain.m_VDomain.m_dMin;
                        }
                    }
                else if(SGM::NearEqual(fabs(SGM::UnitVector3D(Pos-Center)%ZAxis),1.0,SGM_MIN_TOL,false))
                    {
                    uv.m_u=pGuess->m_u;
                    }
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
        case SGM::NURBSurfaceType:
            {
            NURBsurface const *pNURBSurface=(NURBsurface const *)this;

            SGM::Point2D StartUV(0.0,0.0);
            if(pGuess)
                {
                StartUV=*pGuess;
                }
            else
                {
                std::vector<SGM::Point3D> const &aSeedPoints=pNURBSurface->GetSeedPoints();
                std::vector<SGM::Point2D> const &aSeedParams=pNURBSurface->GetSeedParams();
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
        case SGM::RevolveType:
            {
            revolve const *pRevolve=(revolve *)this;

            SGM::Point3D      const &Origin =pRevolve->m_Origin;
            SGM::UnitVector3D const &XAxis  =pRevolve->m_XAxis;
            SGM::UnitVector3D const &ZAxis  =pRevolve->m_ZAxis;

            uv.m_u = 0; // default u output to 0

            // get x and y in the local coordinate system
            double dLocalX = (Pos - pRevolve->m_Origin) % pRevolve->m_XAxis;
            double dLocalY = (Pos - pRevolve->m_Origin) % pRevolve->m_YAxis;
            double dLocalZ = (Pos - pRevolve->m_Origin) % pRevolve->m_ZAxis;

            // get the u parameter - angle around the rotation
            if (fabs(dLocalX) < SGM_ZERO) // on local X-Z plane - seam, axis, or PI/2
            {
                if (fabs(dLocalY) < SGM_ZERO) // on the axis of rotation
                {
                    if (pGuess != nullptr)
                    {
                        if (pRevolve->m_pCurve->GetDomain().InInterval(pGuess->m_u,SGM_ZERO))
                            uv.m_u = pGuess->m_u;
                    }
                }
                else
                {
                    uv.m_u = SGM::SAFEatan2(dLocalY, dLocalX);

                    if (fabs(uv.m_u) < SGM_MIN_TOL) // on the seam
                    {
                        if (pGuess != nullptr)
                        {
                            if (fabs(SGM_TWO_PI - pGuess->m_u) < SGM_ZERO)
                                uv.m_u = SGM_TWO_PI;
                        }
                    }
                }
            }
            else
            {
                uv.m_u = SGM::SAFEatan2(dLocalY, dLocalX);
            }

            if (uv.m_u < 0)
              uv.m_u += SGM_TWO_PI;

            // rotate the point to the local X-Z plane and use the curve Inverse to find v
            double dRadius = sqrt(dLocalX*dLocalX + dLocalY*dLocalY);
            SGM::Point3D PointToProject = Origin + dRadius*XAxis + dLocalZ * ZAxis;

            uv.m_v = pRevolve->m_pCurve->Inverse(PointToProject);

            if(ClosePos != nullptr)
            {
                pRevolve->Evaluate(uv,ClosePos);
            }

            break;
            }
        
        case SGM::ExtrudeType:
            {
            extrude const *pExtrude=(extrude const *)this;
            SGM::UnitVector3D const &vAxis=pExtrude->m_vAxis;
            SGM::Point3D const &Origin=pExtrude->m_Origin;
            
            uv.m_v=(Pos-Origin)%vAxis;
            SGM::Point3D PlanePos=Pos-vAxis*uv.m_v;
            if(pGuess)
                {
                uv.m_u=pExtrude->m_pCurve->Inverse(PlanePos,ClosePos,&(pGuess->m_u));
                }
            else
                {
                uv.m_u=pExtrude->m_pCurve->Inverse(PlanePos,ClosePos);
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
}
