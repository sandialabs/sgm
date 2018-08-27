#include "SGMEntityClasses.h"
#include "SGMMathematics.h"
#include "SGMTransform.h"

#include "EntityClasses.h"
#include "Surface.h"
#include "Curve.h"

namespace SGMInternal
{
    revolve::revolve(SGM::Result             &rResult,
                     SGM::Point3D      const &pAxisOrigin,
                     SGM::UnitVector3D const &uAxisVector,
                     curve                   *pCurve) :
        surface(rResult, SGM::RevolveType),
        m_pCurve(nullptr),
        m_Origin(pAxisOrigin),
        m_XAxis(),
        m_YAxis(),
        m_ZAxis(uAxisVector)
    {
    this->m_bClosedU = true;
    m_Domain.m_UDomain.m_dMin = 0.0;
    m_Domain.m_UDomain.m_dMax = SGM_TWO_PI;

    if (pCurve)
        SetCurve(pCurve);
    }

bool revolve::IsSame(surface const *pOther,double dTolerance) const
    {
    if(pOther->GetSurfaceType()!=m_SurfaceType)
        {
        return false;
        }
    revolve const *pRevolve2=(revolve const *)pOther;
    if(SGM::NearEqual(m_Origin,pRevolve2->m_Origin,dTolerance)==false)
        {
        return false;
        }
    if(SGM::NearEqual(m_ZAxis,pRevolve2->m_ZAxis,dTolerance)==false)
        {
        return false;
        }
    if(m_pCurve->IsSame(pRevolve2->m_pCurve,dTolerance)==false)
        {
        return false;
        }
    return true;
    }

revolve::~revolve()
    {
    if (m_pCurve)
        m_pCurve->RemoveOwner(this);
    }

revolve::revolve(SGM::Result &rResult, revolve const &other) :
        surface(rResult, other),
        m_pCurve(nullptr),
        m_Origin(other.m_Origin),
        m_XAxis(other.m_XAxis),
        m_YAxis(other.m_YAxis),
        m_ZAxis(other.m_ZAxis)
    {
    if (other.m_pCurve)
        SetCurve(other.m_pCurve->Clone(rResult));
    }

revolve* revolve::Clone(SGM::Result &rResult) const
{ return new revolve(rResult, *this); }

void revolve::FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const
    {
    sChildren.insert(m_pCurve);
    }

void revolve::Evaluate(SGM::Point2D const &uv,
                       SGM::Point3D       *Pos,
                       SGM::Vector3D      *Du,
                       SGM::Vector3D      *Dv,
                       SGM::UnitVector3D  *Norm,
                       SGM::Vector3D      *Duu,
                       SGM::Vector3D      *Duv,
                       SGM::Vector3D      *Dvv) const
    {
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
    m_pCurve->Evaluate(uv.m_v, &CurvePos, pDvCurve, pDvvCurve);
    SGM::Point3D AxisPos = m_Origin + ((CurvePos - m_Origin) % m_ZAxis) * m_ZAxis;
    SGM::Vector3D vRadius = CurvePos - AxisPos;
    double dRadius = vRadius.Magnitude();

    if (pDvCurve)
        {
        dvAxisPos = (DvCurve % m_ZAxis) * m_ZAxis;
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
        *Pos = AxisPos + dRCos * m_XAxis + dRSin * m_YAxis;

    if (nullptr != Du || nullptr != Norm )
        DuLocal = (dRCos * m_YAxis) - (dRSin * m_XAxis);

    if (nullptr != Dv || nullptr != Norm )
        DvLocal = dvAxisPos + (dvRadius * dCos * m_XAxis) + (dvRadius * dSin * m_YAxis);

    if (nullptr != Du)
        *Du = DuLocal;

    if (nullptr != Dv)
        *Dv = DvLocal;

    if (nullptr != Norm)
        *Norm = DuLocal * DvLocal;

    if (nullptr != Duu)
        *Duu = (-1) * ( (dRSin * m_YAxis) + (dRCos * m_XAxis) );

    if (nullptr != Duv)
        *Duv = (dvRadius * dCos * m_YAxis) - (dvRadius * dSin * m_XAxis);

    if (nullptr != Dvv)
        {
        dvvAxisPos = (DvvCurve % m_ZAxis) * m_ZAxis;
        double A2_half = (((DvCurve.m_x - dvAxisPos.m_x) * (DvCurve.m_x - dvAxisPos.m_x) + (vRadius.m_x) * (DvvCurve.m_x - dvvAxisPos.m_x)) +
                          ((DvCurve.m_y - dvAxisPos.m_y) * (DvCurve.m_y - dvAxisPos.m_y) + (vRadius.m_y) * (DvvCurve.m_y - dvvAxisPos.m_y)) +
                          ((DvCurve.m_z - dvAxisPos.m_z) * (DvCurve.m_z - dvAxisPos.m_z) + (vRadius.m_z) * (DvvCurve.m_z - dvvAxisPos.m_z)));
        double dvvRadius = ((-1 * A1_half * A1_half) / (dRadius * dRadius * dRadius)) + ((A2_half) / (dRadius));
        *Dvv = dvvAxisPos + (dvvRadius * dCos * m_XAxis) + (dvvRadius * dSin * m_YAxis);
        }
    }

SGM::Point2D revolve::Inverse(SGM::Point3D const &Pos,
                              SGM::Point3D       *ClosePos,
                              SGM::Point2D const *pGuess) const
    { 
    SGM::Point2D uv;
    
    uv.m_u = 0; // default u output to 0

    // get x and y in the local coordinate system
    double dLocalX = (Pos - m_Origin) % m_XAxis;
    double dLocalY = (Pos - m_Origin) % m_YAxis;
    double dLocalZ = (Pos - m_Origin) % m_ZAxis;

    // get the u parameter - angle around the rotation
    if (fabs(dLocalX) < SGM_ZERO) // on local X-Z plane - seam, axis, or PI/2
        {
        if (fabs(dLocalY) < SGM_ZERO) // on the axis of rotation
            {
            if (pGuess != nullptr)
                {
                if (m_pCurve->GetDomain().InInterval(pGuess->m_u,SGM_ZERO))
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
    SGM::Point3D PointToProject = m_Origin + dRadius*m_XAxis + dLocalZ * m_ZAxis;

    uv.m_v = m_pCurve->Inverse(PointToProject);

    if(ClosePos != nullptr)
        {
        Evaluate(uv,ClosePos);
        }

    return uv;
    }
    
void revolve::Transform(SGM::Transform3D const &Trans)
    {
    m_Origin=Trans*m_Origin;
    m_XAxis=Trans*m_XAxis;
    m_YAxis=Trans*m_YAxis;
    m_ZAxis=Trans*m_ZAxis;
    if(m_pCurve->GetEdges().empty() && m_pCurve->GetOwners().size()==1)
        {
        m_pCurve->Transform(Trans);
        }
    else
        {
        //TODO: Make a copy and transform the copy.
        throw std::logic_error("Missing implementation of Transform() when curve has other owners");
        }
    }

curve *revolve::UParamLine(SGM::Result &rResult, double) const
    {
    curve *pParam=m_pCurve->Clone(rResult);
    return pParam;
    }

curve *revolve::VParamLine(SGM::Result &, double) const
    { throw std::logic_error("Derived class of surface must override VParamLine()"); }

void revolve::SetCurve(curve *pCurve)
    {
    pCurve->AddOwner(this);
    m_pCurve = pCurve;

    SGM::Point3D start;
    pCurve->Evaluate(pCurve->GetDomain().MidPoint(), &start);
    m_Origin = m_Origin + ((start - m_Origin) % m_ZAxis) * m_ZAxis;

    m_XAxis = start - m_Origin;
    m_YAxis = m_ZAxis * m_XAxis;

    this->m_bClosedV = pCurve->GetClosed();
    m_Domain.m_VDomain = pCurve->GetDomain();
    }
}
