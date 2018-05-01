#include "SGMDataClasses.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include <math.h>
#include <algorithm>

///////////////////////////////////////////////////////////////////////////////
//
//  Point2D methods
//
///////////////////////////////////////////////////////////////////////////////

double SGM::Point2D::Distance(SGM::Point2D const &Pos) const
    {
    double dU=Pos.m_u-m_u;
    double dV=Pos.m_v-m_v;
    return sqrt(dU*dU+dV*dV);
    }

double SGM::Point2D::DistanceSquared(SGM::Point2D const &Pos) const
    {
    double dU=Pos.m_u-m_u;
    double dV=Pos.m_v-m_v;
    return dU*dU+dV*dV;
    }

SGM::Point2D SGM::Point2D::operator+=(SGM::Vector2D const &Vec)
    {
    m_u+=Vec.m_u;
    m_v+=Vec.m_v;
    return *this;
    }

bool SGM::Point2D::operator<(SGM::Point2D const &Pos) const
    {
    if(m_u<Pos.m_u)
        {
        return true;
        }
    else if(Pos.m_u<m_u)
        {
        return false;
        }
    else
        {
        if(m_v<Pos.m_v)
            {
            return true;
            }
        }
    return false;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Vector3D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::Vector3D::Vector3D(Point3D const &Pos):m_x(Pos.m_x),m_y(Pos.m_y),m_z(Pos.m_z)
    {
    }

SGM::Vector3D SGM::Vector3D::Orthogonal() const
    {
    double dX=fabs(m_x);
    double dY=fabs(m_y);
    double dZ=fabs(m_z);
    if(dY<=dX && dZ<=dX)
        {
        return SGM::Vector3D(-m_y,m_x,0.0);
        }
    else if(dX<=dY && dZ<=dY)
        {
        return SGM::Vector3D(-m_y,m_x,0.0);
        }
    else
        {
        return SGM::Vector3D(m_z,0.0,-m_x);
        }
    }

SGM::Vector3D SGM::Vector3D::operator*(double dScale) const
    {
    return SGM::Vector3D(m_x*dScale,m_y*dScale,m_z*dScale);
    }

SGM::Vector3D SGM::Vector3D::operator/(double dScale) const
    {
    return SGM::Vector3D(m_x/dScale,m_y/dScale,m_z/dScale);
    }

SGM::Vector3D SGM::Vector3D::operator*=(SGM::Transform3D const &Trans)
    {
    *this=Trans*(*this);
    return *this;
    }

double SGM::Vector3D::Magnitude() const
    {
    return sqrt(m_x*m_x+m_y*m_y+m_z*m_z);
    }

double SGM::Vector3D::MagnitudeSquared() const
    {
    return m_x*m_x+m_y*m_y+m_z*m_z;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Vector4D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::Vector4D::Vector4D(Point4D const &Pos):m_x(Pos.m_x),m_y(Pos.m_y),
    m_z(Pos.m_z),m_w(Pos.m_w)
    {
    }

SGM::Vector4D SGM::Vector4D::operator*(double dScale) const
    {
    return SGM::Vector4D(m_x*dScale,m_y*dScale,m_z*dScale,m_w*dScale);
    }

SGM::Vector4D SGM::Vector4D::operator/(double dScale) const
    {
    return SGM::Vector4D(m_x/dScale,m_y/dScale,m_z/dScale,m_w/dScale);
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Point3D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::Point3D::Point3D(Vector3D const &Vec):m_x(Vec.m_x),m_y(Vec.m_y),m_z(Vec.m_z)
    {
    }

double SGM::Point3D::Distance(SGM::Point3D const &Pos) const
    {
    double dX=Pos.m_x-m_x;
    double dY=Pos.m_y-m_y;
    double dZ=Pos.m_z-m_z;
    return sqrt(dX*dX+dY*dY+dZ*dZ);
    }

double SGM::Point3D::DistanceSquared(SGM::Point3D const &Pos) const
    {
    double dX=Pos.m_x-m_x;
    double dY=Pos.m_y-m_y;
    double dZ=Pos.m_z-m_z;
    return dX*dX+dY*dY+dZ*dZ;
    }

SGM::Point3D SGM::Point3D::operator+=(SGM::Vector3D const &Vec) 
    {
    m_x+=Vec.m_x;
    m_y+=Vec.m_y;
    m_z+=Vec.m_z;
    return *this;
    }

SGM::Point3D SGM::Point3D::operator*=(SGM::Transform3D const &Trans)
    {
    *this=Trans*(*this);
    return *this;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Point4D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::Point4D SGM::Point4D::operator+=(SGM::Vector4D const &Vec) 
    {
    m_x+=Vec.m_x;
    m_y+=Vec.m_y;
    m_z+=Vec.m_z;
    m_w+=Vec.m_w;
    return *this;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  UnitVector2D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::UnitVector2D::UnitVector2D(SGM::Vector2D const &Vec)
    {
    double dMagSquared=Vec.m_u*Vec.m_u+Vec.m_v*Vec.m_v;
    if( 0.999999999999<dMagSquared &&
        dMagSquared<1.000000000001)
        {
        m_u=Vec.m_u;
        m_v=Vec.m_v;
        }
    else if(1E-24<dMagSquared)
        {
        double dMag=sqrt(dMagSquared);
        double dScale=1.0/dMag;
        m_u=Vec.m_u*dScale;
        m_v=Vec.m_v*dScale;
        }
    else
        {
        m_u=0.0;
        m_v=1.0;
        }
    }

SGM::UnitVector2D::UnitVector2D(double u,double v)
    {
    double dMagSquared=u*u+v*v;
    if( 0.999999999999<dMagSquared &&
        dMagSquared<1.000000000001)
        {
        m_u=u;
        m_v=v;
        }
    else if(1E-24<dMagSquared)
        {
        double dMag=sqrt(dMagSquared);
        double dScale=1.0/dMag;
        m_u=u*dScale;
        m_v=v*dScale;
        }
    else
        {
        m_u=0.0;
        m_v=1.0;
        }
    }

SGM::UnitVector2D SGM::UnitVector2D::operator*(double dScale) const
    {
    return SGM::UnitVector2D(m_u*dScale,m_v*dScale);
    }

///////////////////////////////////////////////////////////////////////////////
//
//  UnitVector3D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::UnitVector3D::UnitVector3D(SGM::Vector3D const &Vec)
    {
    double dMagSquared=Vec.m_x*Vec.m_x+Vec.m_y*Vec.m_y+Vec.m_z*Vec.m_z;
    if( 0.999999999999<dMagSquared &&
        dMagSquared<1.000000000001)
        {
        m_x=Vec.m_x;
        m_y=Vec.m_y;
        m_z=Vec.m_z;
        }
    else if(1E-24<dMagSquared)
        {
        double dMag=sqrt(dMagSquared);
        double dScale=1.0/dMag;
        m_x=Vec.m_x*dScale;
        m_y=Vec.m_y*dScale;
        m_z=Vec.m_z*dScale;
        }
    else
        {
        m_x=0.0;
        m_y=0.0;
        m_z=1.0;
        }
    }

SGM::UnitVector3D::UnitVector3D(double x,double y,double z)
    {
    double dMagSquared=x*x+y*y+z*z;
    if( 0.999999999999<dMagSquared &&
        dMagSquared<1.000000000001)
        {
        m_x=x;
        m_y=y;
        m_z=z;
        }
    else if(1E-24<dMagSquared)
        {
        double dMag=sqrt(dMagSquared);
        double dScale=1.0/dMag;
        m_x=x*dScale;
        m_y=y*dScale;
        m_z=z*dScale;
        }
    else
        {
        m_x=0.0;
        m_y=0.0;
        m_z=1.0;
        }
    }

SGM::UnitVector3D SGM::UnitVector3D::operator*=(SGM::Transform3D const &Trans)
    {
    *this=Trans*(*this);
    return *this;
    }

void SGM::UnitVector3D::Negate()
    {
    m_x=-m_x;
    m_y=-m_y;
    m_z=-m_z;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  UnitVector4D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::UnitVector4D::UnitVector4D(SGM::Vector4D const &Vec)
    {
    double dMagSquared=Vec.m_x*Vec.m_x+Vec.m_y*Vec.m_y+Vec.m_z*Vec.m_z+Vec.m_w*Vec.m_w;
    if( 0.999999999999<dMagSquared &&
        dMagSquared<1.000000000001)
        {
        m_x=Vec.m_x;
        m_y=Vec.m_y;
        m_z=Vec.m_z;
        m_w=Vec.m_w;
        }
    else if(1E-24<dMagSquared)
        {
        double dMag=sqrt(dMagSquared);
        double dScale=1.0/dMag;
        m_x=Vec.m_x*dScale;
        m_y=Vec.m_y*dScale;
        m_z=Vec.m_z*dScale;
        m_w=Vec.m_w*dScale;
        }
    else
        {
        m_x=0.0;
        m_y=0.0;
        m_z=0.0;
        m_w=1.0;
        }
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Interval1D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::Interval1D const &SGM::Interval1D::operator+=(SGM::Interval1D const &domain)
    {
    if(m_dMax<m_dMin)   // This is empty so the answer is the given domain.
        {
        m_dMin=domain.m_dMin;
        m_dMax=domain.m_dMax;
        }
    else if(domain.m_dMin<=domain.m_dMax) // The given domain is not empty.
        {
        m_dMin=std::min(m_dMin,domain.m_dMin);
        m_dMax=std::max(m_dMax,domain.m_dMax);
        }
    return *this;
    }

SGM::Interval1D const &SGM::Interval1D::operator&=(SGM::Interval1D const &domain)
    {
    if(m_dMax<m_dMin)   // This is empty so the answer is empty.
        {
        m_dMin=1;
        m_dMax=0;
        }
    else if(domain.m_dMin<=domain.m_dMax) // The given domain is not empty.
        {
        m_dMin=std::max(m_dMin,domain.m_dMin);
        m_dMax=std::min(m_dMax,domain.m_dMax);
        }
    return *this;
    }

bool SGM::Interval1D::operator&&(SGM::Interval1D const &domain) const
    {
    if(m_dMax<m_dMin || domain.m_dMax<domain.m_dMin)   
        {
        return false; // One interval is empty so the answer is false.
        }
    return std::max(m_dMin,domain.m_dMin)<=std::min(m_dMax,domain.m_dMax);
    }

bool SGM::Interval1D::InInterval(double Pos) const
    {
    if(m_dMax<m_dMin)
        {
        return false;
        }
    return m_dMin<=Pos && Pos<=m_dMax;
    }

bool SGM::Interval1D::OnBoundary(double Pos,double dTol) const
    {
    if(m_dMax<m_dMin)
        {
        return false;
        }
    return fabs(Pos-m_dMax)<dTol || fabs(Pos-m_dMin)<dTol;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Interval2D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::Interval2D::Interval2D(SGM::Point2D const &Min,SGM::Point2D const &Max):
    m_UDomain(std::min(Min.m_u,Max.m_u),std::max(Min.m_u,Max.m_u)),
    m_VDomain(std::min(Min.m_v,Max.m_v),std::max(Min.m_v,Max.m_v))
    {
    }

SGM::Interval2D::Interval2D(SGM::Point2D const &Pos):
    m_UDomain(Pos.m_u),m_VDomain(Pos.m_v)
    {
    }

double SGM::Interval2D::Area() const 
    {
    return m_UDomain.Length()*m_VDomain.Length();
    }

double SGM::Interval2D::HalfPerimeter() const 
    {
    return m_UDomain.Length()+m_VDomain.Length();
    }

SGM::Interval2D const &SGM::Interval2D::operator+=(SGM::Interval2D const &domain)
    {
    if(m_UDomain.m_dMax<m_UDomain.m_dMin)   // This is empty so the answer is the given domain.
        {
        m_UDomain=domain.m_UDomain;
        m_VDomain=domain.m_VDomain;
        }
    else if(domain.m_UDomain.m_dMin<=domain.m_UDomain.m_dMax) // The given domain is not empty.
        {
        m_UDomain.m_dMin=std::min(m_UDomain.m_dMin,domain.m_UDomain.m_dMin);
        m_UDomain.m_dMax=std::max(m_UDomain.m_dMax,domain.m_UDomain.m_dMax);
        m_VDomain.m_dMin=std::min(m_VDomain.m_dMin,domain.m_VDomain.m_dMin);
        m_VDomain.m_dMax=std::max(m_VDomain.m_dMax,domain.m_VDomain.m_dMax);
        }
    return *this;
    }

SGM::Interval2D const &SGM::Interval2D::operator&=(SGM::Interval2D const &domain)
    {
    if(m_UDomain.m_dMax<m_UDomain.m_dMin)   // This is empty so the answer is empty.
        {
        m_UDomain.m_dMin=1;
        m_UDomain.m_dMax=0;
        }
    else if(domain.m_UDomain.m_dMin<=domain.m_UDomain.m_dMax) // The given domain is not empty.
        {
        m_UDomain.m_dMin=std::max(m_UDomain.m_dMin,domain.m_UDomain.m_dMin);
        m_UDomain.m_dMax=std::min(m_UDomain.m_dMax,domain.m_UDomain.m_dMax);
        m_VDomain.m_dMin=std::max(m_VDomain.m_dMin,domain.m_VDomain.m_dMin);
        m_VDomain.m_dMax=std::min(m_VDomain.m_dMax,domain.m_VDomain.m_dMax);
        }
    return *this;
    }

bool SGM::Interval2D::operator&&(SGM::Interval2D const &domain) const
    {
    if(m_UDomain.m_dMax<m_UDomain.m_dMin || domain.m_UDomain.m_dMax<domain.m_UDomain.m_dMin)   
        {
        return false; // One interval is empty so the answer is false.
        }
    return std::max(m_UDomain.m_dMin,domain.m_UDomain.m_dMin)<=std::min(m_UDomain.m_dMax,domain.m_UDomain.m_dMax) &&
           std::max(m_VDomain.m_dMin,domain.m_VDomain.m_dMin)<=std::min(m_VDomain.m_dMax,domain.m_VDomain.m_dMax);
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Interval3D methods
//
///////////////////////////////////////////////////////////////////////////////

SGM::Interval3D::Interval3D(SGM::Point3D const &Min,SGM::Point3D const &Max):
    m_XDomain(std::min(Min.m_x,Max.m_x),std::max(Min.m_x,Max.m_x)),
    m_YDomain(std::min(Min.m_y,Max.m_y),std::max(Min.m_y,Max.m_y)),
    m_ZDomain(std::min(Min.m_z,Max.m_z),std::max(Min.m_z,Max.m_z))
    {
    }

SGM::Interval3D::Interval3D(SGM::Point3D const &Pos):
    m_XDomain(Pos.m_x),m_YDomain(Pos.m_y),m_ZDomain(Pos.m_z)
    {
    }

double SGM::Interval3D::Volume() const 
    {
    return m_XDomain.Length()*m_YDomain.Length()*m_ZDomain.Length();
    }

double SGM::Interval3D::HalfArea() const
    {
    double dX=m_XDomain.Length();
    double dY=m_YDomain.Length();
    double dZ=m_ZDomain.Length();
    return dX*dY+dX*dZ+dY*dZ;
    }

SGM::Interval3D const &SGM::Interval3D::operator+=(SGM::Interval3D const &domain)
    {
    if(m_XDomain.m_dMax<m_XDomain.m_dMin)   // This is empty so the answer is the given domain.
        {
        m_XDomain=domain.m_XDomain;
        m_YDomain=domain.m_YDomain;
        m_ZDomain=domain.m_ZDomain;
        }
    else if(domain.m_XDomain.m_dMin<=domain.m_XDomain.m_dMax) // The given domain is not empty.
        {
        m_XDomain+=domain.m_XDomain;
        m_YDomain+=domain.m_YDomain;
        m_ZDomain+=domain.m_ZDomain;
        }
    return *this;
    }

SGM::Interval3D const &SGM::Interval3D::operator&=(SGM::Interval3D const &domain)
    {
    if(m_XDomain.m_dMax<m_XDomain.m_dMin)   // This is empty so the answer is empty.
        {
        m_XDomain.m_dMin=1;
        m_XDomain.m_dMax=0;
        }
    else if(domain.m_XDomain.m_dMin<=domain.m_XDomain.m_dMax) // The given domain is not empty.
        {
        m_XDomain&=domain.m_XDomain;
        m_YDomain&=domain.m_YDomain;
        m_ZDomain&=domain.m_ZDomain;
        }
    return *this;
    }

bool SGM::Interval3D::operator&&(SGM::Interval3D const &domain) const
    {
    if(m_XDomain.m_dMax<m_XDomain.m_dMin || domain.m_XDomain.m_dMax<domain.m_XDomain.m_dMin)   
        {
        return false; // One interval is empty so the answer is false.
        }
    return std::max(m_XDomain.m_dMin,domain.m_XDomain.m_dMin)<=std::min(m_XDomain.m_dMax,domain.m_XDomain.m_dMax) &&
           std::max(m_YDomain.m_dMin,domain.m_YDomain.m_dMin)<=std::min(m_YDomain.m_dMax,domain.m_YDomain.m_dMax) &&
           std::max(m_ZDomain.m_dMin,domain.m_ZDomain.m_dMin)<=std::min(m_ZDomain.m_dMax,domain.m_ZDomain.m_dMax);
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Result class methods
//
///////////////////////////////////////////////////////////////////////////////

    
void SGM::Result::SetResult(SGM::ResultType nType)
    {
    m_nType=nType;
    }

entity *SGM::Result::FindEntity(size_t nID) const
    {
    return m_pThing->FindEntity(nID);
    }

void SGM::Result::SetMessage(std::string const &sMessage)
    {
    m_sMessage+=sMessage;
    }

void SGM::Result::Clear()
    {
    m_sMessage.clear();
    m_nType=SGM::ResultType::ResultTypeOK;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Segment class operators
//
///////////////////////////////////////////////////////////////////////////////


SGM::Segment2D::Segment2D() 
    {
    }

SGM::Segment2D::Segment2D(Point2D const &Start,
                          Point2D const &End):m_Start(Start),m_End(End)
    {
    }

bool SGM::Segment2D::Intersect(Segment2D const &other,
                               SGM::Point2D    &Pos) const
    {
    double a1=m_End.m_u-other.m_Start.m_u;
    double b1=m_Start.m_u-other.m_End.m_u;
    double c1=other.m_Start.m_u-m_End.m_u;
    double a2=m_End.m_v-other.m_Start.m_v;
    double b2=m_Start.m_v-other.m_End.m_v;
    double c2=other.m_Start.m_v-m_End.m_v;
    double x,y;
    bool bAnswer=SGM::CramersRule(a1,b1,c1,a2,b2,c2,x,y);
    double dTol=1E-12;
    if(bAnswer && -dTol<x && x<1+dTol && -dTol<y && y<1+dTol)
        {
        Pos.m_u=m_Start.m_u+x*(m_End.m_u-m_Start.m_u);
        Pos.m_v=m_Start.m_v+x*(m_End.m_v-m_Start.m_v);
        }
    else
        {
        bAnswer=false;
        }
    return bAnswer;
    }

SGM::Segment3D::Segment3D() 
    {
    }

SGM::Segment3D::Segment3D(Point3D const &Start,
                          Point3D const &End):m_Start(Start),m_End(End)
    {
    }

bool SGM::Segment3D::Intersect(SGM::Segment3D const &Seg,
                               SGM::Point3D         &Pos1,
                               SGM::Point3D         &Pos2) const
    {
    SGM::Vector3D v=Seg.m_End-Seg.m_Start;
    SGM::Vector3D u=m_End-m_Start;
    SGM::Vector3D w=m_Start-Seg.m_Start;
    double a=u%u;
    double b=u%v;
    double c=v%v;
    double d=u%w;
    double e=v%w;
    double denom=a*c-b*b;
    bool bAnswer=true;
    if(1E-12<denom)
        {
        double s=(b*e-c*d)/denom;
        double t=(a*e-b*d)/denom;
        Pos1=m_Start+u*s;
        Pos2=Seg.m_Start+v*t;
        if(s<0 || 1<s || t<0 || 1<t)
            {
            bAnswer=false;
            }
        }
    else
        {
        double t=e/c;
        Pos1=m_Start;
        Pos2=Seg.m_Start+v*t;
        if(t<0 || 1<t)
            {
            bAnswer=false;
            }
        }
    return bAnswer;
    }

///////////////////////////////////////////////////////////////////////////
//
//  Transform classes
//
///////////////////////////////////////////////////////////////////////////

double SGM::Transform3D::Scale(SGM::UnitVector3D const &Direction) const
    {
    return ((*this)*Vector3D(Direction)).Magnitude();
    }

double SGM::Transform3D::Scale() const
    {
    double dX=Scale(SGM::UnitVector3D(1.0,0.0,0.0));
    double dY=Scale(SGM::UnitVector3D(0.0,1.0,0.0));
    double dZ=Scale(SGM::UnitVector3D(0.0,0.0,1.0));
    if(SGM::NearEqual(dX,dY,SGM_ZERO,false) && SGM::NearEqual(dZ,dY,SGM_ZERO,false))
        {
        return dX;
        }
    return 0.0;
    }

void SGM::Transform3D::Inverse(SGM::Transform3D &Trans) const
    {
    double s0 = m_Matrix[0].m_x * m_Matrix[1].m_y - m_Matrix[1].m_x * m_Matrix[0].m_y;
    double s1 = m_Matrix[0].m_x * m_Matrix[1].m_z - m_Matrix[1].m_x * m_Matrix[0].m_z;
    double s2 = m_Matrix[0].m_x * m_Matrix[1].m_w - m_Matrix[1].m_x * m_Matrix[0].m_w;
    double s3 = m_Matrix[0].m_y * m_Matrix[1].m_z - m_Matrix[1].m_y * m_Matrix[0].m_z;
    double s4 = m_Matrix[0].m_y * m_Matrix[1].m_w - m_Matrix[1].m_y * m_Matrix[0].m_w;
    double s5 = m_Matrix[0].m_z * m_Matrix[1].m_w - m_Matrix[1].m_z * m_Matrix[0].m_w;

    double c5 = m_Matrix[2].m_z * m_Matrix[3].m_w - m_Matrix[3].m_z * m_Matrix[2].m_w;
    double c4 = m_Matrix[2].m_y * m_Matrix[3].m_w - m_Matrix[3].m_y * m_Matrix[2].m_w;
    double c3 = m_Matrix[2].m_y * m_Matrix[3].m_z - m_Matrix[3].m_y * m_Matrix[2].m_z;
    double c2 = m_Matrix[2].m_x * m_Matrix[3].m_w - m_Matrix[3].m_x * m_Matrix[2].m_w;
    double c1 = m_Matrix[2].m_x * m_Matrix[3].m_z - m_Matrix[3].m_x * m_Matrix[2].m_z;
    double c0 = m_Matrix[2].m_x * m_Matrix[3].m_y - m_Matrix[3].m_x * m_Matrix[2].m_y;

    // Note that a transformation matrix may never have a zero determinate.

    double invdet = 1.0 / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);

    Trans.m_Matrix[0].m_x = ( m_Matrix[1].m_y * c5 - m_Matrix[1].m_z * c4 + m_Matrix[1].m_w * c3) * invdet;
    Trans.m_Matrix[0].m_y = (-m_Matrix[0].m_y * c5 + m_Matrix[0].m_z * c4 - m_Matrix[0].m_w * c3) * invdet;
    Trans.m_Matrix[0].m_z = ( m_Matrix[3].m_y * s5 - m_Matrix[3].m_z * s4 + m_Matrix[3].m_w * s3) * invdet;
    Trans.m_Matrix[0].m_w = (-m_Matrix[2].m_y * s5 + m_Matrix[2].m_z * s4 - m_Matrix[2].m_w * s3) * invdet;
    
    Trans.m_Matrix[1].m_x = (-m_Matrix[1].m_x * c5 + m_Matrix[1].m_z * c2 - m_Matrix[1].m_w * c1) * invdet;
    Trans.m_Matrix[1].m_y = ( m_Matrix[0].m_x * c5 - m_Matrix[0].m_z * c2 + m_Matrix[0].m_w * c1) * invdet;
    Trans.m_Matrix[1].m_z = (-m_Matrix[3].m_x * s5 + m_Matrix[3].m_z * s2 - m_Matrix[3].m_w * s1) * invdet;
    Trans.m_Matrix[1].m_w = ( m_Matrix[2].m_x * s5 - m_Matrix[2].m_z * s2 + m_Matrix[2].m_w * s1) * invdet;
    
    Trans.m_Matrix[2].m_x = ( m_Matrix[1].m_x * c4 - m_Matrix[1].m_y * c2 + m_Matrix[1].m_w * c0) * invdet;
    Trans.m_Matrix[2].m_y = (-m_Matrix[0].m_x * c4 + m_Matrix[0].m_y * c2 - m_Matrix[0].m_w * c0) * invdet;
    Trans.m_Matrix[2].m_z = ( m_Matrix[3].m_x * s4 - m_Matrix[3].m_y * s2 + m_Matrix[3].m_w * s0) * invdet;
    Trans.m_Matrix[2].m_w = (-m_Matrix[2].m_x * s4 + m_Matrix[2].m_y * s2 - m_Matrix[2].m_w * s0) * invdet;
    
    Trans.m_Matrix[3].m_x = (-m_Matrix[1].m_x * c3 + m_Matrix[1].m_y * c1 - m_Matrix[1].m_z * c0) * invdet;
    Trans.m_Matrix[3].m_y = ( m_Matrix[0].m_x * c3 - m_Matrix[0].m_y * c1 + m_Matrix[0].m_z * c0) * invdet;
    Trans.m_Matrix[3].m_z = (-m_Matrix[3].m_x * s3 + m_Matrix[3].m_y * s1 - m_Matrix[3].m_z * s0) * invdet;
    Trans.m_Matrix[3].m_w = ( m_Matrix[2].m_x * s3 - m_Matrix[2].m_y * s1 + m_Matrix[2].m_z * s0) * invdet;
    }

SGM::Transform3D SGM::operator*(SGM::Transform3D const &Trans0,SGM::Transform3D const &Trans1)
    {
    SGM::Vector4D const *Matrix0=Trans0.GetData();
    SGM::Vector4D const &XAxis0=Matrix0[0];
    SGM::Vector4D const &YAxis0=Matrix0[1];
    SGM::Vector4D const &ZAxis0=Matrix0[2];
    SGM::Vector4D const &WAxis0=Matrix0[3];

    SGM::Vector4D const *Matrix1=Trans1.GetData();
    SGM::Vector4D const &XAxis1=Matrix1[0];
    SGM::Vector4D const &YAxis1=Matrix1[1];
    SGM::Vector4D const &ZAxis1=Matrix1[2];
    SGM::Vector4D const &WAxis1=Matrix1[3];

    SGM::Vector4D X,Y,Z,W;

    X.m_x = XAxis0.m_x*XAxis1.m_x + YAxis0.m_x*XAxis1.m_y + ZAxis0.m_x*XAxis1.m_z + WAxis0.m_x*XAxis1.m_w;
    X.m_y = XAxis0.m_y*XAxis1.m_x + YAxis0.m_y*XAxis1.m_y + ZAxis0.m_y*XAxis1.m_z + WAxis0.m_y*XAxis1.m_w;
    X.m_z = XAxis0.m_z*XAxis1.m_x + YAxis0.m_z*XAxis1.m_y + ZAxis0.m_z*XAxis1.m_z + WAxis0.m_z*XAxis1.m_w;
    X.m_w = XAxis0.m_w*XAxis1.m_x + YAxis0.m_w*XAxis1.m_y + ZAxis0.m_w*XAxis1.m_z + WAxis0.m_w*XAxis1.m_w;
                                                                                    
    Y.m_x = XAxis0.m_x*YAxis1.m_x + YAxis0.m_x*YAxis1.m_y + ZAxis0.m_x*YAxis1.m_z + WAxis0.m_x*YAxis1.m_w;
    Y.m_y = XAxis0.m_y*YAxis1.m_x + YAxis0.m_y*YAxis1.m_y + ZAxis0.m_y*YAxis1.m_z + WAxis0.m_y*YAxis1.m_w;
    Y.m_z = XAxis0.m_z*YAxis1.m_x + YAxis0.m_z*YAxis1.m_y + ZAxis0.m_z*YAxis1.m_z + WAxis0.m_z*YAxis1.m_w;
    Y.m_w = XAxis0.m_w*YAxis1.m_x + YAxis0.m_w*YAxis1.m_y + ZAxis0.m_w*YAxis1.m_z + WAxis0.m_w*YAxis1.m_w;
                                                                                    
    Z.m_x = XAxis0.m_x*ZAxis1.m_x + YAxis0.m_x*ZAxis1.m_y + ZAxis0.m_x*ZAxis1.m_z + WAxis0.m_x*ZAxis1.m_w;
    Z.m_y = XAxis0.m_y*ZAxis1.m_x + YAxis0.m_y*ZAxis1.m_y + ZAxis0.m_y*ZAxis1.m_z + WAxis0.m_y*ZAxis1.m_w;
    Z.m_z = XAxis0.m_z*ZAxis1.m_x + YAxis0.m_z*ZAxis1.m_y + ZAxis0.m_z*ZAxis1.m_z + WAxis0.m_z*ZAxis1.m_w;
    Z.m_w = XAxis0.m_w*ZAxis1.m_x + YAxis0.m_w*ZAxis1.m_y + ZAxis0.m_w*ZAxis1.m_z + WAxis0.m_w*ZAxis1.m_w;
                                                                                    
    W.m_x = XAxis0.m_x*WAxis1.m_x + YAxis0.m_x*WAxis1.m_y + ZAxis0.m_x*WAxis1.m_z + WAxis0.m_x*WAxis1.m_w;
    W.m_y = XAxis0.m_y*WAxis1.m_x + YAxis0.m_y*WAxis1.m_y + ZAxis0.m_y*WAxis1.m_z + WAxis0.m_y*WAxis1.m_w;
    W.m_z = XAxis0.m_z*WAxis1.m_x + YAxis0.m_z*WAxis1.m_y + ZAxis0.m_z*WAxis1.m_z + WAxis0.m_z*WAxis1.m_w;
    W.m_w = XAxis0.m_w*WAxis1.m_x + YAxis0.m_w*WAxis1.m_y + ZAxis0.m_w*WAxis1.m_z + WAxis0.m_w*WAxis1.m_w;

    return SGM::Transform3D(X,Y,Z,W);
    }

SGM::Point3D SGM::operator*(SGM::Transform3D const &Trans,SGM::Point3D const &Pos)
    {
    SGM::Vector4D const *Matrix=Trans.GetData();
    SGM::Vector4D const &XAxis=Matrix[0];
    SGM::Vector4D const &YAxis=Matrix[1];
    SGM::Vector4D const &ZAxis=Matrix[2];
    double x=Pos.m_x*XAxis.m_x+Pos.m_y*XAxis.m_y+Pos.m_z*XAxis.m_z+XAxis.m_w;
    double y=Pos.m_x*YAxis.m_x+Pos.m_y*YAxis.m_y+Pos.m_z*YAxis.m_z+YAxis.m_w;
    double z=Pos.m_x*ZAxis.m_x+Pos.m_y*ZAxis.m_y+Pos.m_z*ZAxis.m_z+ZAxis.m_w;
    return SGM::Point3D(x,y,z);
    }

SGM::Vector3D SGM::operator*(SGM::Transform3D const &Trans,SGM::Vector3D const &Vec)
    {
    SGM::Vector4D const *Matrix=Trans.GetData();
    SGM::Vector4D const &XAxis=Matrix[0];
    SGM::Vector4D const &YAxis=Matrix[1];
    SGM::Vector4D const &ZAxis=Matrix[2];
    double x=Vec.m_x*XAxis.m_x+Vec.m_y*XAxis.m_y+Vec.m_z*XAxis.m_z;
    double y=Vec.m_x*YAxis.m_x+Vec.m_y*YAxis.m_y+Vec.m_z*YAxis.m_z;
    double z=Vec.m_x*ZAxis.m_x+Vec.m_y*ZAxis.m_y+Vec.m_z*ZAxis.m_z;
    return SGM::Vector3D(x,y,z);   
    }

SGM::UnitVector3D SGM::operator*(SGM::Transform3D const &Trans,SGM::UnitVector3D const &UVec)
    {
    SGM::Vector4D const *Matrix=Trans.GetData();
    SGM::Vector4D const &XAxis=Matrix[0];
    SGM::Vector4D const &YAxis=Matrix[1];
    SGM::Vector4D const &ZAxis=Matrix[2];
    double x=UVec.m_x*XAxis.m_x+UVec.m_y*XAxis.m_y+UVec.m_z*XAxis.m_z;
    double y=UVec.m_x*YAxis.m_x+UVec.m_y*YAxis.m_y+UVec.m_z*YAxis.m_z;
    double z=UVec.m_x*ZAxis.m_x+UVec.m_y*ZAxis.m_y+UVec.m_z*ZAxis.m_z;
    return SGM::UnitVector3D(x,y,z); 
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Multi class operators
//
///////////////////////////////////////////////////////////////////////////////

SGM::Vector2D SGM::operator-(SGM::Point2D const &Pos0,SGM::Point2D const &Pos1)
    {
    return SGM::Vector2D(Pos0.m_u-Pos1.m_u,Pos0.m_v-Pos1.m_v);
    }

SGM::Point2D SGM::operator+(SGM::Point2D const &Pos,SGM::Vector2D const &Vec)
    {
    return SGM::Point2D(Pos.m_u+Vec.m_u,Pos.m_v+Vec.m_v);
    }

SGM::Point2D SGM::operator-(SGM::Point2D const &Pos,SGM::Vector2D const &Vec)
    {
    return SGM::Point2D(Pos.m_u-Vec.m_u,Pos.m_v-Vec.m_v);
    }

SGM::Vector3D SGM::operator-(SGM::Point3D const &Pos0,SGM::Point3D const &Pos1)
    {
    return SGM::Vector3D(Pos0.m_x-Pos1.m_x,Pos0.m_y-Pos1.m_y,Pos0.m_z-Pos1.m_z);
    }

SGM::Vector3D SGM::operator-(SGM::Vector3D const &Vec)
    {
    return SGM::Vector3D(-Vec.m_x,-Vec.m_y,-Vec.m_z);
    }

SGM::UnitVector3D SGM::operator-(SGM::UnitVector3D const &UVec)
    {
    return SGM::Vector3D(-UVec.m_x,-UVec.m_y,-UVec.m_z);
    }

SGM::Vector3D SGM::operator*(double dValue,SGM::Vector3D const &Vec)
    {
    return SGM::Vector3D(Vec.m_x*dValue,Vec.m_y*dValue,Vec.m_z*dValue);
    }

SGM::Vector4D SGM::operator*(double dValue,SGM::Vector4D const &Vec)
    {
    return SGM::Vector4D(Vec.m_x*dValue,Vec.m_y*dValue,Vec.m_z*dValue,Vec.m_w*dValue);
    }

SGM::Vector3D SGM::operator*(SGM::Vector3D const &Vec0,SGM::Vector3D const &Vec1)
    {
    return SGM::Vector3D(Vec0.m_y*Vec1.m_z-Vec0.m_z*Vec1.m_y,
                         Vec0.m_z*Vec1.m_x-Vec0.m_x*Vec1.m_z,
                         Vec0.m_x*Vec1.m_y-Vec0.m_y*Vec1.m_x);
    }

SGM::Vector3D SGM::operator+(SGM::Vector3D const &Vec0,SGM::Vector3D const &Vec1)
    {
    return SGM::Vector3D(Vec0.m_x+Vec1.m_x,Vec0.m_y+Vec1.m_y,Vec0.m_z+Vec1.m_z);
    }

SGM::Point3D SGM::operator+(SGM::Point3D const &Pos,SGM::Vector3D const &Vec)
    {
    return SGM::Point3D(Pos.m_x+Vec.m_x,Pos.m_y+Vec.m_y,Pos.m_z+Vec.m_z);
    }

SGM::Point4D SGM::operator+(SGM::Point4D const &Pos,SGM::Vector4D const &Vec)
    {
    return SGM::Point4D(Pos.m_x+Vec.m_x,Pos.m_y+Vec.m_y,Pos.m_z+Vec.m_z,Pos.m_w+Vec.m_w);
    }

SGM::Point3D SGM::operator-(SGM::Point3D const &Pos,SGM::Vector3D const &Vec)
    {
    return SGM::Point3D(Pos.m_x-Vec.m_x,Pos.m_y-Vec.m_y,Pos.m_z-Vec.m_z);
    }

SGM::Point4D SGM::operator-(SGM::Point4D const &Pos,SGM::Vector4D const &Vec)
    {
    return SGM::Point4D(Pos.m_x-Vec.m_x,Pos.m_y-Vec.m_y,Pos.m_z-Vec.m_z,Pos.m_w-Vec.m_w);
    }

SGM::Vector3D SGM::operator-(SGM::Vector3D const &Vec0,SGM::Vector3D const &Vec1)
    {
    return SGM::Vector3D(Vec0.m_x-Vec1.m_x,Vec0.m_y-Vec1.m_y,Vec0.m_z-+Vec1.m_z);
    }

double SGM::operator%(SGM::Vector2D const &Vec0,SGM::Vector2D const &Vec1)
    {
    return Vec0.m_u*Vec1.m_u+Vec0.m_v*Vec1.m_v;
    }

double SGM::operator%(SGM::Vector3D const &Vec0,SGM::Vector3D const &Vec1)
    {
    return Vec0.m_x*Vec1.m_x+Vec0.m_y*Vec1.m_y+Vec0.m_z*Vec1.m_z;
    }

SGM::Point3D SGM::MidPoint(SGM::Point3D const &Pos0,SGM::Point3D const &Pos1,double dFraction)
    {
    return SGM::Point3D((Pos0.m_x+Pos1.m_x)*dFraction,
                        (Pos0.m_y+Pos1.m_y)*dFraction,
                        (Pos0.m_z+Pos1.m_z)*dFraction);
    }
    
SGM::Point2D SGM::MidPoint(SGM::Point2D const &Pos0,SGM::Point2D const &Pos1,double dFraction)
    {
    return SGM::Point2D((Pos0.m_u+Pos1.m_u)*dFraction,
                        (Pos0.m_v+Pos1.m_v)*dFraction);
    }

bool SGM::NearEqual(double d1,double d2,double dTolerance,bool bPercent)
    {
    if(bPercent)
        {
        double dAverage=fabs(d1+d2)*0.5;
        if(dAverage<SGM_ZERO)
            {
            dAverage=std::max(fabs(d1),fabs(d2));
            if(dAverage<SGM_ZERO)
                {
                return true;
                }
            }
        return fabs(d1-d2)/dAverage<dTolerance;
        }
    else
        {
        return fabs(d1-d2)<dTolerance;
        }
    }

bool SGM::NearEqual(SGM::Point3D const &Pos1,SGM::Point3D const &Pos2,double dTolerance)
    {
    return Pos1.DistanceSquared(Pos2)<dTolerance*dTolerance;
    }

bool SGM::NearEqual(SGM::Vector3D const &Vec1,SGM::Vector3D const &Vec2,double dTolerance)
    {
    return (Vec1-Vec2).MagnitudeSquared()<dTolerance*dTolerance;
    }

bool SGM::NearEqual(SGM::Point2D const &Pos1,SGM::Point2D const &Pos2,double dTolerance)
    {
    return Pos1.DistanceSquared(Pos2)<dTolerance*dTolerance;
    }
