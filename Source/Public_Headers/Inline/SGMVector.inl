#ifndef SGM_VECTOR_INL
#define SGM_VECTOR_INL

#include <algorithm>
#include <cmath>

#include "SGMConstants.h"
#include "SGMMathematics.h"

namespace SGM {

///////////////////////////////////////////////////////////////////////////////
//
//  Point2D methods
//
///////////////////////////////////////////////////////////////////////////////

    inline double Point2D::Distance(Point2D const &Pos) const
    {
        double dU = Pos.m_u - m_u;
        double dV = Pos.m_v - m_v;
        return sqrt(dU * dU + dV * dV);
    }

    inline double Point2D::DistanceSquared(Point2D const &Pos) const
    {
        double dU = Pos.m_u - m_u;
        double dV = Pos.m_v - m_v;
        return dU * dU + dV * dV;
    }

    inline Point2D Point2D::operator+=(Vector2D const &Vec)
    {
        m_u += Vec.m_u;
        m_v += Vec.m_v;
        return *this;
    }

    inline bool Point2D::operator<(Point2D const &Pos) const
    {
        if (m_u < Pos.m_u)
            {
            return true;
            }
        else if (Pos.m_u < m_u)
            {
            return false;
            }
        else
            {
            if (m_v < Pos.m_v)
                {
                return true;
                }
            }
        return false;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Vector2D methods
//
///////////////////////////////////////////////////////////////////////////////

    inline Vector2D Vector2D::operator*(double dScale) const
    {
        return Vector2D(m_u*dScale,m_v*dScale);
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Vector3D methods
//
///////////////////////////////////////////////////////////////////////////////

    inline Vector3D::Vector3D(Point3D const &Pos):m_x(Pos.m_x),m_y(Pos.m_y),m_z(Pos.m_z)
    {
    }

    inline Vector3D Vector3D::Orthogonal() const
    {
        double dX=fabs(m_x);
        double dY=fabs(m_y);
        double dZ=fabs(m_z);
        if(dY<=dX && dZ<=dX)
            {
            return Vector3D(-m_y,m_x,0.0);
            }
        else if(dX<=dY && dZ<=dY)
            {
            return Vector3D(-m_y,m_x,0.0);
            }
        else
            {
            return Vector3D(m_z,0.0,-m_x);
            }
    }

    inline Vector3D Vector3D::operator*(double dScale) const
    {
        return Vector3D(m_x*dScale,m_y*dScale,m_z*dScale);
    }

    inline Vector3D Vector3D::operator/(double dScale) const
    {
        return Vector3D(m_x/dScale,m_y/dScale,m_z/dScale);
    }


    inline double Vector3D::Magnitude() const
    {
        return sqrt(m_x*m_x+m_y*m_y+m_z*m_z);
    }

    inline double Vector3D::MagnitudeSquared() const
    {
        return m_x*m_x+m_y*m_y+m_z*m_z;
    }

    inline void Vector3D::Negate()
    {
        m_x=-m_x;
        m_y=-m_y;
        m_z=-m_z;
    }

///////////////////////////////////////////////////////////////////////////////
//
//  Vector4D methods
//
///////////////////////////////////////////////////////////////////////////////

    inline Vector4D::Vector4D(Point4D const &Pos):m_x(Pos.m_x),m_y(Pos.m_y),
                                                  m_z(Pos.m_z),m_w(Pos.m_w)
    {
    }

    inline Vector4D Vector4D::operator*(double dScale) const
    {
        return Vector4D(m_x*dScale,m_y*dScale,m_z*dScale,m_w*dScale);
    }

    inline Vector4D Vector4D::operator/(double dScale) const
    {
        return Vector4D(m_x/dScale,m_y/dScale,m_z/dScale,m_w/dScale);
    }

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Point4 methods
    //
    ///////////////////////////////////////////////////////////////////////////

    inline double Point4D::Distance(Point4D const &Pos) const
    {
        double dX=Pos.m_x-m_x;
        double dY=Pos.m_y-m_y;
        double dZ=Pos.m_z-m_z;
        double dW=Pos.m_z-m_z;
        return sqrt(dX*dX+dY*dY+dZ*dZ+dW*dW);
    }

    inline double Point4D::DistanceSquared(Point4D const &Pos) const
    {
        double dX=Pos.m_x-m_x;
        double dY=Pos.m_y-m_y;
        double dZ=Pos.m_z-m_z;
        double dW=Pos.m_z-m_z;
        return dX*dX+dY*dY+dZ*dZ+dW*dW;
    }

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Point3D methods
    //
    ///////////////////////////////////////////////////////////////////////////

    inline Point3D::Point3D(Vector3D const &Vec):m_x(Vec.m_x),m_y(Vec.m_y),m_z(Vec.m_z)
    {
    }

    inline double Point3D::Distance(Point3D const &Pos) const
    {
        double dX=Pos.m_x-m_x;
        double dY=Pos.m_y-m_y;
        double dZ=Pos.m_z-m_z;
        return sqrt(dX*dX+dY*dY+dZ*dZ);
    }

    inline double Point3D::DistanceSquared(Point3D const &Pos) const
    {
        double dX=Pos.m_x-m_x;
        double dY=Pos.m_y-m_y;
        double dZ=Pos.m_z-m_z;
        return dX*dX+dY*dY+dZ*dZ;
    }

    inline Point3D Point3D::operator+=(Vector3D const &Vec)
    {
        m_x+=Vec.m_x;
        m_y+=Vec.m_y;
        m_z+=Vec.m_z;
        return *this;
    }


    inline bool Point3D::operator<(Point3D const &Pos) const
    {
        if(m_x<Pos.m_x)
            {
            return true;
            }
        if(Pos.m_x<m_x)
            {
            return false;
            }
        if(m_y<Pos.m_y)
            {
            return true;
            }
        if(Pos.m_y<m_y)
            {
            return false;
            }
        if(m_z<Pos.m_z)
            {
            return true;
            }
        if(Pos.m_z<m_z)
            {
            return false;
            }
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Point4D methods
    //
    ///////////////////////////////////////////////////////////////////////////

    inline Point4D Point4D::operator+=(Vector4D const &Vec)
    {
        m_x+=Vec.m_x;
        m_y+=Vec.m_y;
        m_z+=Vec.m_z;
        m_w+=Vec.m_w;
        return *this;
    }

    inline bool Point4D::operator<(Point4D const &Pos) const
    {
        if(m_x<Pos.m_x)
            {
            return true;
            }
        if(Pos.m_x<m_x)
            {
            return false;
            }
        if(m_y<Pos.m_y)
            {
            return true;
            }
        if(Pos.m_y<m_y)
            {
            return false;
            }
        if(m_z<Pos.m_z)
            {
            return true;
            }
        if(Pos.m_z<m_z)
            {
            return false;
            }
        if(m_z<Pos.m_w)
            {
            return true;
            }
        if(Pos.m_z<m_w)
            {
            return false;
            }
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////
    //
    //  UnitVector2D methods
    //
    ///////////////////////////////////////////////////////////////////////////

    inline UnitVector2D::UnitVector2D(Vector2D const &Vec)
    {
        double dMagSquared = Vec.m_u * Vec.m_u + Vec.m_v * Vec.m_v;
        if (1-SGM_ZERO < dMagSquared &&
            dMagSquared < 1+SGM_ZERO)
            {
            m_u = Vec.m_u;
            m_v = Vec.m_v;
            }
        else if (SGM_ZERO_SQUARED < dMagSquared)
            {
            double dMag = sqrt(dMagSquared);
            double dScale = 1.0 / dMag;
            m_u = Vec.m_u * dScale;
            m_v = Vec.m_v * dScale;
            }
        else
            {
            m_u = 0.0;
            m_v = 1.0;
            }
    }

    inline UnitVector2D::UnitVector2D(double u,double v)
    {
        double dMagSquared=u*u+v*v;
        if( 1-SGM_ZERO<dMagSquared &&
            dMagSquared<1+SGM_ZERO)
            {
            m_u=u;
            m_v=v;
            }
        else if(SGM_ZERO_SQUARED<dMagSquared)
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

    ///////////////////////////////////////////////////////////////////////////
    //
    //  UnitVector3D methods
    //
    ///////////////////////////////////////////////////////////////////////////

    inline UnitVector3D::UnitVector3D(Vector3D const &Vec)
    {
        double dMagSquared=Vec.m_x*Vec.m_x+Vec.m_y*Vec.m_y+Vec.m_z*Vec.m_z;
        if( 1-SGM_ZERO<dMagSquared &&
            dMagSquared<1+SGM_ZERO)
            {
            m_x=Vec.m_x;
            m_y=Vec.m_y;
            m_z=Vec.m_z;
            }
        else if(SGM_ZERO_SQUARED<dMagSquared)
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

    inline UnitVector3D::UnitVector3D(double x,double y,double z)
    {
        double dMagSquared=x*x+y*y+z*z;
        if( 1-SGM_ZERO<dMagSquared &&
            dMagSquared<1+SGM_ZERO)
            {
            m_x=x;
            m_y=y;
            m_z=z;
            }
        else if(SGM_ZERO_SQUARED<dMagSquared)
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

    ///////////////////////////////////////////////////////////////////////////
    //
    //  UnitVector4D methods
    //
    ///////////////////////////////////////////////////////////////////////////

    inline UnitVector4D::UnitVector4D(Vector4D const &Vec)
    {
        double dMagSquared=Vec.m_x*Vec.m_x+Vec.m_y*Vec.m_y+Vec.m_z*Vec.m_z+Vec.m_w*Vec.m_w;
        if( 1-SGM_ZERO<dMagSquared &&
            dMagSquared<1+SGM_ZERO)
            {
            m_x=Vec.m_x;
            m_y=Vec.m_y;
            m_z=Vec.m_z;
            m_w=Vec.m_w;
            }
        else if(SGM_ZERO_SQUARED<dMagSquared)
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
//  Multi class operators
//
///////////////////////////////////////////////////////////////////////////////

    inline Vector2D operator-(Point2D const &Pos0,Point2D const &Pos1)
    {
        return Vector2D(Pos0.m_u-Pos1.m_u,Pos0.m_v-Pos1.m_v);
    }

    inline Point2D operator+(Point2D const &Pos,Vector2D const &Vec)
    {
        return Point2D(Pos.m_u+Vec.m_u,Pos.m_v+Vec.m_v);
    }

    inline Point2D operator-(Point2D const &Pos,Vector2D const &Vec)
    {
        return Point2D(Pos.m_u-Vec.m_u,Pos.m_v-Vec.m_v);
    }

    inline Vector3D operator-(Point3D const &Pos0,Point3D const &Pos1)
    {
        return Vector3D(Pos0.m_x-Pos1.m_x,Pos0.m_y-Pos1.m_y,Pos0.m_z-Pos1.m_z);
    }

    inline Vector3D operator-(Vector3D const &Vec)
    {
        return Vector3D(-Vec.m_x,-Vec.m_y,-Vec.m_z);
    }

    inline UnitVector3D operator-(UnitVector3D const &UVec)
    {
        return Vector3D(-UVec.m_x,-UVec.m_y,-UVec.m_z);
    }

    inline Vector2D operator*(double dValue,Vector2D const &Vec)
    {
        return Vector2D(Vec.m_u*dValue,Vec.m_v*dValue);
    }

    inline Vector3D operator*(double dValue,Vector3D const &Vec)
    {
        return Vector3D(Vec.m_x*dValue,Vec.m_y*dValue,Vec.m_z*dValue);
    }

    inline Vector4D operator*(double dValue,Vector4D const &Vec)
    {
        return Vector4D(Vec.m_x*dValue,Vec.m_y*dValue,Vec.m_z*dValue,Vec.m_w*dValue);
    }

    inline Vector3D operator*(Vector3D const &Vec0,Vector3D const &Vec1)
    {
        return Vector3D(Vec0.m_y*Vec1.m_z-Vec0.m_z*Vec1.m_y,
                        Vec0.m_z*Vec1.m_x-Vec0.m_x*Vec1.m_z,
                        Vec0.m_x*Vec1.m_y-Vec0.m_y*Vec1.m_x);
    }

    inline Vector3D operator+(Vector3D const &Vec0,Vector3D const &Vec1)
    {
        return Vector3D(Vec0.m_x+Vec1.m_x,Vec0.m_y+Vec1.m_y,Vec0.m_z+Vec1.m_z);
    }

    inline Point3D operator+(Point3D const &Pos,Vector3D const &Vec)
    {
        return Point3D(Pos.m_x+Vec.m_x,Pos.m_y+Vec.m_y,Pos.m_z+Vec.m_z);
    }

    inline Point4D operator+(Point4D const &Pos,Vector4D const &Vec)
    {
        return Point4D(Pos.m_x+Vec.m_x,Pos.m_y+Vec.m_y,Pos.m_z+Vec.m_z,Pos.m_w+Vec.m_w);
    }

    inline Point3D operator-(Point3D const &Pos,Vector3D const &Vec)
    {
        return Point3D(Pos.m_x-Vec.m_x,Pos.m_y-Vec.m_y,Pos.m_z-Vec.m_z);
    }

    inline Point4D operator-(Point4D const &Pos,Vector4D const &Vec)
    {
        return Point4D(Pos.m_x-Vec.m_x,Pos.m_y-Vec.m_y,Pos.m_z-Vec.m_z,Pos.m_w-Vec.m_w);
    }

    inline Vector2D operator+(Vector2D const &Vec0,Vector2D const &Vec1)
    {
        return Vector2D(Vec0.m_u+Vec1.m_u,Vec0.m_v+Vec1.m_v);
    }

    inline Vector3D operator-(Vector3D const &Vec0,Vector3D const &Vec1)
    {
        return Vector3D(Vec0.m_x-Vec1.m_x,Vec0.m_y-Vec1.m_y,Vec0.m_z-+Vec1.m_z);
    }

    inline double operator%(Vector2D const &Vec0,Vector2D const &Vec1)
    {
        return Vec0.m_u*Vec1.m_u+Vec0.m_v*Vec1.m_v;
    }

    inline double operator%(Vector3D const &Vec0,Vector3D const &Vec1)
    {
        return Vec0.m_x*Vec1.m_x+Vec0.m_y*Vec1.m_y+Vec0.m_z*Vec1.m_z;
    }

    inline Point3D MidPoint(Point3D const &Pos0,Point3D const &Pos1,double dFraction)
    {
        return Point3D(Pos0.m_x*(1.0-dFraction)+Pos1.m_x*dFraction,
                       Pos0.m_y*(1.0-dFraction)+Pos1.m_y*dFraction,
                       Pos0.m_z*(1.0-dFraction)+Pos1.m_z*dFraction);
    }

    inline Point2D MidPoint(Point2D const &Pos0,Point2D const &Pos1,double dFraction)
    {
        return Point2D(Pos0.m_u*(1.0-dFraction)+Pos1.m_u*dFraction,
                       Pos0.m_v*(1.0-dFraction)+Pos1.m_v*dFraction);
    }

    inline bool NearEqual(double d1,double d2,double dTolerance,bool bPercent)
    {
        if(bPercent)
            {
            double dAverage=fabs(d1+d2)*0.5;
            if(dAverage<SGM_ZERO)
                {
                dAverage=(std::max)(fabs(d1),fabs(d2));
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

    inline bool NearEqual(Point3D const &Pos1,Point3D const &Pos2,double dTolerance)
    {
        return Pos1.DistanceSquared(Pos2)<dTolerance*dTolerance;
    }

    inline bool NearEqual(Point4D const &Pos1,Point4D const &Pos2,double dTolerance)
    {
        return Pos1.DistanceSquared(Pos2)<dTolerance*dTolerance;
    }

    inline bool NearEqual(Vector3D const &Vec1,Vector3D const &Vec2,double dTolerance)
    {
        return (Vec1-Vec2).MagnitudeSquared()<dTolerance*dTolerance;
    }

    inline bool NearEqual(Point2D const &Pos1,Point2D const &Pos2,double dTolerance)
    {
        return Pos1.DistanceSquared(Pos2)<dTolerance*dTolerance;
    }

} // namespace SGM

#endif //SGM_VECTOR_INL
