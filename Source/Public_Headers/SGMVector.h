#ifndef SGM_POINT_H
#define SGM_POINT_H

#include <cassert>
#include <cstddef>
#include <vector>

#include "sgm_export.h"
#include "SGMEnums.h"

///////////////////////////////////////////////////////////////////////////////
//
//  Point2D,3D, Vector2D,3D,4D, UnitVector2D,3D,4D and Ray3D 
//
///////////////////////////////////////////////////////////////////////////////

namespace SGM
{
    class Vector2D;
    class Vector3D;
    class Vector4D;
    class Transform3D;

    // Note that for performance reasons the basic data classes DO NOT
    // initialize their data members with the default constructor.

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Point classes 
    //
    ///////////////////////////////////////////////////////////////////////////

    class SGM_EXPORT Point2D
    {
    public:

        Point2D() = default;

        Point2D(double u,double v):m_u(u),m_v(v) {}

        double Distance(Point2D const &Pos) const;

        double DistanceSquared(Point2D const &Pos) const;

        // Orders by strict dictionary order, i.e. m_u first then m_v if the 
        // m_u's are equal.

        bool operator<(Point2D const &Pos) const;

        Point2D operator+=(Vector2D const &Vec);

        double m_u;
        double m_v;
    };

    class SGM_EXPORT Point3D
    {
    public:

        enum { N = 3 };

        Point3D() = default;

        Point3D(double x,double y,double z):m_x(x),m_y(y),m_z(z) {}

        explicit Point3D(Vector3D const &Vec);

        const double& operator []( const size_t axis ) const { assert(axis < N); return (&m_x)[axis]; }
        double& operator []( const size_t axis )             { assert(axis < N); return (&m_x)[axis]; }

        double Distance(Point3D const &Pos) const;

        double DistanceSquared(Point3D const &Pos) const;

        Point3D operator+=(Vector3D const &Vec);

        Point3D operator*=(Transform3D const &Trans);

        bool operator<(Point3D const &Pos) const;

        double m_x;
        double m_y;
        double m_z;
    };

    class SGM_EXPORT Point4D
    {
    public:

        Point4D() = default;

        Point4D(double x,double y,double z,double w):m_x(x),m_y(y),m_z(z),m_w(w) {}

        Point4D operator+=(Vector4D const &Vec);

        double m_x;
        double m_y;
        double m_z;
        double m_w;
    };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Vector classes 
    //
    ///////////////////////////////////////////////////////////////////////////

    class SGM_EXPORT Vector2D
    {
    public:

        Vector2D() = default;

        Vector2D(double u,double v):m_u(u),m_v(v) {}

        Vector2D operator*(double dScale) const;

        double m_u;
        double m_v;
    };

    class SGM_EXPORT Vector3D
    {
    public:

        enum { N = 3 };

        Vector3D() = default;

        Vector3D(double x,double y,double z):m_x(x),m_y(y),m_z(z) {}

        explicit Vector3D(Point3D const &Pos);

        double Dot(Vector3D const &v) const {
            return m_x*v.m_x + m_y*v.m_y + m_z*v.m_z;
        }

        double Magnitude() const;

        double MagnitudeSquared() const;

        Vector3D Orthogonal() const;

        Vector3D operator*(double dScale) const;

        Vector3D operator/(double dScale) const;

        Vector3D operator*=(Transform3D const &Trans);

        const double& operator []( const size_t axis ) const { assert(axis < N); return (&m_x)[axis]; }
        double& operator []( const size_t axis )             { assert(axis < N); return (&m_x)[axis]; }

        void Negate();

        double m_x;
        double m_y;
        double m_z;
    };

    class SGM_EXPORT Vector4D
    {
    public:

        Vector4D() = default;

        Vector4D(double x,double y,double z,double w):m_x(x),m_y(y),m_z(z),m_w(w) {}

        explicit Vector4D(Point4D const &Pos);

        Vector4D operator*(double dScale) const;

        Vector4D operator/(double dScale) const;

        double m_x;
        double m_y;
        double m_z;
        double m_w;
    };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Unit vector classes 
    //
    ///////////////////////////////////////////////////////////////////////////

    class SGM_EXPORT UnitVector2D : public Vector2D
    {
    public:

        UnitVector2D() = default;

        UnitVector2D(double u,double v);

        UnitVector2D(Vector2D const &Vec);
    };

    class SGM_EXPORT UnitVector3D : public Vector3D
    {
    public:

        UnitVector3D() = default;

        UnitVector3D(double x,double y,double z);

        UnitVector3D(Vector3D const &Vec);

        // Returns the a number between zero and pi which is the angle between
        // this vector and the given vector.

        double Angle(UnitVector3D const &Vec) const;

        // Returns the a number between zero and 2*pi which is the angle between
        // this vector and the given vector with respect to the given normal 
        // vector in the right handed direction.

        double Angle(UnitVector3D const &Vec,
                     UnitVector3D const &Norm) const;

        UnitVector3D operator*=(Transform3D const &Trans);
    };

    class SGM_EXPORT UnitVector4D : public Vector4D
    {
    public:

        UnitVector4D() = default;

        UnitVector4D(Vector4D const &Vec);
    };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Ray class
    //
    ///////////////////////////////////////////////////////////////////////////

    class Ray3D
    {
    public:
        
        Ray3D(const Point3D &orig, const UnitVector3D &dir) :
                m_Origin(orig),
                m_Direction(dir),
                m_InverseDirection(1./dir.m_x, 1./dir.m_y, 1./dir.m_z),
                m_xSign(m_InverseDirection.m_x < 0),
                m_ySign(m_InverseDirection.m_y < 0),
                m_zSign(m_InverseDirection.m_z < 0)
        { }

        // The Ray is pure constant value class to store direction and origin, which
        // is used to implement fast ray, line, segment intersections with axis
        // aligned bounding boxes (Interval3D). This allows the inverse direction and
        // sign to be calculated only once in the constructor, and so we disallow
        // changing member variables so we do not violate the inverse direction and sign.

        Point3D      m_Origin;
        UnitVector3D m_Direction;

        Vector3D     m_InverseDirection;
        int          m_xSign;
        int          m_ySign;
        int          m_zSign;
    };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Multi-class operators and functions
    //
    ///////////////////////////////////////////////////////////////////////////

    inline Point2D operator+(Point2D const &Pos,Vector2D const &Vec);

    inline Point2D operator-(Point2D const &Pos,Vector2D const &Vec);

    inline Vector2D operator-(Point2D const &Pos0,Point2D const &Pos1);

    inline Vector2D operator+(Vector2D const &Vec0,Vector2D const &Vec1);

    inline Vector2D operator-(Vector2D const &Vec0,Vector2D const &Vec1);

    inline Vector2D operator*(double dValue,Vector2D const &Vec);

    inline Point3D operator+(Point3D const &Pos,Vector3D const &Vec);

    inline Point3D operator-(Point3D const &Pos,Vector3D const &Vec);

    inline Vector3D operator-(Point3D const &Pos0,Point3D const &Pos1);

    inline Vector3D operator-(Vector3D const &Vec);

    inline Vector3D operator+(Vector3D const &Vec0,Vector3D const &Vec1);

    inline Vector3D operator-(Vector3D const &Vec0,Vector3D const &Vec1);

    inline Vector3D operator*(double dValue,Vector3D const &Vec);

    inline Vector3D operator*(Vector3D const &Vec0,Vector3D const &Vec1);

    inline UnitVector3D operator-(UnitVector3D const &UVec);

    inline Point4D operator+(Point4D const &Pos,Vector4D const &Vec);

    inline Point4D operator-(Point4D const &Pos,Vector4D const &Vec);

    inline Vector4D operator*(double dValue,Vector4D const &Vec);

    inline double operator%(Vector2D const &Vec0,Vector2D const &Vec1);

    inline double operator%(Vector3D const &Vec0,Vector3D const &Vec1);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  MidPoint and NearEqual functions.
    //
    ///////////////////////////////////////////////////////////////////////////

    inline Point3D MidPoint(Point3D const &Pos0,Point3D const &Pos1,double dFraction=0.5);

    inline Point2D MidPoint(Point2D const &Pos0,Point2D const &Pos1,double dFraction=0.5);

    inline bool NearEqual(double d1,double d2,double dTolerance,bool bPercent);

    inline bool NearEqual(Point2D const &Pos1,Point2D const &Pos2,double dTolerance);

    inline bool NearEqual(Point3D const &Pos1,Point3D const &Pos2,double dTolerance);

    inline bool NearEqual(Vector3D const &Pos1,Vector3D const &Pos2,double dTolerance);

} // End of SGM namespace

#include "Inline/SGMVector.inl" // inline implementations

#endif //SGM_POINT_H
