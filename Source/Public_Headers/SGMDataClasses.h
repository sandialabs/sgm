#ifndef SGM_DATA_CLASSES_H
#define SGM_DATA_CLASSES_H

#include "SGMEnums.h"
#include <string>

class thing;
class entity;

namespace SGM
    {
    // Note that for performance reasons the basic data classes DO NOT
    // initialize their data members with the default constructor.

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Point classes 
    //
    ///////////////////////////////////////////////////////////////////////////

    class Point2D
        {
        public:

        Point2D() {}

        Point2D(double u,double v):m_u(u),m_v(v) {}

        double Distance(SGM::Point2D const &Pos) const;

        double DistanceSquared(SGM::Point2D const &Pos) const;

        double m_u;
        double m_v;
        };

    class Point3D
        {
        public:

        Point3D() {}

        Point3D(double x,double y,double z):m_x(x),m_y(y),m_z(z) {}

        double Distance(SGM::Point3D const &Pos) const;

        double DistanceSquared(SGM::Point3D const &Pos) const;

        double m_x;
        double m_y;
        double m_z;
        };

    class Point4D
        {
        public:

        Point4D() {}

        Point4D(double x,double y,double z,double w):m_x(x),m_y(y),m_z(z),m_w(w) {}

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

    class Vector2D
        {
        public:

        Vector2D() {}

        Vector2D(double u,double v):m_u(u),m_v(v) {}

        double m_u;
        double m_v;
        };

    class Vector3D
        {
        public:

        Vector3D() {}

        Vector3D(double x,double y,double z):m_x(x),m_y(y),m_z(z) {}

        double Magnitude() const;

        Vector3D Orthogonal() const;

        Vector3D operator*(double dScale) const;

        double m_x;
        double m_y;
        double m_z;
        };

    class Vector4D
        {
        public:

        Vector4D() {}

        Vector4D(double x,double y,double z,double w):m_x(x),m_y(y),m_z(z),m_w(w) {}

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

    class UnitVector2D : public SGM::Vector2D
        {
        public:

        UnitVector2D() {}

        UnitVector2D(SGM::Vector2D const &Vec);
        };

    class UnitVector3D : public SGM::Vector3D
        {
        public:

        UnitVector3D() {}

        UnitVector3D(double x,double y,double z);

        UnitVector3D(SGM::Vector3D const &Vec);
        };

    class UnitVector4D : public SGM::Vector4D
        {
        public:

        UnitVector4D() {}

        UnitVector4D(SGM::Vector4D const &Vec);
        };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Bounding Boxes for dimensions one, two and three.
    //
    ///////////////////////////////////////////////////////////////////////////

    class Interval1D
        {
        public:

            // Note that if m_dMax<m_dMin then the interval is consider to be empty.
            // Infinite intervals may be defined by using -DBL_MAX and DBL_MAX.

            Interval1D():m_dMin(1),m_dMax(0) {}

            Interval1D(double dStart,double dEnd):m_dMin(dStart),m_dMax(dEnd) {}

            explicit Interval1D(double dPoint):m_dMin(dPoint),m_dMax(dPoint) {}

            // Interigation methods.

            bool Empty() const {return m_dMax<m_dMin;}

            double MidPoint(double dFraction=0.5) const {return m_dMin*(1-dFraction)+m_dMax*dFraction;}

            double Length() const {return m_dMax-m_dMin;}

            bool InInterval(double Pos) const;

            bool OnBoundary(double Pos) const;

            // Unites this interval with the given interval.

            SGM::Interval1D const &operator+=(SGM::Interval1D const &);

            // Intersects this interval with the given interval.

            SGM::Interval1D const &operator&=(SGM::Interval1D const &);

            // Returns true if the two intervals have a non-empty intersection.

            bool operator&&(SGM::Interval1D const &) const;

            double m_dMin;
            double m_dMax;
        };

    class Interval2D
        {
        public:
            
            // Note that if m_dMaxU<m_dMinU then the interval is consider to be empty.
            // Infinite intervals may be defined by using -DBL_MAX and DBL_MAX.

            Interval2D() {}

            Interval2D(SGM::Point2D const &Pos0,SGM::Point2D const &Pos1);

            explicit Interval2D(SGM::Point2D const &Pos);

            // Interigation methods.

            double Area() const;

            // Note that if one is sorting rectangle by perimeter, then
            // half the perimeter can be found in less time resulting
            // in the same order.  To get the perimeter muliply by two.

            double HalfPerimeter() const; 

            bool InInterval(SGM::Point2D const &Pos) const;

            // Unites this interval with the given interval.

            SGM::Interval2D const &operator+=(SGM::Interval2D const &);

            // Intersects this interval with the given interval.

            SGM::Interval2D const &operator&=(SGM::Interval2D const &);

            // Returns true if the two intervals have a non-empty intersection.

            bool operator&&(SGM::Interval2D const &) const;

        public:

            SGM::Interval1D m_UDomain;
            SGM::Interval1D m_VDomain;
        };

    class Interval3D
        {
        public:

           // Note that if m_dMaxX<m_dMinX then the interval is consider to be empty.
           // Infinite intervals may be defined by using -DBL_MAX and DBL_MAX.

           Interval3D() {}

           Interval3D(SGM::Point3D const &Pos0,SGM::Point3D const &Pos1);

           explicit Interval3D(SGM::Point3D const &Pos);

           // Interigation methods.

           bool IsEmpty() const {return m_XDomain.m_dMax<m_XDomain.m_dMin;}

           double Volume() const;

           // Note that if one is sorting boxes by area, then
           // half the area can be found in less time resulting
           // in the same order.  To get the area muliply by two.

           double HalfArea() const; 

           bool InInterval(SGM::Point3D const &Pos) const;

           // Unites this interval with the given interval.

           SGM::Interval3D const &operator+=(SGM::Interval3D const &);

           // Intersects this interval with the given interval.

           SGM::Interval3D const &operator&=(SGM::Interval3D const &);

           // Returns true if the two intervals have a non-empty intersection.

           bool operator&&(SGM::Interval3D const &) const;

           // Modify methods

           void clear() {m_XDomain.m_dMax=1;m_XDomain.m_dMin=0;}

        public:

            SGM::Interval1D m_XDomain;
            SGM::Interval1D m_YDomain;
            SGM::Interval1D m_ZDomain;
        };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Segment classes for dimensions two and three.
    //
    ///////////////////////////////////////////////////////////////////////////

    class Segment2D
        {
        public:

            Segment2D();

            Segment2D(Point2D const &Start,
                      Point2D const &End);

            // Returns true, and the point of intersection, if this 
            // segment and the given segment intersect.

            bool Intersect(Segment2D const &other,
                           Point2D         &Pos) const;

            // Returns true, and the point of intersection, if this 
            // segment and the given ray intersect.

            bool Intersect(Point2D      const &RayOrigin,
                           UnitVector3D const &RayDirection,
                           Point2D            &Pos) const;

            Point2D m_Start;
            Point2D m_End;
        };

    class Segment3D
        {
        public:

            Segment3D();

            Segment3D(Point3D const &Start,
                      Point3D const &End);
            
            // Returns the closest pairs of points on the two lines defined by
            // this and the given segment.  If the two points are not on the two
            //  segments, then false is returned.

            bool Intersect(SGM::Segment3D const &Seg,
                           SGM::Point3D         &Pos1,
                           SGM::Point3D         &Pos2) const;

            Point3D m_Start;
            Point3D m_End;
        };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Tranformation classes
    //
    ///////////////////////////////////////////////////////////////////////////

    class Transform3D
        {
        public:

            // Returns the identity transform.

            Transform3D() {m_Matrix[0]=SGM::Vector4D(1,0,0,0);
                           m_Matrix[1]=SGM::Vector4D(0,1,0,0);
                           m_Matrix[2]=SGM::Vector4D(0,0,1,0);
                           m_Matrix[3]=SGM::Vector4D(0,0,0,1);}

            // Returns a transform that scales space by dS.

            explicit Transform3D(double dS) 
                          {m_Matrix[0]=SGM::Vector4D(dS,0,0,0);
                           m_Matrix[1]=SGM::Vector4D(0,dS,0,0);
                           m_Matrix[2]=SGM::Vector4D(0,0,dS,0);
                           m_Matrix[3]=SGM::Vector4D(0,0,0,1);}

            // Returns a transform that translates space by the given vector.

            explicit Transform3D(SGM::Vector3D const &Translate) 
                          {m_Matrix[0]=SGM::Vector4D(1,0,0,Translate.m_x);
                           m_Matrix[1]=SGM::Vector4D(0,1,0,Translate.m_y);
                           m_Matrix[2]=SGM::Vector4D(0,0,1,Translate.m_z);
                           m_Matrix[3]=SGM::Vector4D(0,0,0,1);}

            // Returns a transform that scales by different amounts in each of
            // the three coordinate axies.

            Transform3D(double dXScale,
                        double dYScale,
                        double dZScale) 
                          {m_Matrix[0]=SGM::Vector4D(dXScale,0,0,0);
                           m_Matrix[1]=SGM::Vector4D(0,dYScale,0,0);
                           m_Matrix[2]=SGM::Vector4D(0,0,dZScale,0);
                           m_Matrix[3]=SGM::Vector4D(0,0,0,1);}

            // Returns a transform that rotates space about the origin by 
            // moving the X, Y and Z axies to the given values.

            Transform3D(SGM::UnitVector3D const &XAxis,
                        SGM::UnitVector3D const &YAxis,
                        SGM::UnitVector3D const &ZAxis) 
                          {m_Matrix[0]=SGM::Vector4D(XAxis.m_x,YAxis.m_x,ZAxis.m_x,0);
                           m_Matrix[1]=SGM::Vector4D(XAxis.m_y,YAxis.m_y,ZAxis.m_y,0);
                           m_Matrix[2]=SGM::Vector4D(XAxis.m_z,YAxis.m_z,ZAxis.m_z,0);
                           m_Matrix[3]=SGM::Vector4D(0,0,0,1);}

            // Returns a transform that first rotates space and then translates.

            Transform3D(SGM::UnitVector3D const &XAxis,
                        SGM::UnitVector3D const &YAxis,
                        SGM::UnitVector3D const &ZAxis,
                        SGM::Vector3D     const &Translate) 
                          {m_Matrix[0]=SGM::Vector4D(XAxis.m_x,YAxis.m_x,ZAxis.m_x,Translate.m_x);
                           m_Matrix[1]=SGM::Vector4D(XAxis.m_y,YAxis.m_y,ZAxis.m_y,Translate.m_y);
                           m_Matrix[2]=SGM::Vector4D(XAxis.m_z,YAxis.m_z,ZAxis.m_z,Translate.m_z);
                           m_Matrix[3]=SGM::Vector4D(0,0,0,1);}

            // Returns the four by four transform from all the data.

            Transform3D(SGM::Vector4D const &XAxis,
                        SGM::Vector4D const &YAxis,
                        SGM::Vector4D const &ZAxis,
                        SGM::Vector4D const &Translate) 
                          {m_Matrix[0]=SGM::Vector4D(XAxis.m_x,YAxis.m_x,ZAxis.m_x,Translate.m_x);
                           m_Matrix[1]=SGM::Vector4D(XAxis.m_y,YAxis.m_y,ZAxis.m_y,Translate.m_y);
                           m_Matrix[2]=SGM::Vector4D(XAxis.m_z,YAxis.m_z,ZAxis.m_z,Translate.m_z);
                           m_Matrix[3]=SGM::Vector4D(XAxis.m_w,YAxis.m_w,ZAxis.m_w,Translate.m_w);}

            // Returns a transform Trans such that this*Trans = Trans*this = Identity.

            void Inverse(Transform3D &Trans) const;

            // Get methods.

            SGM::Vector4D const *GetData() const {return m_Matrix;}

        private:

            SGM::Vector4D m_Matrix[4];
        };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Miscellaneous data classes
    //
    ///////////////////////////////////////////////////////////////////////////

    class Result
        {
        public:

            Result(thing *pThing):m_pThing(pThing) {}

            void SetResult(SGM::ResultType nType);

            void SetMessage(std::string const &sMessage);

            void Clear();

            SGM::ResultType GetResult() const {return m_nType;}

            std::string const &GetMessage() const {return m_sMessage;}

            thing *GetThing() const {return m_pThing;}

            entity *FindEntity(size_t nID) const;

        private:

            SGM::ResultType  m_nType;
            std::string      m_sMessage;
            thing           *m_pThing;
        };

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Multi-class operators and functions
    //
    ///////////////////////////////////////////////////////////////////////////

    SGM::Vector2D operator-(SGM::Point2D const &Pos0,SGM::Point2D const &Pos1);

    SGM::Vector3D operator-(SGM::Point3D const &Pos0,SGM::Point3D const &Pos1);

    SGM::Vector3D operator+(SGM::Vector3D const &Vec0,SGM::Vector3D const &Vec1);

    SGM::Point3D operator+(SGM::Point3D const &Pos,SGM::Vector3D const &Vec);

    SGM::Point3D operator-(SGM::Point3D const &Pos,SGM::Vector3D const &Vec);

    SGM::Vector3D operator-(SGM::Vector3D const &Vec0,SGM::Vector3D const &Vec1);

    SGM::Vector3D operator*(SGM::Vector3D const &Vec0,SGM::Vector3D const &Vec1);

    SGM::Transform3D operator*(SGM::Transform3D const &Trans0,SGM::Transform3D const &Trans1);

    SGM::Point3D operator*(SGM::Transform3D const &Trans,SGM::Point3D const &Pos);

    SGM::Vector3D operator*(SGM::Transform3D const &Trans,SGM::Vector3D const &Vec);

    SGM::UnitVector3D operator*(SGM::Transform3D const &Trans,SGM::UnitVector3D const &UVec);
    
    double operator%(SGM::Vector2D const &Vec0,SGM::Vector2D const &Vec1);

    double operator%(SGM::Vector3D const &Vec0,SGM::Vector3D const &Vec1);

    SGM::Vector3D operator-(SGM::Vector3D const &Vec);

    SGM::UnitVector3D operator-(SGM::UnitVector3D const &UVec);

    SGM::Point3D MidPoint(SGM::Point3D const &Pos0,SGM::Point3D const &Pos1,double dFraction=0.5);
    
    SGM::Point2D MidPoint(SGM::Point2D const &Pos0,SGM::Point2D const &Pos1,double dFraction=0.5);

    bool NearEqual(double d1,double d2,double dTolerance,bool bPercent);

    bool NearEqual(SGM::Point3D const &Pos1,SGM::Point3D const &Pos2,double dTolerance);

    } // End of SGM namespace

#endif // SGM_DATA_CLASSES_H
