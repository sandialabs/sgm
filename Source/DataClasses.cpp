#include "SGMVector.h"
#include "SGMTransform.h"
#include "SGMSegment.h"
#include "SGMMathematics.h"
#include "EntityClasses.h"
#include <math.h>
#include <algorithm>

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

SGM::Vector2D SGM::operator*(double dValue,SGM::Vector2D const &Vec)
    {
    return SGM::Vector2D(Vec.m_u*dValue,Vec.m_v*dValue);
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

SGM::Vector2D SGM::operator+(SGM::Vector2D const &Vec0,SGM::Vector2D const &Vec1)
    {
    return SGM::Vector2D(Vec0.m_u+Vec1.m_u,Vec0.m_v+Vec1.m_v);
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
    return SGM::Point3D(Pos0.m_x*(1.0-dFraction)+Pos1.m_x*dFraction,
                        Pos0.m_y*(1.0-dFraction)+Pos1.m_y*dFraction,
                        Pos0.m_z*(1.0-dFraction)+Pos1.m_z*dFraction);
    }
    
SGM::Point2D SGM::MidPoint(SGM::Point2D const &Pos0,SGM::Point2D const &Pos1,double dFraction)
    {
    return SGM::Point2D(Pos0.m_u*(1.0-dFraction)+Pos1.m_u*dFraction,
                        Pos0.m_v*(1.0-dFraction)+Pos1.m_v*dFraction);
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
