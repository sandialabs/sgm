#include <iostream>
#include "SGMVector.h"
#include "SGMMathematics.h"
#include "SGMTransform.h"

namespace SGM {

// Functions defined here in .cpp in order to avoid cyclic dependency
// on SGMMathematics.h and SGMVector.inl

// Returns the a number between zero and pi which is the angle between
// this vector and the given vector.
    double UnitVector3D::Angle(UnitVector3D const &Vec) const
    {
        return SAFEacos((*this)%Vec);
    }

// Returns the a number between zero and 2*pi which is the angle between
// this vector and the given vector with respect to the given normal
// vector in the right handed direction.

    double UnitVector3D::Angle(UnitVector3D const &Vec,
                               UnitVector3D const &Norm) const
    {
        UnitVector3D YVec=Norm*(*this);
        double dAnswer=SAFEatan2(YVec%Vec,(*this)%Vec);
        return dAnswer<0 ? dAnswer+SGM_TWO_PI : dAnswer;
    }

///////////////////////////////////////////////////////////////////////////
//
// Transform operators for Point, Vector, UnitVector
//
///////////////////////////////////////////////////////////////////////////

// These must be defined in .cpp because of cyclic dependency of Transform and Point3D, Vector3D.

    Point3D Point3D::operator*=(Transform3D const &Trans)
    {
        *this = Trans*(*this);
        return *this;
    }

    Vector3D Vector3D::operator*=(Transform3D const &Trans)
    {
        *this = Trans*(*this);
        return *this;
    }

    UnitVector3D UnitVector3D::operator*=(Transform3D const &Trans)
    {
        *this = Trans*(*this);
        return *this;
    }

} // namespace SGM

