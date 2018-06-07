#include "SGMVector.h"
#include "SGMMathematics.h"

// Functions defined here in .cpp in order to avoid cyclic dependency
// on SGMMathematics.h and SGMVector.inl

// Returns the a number between zero and pi which is the angle between
// this vector and the given vector.
double SGM::UnitVector3D::Angle(SGM::UnitVector3D const &Vec) const
{
    return SGM::SAFEacos((*this)%Vec);
}

// Returns the a number between zero and 2*pi which is the angle between
// this vector and the given vector with respect to the given normal
// vector in the right handed direction.

double SGM::UnitVector3D::Angle(SGM::UnitVector3D const &Vec,
                                SGM::UnitVector3D const &Norm) const
{
    SGM::UnitVector3D YVec=Norm*(*this);
    return SAFEatan2(YVec%Vec,(*this)%Vec);
}
