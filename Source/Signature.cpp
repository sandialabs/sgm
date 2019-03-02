#include "Signature.h"

namespace SGMInternal
{

bool Signature::Matches(Signature const &other, bool bIgnoreScale) const
{
    bool bRelative = false;
    if (false == bIgnoreScale)
    {
        if (!SGM::NearEqual(other.Scale, Scale, SGM_MIN_TOL, bRelative))
        {
            return false;
        }
    }

    return (SGM::NearEqual(Rubric, other.Rubric, SGM_MIN_TOL, bRelative));
}

}
