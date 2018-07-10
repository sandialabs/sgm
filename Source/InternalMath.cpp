#include "Mathematics.h"
#include "SGMConstants.h"
#include "SGMVector.h"

namespace SGMInternal
{

SGM::Vector3D Snap(SGM::Vector3D const &Vec)
    {
    SGM::Vector3D Answer=Vec;
    if(fabs(Vec.m_x)<SGM_ZERO)
        {
        Answer.m_x=0.0;
        }
    if(fabs(Vec.m_y)<SGM_ZERO)
        {
        Answer.m_y=0.0;
        }
    if(fabs(Vec.m_z)<SGM_ZERO)
        {
        Answer.m_z=0.0;
        }
    return Answer;
    }

} // End namespace SGMInternal