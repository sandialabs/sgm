#include "EntityClasses.h"

namespace SGMInternal
{

void body::AddVolume(volume *pVolume) 
    {
    m_sVolumes.insert(pVolume);
    pVolume->SetBody(this);
    }

}