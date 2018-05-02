#include "EntityClasses.h"

namespace SGM { namespace Impl {

void body::AddVolume(volume *pVolume) 
    {
    m_sVolumes.insert(pVolume);
    pVolume->SetBody(this);
    }

}}
