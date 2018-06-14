#include "EntityClasses.h"

namespace SGMInternal
{

void body::AddVolume(volume *pVolume) 
    {
    m_sVolumes.insert(pVolume);
    pVolume->SetBody(this);
    }

void body::ClearBox(SGM::Result &rResult)
    {
    m_Box.Reset();
    rResult.GetThing()->ClearBox();
    }

double body::FindVolume(SGM::Result &rResult,bool bApproximate) const
    {
    double dAnswer=0;
    std::set<volume *,EntityCompare>::const_iterator iter=m_sVolumes.begin();
    while(iter!=m_sVolumes.end())
        {
        volume *pVolume=*iter;
        dAnswer+=pVolume->FindVolume(rResult,bApproximate);
        ++iter;
        }
    return dAnswer;
    }

}