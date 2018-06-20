#include "EntityClasses.h"

namespace SGMInternal
{

void body::AddVolume(volume *pVolume) 
    {
    m_sVolumes.insert(pVolume);
    pVolume->SetBody(this);
    }

void body::ClearBox(SGM::Result &rResult) const
    {
    m_Box.Reset();
    rResult.GetThing()->ClearBox();
    }

double body::FindVolume(SGM::Result &rResult,bool bApproximate) const
    {
    double dAnswer=0;
    for (auto &&pVolume : m_sVolumes)
        {
        dAnswer+=pVolume->FindVolume(rResult,bApproximate);
        }
    return dAnswer;
    }

}