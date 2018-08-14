#include <EntityFunctions.h>
#include "EntityClasses.h"
#include "Topology.h"

namespace SGMInternal
{

body::body(SGM::Result &rResult):
    topology(rResult,SGM::EntityType::BodyType) 
    {
    }

body *body::Clone(SGM::Result &rResult) const
    {
    body *pAnswer=new body(rResult);
    pAnswer->m_sVolumes=m_sVolumes;
    pAnswer->m_aPoints=m_aPoints;
    return pAnswer;
    }

void body::FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const
    {
    for (auto pVolume : GetVolumes())
        {
        sChildren.insert(pVolume);
        pVolume->FindAllChildren(sChildren);
        }
    }

void body::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
    {
    // Run though all the pointers and change them if they are in the map.
    
    std::set<volume *,EntityCompare> m_sFixedVolumes;
    for(auto pVolume : m_sVolumes)
        {
        auto MapValue=mEntityMap.find(pVolume);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedVolumes.insert((volume *)MapValue->second);
            }
        else
            {
            m_sFixedVolumes.insert(pVolume);
            }
        }
    m_sVolumes=m_sFixedVolumes;

    std::set<attribute *,EntityCompare> m_sFixedAttributes;
    for(auto pAttribute : m_sAttributes)
        {
        auto MapValue=mEntityMap.find(pAttribute);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedAttributes.insert((attribute *)MapValue->second);
            }
        else
            {
            m_sFixedAttributes.insert(pAttribute);
            }
        }
    m_sAttributes=m_sFixedAttributes;

    std::set<entity *,EntityCompare> m_sFixedOwners;
    for(auto pEntity : m_sOwners)
        {
        auto MapValue=mEntityMap.find(pEntity);
        if(MapValue!=mEntityMap.end())
            {
            m_sFixedOwners.insert((attribute *)MapValue->second);
            }
        else
            {
            m_sFixedOwners.insert(pEntity);
            }
        }
    m_sOwners=m_sFixedOwners;
    }

void body::AddVolume(volume *pVolume) 
    {
    m_sVolumes.insert(pVolume);
    pVolume->SetBody(this);
    }

void body::RemoveVolume(volume *pVolume)
    {
    m_sVolumes.erase(pVolume);
    pVolume->SetBody(nullptr);
    }

void body::AddPoint(SGM::Point3D const &Pos)
    {
    m_aPoints.push_back(Pos);
    }

SGM::Interval3D const &body::GetBox(SGM::Result &rResult) const
    {
    if (m_Box.IsEmpty())
        {
        std::set<volume *,EntityCompare> const &sVolumes = GetVolumes();
        StretchBox(rResult,m_Box,sVolumes.begin(),sVolumes.end());
        }
    return m_Box;
    }

void body::ResetBox(SGM::Result &rResult) const
    {
    m_Box.Reset();
    rResult.GetThing()->ResetBox(rResult);
    }

void body::SeverRelations(SGM::Result &rResult)
    {
    std::set<volume *,EntityCompare> sVolumes=GetVolumes();
    for(volume *pVolume : sVolumes)
        RemoveVolume(pVolume);
    RemoveAllOwners();
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

bool body::IsSheetBody(SGM::Result &rResult) const
    {
    std::set<edge *,EntityCompare> sEdges;
    FindWireEdges(rResult,this,sEdges);
    if(sEdges.size())
        {
        return false;
        }
    std::set<face *,EntityCompare> sFaces;
    FindFaces(rResult,this,sFaces);
    auto iter=sFaces.begin();
    while(iter!=sFaces.end())
        {
        face *pFace=*iter;
        if(pFace->GetSides()!=2)
            {
            return false;
            }
        ++iter;
        }
    if(sFaces.size())
        {
        return true;
        }
    return false;
    }

bool body::IsWireBody(SGM::Result &rResult) const
    {
    std::set<face *,EntityCompare> sFaces;
    FindFaces(rResult,this,sFaces);
    if(sFaces.size())
        {
        return false;
        }
    std::set<edge *,EntityCompare> sEdges;
    FindWireEdges(rResult,this,sEdges);
    if(sEdges.size())
        {
        return true;
        }
    return false;
    }

}