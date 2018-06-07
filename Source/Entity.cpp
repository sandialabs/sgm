#include "EntityClasses.h"
#include "Curve.h"
#include "Surface.h"

///////////////////////////////////////////////////////////////////////////////
//
//  entity methods
//
///////////////////////////////////////////////////////////////////////////////
namespace SGMInternal
{

bool EntityCompare::operator()(entity const *ent1,entity const *ent2)
    {
    return ent1->GetID()<ent2->GetID();
    }

bool EntityPointerCompare(entity *pEnt0,entity *pEnt1)
    {
    return pEnt0->GetID()<pEnt1->GetID();
    }

entity::entity(SGM::Result &rResult,SGM::EntityType nType):
    m_ID(rResult.GetThing()->GetNextID()),m_Type(nType) 
    {
    rResult.GetThing()->AddToMap(m_ID,this);
    }

entity::entity():
    m_ID(0),m_Type(SGM::ThingType) 
    {
    }

entity *entity::Copy(SGM::Result &rResult) const
    {
    switch(m_Type)
        {
        case SGM::CurveType:
            {
            curve const *pCurve=(curve const *)this;
            return pCurve->MakeCopy(rResult);
            }
        default:
            {
            throw;
            }
        }
    }

void entity::SeverOwners() const
    {
    for (entity *pOwner : m_Owners)
        {
        pOwner->RemoveOwner((entity*)this);
        }
    }

void entity::FindAllChildern(std::set<entity *,EntityCompare> &sChildern) const
    {
    switch(m_Type)
        {
        case SGM::BodyType:
            {
            body const *pBody=(body const *)this;
            std::set<volume *,EntityCompare> const &sVolumes=pBody->GetVolumes();
            std::set<volume *,EntityCompare>::iterator iter=sVolumes.begin();
            while(iter!=sVolumes.end())
                {
                volume *pVolume=*iter;
                sChildern.insert(pVolume);
                pVolume->FindAllChildern(sChildern);
                ++iter;
                }
            break;
            }
        case SGM::VolumeType:
            {
            volume const *pVolume=(volume const *)this;
            std::set<face *,EntityCompare> const &sFaces=pVolume->GetFaces();
            std::set<face *,EntityCompare>::iterator iter1=sFaces.begin();
            while(iter1!=sFaces.end())
                {
                face *pFace=*iter1;
                sChildern.insert(pFace);
                pFace->FindAllChildern(sChildern);
                ++iter1;
                }
            std::set<edge *,EntityCompare> const &sEdges=pVolume->GetEdges();
            std::set<edge *,EntityCompare>::iterator iter2=sEdges.begin();
            while(iter2!=sEdges.end())
                {
                edge *pEdge=*iter2;
                sChildern.insert(pEdge);
                pEdge->FindAllChildern(sChildern);
                ++iter2;
                }
            break;
            }
        case SGM::FaceType:
            {
            face const *pFace=(face const *)this;
            sChildern.insert((entity *)(pFace->GetSurface()));
            std::set<edge *,EntityCompare> const &sEdges=pFace->GetEdges();
            std::set<edge *,EntityCompare>::iterator iter=sEdges.begin();
            while(iter!=sEdges.end())
                {
                edge *pEdge=*iter;
                sChildern.insert(pEdge);
                pEdge->FindAllChildern(sChildern);
                ++iter;
                }
            break;
            }
        case SGM::EdgeType:
            {
            edge const *pEdge=(edge const *)this;
            sChildern.insert((entity *)(pEdge->GetCurve()));
            if(pEdge->GetStart())
                {
                sChildern.insert(pEdge->GetStart());
                }
            if(pEdge->GetEnd())
                {
                sChildern.insert(pEdge->GetEnd());
                }
            break;
            }
        case SGM::VertexType:
            {
            break;
            }
        case SGM::CurveType:
            {
            break;
            }
        case SGM::SurfaceType:
            {
            surface const *pSurface=(surface const *)this;
            switch(pSurface->GetSurfaceType())
                {
                case SGM::RevolveType:
                    {
                    revolve const *pRevole=(revolve const *)this;
                    sChildern.insert((entity *)(pRevole->m_pCurve));
                    break;
                    }
                default:
                    break;
                }
            break;
            }
        default:
            {
            throw;
            }
        }
    }

void entity::Transform(SGM::Result            &,//rResult,
                       SGM::Transform3D const &Trans)
    {
    switch(m_Type)
        {
        case SGM::CurveType:
            {
            curve *pCurve=(curve *)this;
            pCurve->Transform(Trans);
            break;
            }
        default:
            {
            throw;
            }
        }
    }
}