#include "EntityClasses.h"
#include "Curve.h"

///////////////////////////////////////////////////////////////////////////////
//
//  entity methods
//
///////////////////////////////////////////////////////////////////////////////
namespace SGMInternal
{
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