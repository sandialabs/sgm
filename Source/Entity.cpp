#include "EntityClasses.h"
#include "EntityFunctions.h"
#include "Curve.h"
#include "Surface.h"

///////////////////////////////////////////////////////////////////////////////
//
//  entity method implementations
//
///////////////////////////////////////////////////////////////////////////////
namespace SGMInternal
{

bool entity::Check(SGM::Result              &,
                   SGM::CheckOptions const  &,
                   std::vector<std::string> &aCheckStrings,
                   bool                      ) const
{
    // we do not throw so that checking may continue
    aCheckStrings.emplace_back("Derived class must override Check()\n");
    return false;
}

entity *entity::Clone(SGM::Result &) const
    {
    throw std::logic_error("Derived classes must override Clone()");
    }

SGM::Interval3D const &entity::GetBox(SGM::Result &) const
    {
    throw std::logic_error("Derived class must implement GetBox()");
    }

void entity::ResetBox(SGM::Result &) const
    {
    throw std::logic_error("Derived class must implement ResetBox()");
    }

void entity::ReplacePointers(std::map<entity *,entity *> const &)
    {
    throw std::logic_error("Derived class must implement ReplacePointers()");
    }

void entity::ChangeColor(SGM::Result &rResult,
                         int nRed,int nGreen,int nBlue)
    {
    RemoveColor(rResult);
    std::vector<int> aData;
    aData.reserve(3);
    aData.push_back(nRed);
    aData.push_back(nGreen);
    aData.push_back(nBlue);
    IntegerAttribute *pColor=new IntegerAttribute(rResult,"SGM Color",aData);
    AddAttribute(pColor);
    pColor->AddOwner(this);
    }

void entity::RemoveColor(SGM::Result &rResult)
    {
    auto iter=m_sAttributes.begin();
    while(iter!=m_sAttributes.end())
        {
        attribute *pAttribute=*iter;
        if(pAttribute->GetName()=="SGM Color")
            {
            RemoveAttribute(pAttribute);
            pAttribute->RemoveOwner(this);
            if(pAttribute->GetOwners().empty())
                {
                rResult.GetThing()->DeleteEntity(pAttribute);
                }
            break;
            }
        ++iter;
        }
    }

bool entity::GetColor(int &nRed,int &nGreen,int &nBlue) const
{
    for (auto attr : m_sAttributes)
        {
        if (attr->GetName() == "SGM Color")
            {
            auto *pIntegerAttribute = dynamic_cast<IntegerAttribute *>(attr);
            std::vector<int> const &aData = pIntegerAttribute->GetData();
            nRed = aData[0];
            nGreen = aData[1];
            nBlue = aData[2];
            return true;
            }
        }
    return false;
}

void entity::TransformBox(SGM::Transform3D const &Trans)
    {
    m_Box*=Trans;
    }

void entity::FindAllChildren(std::set<entity *, EntityCompare> &) const
    {
        throw std::logic_error("Derived class must implement FindAllChildren()");
    }


}
