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

entity *entity::Clone(SGM::Result &rResult) const
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

void entity::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
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

//TODO: make virtual
void entity::FindAllChildren(std::set<entity *, EntityCompare> &sChildren) const
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
                sChildren.insert(pVolume);
                pVolume->FindAllChildren(sChildren);
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
                sChildren.insert(pFace);
                pFace->FindAllChildren(sChildren);
                ++iter1;
                }
            std::set<edge *,EntityCompare> const &sEdges=pVolume->GetEdges();
            std::set<edge *,EntityCompare>::iterator iter2=sEdges.begin();
            while(iter2!=sEdges.end())
                {
                edge *pEdge=*iter2;
                sChildren.insert(pEdge);
                pEdge->FindAllChildren(sChildren);
                ++iter2;
                }
            break;
            }
        case SGM::FaceType:
            {
            face const *pFace=(face const *)this;
            sChildren.insert((entity *)(pFace->GetSurface()));
            std::set<edge *,EntityCompare> const &sEdges=pFace->GetEdges();
            std::set<edge *,EntityCompare>::iterator iter=sEdges.begin();
            while(iter!=sEdges.end())
                {
                edge *pEdge=*iter;
                sChildren.insert(pEdge);
                pEdge->FindAllChildren(sChildren);
                ++iter;
                }
            break;
            }
        case SGM::EdgeType:
            {
            edge const *pEdge=(edge const *)this;
            sChildren.insert((entity *)(pEdge->GetCurve()));
            if(pEdge->GetStart())
                {
                sChildren.insert(pEdge->GetStart());
                }
            if(pEdge->GetEnd())
                {
                sChildren.insert(pEdge->GetEnd());
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
        case SGM::ComplexType:
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
                    revolve const *pRevolve=(revolve const *)this;
                    sChildren.insert((entity *)(pRevolve->m_pCurve));
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

}
