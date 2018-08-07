#include <EntityFunctions.h>
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

bool EntityPointerCompare(entity *pEnt0,entity *pEnt1)
    {
    return pEnt0->GetID()<pEnt1->GetID();
    }

size_t entity::GetID() const
    {
    return m_ID;
    }

entity *entity::MakeCopy(SGM::Result &rResult) const
    {
    switch(m_Type)
        {
        case SGM::VertexType:
            {
            vertex const *pVertex=(vertex const *)this;
            return pVertex->MakeCopy(rResult);
            }
        case SGM::CurveType:
            {
            curve const *pCurve=(curve const *)this;
            return pCurve->MakeCopy(rResult);
            }
        case SGM::SurfaceType:
            {
            surface const *pSurface=(surface const *)this;
            return pSurface->MakeCopy(rResult);
            }
        case SGM::EdgeType:
            {
            edge const *pEdge=(edge const *)this;
            return pEdge->MakeCopy(rResult);
            }
        case SGM::FaceType:
            {
            face const *pFace=(face const *)this;
            return pFace->MakeCopy(rResult);
            }
        case SGM::VolumeType:
            {
            volume const *pVolume=(volume const *)this;
            return pVolume->MakeCopy(rResult);
            }
        case SGM::BodyType:
            {
            body const *pBody=(body const *)this;
            return pBody->MakeCopy(rResult);
            }
        default:
            {
            throw;
            }
        }
    }

void entity::ReplacePointers(std::map<entity *,entity *> const &mEntityMap)
    {
    switch(m_Type)
        {
        case SGM::VertexType:
            {
            vertex *pVertex=(vertex *)this;
            pVertex->ReplacePointers(mEntityMap);
            break;
            }
        case SGM::CurveType:
            {
            curve *pCurve=(curve *)this;
            pCurve->ReplacePointers(mEntityMap);
            break;
            }
        case SGM::SurfaceType:
            {
            surface *pSurface=(surface *)this;
            pSurface->ReplacePointers(mEntityMap);
            break;
            }
        case SGM::EdgeType:
            {
            edge *pEdge=(edge *)this;
            pEdge->ReplacePointers(mEntityMap);
            break;
            }
        case SGM::FaceType:
            {
            face *pFace=(face *)this;
            pFace->ReplacePointers(mEntityMap);
            break;
            }
        case SGM::VolumeType:
            {
            volume *pVolume=(volume *)this;
            pVolume->ReplacePointers(mEntityMap);
            break;
            }
        case SGM::BodyType:
            {
            body *pBody=(body *)this;
            pBody->ReplacePointers(mEntityMap);
            break;
            }
        default:
            {
            throw;
            }
        }
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
    std::set<attribute *,EntityCompare>::const_iterator iter=m_sAttributes.begin();
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
    std::set<attribute *,EntityCompare>::const_iterator iter=m_sAttributes.begin();
    while(iter!=m_sAttributes.end())
        {
        attribute *pAttribute=*iter;
        if(pAttribute->GetName()=="SGM Color")
            {
            IntegerAttribute *pIntegerAttribute=(IntegerAttribute *)pAttribute;
            std::vector<int> const &aData=pIntegerAttribute->GetData();
            nRed=aData[0];
            nGreen=aData[1];
            nBlue=aData[2];
            return true;
            }
        ++iter;
        }

    // If the object does not have a color then it may get the color
    // from the volume or body.

    switch(m_Type)
        {
        case SGM::VolumeType:
            {
            volume const *pVolume=(volume const *)this;
            if(pVolume->GetBody())
                {
                return pVolume->GetBody()->GetColor(nRed,nGreen,nBlue);
                }
            break;
            }
        case SGM::FaceType:
            {
            face const *pFace=(face const *)this;
            if(pFace->GetVolume())
                {
                return pFace->GetVolume()->GetColor(nRed,nGreen,nBlue);
                }
            break;
            }
        default:
            {
            break;
            }
        }
    
    return false;
    }

SGM::Interval3D const &entity::GetBox(SGM::Result &rResult) const
    {
    if(m_Box.IsEmpty())
        {
        switch(m_Type)
            {
            case SGM::EntityType::ThingType:
                {
                
                break;
                }
            case SGM::EntityType::BodyType:
                {
                body *pBody=(body *)this;
                std::set<volume *,EntityCompare> const &sVolumes=pBody->GetVolumes();
                StretchBox(rResult,m_Box,sVolumes.begin(),sVolumes.end());
                break;
                }
            case SGM::EntityType::VolumeType:
                {
                volume *pVolume=(volume *)this;
                std::set<face *,EntityCompare> const &sFaces=pVolume->GetFaces();
                std::set<edge *,EntityCompare> const &sEdges=pVolume->GetEdges();
                StretchBox(rResult,m_Box,sEdges.begin(),sEdges.end());
                StretchBox(rResult,m_Box,sFaces.begin(),sFaces.end());
                break;
                }
            case SGM::EntityType::FaceType:
                {
                face *pFace=(face *)this;
                switch(pFace->GetSurface()->GetSurfaceType())
                    {
                    case SGM::EntityType::ConeType:
                    case SGM::EntityType::CylinderType:
                    case SGM::EntityType::PlaneType:
                        {
                        // Only use the edge boxes.
                        std::set<edge *,EntityCompare> const &sEdges=pFace->GetEdges();
                        StretchBox(rResult,m_Box,sEdges.begin(),sEdges.end());
                        break;
                        }
                    default:
                        {
                        // Use all the points.
                        std::vector<SGM::Point3D> const &aPoints=pFace->GetPoints3D(rResult);
                        std::vector<unsigned int> const &aTriangles=pFace->GetTriangles(rResult);
                        double dMaxLength=SGM::FindMaxEdgeLength(aPoints,aTriangles);
                        m_Box=SGM::Interval3D(aPoints);
                        m_Box.Extend(sqrt(dMaxLength)*0.08908870145605166538285132205469); // 0.5*tan(15)
                        }
                    }
                break;
                }
            case SGM::EntityType::EdgeType:
                {
                edge *pEdge=(edge *)this;
                std::vector<SGM::Point3D> const &aPoints=pEdge->GetFacets(rResult);
                size_t nPoints=aPoints.size();
                size_t Index1;
                double dMaxLength=0;
                for(Index1=1;Index1<nPoints;++Index1)
                    {
                    double dLength=aPoints[Index1].DistanceSquared(aPoints[Index1-1]);
                    if(dMaxLength<dLength)
                        {
                        dMaxLength=dLength;
                        }
                    }
                m_Box=SGM::Interval3D(aPoints);
                m_Box.Extend(sqrt(dMaxLength)*0.08908870145605166538285132205469); // 0.5*tan(15)
                break;
                }
            case SGM::EntityType::VertexType:
                {
                vertex *pVertex=(vertex *)this;
                m_Box=SGM::Interval3D(pVertex->GetPoint());
                break;
                }
            case SGM::EntityType::ComplexType:
                {
                complex *pComplex=(complex *)this;
                m_Box=SGM::Interval3D(pComplex->GetPoints());
                break;
                }
            default:
                {
                throw;
                }
            }
        }
    return m_Box;
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

void entity::TransformBox(SGM::Transform3D const &Trans)
    {
    m_Box*=Trans;
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
        case SGM::SurfaceType:
            {
            surface const *pSurface=(surface const *)this;
            return pSurface->MakeCopy(rResult);
            }
        case SGM::EdgeType:
            {
            edge const *pEdge=(edge const *)this;
            return pEdge->MakeCopy(rResult);
            }
        case SGM::VertexType:
            {
            vertex const *pVertex=(vertex const *)this;
            return pVertex->MakeCopy(rResult);
            }
        case SGM::VolumeType:
            {
            volume const *pVolume=(volume const *)this;
            return pVolume->MakeCopy(rResult);
            }
        case SGM::BodyType:
            {
            body const *pBody=(body const *)this;
            return pBody->MakeCopy(rResult);
            }
        default:
            {
            throw;
            }
        }
    }

void entity::SeverRelations(SGM::Result &rResult)
    {
    switch(GetType())
        {
        case SGM::BodyType:
            {
            body *pBody=(body *)this;
            std::set<volume *,EntityCompare> sVolumes=pBody->GetVolumes();
            for(volume *pVolume : sVolumes)
                {
                pBody->RemoveVolume(pVolume);
                }
            break;
            }
        case SGM::VolumeType:
            {
            volume *pVolume=(volume *)this;
            if(pVolume->GetBody())
                {
                pVolume->GetBody()->RemoveVolume(pVolume);
                }
            std::set<edge *,EntityCompare> sEdges=pVolume->GetEdges();
            for(edge *pEdge : sEdges)
                {
                pVolume->RemoveEdge(pEdge);
                }
            std::set<face *,EntityCompare> sFaces=pVolume->GetFaces();
            for(face *pFace : sFaces)
                {
                pVolume->RemoveFace(pFace);
                }
            break;
            }
        case SGM::FaceType:
            {
            face *pFace=(face *)this;
            if(pFace->GetVolume())
                {
                pFace->GetVolume()->RemoveFace(pFace);
                }
            std::set<edge *,EntityCompare> sEdges=pFace->GetEdges();
            for(edge *pEdge : sEdges)
                {
                pEdge->RemoveFace(pFace);
                }
            if(pFace->GetSurface())
                {
                pFace->SetSurface(nullptr);
                }
            break;
            }
        case SGM::EdgeType:
            {
            edge *pEdge=(edge *)this;
            std::set<face *,EntityCompare> sFaces=pEdge->GetFaces();
            for(auto pFace : sFaces)
                {
                pFace->RemoveEdge(rResult,pEdge);
                }
            if(pEdge->GetStart())
                {
                pEdge->GetStart()->RemoveEdge(pEdge);
                }
            if(pEdge->GetEnd())
                {
                pEdge->GetEnd()->RemoveEdge(pEdge);
                }
            break;
            }
        case SGM::VertexType:
            {
            vertex *pVertex=(vertex *)this;
            std::set<edge *,EntityCompare> sEdges=pVertex->GetEdges();
            for(auto pEdge : sEdges)
                {
                if(pEdge->GetStart()==pVertex)
                    {
                    pEdge->SetStart(nullptr);
                    }
                if(pEdge->GetEnd()==pVertex)
                    {
                    pEdge->SetEnd(nullptr);
                    }
                }
            break;
            }
        default:
            {
            break;
            }
        }
    for (entity *pOwner : m_sOwners)
        {
        pOwner->RemoveOwner(this);
        }
    }

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
