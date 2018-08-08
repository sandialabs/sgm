#include <cfloat>
#include <cmath>
#include <vector>
#include <algorithm>
#include <list>

#include "SGMVector.h"
#include "SGMInterval.h"
#include "SGMSegment.h"
#include "SGMTransform.h"
#include "SGMIntersector.h"

#include "Intersectors.h"
#include "EntityClasses.h"
#include "FacetToBRep.h"
#include "Topology.h"

namespace SGMInternal
{

size_t OrderAndRemoveDuplicates(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                double                              dTolerance,
                                bool                                bUseWholeLine,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes)
    {
    size_t nAnswer=0;
    size_t Index1;
    if(size_t nAllHits=aPoints.size())
        {
        // Save all the points that occur in the right direction from the ray origin
        // Put them into aParams
        std::vector<std::pair<double,size_t> > aParams;
        aParams.reserve(nAllHits);
        for(Index1=0;Index1<nAllHits;++Index1)
            {
            if(bUseWholeLine || -dTolerance<(aPoints[Index1]-Origin)%Axis)
                {
                double dDist=Origin.Distance(aPoints[Index1]);
                aParams.emplace_back(dDist,Index1);
                }
            }

        // no points were valid
        if ( aParams.empty() )
            {
            aPoints.clear();
            aTypes.clear();
            return 0;    
            }

        // Order them
        std::sort(aParams.begin(),aParams.end());

        // Remove duplicates. They are consecutive.
        size_t nGoodHits=aParams.size();
        std::vector<SGM::Point3D> aTempPoints;
        std::vector<SGM::IntersectionType> aTempTypes;
        aTempPoints.reserve(nGoodHits);
        aTempTypes.reserve(nGoodHits);
        aTempPoints.push_back(aPoints[aParams[0].second]);
        aTempTypes.push_back(aTypes[aParams[0].second]);
        double dLastParam=aParams[0].first;
        ++nAnswer;
        for(Index1=1;Index1<nGoodHits;++Index1)
            {
            if(dTolerance<aParams[Index1].first-dLastParam)
                {
                dLastParam=aParams[Index1].first;
                aTempPoints.push_back(aPoints[aParams[Index1].second]);
                aTempTypes.push_back(aTypes[aParams[Index1].second]);
                ++nAnswer;
                }
            }
        aPoints.swap(aTempPoints);
        aTypes.swap(aTempTypes);
        }
    return nAnswer;
    }

size_t RayFireFace(SGM::Result                        &rResult,
                   SGM::Point3D                 const &Origin,
                   SGM::UnitVector3D            const &Axis,
                   face                         const *pFace,
                   std::vector<SGM::Point3D>          &aPoints,
                   std::vector<SGM::IntersectionType> &aTypes,
                   double                              dTolerance,
                   bool                                bUseWholeLine)
    {
    std::vector<SGM::Point3D> aAllPoints;
    std::vector<SGM::IntersectionType> aAllTypes;
    SGM::Interval1D Domain(-dTolerance,SGM_MAX);
    surface const *pSurface=pFace->GetSurface();
    std::vector<SGM::Point3D> aTempPoints;
    std::vector<SGM::IntersectionType> aTempTypes;
    size_t nHits=IntersectLineAndSurface(rResult,Origin,Axis,Domain,pSurface,dTolerance,aTempPoints,aTempTypes);
    size_t Index1;
    for(Index1=0;Index1<nHits;++Index1)
        {
        SGM::Point3D const &Pos=aTempPoints[Index1];
        SGM::Point2D uv=pSurface->Inverse(Pos);
        if(pFace->PointInFace(rResult,uv))
            {
            aAllPoints.push_back(Pos);
            aAllTypes.push_back(aTempTypes[Index1]);
            }
        }
    
    size_t nAnswer=OrderAndRemoveDuplicates(Origin,Axis,dTolerance,bUseWholeLine,aAllPoints,aAllTypes);
    aPoints=aAllPoints;
    aTypes=aAllTypes;
    return nAnswer;
    }

size_t RayFireVolume(SGM::Result                        &rResult,
                     SGM::Point3D                 const &Origin,
                     SGM::UnitVector3D            const &Axis,
                     volume                       const *pVolume,
                     std::vector<SGM::Point3D>          &aPoints,
                     std::vector<SGM::IntersectionType> &aTypes,
                     double                              dTolerance,
                     bool                                bUseWholeLine)
    {
    // Find all ray hits for all faces.

    SGM::BoxTree const &FaceTree=pVolume->GetFaceTree(rResult);
    SGM::Ray3D Ray(Origin,Axis);
    std::vector<SGM::BoxTree::BoundedItemType> aHitFaces=FaceTree.FindIntersectsRay(Ray,dTolerance);

    std::vector<SGM::BoxTree::BoundedItemType>::iterator FaceIter=aHitFaces.begin();
    std::vector<SGM::Point3D> aAllPoints;
    std::vector<SGM::IntersectionType> aAllTypes;
    size_t Index1;
    while(FaceIter!=aHitFaces.end())
        {
        face *pFace=(face *)(FaceIter->first);
        std::vector<SGM::Point3D> aSubPoints;
        std::vector<SGM::IntersectionType> aSubTypes;
        size_t nHits=RayFireFace(rResult,Origin,Axis,pFace,aSubPoints,aSubTypes,dTolerance,bUseWholeLine);
        for(Index1=0;Index1<nHits;++Index1)
            {
            aAllPoints.push_back(aSubPoints[Index1]);
            aAllTypes.push_back(aSubTypes[Index1]);
            }
        ++FaceIter;
        }

    size_t nAnswer=OrderAndRemoveDuplicates(Origin,Axis,dTolerance,bUseWholeLine,aAllPoints,aAllTypes);
    aPoints=aAllPoints;
    aTypes=aAllTypes;
    return nAnswer;
    }

size_t RayFireBody(SGM::Result                        &rResult,
                   SGM::Point3D                 const &Origin,
                   SGM::UnitVector3D            const &Axis,
                   body                         const *pBody,
                   std::vector<SGM::Point3D>          &aPoints,
                   std::vector<SGM::IntersectionType> &aTypes,
                   double                              dTolerance,
                   bool                                bUseWholeLine)
    {
    // Find all ray hits for all volumes.

    std::set<volume *,EntityCompare> const &sVolumes=pBody->GetVolumes();
    std::set<volume *,EntityCompare>::const_iterator iter=sVolumes.begin();
    std::vector<SGM::Point3D> aAllPoints;
    std::vector<SGM::IntersectionType> aAllTypes;
    size_t Index1;
    while(iter!=sVolumes.end())
        {
        volume *pVolume=*iter;
        std::vector<SGM::Point3D> aSubPoints;
        std::vector<SGM::IntersectionType> aSubTypes;
        size_t nHits=RayFireVolume(rResult,Origin,Axis,pVolume,aSubPoints,aSubTypes,dTolerance,bUseWholeLine);
        for(Index1=0;Index1<nHits;++Index1)
            {
            aAllPoints.push_back(aSubPoints[Index1]);
            aAllTypes.push_back(aSubTypes[Index1]);
            }
        ++iter;
        }

    size_t nAnswer=OrderAndRemoveDuplicates(Origin,Axis,dTolerance,bUseWholeLine,aAllPoints,aAllTypes);
    aPoints=aAllPoints;
    aTypes=aAllTypes;
    return nAnswer;
    }

size_t RayFireThing(SGM::Result                        &rResult,
                    SGM::Point3D                 const &Origin,
                    SGM::UnitVector3D            const &Axis,
                    thing                        const *pThing,
                    std::vector<SGM::Point3D>          &aPoints,
                    std::vector<SGM::IntersectionType> &aTypes,
                    double                              dTolerance,
                    bool                                bUseWholeLine)
    {
    // Find all top level bodies, volumes, and faces.

    std::set<body *,EntityCompare> sBodies;
    FindBodies(rResult,pThing,sBodies,true);
    std::set<body *,EntityCompare>::const_iterator BodyIter=sBodies.begin();
    std::vector<SGM::Point3D> aAllPoints;
    std::vector<SGM::IntersectionType> aAllTypes;
    size_t Index1;
    while(BodyIter!=sBodies.end())
        {
        body *pBody=*BodyIter;
        std::vector<SGM::Point3D> aSubPoints;
        std::vector<SGM::IntersectionType> aSubTypes;
        size_t nHits=RayFireBody(rResult,Origin,Axis,pBody,aSubPoints,aSubTypes,dTolerance,bUseWholeLine);
        for(Index1=0;Index1<nHits;++Index1)
            {
            aAllPoints.push_back(aSubPoints[Index1]);
            aAllTypes.push_back(aSubTypes[Index1]);
            }
        ++BodyIter;
        }
    
    std::set<volume *,EntityCompare> sVolumes;
    FindVolumes(rResult,pThing,sVolumes,true);
    std::set<volume *,EntityCompare>::const_iterator VolumeIter=sVolumes.begin();
    while(VolumeIter!=sVolumes.end())
        {
        volume *pVolume=*VolumeIter;
        std::vector<SGM::Point3D> aSubPoints;
        std::vector<SGM::IntersectionType> aSubTypes;
        size_t nHits=RayFireVolume(rResult,Origin,Axis,pVolume,aSubPoints,aSubTypes,dTolerance,bUseWholeLine);
        for(Index1=0;Index1<nHits;++Index1)
            {
            aAllPoints.push_back(aSubPoints[Index1]);
            aAllTypes.push_back(aSubTypes[Index1]);
            }
        ++VolumeIter;
        }
    
    std::set<face *,EntityCompare> sFaces;
    FindFaces(rResult,pThing,sFaces,true);
    std::set<face *,EntityCompare>::const_iterator FaceIter=sFaces.begin();
    while(FaceIter!=sFaces.end())
        {
        face *pFace=*FaceIter;
        std::vector<SGM::Point3D> aSubPoints;
        std::vector<SGM::IntersectionType> aSubTypes;
        size_t nHits=RayFireFace(rResult,Origin,Axis,pFace,aSubPoints,aSubTypes,dTolerance,bUseWholeLine);
        for(Index1=0;Index1<nHits;++Index1)
            {
            aAllPoints.push_back(aSubPoints[Index1]);
            aAllTypes.push_back(aSubTypes[Index1]);
            }
        ++FaceIter;
        }

    size_t nAnswer=OrderAndRemoveDuplicates(Origin,Axis,dTolerance,bUseWholeLine,aAllPoints,aAllTypes);
    aPoints=aAllPoints;
    aTypes=aAllTypes;
    return nAnswer;
    }

size_t IntersectSegment(SGM::Result               &rResult,
                        SGM::Segment3D      const &Segment,
                        entity              const *pEntity,
                        std::vector<SGM::Point3D> &aPoints,
                        double                     dTolerance)
    {
    std::vector<SGM::IntersectionType> aTypes;
    std::vector<SGM::Point3D> aTempPoints;
    SGM::Point3D Origin=Segment.m_Start;
    SGM::UnitVector3D Axis=Segment.m_End-Origin;
    size_t nHits=RayFire(rResult,Origin,Axis,pEntity,aTempPoints,aTypes,dTolerance);
    double dLengthSquared=Segment.LengthSquared();
    size_t Index1,nAnswer=0;
    for(Index1=0;Index1<nHits;++Index1)
        {
        double dTest=Origin.DistanceSquared(aTempPoints[Index1]);
        if(dTest<dLengthSquared+dTolerance)
            {
            aPoints.push_back(aTempPoints[Index1]);
            ++nAnswer;
            }
        }
    return nAnswer;
    }

size_t RayFire(SGM::Result                        &rResult,
               SGM::Point3D                 const &Origin,
               SGM::UnitVector3D            const &Axis,
               entity                       const *pEntity,
               std::vector<SGM::Point3D>          &aPoints,
               std::vector<SGM::IntersectionType> &aTypes,
               double                              dTolerance,
               bool                                bUseWholeLine)
    {
    switch(pEntity->GetType())
        {
        case SGM::ThingType:
            {
            return RayFireThing(rResult,Origin,Axis,(thing const *)pEntity,aPoints,aTypes,dTolerance,bUseWholeLine);
            }
        case SGM::BodyType:
            {
            return RayFireBody(rResult,Origin,Axis,(body const *)pEntity,aPoints,aTypes,dTolerance,bUseWholeLine);
            }
        case SGM::VolumeType:
            {
            return RayFireVolume(rResult,Origin,Axis,(volume const *)pEntity,aPoints,aTypes,dTolerance,bUseWholeLine);
            }
        case SGM::FaceType:
            {
            return RayFireFace(rResult,Origin,Axis,(face const *)pEntity,aPoints,aTypes,dTolerance,bUseWholeLine);
            }
        default:
            {
            throw;
            break;
            }
        }
    }

void IntersectNonParallelPlanes(SGM::Point3D      const &Origin1,
                                SGM::UnitVector3D const &Normal1,
                                SGM::Point3D      const &Origin2,
                                SGM::UnitVector3D const &Normal2,
                                SGM::Point3D            &Origin,
                                SGM::UnitVector3D       &Axis)
    {
    double AxisX=Normal1.m_y*Normal2.m_z-Normal1.m_z*Normal2.m_y;
    double AxisY=Normal1.m_z*Normal2.m_x-Normal1.m_x*Normal2.m_z;
    double AxisZ=Normal1.m_x*Normal2.m_y-Normal1.m_y*Normal2.m_x;

    double dx=fabs(AxisX);
    double dy=fabs(AxisY);
    double dz=fabs(AxisZ);

    double dLength=sqrt(AxisX*AxisX+AxisY*AxisY+AxisZ*AxisZ);
    Axis.m_x=AxisX/dLength;
    Axis.m_y=AxisY/dLength;
    Axis.m_z=AxisZ/dLength;

    double dPlaneDist1=-Origin1.m_x*Normal1.m_x-Origin1.m_y*Normal1.m_y-Origin1.m_z*Normal1.m_z;
    double dPlaneDist2=-Origin2.m_x*Normal2.m_x-Origin2.m_y*Normal2.m_y-Origin2.m_z*Normal2.m_z;
    if(dy<=dx && dz<=dx)
        {
        Origin.m_x=0.0;
        Origin.m_y=(dPlaneDist2*Normal1.m_z-dPlaneDist1*Normal2.m_z)/AxisX;
        Origin.m_z=(dPlaneDist1*Normal2.m_y-dPlaneDist2*Normal1.m_y)/AxisX;
        }
    else if(dz<dy)
        {
        Origin.m_x=(dPlaneDist1*Normal2.m_z-dPlaneDist2*Normal1.m_z)/AxisY;
        Origin.m_y=0.0;
        Origin.m_z=(dPlaneDist2*Normal1.m_x-dPlaneDist1*Normal2.m_x)/AxisY;
        }
    else
        {
        Origin.m_x=(dPlaneDist2*Normal1.m_y-dPlaneDist1*Normal2.m_y)/AxisZ;
        Origin.m_y=(dPlaneDist1*Normal2.m_x-dPlaneDist2*Normal1.m_x)/AxisZ;
        Origin.m_z=0.0;
        }
    }

size_t IntersectLineAndCylinder(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                SGM::Interval1D              const &Domain,
                                cylinder                     const *pCylinder,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes)
    {
    // IsEmpty, coincident, one tangent point, or two points.

    SGM::Point3D Center=pCylinder->m_Origin;
    SGM::UnitVector3D CylinderAxis=pCylinder->m_ZAxis;
    double dRadius=pCylinder->m_dRadius;
    SGM::Segment3D Seg1(Origin,Origin+Axis);
    SGM::Segment3D Seg2(Center,Center+CylinderAxis);
    SGM::Point3D Pos1,Pos2;
    Seg1.Intersect(Seg2,Pos1,Pos2);
    if(SGM::NearEqual(Pos1,Pos2,dTolerance))
        {
        if(fabs(Axis%CylinderAxis)<1.0-dTolerance)
            {
            SGM::UnitVector3D LineAxis=CylinderAxis*Axis*CylinderAxis;
            SGM::Point3D LineOrigin=Center-((Origin-Center)%CylinderAxis)*CylinderAxis;
            double dCloseT=(Center-LineOrigin)%LineAxis;
            SGM::Point3D Pos=LineOrigin+dCloseT*LineAxis;
            double dDist=Pos.Distance(Center);
            SGM::Point3D TestPos=Origin+Axis;
            SGM::Point3D ProjectedTestPos=TestPos-((TestPos-Center)%CylinderAxis)*CylinderAxis;
            double dScale=LineOrigin.Distance(ProjectedTestPos);
            if(fabs(dDist-dRadius)<dTolerance)
                {
                aPoints.push_back(Origin+(dCloseT*dScale)*Axis);
                aTypes.push_back(SGM::IntersectionType::TangentType);
                }
            else if(dDist<dRadius)
                {
                SGM::Point3D ClosePos=LineOrigin+(dCloseT*dScale)*Axis;
                double t=sqrt(dRadius*dRadius-dDist*dDist);
                aPoints.push_back(ClosePos+(t*dScale)*Axis);
                aTypes.push_back(SGM::IntersectionType::PointType);
                aPoints.push_back(ClosePos-(t*dScale)*Axis);
                aTypes.push_back(SGM::IntersectionType::PointType);
                }
            }
        else 
            {
            SGM::Point3D CylinderPos;
            pCylinder->Inverse(Origin,&CylinderPos);
            if(CylinderPos.DistanceSquared(Origin)<dTolerance*dTolerance)
                {
                aPoints.push_back(Origin+Domain.m_dMin*Axis);
                aTypes.push_back(SGM::IntersectionType::CoincidentType);
                aPoints.push_back(Origin+Domain.m_dMax*Axis);
                aTypes.push_back(SGM::IntersectionType::CoincidentType);
                }
            }
        return aPoints.size();
        }

    return 0;
    }

size_t IntersectLineAndLine(SGM::Point3D                 const &Origin,
                            SGM::UnitVector3D            const &Axis,
                            SGM::Interval1D              const &Domain,
                            line                         const *pLine,
                            double                              dTolerance,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &aTypes)
    {
    SGM::Segment3D Seg1(Origin,Origin+Axis);
    SGM::Point3D const &Origin2=pLine->GetOrigin();
    SGM::UnitVector3D const &Axis2=pLine->GetAxis();
    SGM::Segment3D Seg2(Origin2,Origin2+Axis2);
    SGM::Point3D Pos1,Pos2;
    Seg1.Intersect(Seg2,Pos1,Pos2);
    if(SGM::NearEqual(Pos1,Pos2,dTolerance))
        {
        if(fabs(Axis%Axis2)<1.0-dTolerance)
            {
            aPoints.push_back(SGM::MidPoint(Pos1,Pos2));
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        else
            {
            aPoints.push_back(Origin+Axis*Domain.m_dMin);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            aPoints.push_back(Origin+Axis*Domain.m_dMax);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            }
        }
    return aPoints.size();
    }

size_t IntersectLineAndPlane(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             SGM::Point3D                 const &PlaneOrigin,
                             SGM::UnitVector3D            const &PlaneNorm,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes)
    {
    // IsEmpty, coincident, or one point.

    SGM::Point3D Pos0=Origin+Axis*Domain.m_dMin;
    SGM::Point3D Pos1=Origin+Axis*Domain.m_dMax;
    SGM::Point3D PosMid=Origin+Axis*Domain.MidPoint();
    double dDist0=(Pos0-PlaneOrigin)%PlaneNorm;
    double dDist1=(Pos1-PlaneOrigin)%PlaneNorm;
    double dDistMid=(PosMid-PlaneOrigin)%PlaneNorm;
    if(fabs(dDist0)<dTolerance && fabs(dDist1)<dTolerance && fabs(dDistMid)<dTolerance)
        {
        aPoints.push_back(Pos0);
        aPoints.push_back(Pos1);
        aTypes.push_back(SGM::IntersectionType::CoincidentType);
        aTypes.push_back(SGM::IntersectionType::CoincidentType);
        return 2;
        }
    if (fabs(Axis%PlaneNorm)<dTolerance)
      return 0;
    if(dDist0*dDist1<0)
        {
        double t=-((Origin.m_x-PlaneOrigin.m_x)*PlaneNorm.m_x+
                   (Origin.m_y-PlaneOrigin.m_y)*PlaneNorm.m_y+
                   (Origin.m_z-PlaneOrigin.m_z)*PlaneNorm.m_z)/
            (Axis.m_x*PlaneNorm.m_x+Axis.m_y*PlaneNorm.m_y+Axis.m_z*PlaneNorm.m_z);
        aPoints.push_back(Origin+t*Axis);
        aTypes.push_back(SGM::IntersectionType::PointType);
        return 1;
        }
    else
        {
        if(fabs(dDist0)<dTolerance)
            {
            aPoints.push_back(Pos0);
            aTypes.push_back(SGM::IntersectionType::PointType);
            return 1;
            }
        else if(fabs(dDist1)<dTolerance)
            {
            aPoints.push_back(Pos1);
            aTypes.push_back(SGM::IntersectionType::PointType);
            return 1;
            }
        return 0;
        }
    }

size_t IntersectLineAndPlane(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &Domain,
                             plane                        const *pPlane,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes)
    {
    return IntersectLineAndPlane(Origin,Axis,Domain,pPlane->m_Origin,pPlane->m_ZAxis,
                                 dTolerance,aPoints,aTypes);
    }

size_t IntersectLineAndCircle(SGM::Point3D                 const &Origin,
                              SGM::UnitVector3D            const &Axis,
                              SGM::Interval1D              const &Domain,
                              SGM::Point3D                 const &Center,
                              SGM::UnitVector3D            const &Normal,
                              double                              dRadius,
                              double                              dTolerance,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes)
    {
    std::vector<SGM::Point3D> aPoints2;
    std::vector<SGM::IntersectionType> aTypes2;
    dTolerance=std::max(dTolerance,SGM_ZERO);
    IntersectLineAndPlane(Origin,Axis,Domain,Center,Normal,dTolerance,aPoints2,aTypes2);
    if(aPoints2.size()==2)
        {
        SGM::Point3D Pos=Origin+((Center-Origin)%Axis)*Axis;
        double dDist=Pos.Distance(Center);
        if(dDist<=dRadius-dTolerance)
            {
            double t=sqrt(dRadius*dRadius-dDist*dDist);
            aPoints.push_back(Pos-t*Axis);
            aTypes.push_back(SGM::IntersectionType::PointType);
            aPoints.push_back(Pos+t*Axis);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        else if(fabs(dDist-dRadius)<dTolerance)
            {
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::TangentType);
            }
        }
    else if(aPoints2.size()==1 && SGM::NearEqual(aPoints[0],Center,SGM_ZERO)==false)
        {
        SGM::UnitVector3D UVec=aPoints[0]-Center;
        SGM::Point3D Pos=Center+UVec*dRadius;
        if(Pos.DistanceSquared(aPoints2[0])<dTolerance*dTolerance)
            {
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        }
    return aPoints.size();
    }

size_t IntersectLineAndCircle(SGM::Point3D                 const &Origin,
                              SGM::UnitVector3D            const &Axis,
                              SGM::Interval1D              const &Domain,
                              circle                       const *pCircle,
                              double                              dTolerance,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes)
    {
    SGM::Point3D const &Center=pCircle->m_Center;
    SGM::UnitVector3D const &Normal=pCircle->m_Normal;
    double dRadius=pCircle->m_dRadius;
    return IntersectLineAndCircle(Origin,Axis,Domain,Center,Normal,dRadius,dTolerance,aPoints,aTypes);
    }

size_t IntersectLineAndParabola(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                SGM::Interval1D              const &Domain,
                                parabola                     const *pParabola,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes)
    {
    std::vector<SGM::Point3D> aPoints2;
    std::vector<SGM::IntersectionType> aTypes2;
    IntersectLineAndPlane(Origin,Axis,Domain,pParabola->m_Center,pParabola->m_Normal,dTolerance,aPoints2,aTypes2);
    if(aPoints2.size()==2)
        {
        SGM::Point3D const &Center=pParabola->m_Center;
        double a=pParabola->m_dA;
        SGM::UnitVector3D XVec=pParabola->m_XAxis;
        SGM::UnitVector3D YVec=pParabola->m_YAxis;
        SGM::Vector3D Vec=Origin-Center;
        SGM::Point2D Origin2D(Vec%XVec,Vec%YVec);
        SGM::Vector3D Vec2=(Origin+Axis)-Center;
        SGM::Point2D Pos2D(Vec2%XVec,Vec2%YVec);
        SGM::UnitVector2D Axis2D=Pos2D-Origin2D;

        // Using y=a*x^2
        // x=Origin2D.m_u+Axis2D.m_u*t=c+d*t
        // y=Origin2D.m_v+Axis2D.m_v*t=e+f*t

        double c=Origin2D.m_u;
        double e=Origin2D.m_v;
        double d=Axis2D.m_u;
        double f=Axis2D.m_v;

        // a*(c+d*t)^2-(e+f*t)=0
        // (a*c^2-e) + (2*a*c*d-f)*t + (a*d^2)*t^2

        double A=a*d*d;
        double B=2.0*a*c*d-f;
        double C=a*c*c-e;

        std::vector<double> aRoots;
        size_t nRoots=SGM::Quadratic(A,B,C,aRoots);
        size_t Index1;
        for(Index1=0;Index1<nRoots;++Index1)
            {
            SGM::Point2D Pos=Origin2D+Axis2D*aRoots[Index1];
            SGM::Point3D Pos3D=Center+Pos.m_u*XVec+Pos.m_v*YVec;
            SGM::Point3D CPos;
            double t=pParabola->Inverse(Pos3D,&CPos);
            if(Pos3D.DistanceSquared(CPos)<dTolerance*dTolerance)
                {
                aPoints.push_back(Center+Pos.m_u*XVec+Pos.m_v*YVec);
                if(nRoots==1)
                    {
                    SGM::Vector3D D1;
                    pParabola->Evaluate(t,nullptr,&D1);
                    SGM::UnitVector3D UD1=D1;
                    if(SGM::NearEqual(fabs(UD1%Axis),1.0,dTolerance,false))
                        {
                        aTypes.push_back(SGM::IntersectionType::TangentType);
                        }
                    else
                        {
                        aTypes.push_back(SGM::IntersectionType::PointType);
                        }
                    }
                else
                    {
                    aTypes.push_back(SGM::IntersectionType::PointType);
                    }
                }
            }
        if(nRoots==0)
            {
            SGM::Point2D Pos=Origin2D-Axis2D*(B/(2.0*A));
            SGM::Point3D Pos3D=Center+Pos.m_u*XVec+Pos.m_v*YVec;
            SGM::Point3D CPos;
            pParabola->Inverse(Pos3D,&CPos);
            if(Pos3D.DistanceSquared(CPos)<dTolerance*dTolerance)
                {
                aPoints.push_back(Pos3D);
                aTypes.push_back(SGM::IntersectionType::TangentType);
                }
            }
        }
    else if(aPoints2.size()==1)
        {
        SGM::Point3D Pos;
        pParabola->Inverse(aPoints2[0],&Pos);
        if(Pos.DistanceSquared(aPoints2[0])<dTolerance*dTolerance)
            {
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        }
    return aPoints.size();
    }

size_t IntersectLineAndHyperbola(SGM::Point3D                 const &Origin,
                                 SGM::UnitVector3D            const &Axis,
                                 SGM::Interval1D              const &Domain,
                                 hyperbola                    const *pHyperbola,
                                 double                              dTolerance,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes)
    {
    std::vector<SGM::Point3D> aPoints2;
    std::vector<SGM::IntersectionType> aTypes2;
    IntersectLineAndPlane(Origin,Axis,Domain,pHyperbola->m_Center,pHyperbola->m_Normal,dTolerance,aPoints2,aTypes2);
    if(aPoints2.size()==2)
        {
        SGM::Point3D const &Center=pHyperbola->m_Center;
        double dA=pHyperbola->m_dA;
        double dB=pHyperbola->m_dB;
        SGM::UnitVector3D XVec=pHyperbola->m_XAxis;
        SGM::UnitVector3D YVec=pHyperbola->m_YAxis;
        SGM::Vector3D Vec=Origin-Center;
        SGM::Point2D Origin2D(Vec%XVec,Vec%YVec);
        SGM::Vector3D Vec2=(Origin+Axis)-Center;
        SGM::Point2D Pos2D(Vec2%XVec,Vec2%YVec);
        SGM::UnitVector2D Axis2D=Pos2D-Origin2D;

        // Using y^2/a^2-x^2/b^2=1
        // x=Origin2D.m_u+Axis2D.m_u*t=c+d*t
        // y=Origin2D.m_v+Axis2D.m_v*t=e+f*t

        double c=Origin2D.m_u;
        double e=Origin2D.m_v;
        double d=Axis2D.m_u;
        double f=Axis2D.m_v;
        double raa=1.0/(dA*dA);
        double rbb=1.0/(dB*dB);
        double A=f*f*raa-d*d*rbb;
        double B=2.0*e*f*raa-2*c*d*rbb;
        double C=e*e*raa-c*c*rbb-1.0;

        // (e+f*t)^2/a^2-(c+d*t)^2/b^2-1=0
        // (e^2/a^2 - c^2/b^2 - 1) + t ((2 e f)/a^2 - (2 c d)/b^2) + t^2 (f^2/a^2 - d^2/b^2) 

        std::vector<double> aRoots;
        size_t nRoots=SGM::Quadratic(A,B,C,aRoots);
        size_t Index1;
        for(Index1=0;Index1<nRoots;++Index1)
            {
            SGM::Point2D Pos=Origin2D+Axis2D*aRoots[Index1];
            SGM::Point3D Pos3D=Center+Pos.m_u*XVec+Pos.m_v*YVec;
            SGM::Point3D CPos;
            double t=pHyperbola->Inverse(Pos3D,&CPos);
            if((Pos3D-Center)%YVec > SGM_ZERO)
                {
                aPoints.push_back(Center+Pos.m_u*XVec+Pos.m_v*YVec);
                if(nRoots==1)
                    {
                    SGM::Vector3D D1;
                    pHyperbola->Evaluate(t,nullptr,&D1);
                    SGM::UnitVector3D UD1=D1;
                    if(SGM::NearEqual(fabs(UD1%Axis),1.0,dTolerance,false))
                        {
                        aTypes.push_back(SGM::IntersectionType::TangentType);
                        }
                    else
                        {
                        aTypes.push_back(SGM::IntersectionType::PointType);
                        }
                    }
                else
                    {
                    aTypes.push_back(SGM::IntersectionType::PointType);
                    }
                }
            }
        if(nRoots==0)
            {
            SGM::Point2D Pos=Origin2D-Axis2D*(B/(2.0*A));
            SGM::Point3D Pos3D=Center+Pos.m_u*XVec+Pos.m_v*YVec;
            SGM::Point3D CPos;
            pHyperbola->Inverse(Pos3D,&CPos);
            if(Pos3D.DistanceSquared(CPos)<dTolerance*dTolerance)
                {
                aPoints.push_back(Pos3D);
                aTypes.push_back(SGM::IntersectionType::TangentType);
                }
            }
        }
    else if(aPoints2.size()==1)
        {
        SGM::Point3D Pos;
        pHyperbola->Inverse(aPoints2[0],&Pos);
        if(Pos.DistanceSquared(aPoints2[0])<dTolerance*dTolerance)
            {
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        }
    return aPoints.size();
    }

size_t IntersectLineAndEllipse(SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               ellipse                      const *pEllipse,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes)
    {
    std::vector<SGM::Point3D> aPoints2;
    std::vector<SGM::IntersectionType> aTypes2;
    IntersectLineAndPlane(Origin,Axis,Domain,pEllipse->m_Center,pEllipse->m_Normal,dTolerance,aPoints2,aTypes2);
    if(aPoints2.size()==2)
        {
        SGM::Point3D const &Center=pEllipse->m_Center;
        double dA=pEllipse->m_dA;
        double dB=pEllipse->m_dB;
        SGM::UnitVector3D XVec=pEllipse->m_XAxis;
        SGM::UnitVector3D YVec=pEllipse->m_YAxis;
        SGM::Vector3D Vec=Origin-Center;
        SGM::Point2D Origin2D(Vec%XVec,Vec%YVec);
        SGM::Vector3D Vec2=(Origin+Axis)-Center;
        SGM::Point2D Pos2D(Vec2%XVec,Vec2%YVec);
        SGM::UnitVector2D Axis2D=Pos2D-Origin2D;

        // Using x^2/a^2+y^2/b^2=1
        // x=Origin2D.m_u+Axis2D.m_u*t=c+d*t
        // y=Origin2D.m_v+Axis2D.m_v*t=e+f*t

        double c=Origin2D.m_u;
        double e=Origin2D.m_v;
        double d=Axis2D.m_u;
        double f=Axis2D.m_v;
        double raa=1.0/(dA*dA);
        double rbb=1.0/(dB*dB);
        double A=d*d*raa+f*f*rbb;
        double B=2.0*c*d*raa+2*e*f*rbb;
        double C=c*c*raa+e*e*rbb-1.0;

        // (c+d*t)^2/a^2+(e+f*t)^2/b^2-1=0
        // (c^2/a^2 + e^2/b^2 - 1) + t ((2 c d)/a^2 + (2 e f)/b^2) + t^2 (d^2/a^2 + f^2/b^2) 

        std::vector<double> aRoots;
        size_t nRoots=SGM::Quadratic(A,B,C,aRoots);
        size_t Index1;
        for(Index1=0;Index1<nRoots;++Index1)
            {
            SGM::Point2D Pos=Origin2D+Axis2D*aRoots[Index1];
            aPoints.push_back(Center+Pos.m_u*XVec+Pos.m_v*YVec);
            if(nRoots==1)
                {
                aTypes.push_back(SGM::IntersectionType::TangentType);
                }
            else
                {
                aTypes.push_back(SGM::IntersectionType::PointType);
                }
            }
        if(nRoots==0)
            {
            SGM::Point2D Pos=Origin2D-Axis2D*(B/(2.0*A));
            SGM::Point3D Pos3D=Center+Pos.m_u*XVec+Pos.m_v*YVec;
            SGM::Point3D CPos;
            pEllipse->Inverse(Pos3D,&CPos);
            if(Pos3D.DistanceSquared(CPos)<dTolerance*dTolerance)
                {
                aPoints.push_back(Pos3D);
                aTypes.push_back(SGM::IntersectionType::TangentType);
                }
            }
        }
    else if(aPoints2.size()==1)
        {
        SGM::Point3D Pos;
        pEllipse->Inverse(aPoints2[0],&Pos);
        if(Pos.DistanceSquared(aPoints2[0])<dTolerance*dTolerance)
            {
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        }
    return aPoints.size();
    }

size_t IntersectLineAndNUBCurve(SGM::Point3D                 const &Origin,
                                SGM::UnitVector3D            const &Axis,
                                SGM::Interval1D              const &,//Domain,
                                NUBcurve                     const *pNUBCurve,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes)
    {
    // Find the starting points.

    SGM::Segment3D LineSeg(Origin,Origin+Axis);
    std::vector<SGM::Point3D> const &aSeedPoints=pNUBCurve->GetSeedPoints();
    size_t nSeedPoints=aSeedPoints.size();
    double dTanHalfAngle=SEED_POINT_HALF_ANGLE_TANGENT;
    std::vector<SGM::Point3D> aStartPoints;
    size_t Index1;
    for(Index1=1;Index1<nSeedPoints;++Index1)
        {
        SGM::Point3D const &Pos0=aSeedPoints[Index1-1];
        SGM::Point3D const &Pos1=aSeedPoints[Index1];
        SGM::Segment3D Seg(Pos0,Pos1);
        SGM::Point3D Pos2,Pos3;
        Seg.Intersect(LineSeg,Pos2,Pos3);
        double dLength=Pos0.Distance(Pos1);
        double dTol=dTolerance+dLength*dTanHalfAngle;
        if(Pos2.DistanceSquared(Pos3)<dTol*dTol)
            {
            aStartPoints.push_back(Pos2);
            }
        }

    // Find the intersection points.

    std::vector<std::pair<double,SGM::Point3D> > aRefinedPoints;
    size_t nStartPoints=aStartPoints.size();
    size_t nCountLimit=100;
    SGM::Segment3D Seg1(Origin,Origin+Axis);
    for(Index1=0;Index1<nStartPoints;++Index1)
        {
        SGM::Point3D Pos=aStartPoints[Index1];
        size_t nCount=0;
        double dOldDist=SGM_MAX;
        while(nCount<nCountLimit)
            {
            SGM::Point3D CPos;
            double dNUBt=pNUBCurve->Inverse(Pos);
            SGM::Vector3D LocalTan;
            pNUBCurve->Evaluate(dNUBt,&CPos,&LocalTan);
            SGM::Segment3D Seg2(CPos,CPos+LocalTan);
            SGM::Point3D Pos0,Pos1;
            Seg1.Intersect(Seg2,Pos0,Pos1);
            SGM::Point3D Temp=SGM::MidPoint(Pos0,Pos1);
            double dDist=Temp.Distance(CPos);
            if(dDist<dOldDist)
                {
                Pos=Temp;
                }
            else
                {
                // Newton lead us astray.
                double t=(CPos-Origin)%Axis;
                Pos=Origin+t*Axis;
                dDist=Pos.Distance(CPos);
                }
            double t=(Pos-Origin)%Axis;
            if(dDist<SGM_ZERO || fabs(dDist-dOldDist)<SGM_ZERO)
                {
                aRefinedPoints.emplace_back(t,Pos);
                break;
                }
            if(nCount==nCountLimit-1 && dDist<dTolerance)
                {
                aRefinedPoints.emplace_back(t,Pos);
                break;
                }
            dOldDist=dDist;
            ++nCount;
            }
        }

    // Remove duplicates and find the types.

    if(size_t nRefinedPoints=aRefinedPoints.size())
        {
        double dDuplicatesTolerance=std::max(dTolerance,SGM_MIN_TOL);
        std::sort(aRefinedPoints.begin(),aRefinedPoints.end());
        for(Index1=0;Index1<nRefinedPoints;++Index1)
            {
            SGM::Point3D const &Pos=aRefinedPoints[Index1].second;
            if(Index1==0 || dDuplicatesTolerance<Pos.Distance(aPoints.back()))
                {
                double t=aRefinedPoints[Index1].first;
                SGM::Vector3D DPos;
                pNUBCurve->Evaluate(t,nullptr,&DPos);
                aPoints.push_back(Pos);
                SGM::UnitVector3D Test(DPos);
                if(fabs(1.0-fabs(Test%Axis))<SGM_MIN_TOL)
                    {
                    aTypes.push_back(SGM::IntersectionType::TangentType);
                    }
                else
                    {
                    aTypes.push_back(SGM::IntersectionType::PointType);
                    }
                }
            }
        }

    return aPoints.size();
    }

size_t IntersectLineAndSphere(SGM::Point3D                 const &Origin,
                              SGM::UnitVector3D            const &Axis,
                              SGM::Interval1D              const &,//Domain,
                              sphere                       const *pSphere,
                              double                              dTolerance,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes)
    {
    // IsEmpty, one tangent point or two points.

    SGM::Point3D const &Center=pSphere->m_Center;
    SGM::Point3D Pos=Origin+((Center-Origin)%Axis)*Axis;
    double dDist=Pos.Distance(Center);
    double dRadius=pSphere->m_dRadius;
    if(dDist<=dRadius-dTolerance)
        {
        double t=sqrt(dRadius*dRadius-dDist*dDist);
        aPoints.push_back(Pos-t*Axis);
        aTypes.push_back(SGM::IntersectionType::PointType);
        aPoints.push_back(Pos+t*Axis);
        aTypes.push_back(SGM::IntersectionType::PointType);
        }
    else if(fabs(dDist-dRadius)<dTolerance)
        {
        aPoints.push_back(Pos);
        aTypes.push_back(SGM::IntersectionType::TangentType);
        }
    return aPoints.size();
    }

size_t IntersectLineAndCone(SGM::Point3D                 const &Origin,
                            SGM::UnitVector3D            const &Axis,
                            SGM::Interval1D              const &,//Domain,
                            cone                         const *pCone,
                            double                              dTolerance,
                            std::vector<SGM::Point3D>          &aPoints,
                            std::vector<SGM::IntersectionType> &)//aTypes)
    {
    //  IsEmpty, one point tangent or not, two points.

    // A point is on a cone with apex at the origin and axis the positive 
    // z-axis if and only if x^2+y^2=(cos(half angle)/sin(half angle))^2*z^2.

    SGM::Point3D Center=pCone->m_Origin;
    SGM::UnitVector3D XVec=pCone->m_XAxis;
    SGM::UnitVector3D YVec=pCone->m_YAxis;
    SGM::UnitVector3D ZVec=pCone->m_ZAxis;
    SGM::Transform3D Trans(XVec,YVec,ZVec,SGM::Vector3D(Center));
    SGM::Transform3D Inverse;
    Trans.Inverse(Inverse);

    SGM::Point3D TOrigin=Inverse*Origin;
    SGM::UnitVector3D TAxis=Inverse*Axis;

    double dCosHalfAngle=pCone->m_dCosHalfAngle;
    double dSinHalfAngle=pCone->m_dSinHalfAngle;
    double dCoTan=dCosHalfAngle/dSinHalfAngle;
    double s=dCoTan*dCoTan;

    // x^2+y^2=s*z^2
    //
    // x=a+t*b
    // y=c+t*d
    // z=e+t*f
    //
    // (a+t*b)^2+(c+t*d)^2-s*(e+t*f)^2=0

    double a=TOrigin.m_x;
    double c=TOrigin.m_y;
    double e=TOrigin.m_z;
    double b=TAxis.m_x;
    double d=TAxis.m_y;
    double f=TAxis.m_z;

    double A = d*d-f*f*s;
    double B = 2*a*b+2*c*d-2*e*f*s;
    double C = a*a+c*c-e*e*s;

    std::vector<SGM::Point3D> aHits;
    std::vector<double> aRoots;
    size_t nRoots=SGM::Quadratic(A,B,C,aRoots);
    if(nRoots==0 && SGM_ZERO<fabs(A))
        {
        double x=-B/(2.0*A);
        aHits.push_back(Origin+x*Axis);
        }
    else if(nRoots==1)
        {
        aHits.push_back(Origin+aRoots[0]*Axis);
        }
    else
        {
        aHits.push_back(Origin+aRoots[0]*Axis);
        aHits.push_back(Origin+aRoots[1]*Axis);
        }

    // Check all the hits.

    size_t nHits=aHits.size();
    size_t Index1;
    for(Index1=0;Index1<nHits;++Index1)
        {
        SGM::Point3D Pos=aHits[Index1];
        Pos*=Trans;
        SGM::Point3D CPos;
        pCone->Inverse(Pos,&CPos);
        if(SGM::NearEqual(Pos,CPos,dTolerance))
            {
            aPoints.push_back(CPos);
            }
        }
    return aPoints.size();
    }

size_t IntersectLineAndTorus(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             double                              dMajorRadius,
                             double                              dMinorRadius,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &)//aTypes)
    {
    // IsEmpty, one tangent point, two points, two tangents points, two points and one tangent point,
    // three points one tangent, four points.

    // A point is on a torus centered at the origin, with normal (0,0,1), major radius s and 
    // minor radius r if and only if (x^2+y^2+z^2+s^2-r^2)^2 = 4*s^2(x^2+y^2).

    // Finding x, y and z in terms of the parameter t of the given line we have the following;
    // x=a+t*b
    // y=c+t*d
    // z=e+t*f

    // Hence, the intersection of the line and the torus are at the roots of the following quartic;
    // ((a+t*b)^2+(c+t*d)^2+(e+t*f)^2+s^2-r^2)^2-4*s^2((a+t*b)^2+(c+t*d)^2)=0

    double a=Origin.m_x;
    double c=Origin.m_y;
    double e=Origin.m_z;
    double b=Axis.m_x;
    double d=Axis.m_y;
    double f=Axis.m_z;
    double r=dMinorRadius;
    double s=dMajorRadius;

    double a2=a*a,a3=a2*a,a4=a2*a2;
    double b2=b*b,b3=b2*b,b4=b2*b2;
    double c2=c*c,c3=c2*c,c4=c2*c2;
    double d2=d*d,d3=d2*d,d4=d2*d2;
    double e2=e*e,e3=e2*e,e4=e2*e2;
    double f2=f*f,f3=f2*f,f4=f2*f2;
    double r2=r*r,r4=r2*r2;
    double s2=s*s,s4=s2*s2;

    double E=a4+2*a2*c2+c4+2*a2*e2+2*c2*e2+e4-2*a2*r2-2*c2*r2-2*e2*r2+r4-2*a2*s2-2*c2*s2+2*e2*s2-2*r2*s2+s4;
    double D=4*a3*b+4*a*b*c2+4*a2*c*d+4*c3*d+4*a*b*e2+4*c*d*e2+4*a2*e*f+4*c2*e*f+4*e3*f-4*a*b*r2-4*c*d*r2-4*e*f*r2-4*a*b*s2-4*c*d*s2+4*e*f*s2;
    double C=6*a2*b2+2*b2*c2+8*a*b*c*d+2*a2*d2+6*c2*d2+2*b2*e2+2*d2*e2+8*a*b*e*f+8*c*d*e*f+2*a2*f2+2*c2*f2+6*e2*f2-2*b2*r2-2*d2*r2-2*f2*r2-2*b2*s2-2*d2*s2+2*f2*s2;
    double B=4*a*b3+4*b2*c*d+4*a*b*d2+4*c*d3+4*b2*e*f+4*d2*e*f+4*a*b*f2+4*c*d*f2+4*e*f3;
    double A=b4+2*b2*d2+d4+2*b2*f2+2*d2*f2+f4;

    std::vector<double> aRoots;
    size_t nRoots=SGM::Quartic(A,B,C,D,E,aRoots,dTolerance);
    size_t Index1;
    for(Index1=0;Index1<nRoots;++Index1)
        {
        aPoints.push_back(Origin+aRoots[Index1]*Axis);
        }
    return aPoints.size();
    }

size_t IntersectLineAndTorus(SGM::Point3D                 const &Origin,
                             SGM::UnitVector3D            const &Axis,
                             SGM::Interval1D              const &,//Domain,
                             torus                        const *pTorus,
                             double                              dTolerance,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes)
    {
    // IsEmpty, one tangent point, two points, two tangents points, two points and one tangent point,
    // three points one tangent, four points.

    // Set up the transforms.

    SGM::Point3D Center=pTorus->m_Center;
    SGM::UnitVector3D XVec=pTorus->m_XAxis;
    SGM::UnitVector3D YVec=pTorus->m_YAxis;
    SGM::UnitVector3D ZVec=pTorus->m_ZAxis;
    SGM::Transform3D Trans(XVec,YVec,ZVec,SGM::Vector3D(Center));
    SGM::Transform3D Inverse;
    Trans.Inverse(Inverse);

    SGM::Point3D TOrigin=Inverse*Origin;
    SGM::UnitVector3D TAxis=Inverse*Axis;

    double dMajorRadius=pTorus->m_dMajorRadius;
    double dMinorRadius=pTorus->m_dMinorRadius;
    size_t nHits=IntersectLineAndTorus(TOrigin,TAxis,dMajorRadius,dMinorRadius,dTolerance,aPoints,aTypes);
    size_t Index1;
    for(Index1=0;Index1<nHits;++Index1)
        {
        SGM::Point3D Pos=aPoints[Index1];
        Pos*=Trans;
        SGM::Point3D SnappedPos;
        pTorus->Inverse(Pos,&SnappedPos);
        aPoints[Index1]=SnappedPos;
        }
    return nHits;
    }

size_t IntersectLineAndNUBSurface(SGM::Point3D                 const &Origin,
                                  SGM::UnitVector3D            const &Axis,
                                  SGM::Interval1D              const &Domain,
                                  NUBsurface                   const *pNUBSurface,
                                  double                              dTolerance,
                                  std::vector<SGM::Point3D>          &aPoints,
                                  std::vector<SGM::IntersectionType> &aTypes)
    {
    // Find the starting points.

    SGM::Segment3D LineSeg(Origin,Origin+Axis);
    std::vector<SGM::Point3D> const &aSeedPoints=pNUBSurface->GetSeedPoints();
    std::vector<SGM::Point2D> const &aSeedParams=pNUBSurface->GetSeedParams();
    size_t nUParams=pNUBSurface->GetUParams();
    size_t nVParams=pNUBSurface->GetVParams();
    std::vector<SGM::Point3D> aStartPoints;
    size_t Index1,Index2,Index3;
    for(Index1=0;Index1<nUParams;++Index1)
        {
        size_t nU=Index1*nUParams;
        for(Index2=0;Index2<nVParams;++Index2)
            {
            SGM::Point3D const &PlaneOrigin=aSeedPoints[nU+Index2];
            SGM::Point2D const uv=aSeedParams[nU+Index2];
            SGM::UnitVector3D Norm;
            pNUBSurface->Evaluate(uv,nullptr,nullptr,nullptr,&Norm);
            std::vector<SGM::Point3D> aHits;
            std::vector<SGM::IntersectionType> aHitTypes;
            size_t nHits=IntersectLineAndPlane(Origin,Axis,Domain,PlaneOrigin,Norm,dTolerance,aHits,aHitTypes);
            for(Index3=0;Index3<nHits;++Index3)
                {
                aStartPoints.push_back(aHits[Index3]);
                }
            }
        }

    // Find the intersection points.

    std::vector<std::pair<double,SGM::Point3D> > aRefinedPoints;
    size_t nStartPoints=aStartPoints.size();
    size_t nCountLimit=100;
    for(Index1=0;Index1<nStartPoints;++Index1)
        {
        SGM::Point3D Pos=aStartPoints[Index1];
        size_t nCount=0;
        double dOldDist=SGM_MAX;
        while(nCount<nCountLimit)
            {
            SGM::Point3D CPos;
            SGM::Point2D uv=pNUBSurface->Inverse(Pos);
            SGM::UnitVector3D LocalNorm;
            pNUBSurface->Evaluate(uv,&CPos,nullptr,nullptr,&LocalNorm);
            std::vector<SGM::Point3D> aTemp;
            std::vector<SGM::IntersectionType> aTempType;
            if(1==SGMInternal::IntersectLineAndPlane(Origin,Axis,Domain,CPos,LocalNorm,SGM_MIN_TOL,aTemp,aTempType))
                {
                double dDist=aTemp[0].Distance(CPos);
                if(dDist<dOldDist)
                    {
                    Pos=aTemp[0];
                    }
                else
                    {
                    // Newton lead us astray.
                    double t=(CPos-Origin)%Axis;
                    Pos=Origin+t*Axis;
                    dDist=Pos.Distance(CPos);
                    }
                double t=(Pos-Origin)%Axis;
                if(dDist<SGM_ZERO || fabs(dDist-dOldDist)<SGM_ZERO)
                    {
                    aRefinedPoints.emplace_back(t,Pos);
                    break;
                    }
                if(nCount==nCountLimit-1 && dDist<dTolerance)
                    {
                    aRefinedPoints.emplace_back(t,Pos);
                    break;
                    }
                dOldDist=dDist;
                }
            else
                {
                break;
                }
            ++nCount;
            }
        }

    // Remove duplicates and find the types.

    if(size_t nRefinedPoints=aRefinedPoints.size())
        {
        double dDuplicatesTolerance=std::max(dTolerance,SGM_MIN_TOL);
        std::sort(aRefinedPoints.begin(),aRefinedPoints.end());
        for(Index1=0;Index1<nRefinedPoints;++Index1)
            {
            SGM::Point3D const &Pos=aRefinedPoints[Index1].second;
            SGM::Point2D uv=pNUBSurface->Inverse(Pos);
            if(Index1==0 || dDuplicatesTolerance<Pos.Distance(aPoints.back()))
                {
                SGM::UnitVector3D Norm;
                pNUBSurface->Evaluate(uv,nullptr,nullptr,nullptr,&Norm);
                aPoints.push_back(Pos);
                if(fabs(Norm%Axis)<dDuplicatesTolerance)
                    {
                    aTypes.push_back(SGM::IntersectionType::TangentType);
                    }
                else
                    {
                    aTypes.push_back(SGM::IntersectionType::PointType);
                    }
                }
            }
        }

    return aPoints.size();
    }

size_t IntersectLineAndNURBSurface(SGM::Point3D                 const &Origin,
                                   SGM::UnitVector3D            const &Axis,
                                   SGM::Interval1D              const &Domain,
                                   NURBsurface                  const *pNURBSurface,
                                   double                              dTolerance,
                                   std::vector<SGM::Point3D>          &aPoints,
                                   std::vector<SGM::IntersectionType> &aTypes)
    {
    // Find the starting points.

    SGM::Segment3D LineSeg(Origin,Origin+Axis);
    std::vector<SGM::Point3D> const &aSeedPoints=pNURBSurface->GetSeedPoints();
    std::vector<SGM::Point2D> const &aSeedParams=pNURBSurface->GetSeedParams();
    size_t nUParams=pNURBSurface->GetUParams();
    size_t nVParams=pNURBSurface->GetVParams();
    std::vector<SGM::Point3D> aStartPoints;
    size_t Index1,Index2,Index3;
    for(Index1=0;Index1<nUParams;++Index1)
        {
        size_t nU=Index1*nUParams;
        for(Index2=0;Index2<nVParams;++Index2)
            {
            SGM::Point3D const &PlaneOrigin=aSeedPoints[nU+Index2];
            SGM::Point2D const uv=aSeedParams[nU+Index2];
            SGM::UnitVector3D Norm;
            pNURBSurface->Evaluate(uv,nullptr,nullptr,nullptr,&Norm);
            std::vector<SGM::Point3D> aHits;
            std::vector<SGM::IntersectionType> aHitTypes;
            size_t nHits=IntersectLineAndPlane(Origin,Axis,Domain,PlaneOrigin,Norm,dTolerance,aHits,aHitTypes);
            for(Index3=0;Index3<nHits;++Index3)
                {
                aStartPoints.push_back(aHits[Index3]);
                }
            }
        }

    // Find the intersection points.

    std::vector<std::pair<double,SGM::Point3D> > aRefinedPoints;
    size_t nStartPoints=aStartPoints.size();
    size_t nCountLimit=100;
    for(Index1=0;Index1<nStartPoints;++Index1)
        {
        SGM::Point3D Pos=aStartPoints[Index1];
        size_t nCount=0;
        double dOldDist=SGM_MAX;
        while(nCount<nCountLimit)
            {
            SGM::Point3D CPos;
            SGM::Point2D uv=pNURBSurface->Inverse(Pos);
            SGM::UnitVector3D LocalNorm;
            pNURBSurface->Evaluate(uv,&CPos,nullptr,nullptr,&LocalNorm);
            std::vector<SGM::Point3D> aTemp;
            std::vector<SGM::IntersectionType> aTempType;
            if(1==SGMInternal::IntersectLineAndPlane(Origin,Axis,Domain,CPos,LocalNorm,SGM_MIN_TOL,aTemp,aTempType))
                {
                double dDist=aTemp[0].Distance(CPos);
                if(dDist<dOldDist)
                    {
                    Pos=aTemp[0];
                    }
                else
                    {
                    // Newton lead us astray.
                    double t=(CPos-Origin)%Axis;
                    Pos=Origin+t*Axis;
                    dDist=Pos.Distance(CPos);
                    }
                double t=(Pos-Origin)%Axis;
                if(dDist<SGM_ZERO || fabs(dDist-dOldDist)<SGM_ZERO)
                    {
                    aRefinedPoints.emplace_back(t,Pos);
                    break;
                    }
                if(nCount==nCountLimit-1 && dDist<dTolerance)
                    {
                    aRefinedPoints.emplace_back(t,Pos);
                    break;
                    }
                dOldDist=dDist;
                }
            else
                {
                break;
                }
            ++nCount;
            }
        }

    // Remove duplicates and find the types.

    if(size_t nRefinedPoints=aRefinedPoints.size())
        {
        double dDuplicatesTolerance=std::max(dTolerance,SGM_MIN_TOL);
        std::sort(aRefinedPoints.begin(),aRefinedPoints.end());
        for(Index1=0;Index1<nRefinedPoints;++Index1)
            {
            SGM::Point3D const &Pos=aRefinedPoints[Index1].second;
            SGM::Point2D uv=pNURBSurface->Inverse(Pos);
            if(Index1==0 || dDuplicatesTolerance<Pos.Distance(aPoints.back()))
                {
                SGM::UnitVector3D Norm;
                pNURBSurface->Evaluate(uv,nullptr,nullptr,nullptr,&Norm);
                aPoints.push_back(Pos);
                if(fabs(Norm%Axis)<dDuplicatesTolerance)
                    {
                    aTypes.push_back(SGM::IntersectionType::TangentType);
                    }
                else
                    {
                    aTypes.push_back(SGM::IntersectionType::PointType);
                    }
                }
            }
        }

    return aPoints.size();
    }

size_t IntersectLineAndRevolve(SGM::Result                        &rResult,
                               SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Direction,
                               SGM::Interval1D              const &Domain,
                               revolve                      const *pRevolve,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes)
    {
    if (Direction % pRevolve->m_ZAxis < SGM_ZERO)
        {
        // find closest points on axis and line
        SGM::Segment3D RevolveAxisSeg(pRevolve->m_Origin, pRevolve->m_Origin + pRevolve->m_ZAxis);
        SGM::Segment3D LineSeg(Origin, Origin + Direction);
        SGM::Point3D PosAxis, PosLine;
        RevolveAxisSeg.Intersect(LineSeg,PosAxis,PosLine);

        // intersect curve with plane
        std::vector<SGM::Point3D> aCurvePlanePoints;
        std::vector<SGM::IntersectionType> aCurvePlaneTypes;
        IntersectCurveAndPlane(rResult, pRevolve->m_pCurve, PosAxis,pRevolve->m_ZAxis, aCurvePlanePoints, aCurvePlaneTypes, dTolerance);

        // if plane does not intersect we're done
        if (aCurvePlanePoints.empty())
            return 0;

        // intersect the line with the circle defined by each curve and plane intersection point
        for (SGM::Point3D Pos : aCurvePlanePoints)
            {
            double dRadius = PosAxis.Distance(Pos);
            IntersectLineAndCircle(Origin, Direction, Domain, PosAxis, pRevolve->m_ZAxis, dRadius, dTolerance, aPoints, aTypes);
            }
        }
    else
        {
          throw;
        }
    return aPoints.size();
    }

size_t IntersectLineAndSurface(SGM::Result                        &rResult,
                               SGM::Point3D                 const &Origin,
                               SGM::UnitVector3D            const &Axis,
                               SGM::Interval1D              const &Domain,
                               surface                      const *pSurface,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes)
    {
    switch(pSurface->GetSurfaceType())
        {
        case SGM::EntityType::CylinderType:
            {
            return IntersectLineAndCylinder(Origin,Axis,Domain,(cylinder const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::PlaneType:
            {
            return IntersectLineAndPlane(Origin,Axis,Domain,(plane const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::SphereType:
            {
            return IntersectLineAndSphere(Origin,Axis,Domain,(sphere const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::TorusType:
            {
            return IntersectLineAndTorus(Origin,Axis,Domain,(torus const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::NUBSurfaceType:
            {
            return IntersectLineAndNUBSurface(Origin,Axis,Domain,(NUBsurface const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::NURBSurfaceType:
            {
            return IntersectLineAndNURBSurface(Origin,Axis,Domain,(NURBsurface const *)pSurface,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::RevolveType:
            {
            return IntersectLineAndRevolve(rResult,Origin,Axis,Domain,(revolve const *)pSurface,dTolerance,aPoints,aTypes);
            }
        default:
            {
            throw;
            }
        }
    }

size_t IntersectLineAndSurface(SGM::Result                        &rResult,
                               line                         const *pLine,
                               surface                      const *pSurface,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes,
                               edge                         const *pEdge,
                               face                         const *pFace)
    {
    std::vector<SGM::Point3D> aTempPoints;
    std::vector<SGM::IntersectionType> aTempTypes;
    //size_t nSize=
        IntersectLineAndSurface(rResult,pLine->m_Origin,pLine->m_Axis,pLine->GetDomain(),
                                           pSurface,dTolerance,aTempPoints,aTempTypes);
    if(pEdge)
        {

        }
    if(pFace)
        {
        
        }
    aPoints=aTempPoints;
    aTypes=aTempTypes;
    return aPoints.size();
    }

size_t IntersectCircleAndPlane(SGM::Point3D                 const &Center,
                               SGM::UnitVector3D            const &Normal,
                               double                              dRadius,
                               SGM::Point3D                 const &PlaneOrigin,
                               SGM::UnitVector3D            const &PlaneNormal,
                               double                              dTolerance,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes)
    {
    if(SGM::NearEqual(fabs(Normal%PlaneNormal),1.0,SGM_MIN_TOL,false))
        {
        if(fabs((Center-PlaneOrigin)%PlaneNormal)<dTolerance)
            {
            SGM::UnitVector3D XAxis=Normal.Orthogonal();
            SGM::Point3D CirclePos=Center+XAxis*dRadius;
            aPoints.push_back(CirclePos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            aPoints.push_back(CirclePos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            }
        }
    else
        {
        SGM::Point3D Origin;
        SGM::UnitVector3D Axis;
        IntersectNonParallelPlanes(Center,Normal,PlaneOrigin,PlaneNormal,Origin,Axis);
        SGM::Point3D Pos=Origin+((Center-Origin)%Axis)*Axis;
        double dDist=Pos.Distance(Center);
        if(dDist<=dRadius-dTolerance)
            {
            double t=sqrt(dRadius*dRadius-dDist*dDist);
            aPoints.push_back(Pos-t*Axis);
            aTypes.push_back(SGM::IntersectionType::PointType);
            aPoints.push_back(Pos+t*Axis);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        else if(fabs(dDist-dRadius)<dTolerance)
            {
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::TangentType);
            }
        }
    return aPoints.size();
    }

size_t IntersectCircleAndCylinder(SGM::Point3D                 const &/*Center*/,
                                  SGM::UnitVector3D            const &/*Normal*/,
                                  double                              /*dRadius*/,
                                  cylinder                     const * /*pCylinder*/,
                                  double                              /*dTolerance*/,
                                  std::vector<SGM::Point3D>          &/*aPoints*/,
                                  std::vector<SGM::IntersectionType> &/*aTypes*/)
    {
    // Intersect the circle's plane and the cylinder, then
    // intersect the line(s), circle, or ellipse with the circle.
    return 0;
    }

size_t IntersectCircleAndSurface(SGM::Point3D                 const &Center,
                                 SGM::UnitVector3D            const &Normal,
                                 double                              dRadius,
                                 surface                      const *pSurface,
                                 double                              dTolerance,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes)
    {
    switch(pSurface->GetSurfaceType())
        {
        case SGM::EntityType::PlaneType:
            {
            plane const *pPlane=(plane const *)pSurface;
            IntersectCircleAndPlane(Center,Normal,dRadius,pPlane->m_Origin,pPlane->m_ZAxis,dTolerance,aPoints,aTypes);
            }
        case SGM::EntityType::CylinderType:
            {
            cylinder const *pCylinder=(cylinder const *)pSurface;
            IntersectCircleAndCylinder(Center,Normal,dRadius,pCylinder,dTolerance,aPoints,aTypes);
            }
        default:
            {
            
            }
        }
    return aPoints.size();
    }

size_t IntersectNUBCurveAndSurface(SGM::Result                        &rResult,
                                   NUBcurve                     const *pNUBcurve,
                                   surface                      const *pSurface,
                                   std::vector<SGM::Point3D>          &aPoints,
                                   std::vector<SGM::IntersectionType> &aTypes,
                                   double                              dTolerance)
    {
    switch(pSurface->GetSurfaceType())
        {
        case SGM::PlaneType:
            {
            plane const *pPlane =(plane const *)pSurface; 
            IntersectNUBCurveAndPlane(rResult, pNUBcurve, pPlane->m_Origin, pPlane->m_ZAxis, aPoints, aTypes, dTolerance);
            break;
            }
        default:
            throw;
        }
    return aPoints.size();
    }

size_t IntersectCurveAndSurface(SGM::Result                        &rResult,
                                curve                        const *pCurve,
                                surface                      const *pSurface,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes,
                                edge                         const *pEdge,
                                face                         const *pFace,
                                double                              dTolerance)
    {
    switch(pCurve->GetCurveType())
        {
        case SGM::LineType:
            {
            double dTol=pEdge ? pEdge->GetTolerance() : SGM_MIN_TOL;
            dTol=std::max(dTol,dTolerance);
            IntersectLineAndSurface(rResult,(line const *)pCurve,pSurface,dTol,aPoints,aTypes,pEdge,pFace);
            break;
            }
        case SGM::CircleType:
            {
            circle const *pCircle=(circle const *)pCurve;
            SGM::Point3D const &Center=pCircle->m_Center;
            SGM::UnitVector3D const &Normal=pCircle->m_Normal;
            double dRadius=pCircle->m_dRadius;
            IntersectCircleAndSurface(Center,Normal,dRadius,pSurface,dTolerance,aPoints,aTypes);
            break;
            }
        case SGM::EllipseType:
            {
            break;
            }
        case SGM::ParabolaType:
            {
            break;
            }
        case SGM::HyperbolaType:
            {
            break;
            }
        case SGM::NUBCurveType:
            {
            NUBcurve const *pNUB = (NUBcurve const *)pCurve;
            IntersectNUBCurveAndSurface(rResult, pNUB, pSurface, aPoints, aTypes, dTolerance);
            break;
            }
        case SGM::NURBCurveType:
            {
            break;
            }
        case SGM::PointCurveType:
            {
            PointCurve const *pPointCurve=(PointCurve const *)pCurve;
            SGM::Point3D const &Pos=pPointCurve->m_Pos;
            SGM::Point3D CPos;
            pSurface->Inverse(Pos,&CPos);
            if(Pos.Distance(CPos)<dTolerance)
                {
                aPoints.push_back(CPos);
                aTypes.push_back(SGM::IntersectionType::CoincidentType);
                }
            break;
            }
        default:
            {
            rResult.SetResult(SGM::ResultType::ResultTypeUnknownType);
            }
        }
    return aPoints.size();
    }

 size_t IntersectLineAndCurve(SGM::Result                        &rResult,
                              SGM::Point3D                 const &Origin,
                              SGM::UnitVector3D            const &Axis,
                              SGM::Interval1D              const &Domain,
                              curve                        const *pCurve,
                              double                              dTolerance,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes)
     {
     switch(pCurve->GetCurveType())
         {
         case SGM::LineType:
             {
             return IntersectLineAndLine(Origin,Axis,Domain,(line const *)pCurve,dTolerance,aPoints,aTypes);
             }
         case SGM::CircleType:
             {
             return IntersectLineAndCircle(Origin,Axis,Domain,(circle const *)pCurve,dTolerance,aPoints,aTypes);
             }
         case SGM::EllipseType:
             {
             return IntersectLineAndEllipse(Origin,Axis,Domain,(ellipse const *)pCurve,dTolerance,aPoints,aTypes);
             }
         case SGM::ParabolaType:
             {
             return IntersectLineAndParabola(Origin,Axis,Domain,(parabola const *)pCurve,dTolerance,aPoints,aTypes);
             }
         case SGM::HyperbolaType:
             {
             return IntersectLineAndHyperbola(Origin,Axis,Domain,(hyperbola const *)pCurve,dTolerance,aPoints,aTypes);
             }
         case SGM::NUBCurveType:
             {
             return IntersectLineAndNUBCurve(Origin,Axis,Domain,(NUBcurve const *)pCurve,dTolerance,aPoints,aTypes);
             }
         case SGM::NURBCurveType:
             {
             break;
             }
         case SGM::PointCurveType:
             {
             break;
             }
         default:
             {
             rResult.SetResult(SGM::ResultType::ResultTypeUnknownType);
             }
         }
     return aPoints.size();
     }

size_t IntersectCircleAndCircle(SGM::Result                        &,//rResult,
                                circle                       const *pCircle1,
                                circle                       const *pCircle2,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes,
                                double                              dTolerance)
    {
    SGM::Point3D const &Center1=pCircle1->m_Center;
    SGM::Point3D const &Center2=pCircle2->m_Center;
    SGM::UnitVector3D const &Norm1=pCircle1->m_Normal;
    SGM::UnitVector3D const &Norm2=pCircle2->m_Normal;
    SGM::Vector3D Vec=Center1-Center2;
    double dDist1=Vec%Norm1;
    double dDist2=Vec%Norm2;
    if(fabs(dDist1)<dTolerance && dDist2<dTolerance && SGM::NearEqual(1.0,fabs(Norm1%Norm2),dTolerance,false))
        {
        // Circle1 and Circle2 are in the same plane.

        double dR1=pCircle1->m_dRadius;
        double dR2=pCircle2->m_dRadius;
        double dDist=Center1.Distance(Center2);
        if(SGM::NearEqual(dR1+dR2,dDist,dTolerance,false))
            {
            double dB=(dR1*dR1-dR2*dR2)/(2*dDist);
            SGM::UnitVector3D Norm=Center1-Center2;
            aPoints.push_back(Center2+Norm*dB);
            aTypes.push_back(SGM::IntersectionType::TangentType);
            }
        else if(dDist<dR1+dR2)
            {
            double dR2Squared=dR2*dR2;
            double dB=(dR2Squared-dR1*dR1)/(2*dDist);
            SGM::UnitVector3D Norm=Center1-Center2;
            double dRadius=sqrt(dR2Squared-dB*dB);
            SGM::Point3D Pos=Center2+Norm*dB;
            SGM::UnitVector3D Offset=Norm*Norm1;
            aPoints.push_back(Pos+Offset*dRadius);
            aTypes.push_back(SGM::IntersectionType::PointType);
            aPoints.push_back(Pos-Offset*dRadius);
            aTypes.push_back(SGM::IntersectionType::PointType);
            }
        }
    else
        {
        std::vector<SGM::Point3D> aHits;
        std::vector<SGM::IntersectionType> aTemp;
        size_t nHits=IntersectCircleAndPlane(Center1,Norm1,pCircle1->m_dRadius,Center2,Norm2,dTolerance,aHits,aTemp);
        size_t Index1;
        for(Index1=0;Index1<nHits;++Index1)
            {
            SGM::Point3D const &Hit=aHits[Index1];
            SGM::Point3D CPos;
            pCircle1->Inverse(Hit,&CPos);
            if(Hit.Distance(CPos)<dTolerance)
                {
                aPoints.push_back(CPos);
                aTypes.push_back(SGM::IntersectionType::PointType);
                }
            }
        }
    return aPoints.size();
    }

size_t IntersectCircleAndCurve(SGM::Result                        &rResult,
                               circle                       const *pCircle,
                               curve                        const *pCurve,
                               std::vector<SGM::Point3D>          &aPoints,
                               std::vector<SGM::IntersectionType> &aTypes,
                               edge                         const *,//pEdge1,
                               edge                         const *pEdge2,
                               double                              dTolerance)
    {
    switch(pCurve->GetCurveType())
        {
        case SGM::LineType:
            {
            line const *pLine=(line const *)pCurve;
            SGM::Interval1D Domain=pCurve->GetDomain();
            if(pEdge2)
                {
                Domain&=pEdge2->GetDomain();
                }
            SGM::Point3D const &Origin=pLine->m_Origin;
            SGM::UnitVector3D const &Axis=pLine->m_Axis;
            return IntersectLineAndCircle(Origin,Axis,Domain,(circle const *)pCurve,dTolerance,aPoints,aTypes);
            }
        case SGM::CircleType:
            {
            return IntersectCircleAndCircle(rResult,pCircle,(circle const *)pCurve,aPoints,aTypes,dTolerance);
            }
        default:
            {
            throw;
            }
        }
    }

 size_t IntersectCurves(SGM::Result                        &rResult,
                        curve                        const *pCurve1,
                        curve                        const *pCurve2,
                        std::vector<SGM::Point3D>          &aPoints,
                        std::vector<SGM::IntersectionType> &aTypes,
                        edge                         const *pEdge1,
                        edge                         const *pEdge2,
                        double                              dTolerance)
      {
      double dTol1=pEdge1 ? pEdge1->GetTolerance() : SGM_MIN_TOL;
      double dTol2=pEdge1 ? pEdge2->GetTolerance() : SGM_MIN_TOL;
      double dTol=dTol1+dTol2+dTolerance;

      switch(pCurve1->GetCurveType())
         {
         case SGM::LineType:
             {
             line const *pLine=(line const *)pCurve1;
             SGM::Interval1D Domain=pCurve1->GetDomain();
             if(pEdge1)
                 {
                 Domain&=pEdge1->GetDomain();
                 }
             IntersectLineAndCurve(rResult,pLine->GetOrigin(),pLine->GetAxis(),Domain,pCurve2,dTol,aPoints,aTypes);
             break;
             }
         case SGM::CircleType:
             {
             circle const *pCircle=(circle const *)pCurve1;
             IntersectCircleAndCurve(rResult,pCircle,pCurve2,aPoints,aTypes,pEdge1,pEdge2,dTol);
             break;
             }
         case SGM::EllipseType:
             {
             break;
             }
         case SGM::ParabolaType:
             {
             break;
             }
         case SGM::HyperbolaType:
             {
             break;
             }
         case SGM::NUBCurveType:
             {
             break;
             }
         case SGM::NURBCurveType:
             {
             break;
             }
         case SGM::PointCurveType:
             {
             PointCurve const *pPointCurve=(PointCurve const *)pCurve2;
             SGM::Point3D const &Pos=pPointCurve->m_Pos;
             SGM::Point3D CPos;
             pCurve1->Inverse(Pos,&CPos);
             if(Pos.Distance(CPos)<dTolerance)
                 {
                 aPoints.push_back(CPos);
                 aTypes.push_back(SGM::IntersectionType::CoincidentType);
                 }
             break;
             }
         default:
             {
             rResult.SetResult(SGM::ResultType::ResultTypeUnknownType);
             }
         }
      return aPoints.size();
      }

 curve *FindConicCurve(SGM::Result             &rResult,
                       SGM::Point3D      const &Pos,
                       SGM::UnitVector3D const &Norm,
                       cone              const *pCone)
     {
     SGM::Interval1D Domain(-SGM_MAX,SGM_MAX);
     std::vector<SGM::Point3D> aPoints;
     std::vector<SGM::IntersectionType> aTypes;
     SGM::UnitVector3D Axis1=Norm.Orthogonal();
     IntersectLineAndCone(Pos,Axis1,Domain,pCone,SGM_MIN_TOL,aPoints,aTypes);
     SGM::UnitVector3D Axis2=Axis1*Norm;
     IntersectLineAndCone(Pos,Axis2,Domain,pCone,SGM_MIN_TOL,aPoints,aTypes);
     IntersectLineAndCone(Pos,Axis1+Axis2,Domain,pCone,SGM_MIN_TOL,aPoints,aTypes);
     IntersectLineAndCone(Pos,Axis1+Axis2,Domain,pCone,SGM_MIN_TOL,aPoints,aTypes);
     IntersectLineAndCone(Pos,Axis1-Axis2,Domain,pCone,SGM_MIN_TOL,aPoints,aTypes);
     return FindConic(rResult,aPoints,SGM_MIN_TOL);
     }

 size_t IntersectPlaneAndCone(SGM::Result                        &rResult,
                           plane                        const *pPlane,
                           cone                         const *pCone,
                           std::vector<curve *>               &aCurves,
                           face                         const *,//pFace1,
                           face                         const *,//pFace2,
                           double                              )//dTolerance)
     {
     SGM::Point3D Apex=pCone->FindApex();
     SGM::UnitVector3D const &Axis=pCone->m_ZAxis;
     SGM::UnitVector3D const &Norm=pPlane->m_ZAxis;
     SGM::Point3D const &Origin=pPlane->m_Origin;
     if(fabs((Apex-Origin)%Norm)<SGM_MIN_TOL)
         {
         // One or two line intersection.
         std::vector<SGM::Point3D> aPoints;
         std::vector<SGM::IntersectionType> aTypes;
         size_t nRoots=IntersectCircleAndPlane(pCone->m_Origin,Axis,
             pCone->m_dRadius,pPlane->m_Origin,pPlane->m_ZAxis,SGM_MIN_TOL,aPoints,aTypes);
         size_t Index1;
         for(Index1=0;Index1<nRoots;++Index1)
             {
             SGM::UnitVector3D LineAxis=aPoints[Index1]-Apex;
             aCurves.push_back(new line(rResult,aPoints[Index1],LineAxis,1.0));
             }
         if(nRoots==0)
             {
             // Single point intersection.
             aCurves.push_back(new PointCurve(rResult,Apex));
             }
         }
     else
         {
         double dSin=pCone->m_dSinHalfAngle;
         double dDot=fabs(Axis%Norm);
         if(dSin+SGM_MIN_TOL<dDot)
             {
             std::vector<SGM::Point3D> aPoints;
             std::vector<SGM::IntersectionType> aTypes;
             SGM::Interval1D Domain(-SGM_MAX,SGM_MAX);
             IntersectLineAndPlane(Apex,Axis,Domain,Origin,Norm,SGM_ZERO,aPoints,aTypes);
             double h=(aPoints[0]-Apex)%Axis;
             if(h<0.0)
                 {
                 if(SGM::NearEqual(dDot,1.0,SGM_MIN_TOL,false))
                     {
                     // Circle intersection.
                     double dRadius=-h*(pCone->m_dCosHalfAngle)/dSin;
                     aCurves.push_back(new circle(rResult,aPoints[0],Axis,dRadius,&pCone->m_XAxis));
                     }
                 else
                     {
                     // Ellipse intersection.
                     aCurves.push_back(FindConicCurve(rResult,aPoints[0],Norm,pCone));
                     }
                 }
             }
         else if(dSin<dDot+SGM_MIN_TOL)
             {
             std::vector<SGM::Point3D> aPoints;
             std::vector<SGM::IntersectionType> aTypes;
             SGM::Interval1D Domain(-SGM_MAX,SGM_MAX);
             IntersectLineAndPlane(Apex,Axis,Domain,Origin,Norm,SGM_ZERO,aPoints,aTypes);
             if((aPoints[0]-Apex)%Axis<0)
                 {
                 // Parabola intersecton.
                 aCurves.push_back(FindConicCurve(rResult,aPoints[0],Norm,pCone));
                 }
             }
         else
             {
             // Hyperbola intersection.
             SGM::UnitVector3D Vec=Norm*Axis*Norm;
             if(0.0<Vec%Axis)
                 {
                 Vec.Negate();
                 }
             std::vector<SGM::Point3D> aPoints;
             std::vector<SGM::IntersectionType> aTypes;
             SGM::Interval1D Domain(-SGM_MAX,SGM_MAX);
             IntersectLineAndCone(Origin,Vec,Domain,pCone,SGM_ZERO,aPoints,aTypes);
             aCurves.push_back(FindConicCurve(rResult,aPoints[0]+Vec,Norm,pCone));
             }
         }
     return aCurves.size();
     }

size_t IntersectPlaneAndPlane(SGM::Result                &rResult,
                              plane                const *pPlane1,
                              plane                const *pPlane2,
                              std::vector<curve *>       &aCurves,
                              face                 const *,//pFace1,
                              face                 const *,//pFace2,
                              double                      dTolerance)
     {
     SGM::UnitVector3D const &Norm1=pPlane1->m_ZAxis;
     SGM::UnitVector3D const &Norm2=pPlane2->m_ZAxis;
     if(fabs(Norm1%Norm2)<1.0-dTolerance)
         {
         SGM::Point3D const &Origin1=pPlane1->m_Origin;
         SGM::Point3D const &Origin2=pPlane2->m_Origin;
         SGM::Point3D Origin;
         SGM::UnitVector3D Axis;
         IntersectNonParallelPlanes(Origin1,Norm1,Origin2,Norm2,Origin,Axis);
         aCurves.push_back(new line(rResult,Origin,Axis,1.0));
         }
     return aCurves.size();
     }

size_t IntersectPlaneAndSphere(SGM::Result                &rResult,
                               plane                const *pPlane,
                               sphere               const *pSphere,
                               std::vector<curve *>       &aCurves,
                               face                 const *,//pFace1,
                               face                 const *,//pFace2,
                               double                      dTolerance)
    {
    SGM::Point3D const &Center=pSphere->m_Center;
    SGM::Point3D const &Origin=pPlane->m_Origin;
    SGM::UnitVector3D const &Norm=pPlane->m_ZAxis;
    double dRadius=pSphere->m_dRadius;
    double dDist=(Center-Origin)%Norm;
    double dFABSDist=fabs(dDist);
    if(dFABSDist<dRadius+dTolerance)
        {
        if(dFABSDist<dRadius-dTolerance)
            {
            SGM::Point3D CircleCenter=Center-dDist*Norm;
            aCurves.push_back(new circle(rResult,CircleCenter,Norm,dRadius));
            }
        else
            {
            aCurves.push_back(new PointCurve(rResult,Center-dRadius*Norm));
            }
        }
    return aCurves.size();
    }

size_t IntersectPlaneAndCylinder(SGM::Result                &rResult,
                                 plane                const *pPlane,
                                 cylinder             const *pCylinder,
                                 std::vector<curve *>       &aCurves,
                                 face                 const *,//pFace1,
                                 face                 const *,//pFace2,
                                 double                      dTolerance)
    {
    // Return one or two lines, a circle, an ellipse, or nothing.

    double dRadius=pCylinder->m_dRadius;
    SGM::Point3D const &Center=pCylinder->m_Origin;
    SGM::Point3D const &Origin=pPlane->m_Origin;
    SGM::UnitVector3D const &Norm=pPlane->m_ZAxis;
    double dDist=fabs((Center-Origin)%Norm);
    if(SGM::NearEqual(dRadius,dDist,dTolerance,false))
        {
        // One line.
        SGM::Point3D Pos=Center-Norm*((Center-Origin)%Norm);
        aCurves.push_back(new line(rResult,Pos,pCylinder->m_ZAxis,1.0));
        }
    if(dDist<dRadius)
        {
        SGM::UnitVector3D const &Axis=pCylinder->m_ZAxis;
        double dFABSDot=fabs(Norm%Axis);
        if(dFABSDot<dTolerance)
            {
            // Two lines.
            SGM::Point3D Pos=Center-Norm*((Center-Origin)%Norm);
            SGM::UnitVector3D UVec=Norm*pCylinder->m_ZAxis;
            double dH=sqrt(dRadius*dRadius-dDist*dDist);
            SGM::Point3D Pos0=Pos+UVec*dH;
            SGM::Point3D Pos1=Pos-UVec*dH;
            aCurves.push_back(new line(rResult,Pos0,pCylinder->m_ZAxis,1.0));
            aCurves.push_back(new line(rResult,Pos1,pCylinder->m_ZAxis,1.0));
            }
        else if(SGM::NearEqual(dFABSDot,1.0,dTolerance,false))
            {
            // Circle.
            SGM::Point3D CircleCenter=Center+(pCylinder->m_ZAxis)*((Origin-Center)%pCylinder->m_ZAxis);
            aCurves.push_back(new circle(rResult,CircleCenter,pCylinder->m_ZAxis,dRadius,&(pCylinder->m_XAxis)));
            }
        else
            {
            // Ellipse.
            SGM::UnitVector3D Minor=Norm*pCylinder->m_ZAxis;
            SGM::UnitVector3D Major=Minor*Norm;
            SGM::Interval1D Domain(-SGM_MAX,SGM_MAX);
            std::vector<SGM::Point3D> aPoints;
            std::vector<SGM::IntersectionType> aTypes;
            IntersectLineAndPlane(Center,pCylinder->m_ZAxis,Domain,Origin,Norm,dTolerance,aPoints,aTypes);
            SGM::Point3D EllipseCenter=aPoints[0];
            double dA=dRadius/dFABSDot;
            aCurves.push_back(new ellipse(rResult,EllipseCenter,Major,Minor,dA,dRadius));
            }
        }
    return aCurves.size();
    }

size_t IntersectPlaneAndTorus(SGM::Result                & /*rResult*/,
                              plane                const * /*pPlane*/,
                              torus                const * /*pTorus*/,
                              std::vector<curve *>       & /*aCurves*/,
                              face                 const * /*pFace1*/,
                              face                 const * /*pFace2*/,
                              double                       /*dTolerance*/)
    {
    return 0;
    }

size_t IntersectPlaneAndSurface(SGM::Result                &rResult,
                                plane                const *pPlane,
                                surface              const *pSurface,
                                std::vector<curve *>       &aCurves,
                                face                 const *pFace1,
                                face                 const *pFace2,
                                double                      dTolerance)
     {
     switch(pSurface->GetSurfaceType())
        {
        case SGM::EntityType::PlaneType:
            {
            plane const *pPlane2=(plane const *)pSurface;
            return IntersectPlaneAndPlane(rResult,pPlane,pPlane2,aCurves,pFace1,pFace2,dTolerance);
            }
        case SGM::EntityType::SphereType:
            {
            sphere const *pSphere=(sphere const *)pSurface;
            return IntersectPlaneAndSphere(rResult,pPlane,pSphere,aCurves,pFace1,pFace2,dTolerance);
            }
        case SGM::EntityType::CylinderType:
            {
            cylinder const *pCylinder=(cylinder const *)pSurface;
            return IntersectPlaneAndCylinder(rResult,pPlane,pCylinder,aCurves,pFace1,pFace2,dTolerance);
            }
        case SGM::EntityType::ConeType:
            {
            cone const *pCone=(cone const *)pSurface;
            return IntersectPlaneAndCone(rResult,pPlane,pCone,aCurves,pFace1,pFace2,dTolerance);
            }
        case SGM::EntityType::TorusType:
            {
            torus const *pTorus=(torus const *)pSurface;
            return IntersectPlaneAndTorus(rResult,pPlane,pTorus,aCurves,pFace1,pFace2,dTolerance);
            }
        case SGM::EntityType::RevolveType:
            {
            throw;
            }
        default:
            {
            return IntersectPlaneAndSurface(rResult,pPlane,pSurface,aCurves,pFace1,pFace2,dTolerance);
            }
        }
     }

size_t IntersectSphereAndSphere(SGM::Result                &rResult,
                             sphere               const *pSphere1,
                             sphere               const *pSphere2,
                             std::vector<curve *>       &aCurves,
                             face                 const *,//pFace1,
                             face                 const *,//pFace2,
                             double                      dTolerance)
    {
    double dR1=pSphere1->m_dRadius;
    double dR2=pSphere2->m_dRadius;
    SGM::Point3D const &Center1=pSphere1->m_Center;
    SGM::Point3D const &Center2=pSphere2->m_Center;
    double dDist=Center1.Distance(Center2);
    if(SGM::NearEqual(dR1+dR2,dDist,dTolerance,false))
        {
        double dB=(dR1*dR1-dR2*dR2)/(2*dDist);
        SGM::UnitVector3D Norm=Center1-Center2;
        SGM::Interval1D Domain(0,SGM_TWO_PI);
        aCurves.push_back(new PointCurve(rResult,Center2+Norm*dB,&Domain));
        }
    else if(dDist<dR1+dR2)
        {
        double dR2Squared=dR2*dR2;
        double dB=(dR2Squared-dR1*dR1)/(2*dDist);
        SGM::UnitVector3D Norm=Center1-Center2;
        double dRadius=sqrt(dR2Squared-dB*dB);
        aCurves.push_back(new circle(rResult,Center2+Norm*dB,Norm,dRadius));
        }
    return aCurves.size();
    }

size_t IntersectSphereAndCylinder(SGM::Result                &rResult,
                                  sphere               const *pSphere,
                                  cylinder             const *pCylinder,
                                  std::vector<curve *>       &aCurves,
                                  face                 const *,//pFace1,
                                  face                 const *,//pFace2,
                                  double                      dTolerance)
    {
    double dSphereRadius=pSphere->m_dRadius;
    double dCylinderRadius=pCylinder->m_dRadius;
    SGM::Point3D const &Center=pSphere->m_Center;
    SGM::Point3D const &Origin=pCylinder->m_Origin;
    SGM::UnitVector3D const &Axis=pCylinder->m_ZAxis;
    SGM::Point3D AxisPos=Origin+Axis*((Center-Origin)%Axis);
    double dDist=Center.Distance(AxisPos);
    if(SGM::NearEqual(AxisPos,Center,dTolerance))
        {
        // Center on Axis
    
        SGM::UnitVector3D const &XAxis=pCylinder->m_XAxis;
        if(SGM::NearEqual(dSphereRadius,dCylinderRadius,dTolerance,false))
            {
            // One circle.
            aCurves.push_back(new circle(rResult,Center,Axis,dSphereRadius,&XAxis));
            }
        else if(dCylinderRadius<dSphereRadius)
            {
            // Two circles.
            double dOffset=sqrt(dSphereRadius*dSphereRadius-dCylinderRadius*dCylinderRadius);
            SGM::Vector3D Offset=Axis*dOffset;
            aCurves.push_back(new circle(rResult,Center+Offset,Axis,dCylinderRadius,&XAxis));
            aCurves.push_back(new circle(rResult,Center-Offset,Axis,dCylinderRadius,&XAxis));
            }
        }
    else if(SGM::NearEqual(dDist,dCylinderRadius+dSphereRadius,dTolerance,false))
        {
        // Sphere outside cylinder and tangent.
        SGM::Point3D Pos=SGM::MidPoint(AxisPos,Center,dCylinderRadius/dDist);
        aCurves.push_back(new PointCurve(rResult,Pos));
        }
    else if( dSphereRadius<dCylinderRadius &&
        SGM::NearEqual(dDist,dCylinderRadius-dSphereRadius,dTolerance,false))
        {
        // Smaller radius sphere is inside cylinder and tangent.
        SGM::UnitVector3D UVec=Center-AxisPos;
        SGM::Point3D Pos=AxisPos+UVec*dCylinderRadius;
        aCurves.push_back(new PointCurve(rResult,Pos));
        }
    else if(dDist<dSphereRadius)
        {
        // Cylinder axis intersects sphere.

        SGM::Point3D LineOrigin;
        pCylinder->Inverse(Center,&LineOrigin);
        SGM::UnitVector3D Normal=Axis*(Center-AxisPos);
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        IntersectLineAndCircle(LineOrigin,Axis,pCylinder->GetDomain().m_VDomain,
                               Center,Normal,dSphereRadius,dTolerance,aPoints,aTypes);

        if(SGM::NearEqual(dDist+dCylinderRadius,dSphereRadius,dTolerance,false))
            {
            // Larger radius sphere is tangent and spanning cylinder,
            // forming a bent figure eight intersection.

            SGM::UnitVector3D UVec=AxisPos-Center;
            SGM::Point3D TanPos=AxisPos+UVec*dCylinderRadius;
            hermite *pCurve0A=WalkFromTo(rResult,aPoints[0],TanPos,pSphere,pCylinder);
            hermite *pCurve0B=WalkFromTo(rResult,aPoints[0],TanPos,pCylinder,pSphere);
            pCurve0A->Negate();
            pCurve0A->Concatenate(pCurve0B);
            aCurves.push_back(pCurve0A);
            rResult.GetThing()->DeleteEntity(pCurve0B);

            hermite *pCurve1A=WalkFromTo(rResult,aPoints[1],TanPos,pSphere,pCylinder);
            hermite *pCurve1B=WalkFromTo(rResult,aPoints[1],TanPos,pCylinder,pSphere);
            pCurve1A->Negate();
            pCurve1A->Concatenate(pCurve1B);
            aCurves.push_back(pCurve1A);
            rResult.GetThing()->DeleteEntity(pCurve1B);
            }
        else if(dDist+dCylinderRadius<dSphereRadius)
            {
            // Larger radius sphere is intected with whole section of cylinder,
            // forming two bent closed curves.

            aCurves.push_back(WalkFromTo(rResult,aPoints[0],aPoints[0],pSphere,pCylinder));
            aCurves.push_back(WalkFromTo(rResult,aPoints[1],aPoints[1],pSphere,pCylinder));
            }
        }
    else 
        {
        // If the closest point on the cylinder to the sphere center is
        // inside the sphere then the sphere intersects but does not cut
        // the cylinder into two parts, resuling in a bent closed curve.

        SGM::Point3D CPos;
        pCylinder->Inverse(Center,&CPos);
        double dDist2=Center.Distance(CPos);
        if(dDist2<dSphereRadius)
            {
            // One closed curve.

            SGM::Point3D LineOrigin;
            pCylinder->Inverse(Center,&LineOrigin);
            SGM::UnitVector3D Normal=Axis*(Center-AxisPos);
            std::vector<SGM::Point3D> aPoints;
            std::vector<SGM::IntersectionType> aTypes;
            IntersectLineAndCircle(LineOrigin,Axis,pCylinder->GetDomain().m_VDomain,
                                   Center,Normal,dSphereRadius,dTolerance,aPoints,aTypes);
            aCurves.push_back(WalkFromTo(rResult,aPoints[0],aPoints[0],pSphere,pCylinder));
            }
        }
    return aCurves.size();
    }

size_t IntersectSphereAndSurface(SGM::Result                &rResult,
                                 sphere               const *pSphere,
                                 surface              const *pSurface,
                                 std::vector<curve *>       &aCurves,
                                 face                 const *pFace1,
                                 face                 const *pFace2,
                                 double                      dTolerance)
    {
    switch(pSurface->GetSurfaceType())
        {
        case SGM::EntityType::PlaneType:
            {
            plane const *pPlane=(plane const *)pSurface;
            return IntersectPlaneAndSphere(rResult,pPlane,pSphere,aCurves,pFace2,pFace1,dTolerance);
            }
        case SGM::EntityType::SphereType:
            {
            sphere const *pSphere2=(sphere const *)pSurface;
            return IntersectSphereAndSphere(rResult,pSphere,pSphere2,aCurves,pFace1,pFace2,dTolerance);
            }
        case SGM::EntityType::CylinderType:
            {
            cylinder const *pCylinder=(cylinder const *)pSurface;
            return IntersectSphereAndCylinder(rResult,pSphere,pCylinder,aCurves,pFace1,pFace2,dTolerance);
            }
        default:
            {
            return IntersectSphereAndSurface(rResult,pSphere,pSurface,aCurves,pFace1,pFace2,dTolerance);
            }
        }
    }


size_t IntersectCylinderAndSurface(SGM::Result                &rResult,
                                   cylinder             const *pCylinder,
                                   surface              const *pSurface,
                                   std::vector<curve *>       &aCurves,
                                   face                 const *pFace1,
                                   face                 const *pFace2,
                                   double                      dTolerance)
    {
    switch(pSurface->GetSurfaceType())
        {
        case SGM::EntityType::SphereType:
            {
            sphere const *pSphere=(sphere const *)pSurface;
            return IntersectSphereAndCylinder(rResult,pSphere,pCylinder,aCurves,pFace1,pFace2,dTolerance);
            }
        default:
            {
            throw;
            }
        }
    }

size_t IntersectSurfaces(SGM::Result                &rResult,
                         surface              const *pSurface1,
                         surface              const *pSurface2,
                         std::vector<curve *>       &aCurves,
                         face                 const *pFace1,
                         face                 const *pFace2,
                         double                      dTolerance)
    {
    switch(pSurface1->GetSurfaceType())
        {
        case SGM::EntityType::PlaneType:
            {
            plane const *pPlane=(plane const *)pSurface1;
            return IntersectPlaneAndSurface(rResult,pPlane,pSurface2,aCurves,pFace1,pFace2,dTolerance);
            }
        case SGM::EntityType::SphereType:
            {
            sphere const *pSphere=(sphere const *)pSurface1;
            return IntersectSphereAndSurface(rResult,pSphere,pSurface2,aCurves,pFace1,pFace2,dTolerance);
            }
        case SGM::EntityType::CylinderType:
            {
            cylinder const *pCylinder=(cylinder const *)pSurface1;
            return IntersectCylinderAndSurface(rResult,pCylinder,pSurface2,aCurves,pFace1,pFace2,dTolerance);
            }
        default:
            {
            throw;
            break;
            }
        }
    }

SGM::Point3D ZoomInFrom(SGM::Point3D const &Pos,
                        surface      const *pSurface1,
                        surface      const *pSurface2)
    {
    SGM::Point3D Answer=Pos;
    double dDist=SGM_MAX;
    size_t nCount=0;
    while(SGM_ZERO<dDist && nCount<100)
        {
        SGM::Point3D OldAnswer=Answer;
        SGM::Point3D CPos;
        pSurface1->Inverse(Answer,&CPos);
        pSurface2->Inverse(CPos,&Answer);
        dDist=OldAnswer.Distance(Answer);
        ++nCount;
        }
    return Answer;
    }

class HermiteNode
    {
    public:

        HermiteNode() = default;

        HermiteNode(double               dParam,
                    SGM::Point3D  const &Pos,
                    SGM::Vector3D const &Tan):m_dParam(dParam),m_Pos(Pos),m_Tan(Tan){}

        double        m_dParam;
        SGM::Point3D  m_Pos;
        SGM::Vector3D m_Tan;
    };

bool MidPointIsOff(HermiteNode const &iter1,
                   HermiteNode const &iter2,
                   surface     const *pSurface1,
                   surface     const *pSurface2,
                   double             dLength,
                   HermiteNode       &HNode)
    {
    SGM::Point3D Pos1=iter1.m_Pos;
    SGM::Point3D Pos2=iter2.m_Pos;
    SGM::Vector3D Vec1=iter1.m_Tan;
    SGM::Vector3D Vec2=iter2.m_Tan;
    double t1=iter1.m_dParam;
    double t2=iter2.m_dParam;
    double t3=(t1+t2)*0.5;
    double s=(t3-t1)/(t2-t1);
    double h1=(s*s)*(2*s-3)+1;
    double h2=1-h1;
    double h3=s*(s*(s-2)+1);
    double h4=(s*s)*(s-1);
    SGM::Point3D MidPos(h1*Pos1.m_x+h2*Pos2.m_x+h3*Vec1.m_x+h4*Vec2.m_x,
                        h1*Pos1.m_y+h2*Pos2.m_y+h3*Vec1.m_y+h4*Vec2.m_y,
                        h1*Pos1.m_z+h2*Pos2.m_z+h3*Vec1.m_z+h4*Vec2.m_z);

    SGM::Point3D ExactMidPos=ZoomInFrom(MidPos,pSurface1,pSurface2);
    double dDist2=ExactMidPos.DistanceSquared(MidPos);
    double dTol2=SGM_FIT*dLength;
    dTol2*=dTol2;
    bool bAnswer=false;
    if(SGM_FIT_SMALL<dDist2)
        {
        SGM::Point2D uv1=pSurface1->Inverse(ExactMidPos);
        SGM::Point2D uv2=pSurface2->Inverse(ExactMidPos);
        SGM::UnitVector3D Norm1,Norm2;
        pSurface1->Evaluate(uv1,nullptr,nullptr,nullptr,&Norm1);
        pSurface2->Evaluate(uv2,nullptr,nullptr,nullptr,&Norm2);
        SGM::UnitVector3D WalkDir=Norm1*Norm2;
        HNode.m_dParam=t3;
        HNode.m_Pos=ExactMidPos;
        HNode.m_Tan=WalkDir*((Vec1.Magnitude()+Vec2.Magnitude())*0.5);
        bAnswer=true;
        }
    return bAnswer;
    }

hermite *WalkFromTo(SGM::Result        &rResult,
                    SGM::Point3D const &StartPos,
                    SGM::Point3D const &EndPos,
                    surface      const *pSurface1,
                    surface      const *pSurface2)
    {
    std::vector<SGM::Point3D> aPoints;
    std::vector<SGM::Vector3D> aTangents;
    std::vector<double> aParams;
    SGM::Point3D CurrentPos=StartPos;
    SGM::Point2D uv1=pSurface1->Inverse(CurrentPos);
    SGM::Point2D uv2=pSurface2->Inverse(CurrentPos);
    SGM::UnitVector3D Norm1,Norm2;
    pSurface1->Evaluate(uv1,nullptr,nullptr,nullptr,&Norm1);
    pSurface2->Evaluate(uv2,nullptr,nullptr,nullptr,&Norm2);
    SGM::UnitVector3D WalkDir=Norm1*Norm2;
    double dWalkFraction=0.5; 
    bool bWalk=true;
    double dScale=0.5;
    while(bWalk)
        {
        // Find how far to walk.

        double c1=pSurface1->DirectionalCurvature(uv1,WalkDir);
        double c2=pSurface2->DirectionalCurvature(uv2,WalkDir);
        double dMax=std::max(fabs(c1),fabs(c2));
        double dR=dMax<SGM_ZERO ? SGM_MAX : 1.0/dMax;
        double dWalkDist=dR*dWalkFraction;
        SGM::Point3D Pos=CurrentPos+WalkDir*dWalkDist;
        aPoints.push_back(CurrentPos);
        aTangents.push_back(WalkDir*dScale);

        // Find the walking direction.

        CurrentPos=ZoomInFrom(Pos,pSurface1,pSurface2);
        SGM::Point2D uv1_walk=pSurface1->Inverse(CurrentPos);
        SGM::Point2D uv2_walk=pSurface2->Inverse(CurrentPos);
        pSurface1->Evaluate(uv1_walk,nullptr,nullptr,nullptr,&Norm1);
        pSurface2->Evaluate(uv2_walk,nullptr,nullptr,nullptr,&Norm2);
        WalkDir=Norm1*Norm2;

        // Check to see if we are at the EndPos.
        
        double dEndDist=EndPos.Distance(CurrentPos);
        SGM::UnitVector3D UVec=EndPos-CurrentPos;
        if(dEndDist<dWalkDist && 0<UVec%WalkDir)
            {
            aPoints.push_back(EndPos);
            if(SGM::NearEqual(StartPos,EndPos,SGM_ZERO))
                {
                aTangents.push_back(aTangents.front());
                }
            else
                {
                uv1_walk=pSurface1->Inverse(EndPos);
                uv2_walk=pSurface2->Inverse(EndPos);
                pSurface1->Evaluate(uv1_walk,nullptr,nullptr,nullptr,&Norm1);
                pSurface2->Evaluate(uv2_walk,nullptr,nullptr,nullptr,&Norm2);
                SGM::Vector3D Vec=Norm1*Norm2;
                if(Vec.Magnitude()<SGM_MIN_TOL)
                    {
                    SGM::Point3D StepBack=SGM::MidPoint(aPoints[aPoints.size()-2],EndPos,0.99);
                    uv1_walk=pSurface1->Inverse(StepBack);
                    uv2_walk=pSurface2->Inverse(StepBack);
                    pSurface1->Evaluate(uv1_walk,nullptr,nullptr,nullptr,&Norm1);
                    pSurface2->Evaluate(uv2_walk,nullptr,nullptr,nullptr,&Norm2);
                    Vec=Norm1*Norm2;
                    }
                Vec=SGM::UnitVector3D(Vec);
                aTangents.push_back(Vec*dScale);
                }
            break;
            }
        }

    // Refine points to meet SGM_FIT times the cord length of the curve tolerance.

    SGM::FindLengths3D(aPoints,aParams);
    std::list<HermiteNode> lNodes;
    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
        {
        lNodes.emplace_back(aParams[Index1],aPoints[Index1],aTangents[Index1]);
        }
    double dLength=aParams.back();
    std::list<HermiteNode>::iterator iter=lNodes.begin();
    std::list<HermiteNode>::iterator LastIter=iter;
    ++iter;
    while(iter!=lNodes.end())
        {
        HermiteNode HNode;
        if(MidPointIsOff(*LastIter,*iter,pSurface1,pSurface2,dLength,HNode))
            {
            iter=lNodes.insert(iter,HNode);
            }
        else
            {
            ++LastIter;
            ++iter;
            }
        }

    // Create the Hermite curve.

    aPoints.clear();
    aTangents.clear();
    aParams.clear();
    iter=lNodes.begin();
    while(iter!=lNodes.end())
        {
        aPoints.push_back(iter->m_Pos);
        aParams.push_back(iter->m_dParam);
        aTangents.push_back(iter->m_Tan);
        ++iter;
        }

    return new hermite(rResult,aPoints,aTangents,aParams);
    }

void IntersectThreeSurfaces(SGM::Result               &rResult,
                            surface             const *pSurface1,
                            surface             const *pSurface2,
                            surface             const *pSurface3,
                            std::vector<SGM::Point3D> &aPoints)
    {
    std::vector<curve *> aCurves;
    size_t nCurves=IntersectSurfaces(rResult,pSurface1,pSurface2,aCurves,nullptr,nullptr,SGM_MIN_TOL);
    size_t Index1;
    for(Index1=0;Index1<nCurves;++Index1)
        {
        curve *pCurve=aCurves[Index1];
        std::vector<SGM::IntersectionType> aTypes;
        IntersectCurveAndSurface(rResult,pCurve,pSurface3,aPoints,aTypes,nullptr,nullptr,SGM_MIN_TOL);
        rResult.GetThing()->DeleteEntity(pCurve);
        }
    }

size_t IntersectCurveAndPlane(SGM::Result                        &rResult,
                              curve                        const *pCurve,
                              plane                        const *pPlane,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes,
                              double                              dTolerance)
{
    return IntersectCurveAndPlane(rResult, pCurve, pPlane->m_Origin, pPlane->m_ZAxis, aPoints, aTypes, dTolerance);
}

size_t IntersectCurveAndPlane(SGM::Result                        &rResult,
                              curve                        const *pCurve,
                              SGM::Point3D                 const &PlaneOrigin,
                              SGM::UnitVector3D            const &PlaneNorm,
                              std::vector<SGM::Point3D>          &aPoints,
                              std::vector<SGM::IntersectionType> &aTypes,
                              double                              dTolerance)
{
    switch(pCurve->GetCurveType())
        {
        case SGM::LineType:
            {
            line const *pLine = (line const *)pCurve;
            IntersectLineAndPlane(pLine->m_Origin, pLine->m_Axis, SGM::Interval1D(-SGM_MAX, SGM_MAX),
                                  PlaneOrigin, PlaneNorm, dTolerance, aPoints, aTypes);
            break;
            }
        case SGM::CircleType:
            {
            circle const *pCircle=(circle const *)pCurve;
            IntersectCircleAndPlane(pCircle->m_Center, pCircle->m_Normal, pCircle->m_dRadius, 
                                    PlaneOrigin, PlaneNorm, dTolerance, aPoints, aTypes);
            break;
            }
        case SGM::EllipseType:
            {
            ellipse const *pEllipse=(ellipse const *)pCurve;
            IntersectEllipseAndPlane(pEllipse, PlaneOrigin, PlaneNorm, dTolerance, aPoints, aTypes);
            break;
            }
        case SGM::ParabolaType:
            {
            parabola const *pParabola = (parabola const *)pCurve;
            IntersectParabolaAndPlane(pParabola, PlaneOrigin, PlaneNorm, dTolerance, aPoints, aTypes);
            break;
            }
        case SGM::HyperbolaType:
            {
            hyperbola const *pHyperbola = (hyperbola const *)pCurve;
            IntersectHyperbolaAndPlane(pHyperbola, PlaneOrigin, PlaneNorm, dTolerance, aPoints, aTypes);
            break;
            }
        case SGM::NUBCurveType:
            {
            NUBcurve const *pNUBCurve = (NUBcurve const *)pCurve;
            IntersectNUBCurveAndPlane(rResult, pNUBCurve, PlaneOrigin, PlaneNorm, aPoints, aTypes, dTolerance);
            break;
            }
        case SGM::NURBCurveType:
            {
            break;
            }
        case SGM::PointCurveType:
            {
            //PointCurve const *pPointCurve=(PointCurve const *)pCurve;
            //SGM::Point3D const &Pos=pPointCurve->m_Pos;
            //SGM::Point3D CPos;
            //pSurface->Inverse(Pos,&CPos);
            //if(Pos.Distance(CPos)<dTolerance)
            //    {
            //    aPoints.push_back(CPos);
            //    aTypes.push_back(SGM::IntersectionType::CoincidentType);
            //    }
            break;
            }
        default:
            {
            rResult.SetResult(SGM::ResultType::ResultTypeUnknownType);
            }
        }
     return aPoints.size();
}

size_t IntersectEdgeAndPlane(SGM::Result                        &rResult,
                             edge                         const *pEdge,
                             SGM::Point3D                 const &PlaneOrigin,
                             SGM::UnitVector3D            const &PlaneNorm,
                             std::vector<SGM::Point3D>          &aPoints,
                             std::vector<SGM::IntersectionType> &aTypes,
                             double                              dTolerance)
    {
    curve const *pCurve=pEdge->GetCurve();
    std::vector<SGM::Point3D> aTempPoints;
    std::vector<SGM::IntersectionType> aTempTypes;
    size_t nHits=IntersectCurveAndPlane(rResult,pCurve,PlaneOrigin,PlaneNorm,aTempPoints,aTempTypes,dTolerance);
    size_t nAnswer=0;
    size_t Index1;
    for(Index1=0;Index1<nHits;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        if(pEdge->PointInEdge(Pos,dTolerance))
            {
            aPoints.push_back(Pos);
            aTypes.push_back(aTempTypes[Index1]);
            ++nAnswer;
            }
        }
    return nAnswer;
    }

size_t IntersectNUBCurveAndPlane(SGM::Result                        &,//rResult,
                                 NUBcurve                     const *pCurve,
                                 SGM::Point3D                 const &PlaneOrigin,
                                 SGM::UnitVector3D            const &PlaneNorm,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes,
                                 double                              dTolerance)
{
    SGM::Point3D      CurvePlaneOrigin;
    SGM::UnitVector3D CurvePlaneNormal;

    if (SGM::ArePointsCoplanar(pCurve->GetControlPoints(), dTolerance, &CurvePlaneOrigin, &CurvePlaneNormal))
    {
        if(SGM::NearEqual(fabs(CurvePlaneNormal%PlaneNorm),1.0,SGM_MIN_TOL,false)) // planes are parallel
        {
            if(fabs((CurvePlaneOrigin-PlaneOrigin)%PlaneNorm)<dTolerance)
            {
                SGM::Point3D StartPos;
                SGM::Point3D EndPos;
                pCurve->Evaluate(pCurve->GetDomain().m_dMin, &StartPos);
                pCurve->Evaluate(pCurve->GetDomain().m_dMax, &EndPos);
                aPoints.push_back(StartPos);
                aTypes.push_back(SGM::IntersectionType::CoincidentType);
                aPoints.push_back(EndPos);
                aTypes.push_back(SGM::IntersectionType::CoincidentType);
            }
        }
        else
        {
            SGM::Point3D LineOrigin;
            SGM::UnitVector3D LineAxis;
            IntersectNonParallelPlanes(CurvePlaneOrigin, CurvePlaneNormal, PlaneOrigin, PlaneNorm, LineOrigin, LineAxis);
            IntersectLineAndNUBCurve(LineOrigin, LineAxis, SGM::Interval1D(-SGM_MAX, +SGM_MAX), pCurve, dTolerance, aPoints, aTypes);
        }
    }
    else
    {
        // Find the starting points.

        std::vector<SGM::Point3D> const &aSeedPoints=pCurve->GetSeedPoints();
        size_t nSeedPoints=aSeedPoints.size();
        double dTanHalfAngle=SEED_POINT_HALF_ANGLE_TANGENT;
        std::vector<SGM::Point3D> aStartPoints;
        size_t Index1;
        for(Index1=1;Index1<nSeedPoints;++Index1)
        {
            SGM::Point3D const &Pos0=aSeedPoints[Index1-1];
            SGM::Point3D const &Pos1=aSeedPoints[Index1];
            double dDist0 = ((Pos0 - PlaneOrigin) % PlaneNorm);
            double dDist1 = ((Pos1 - PlaneOrigin) % PlaneNorm);
            if ((dDist0 * dDist1) < SGM_ZERO) // opposite sides of the plane
            {
                double dFraction = fabs(dDist0) / (fabs(dDist0) + fabs(dDist1));
                SGM::Point3D Start = Pos0 + (dFraction * (Pos1 - Pos0)); 
                aStartPoints.push_back(Start);
            }
            else
            {
                double dLength=Pos0.Distance(Pos1);
                double dTol=dTolerance+dLength*dTanHalfAngle;
                if (fabs(dDist0) < dTol)
                {
                    aStartPoints.push_back(Pos0);
                }
                if (fabs(dDist1) < dTol)
                {
                    aStartPoints.push_back(Pos1);
                }
            }
        }


        // Find the intersection points.

        std::vector<std::pair<double,SGM::Point3D> > aRefinedPoints;
        size_t nStartPoints=aStartPoints.size();
        size_t nCountLimit=100;

        for(Index1=0;Index1<nStartPoints;++Index1)
        {
            SGM::Point3D Pos=aStartPoints[Index1];
            size_t nCount=0;
            double dOldDist=SGM_MAX;
            while(nCount<nCountLimit)
            {
#if 0 // project back and forth
                // project point to NUBCurve
                double dNUBt=pCurve->Inverse(Pos);
                SGM::Point3D CPos;
                pCurve->Evaluate(dNUBt,&CPos);

                // project point to plane
                Pos = CPos + ((PlaneOrigin - CPos) % PlaneNorm) * PlaneNorm;

                double dDist = Pos.Distance(CPos);

                //if(dDist<dOldDist)
                //{
                //    Pos=Temp;
                //}
#endif
#if 1 // newton
                // project point to NUBCurve and get the tangent
                double dNUBt=pCurve->Inverse(Pos);
                SGM::Point3D CPos;
                SGM::Vector3D LocalTan;
                pCurve->Evaluate(dNUBt,&CPos,&LocalTan);

                SGM::UnitVector3D uLocalTan(LocalTan);
                double dT = (PlaneNorm % (PlaneOrigin - CPos)) / (PlaneNorm % uLocalTan);
                SGM::Point3D Temp = CPos + dT * uLocalTan;
                double dDist=Temp.Distance(CPos);
                if(dDist<dOldDist)
                {
                    Pos=Temp;
                }
                else
                {
                    // Newton led us astray.  Just project to plane.
                    double Dist=(CPos-PlaneOrigin)%PlaneNorm;
                    Pos=CPos - Dist*PlaneNorm;
                    dDist=Pos.Distance(CPos);
                }
#endif
                if(dDist<SGM_ZERO || fabs(dDist-dOldDist)<SGM_ZERO)
                {
                    if (dDist<dTolerance)
                        aRefinedPoints.emplace_back(dNUBt,Pos);
                    break;
                }
                if(nCount==nCountLimit-1 && dDist<dTolerance)
                {
                    aRefinedPoints.emplace_back(dNUBt,Pos);
                    break;
                }
                dOldDist=dDist;
                ++nCount;
                }
            }

        // Remove duplicates and find the types.

        if(size_t nRefinedPoints=aRefinedPoints.size())
        {
            double dDuplicatesTolerance=std::max(dTolerance,SGM_MIN_TOL);
            std::sort(aRefinedPoints.begin(),aRefinedPoints.end());
            for(Index1=0;Index1<nRefinedPoints;++Index1)
            {
                SGM::Point3D const &Pos=aRefinedPoints[Index1].second;
                if(Index1==0 || dDuplicatesTolerance<Pos.Distance(aPoints.back()))
                {
                    double t=aRefinedPoints[Index1].first;
                    SGM::Vector3D DPos;
                    pCurve->Evaluate(t,nullptr,&DPos);
                    aPoints.push_back(Pos);
                    SGM::UnitVector3D Test(DPos);
                    if(fabs(Test%PlaneNorm)<SGM_MIN_TOL)
                    {
                        aTypes.push_back(SGM::IntersectionType::TangentType);
                    }
                    else
                    {
                        aTypes.push_back(SGM::IntersectionType::PointType);
                    }
                }
            }
        }
    }

    return aPoints.size();    
}

size_t IntersectEllipseAndPlane(ellipse                      const *pEllipse,
                                SGM::Point3D                 const &PlaneOrigin,
                                SGM::UnitVector3D            const &PlaneNormal,
                                double                              dTolerance,
                                std::vector<SGM::Point3D>          &aPoints,
                                std::vector<SGM::IntersectionType> &aTypes)
{
    if(SGM::NearEqual(fabs(pEllipse->m_Normal%PlaneNormal),1.0,SGM_MIN_TOL,false))
        {
        if(fabs((pEllipse->m_Center-PlaneOrigin)%PlaneNormal)<dTolerance)
            {
            SGM::Point3D EllipsePos;
            pEllipse->Evaluate(0.0, &EllipsePos);
            aPoints.push_back(EllipsePos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            aPoints.push_back(EllipsePos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            }
        }
    else
        {
        SGM::Point3D LineOrigin;
        SGM::UnitVector3D LineDirection;
        IntersectNonParallelPlanes(pEllipse->m_Center,pEllipse->m_Normal,PlaneOrigin,PlaneNormal,LineOrigin,LineDirection);
        IntersectLineAndEllipse(LineOrigin,LineDirection,SGM::Interval1D(-SGM_MAX,SGM_MAX),pEllipse,dTolerance,aPoints,aTypes);
        }
    return aPoints.size();
}

size_t IntersectParabolaAndPlane(parabola                     const *pParabola,
                                 SGM::Point3D                 const &PlaneOrigin,
                                 SGM::UnitVector3D            const &PlaneNormal,
                                 double                              dTolerance,
                                 std::vector<SGM::Point3D>          &aPoints,
                                 std::vector<SGM::IntersectionType> &aTypes)
{
    if(SGM::NearEqual(fabs(pParabola->m_Normal%PlaneNormal),1.0,SGM_MIN_TOL,false))
        {
        if(fabs((pParabola->m_Center-PlaneOrigin)%PlaneNormal)<dTolerance)
            {
            SGM::UnitVector3D XAxis=pParabola->m_Normal.Orthogonal();

            SGM::Point3D Pos;
            pParabola->Evaluate(pParabola->GetDomain().m_dMin, &Pos);
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            pParabola->Evaluate(pParabola->GetDomain().m_dMax, &Pos);
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            }
        }
    else
        {
        SGM::Point3D LineOrigin;
        SGM::UnitVector3D LineDirection;
        IntersectNonParallelPlanes(pParabola->m_Center,pParabola->m_Normal,PlaneOrigin,PlaneNormal,LineOrigin,LineDirection);
        IntersectLineAndParabola(LineOrigin,LineDirection,SGM::Interval1D(-SGM_MAX,SGM_MAX),pParabola,dTolerance,aPoints,aTypes);
        }
    return aPoints.size();
}

size_t IntersectHyperbolaAndPlane(hyperbola                     const *pHyperbola,
                                  SGM::Point3D                  const &PlaneOrigin,
                                  SGM::UnitVector3D             const &PlaneNormal,
                                  double                               dTolerance,
                                  std::vector<SGM::Point3D>           &aPoints,
                                  std::vector<SGM::IntersectionType>  &aTypes)
{
    if(SGM::NearEqual(fabs(pHyperbola->m_Normal%PlaneNormal),1.0,SGM_MIN_TOL,false))
        {
        if(fabs((pHyperbola->m_Center-PlaneOrigin)%PlaneNormal)<dTolerance)
            {
            SGM::UnitVector3D XAxis=pHyperbola->m_Normal.Orthogonal();

            SGM::Point3D Pos;
            pHyperbola->Evaluate(pHyperbola->GetDomain().m_dMin, &Pos);
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            pHyperbola->Evaluate(pHyperbola->GetDomain().m_dMax, &Pos);
            aPoints.push_back(Pos);
            aTypes.push_back(SGM::IntersectionType::CoincidentType);
            }
        }
    else
        {
        SGM::Point3D LineOrigin;
        SGM::UnitVector3D LineDirection;
        IntersectNonParallelPlanes(pHyperbola->m_Center,pHyperbola->m_Normal,PlaneOrigin,PlaneNormal,LineOrigin,LineDirection);
        IntersectLineAndHyperbola(LineOrigin,LineDirection,SGM::Interval1D(-SGM_MAX,SGM_MAX),pHyperbola,dTolerance,aPoints,aTypes);
        }
    return aPoints.size();
}

} // End of SGMInternal namespace
