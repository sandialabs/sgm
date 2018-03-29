#include "SGMDataClasses.h"
#include "SGMEntityClasses.h"
#include "SGMPrimatives.h"
#include "SGMChecker.h"
#include "Primitive.h"

SGM::Result SGM::CreateResult()
    {
    thing *pThing=new thing();
    return SGM::Result(pThing); 
    }

SGM::Body SGM::CreateBlock(SGM::Result        &rResult,
                           SGM::Point3D const &Point1,
                           SGM::Point3D const &Point2)
    {
    thing *pThing=rResult.GetThing();
    body *pBody=CreateBlock(rResult,pThing,Point1,Point2);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateSphere(SGM::Result        &rResult,
                            SGM::Point3D const &Center,
                            double              dRadius)
    {
    thing *pThing=rResult.GetThing();
    body *pBody=CreateSphere(rResult,pThing,Center,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateCylinder(SGM::Result        &rResult,
                              SGM::Point3D const &BottomCenter,
                              SGM::Point3D const &TopCenter,
                              double              dRadius)
    {
    thing *pThing=rResult.GetThing();
    body *pBody=CreateCylinder(rResult,pThing,BottomCenter,TopCenter,dRadius);
    return SGM::Body(pBody->GetID());
    }

SGM::Body SGM::CreateTorus(SGM::Result             &rResult,
                           SGM::Point3D      const &Center,
                           SGM::UnitVector3D const &Axis,
                           double                   dMajorRadius,
                           double                   dMinorRadius,
                           bool                     bApple)
    {
    thing *pThing=rResult.GetThing();
    body *pBody=CreateTorus(rResult,pThing,Center,Axis,dMajorRadius,dMinorRadius,bApple);
    return SGM::Body(pBody->GetID());
    }

bool SGM::CheckEntity(SGM::Result              &rResult,
                      SGM::Entity              &EntityID,
                      SGM::CheckLevel           Level,
                      std::vector<std::string> &aCheckStrings)
    {
    thing *pThing=rResult.GetThing();
    entity *pEntity=pThing->FindEntity(EntityID.m_ID);
    return pEntity->Check(rResult,Level,aCheckStrings);
    }