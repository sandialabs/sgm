#include "SGMChecker.h"
#include "SGMPrimitives.h"
#include "SGMComplex.h"
#include "SGMMathematics.h"
#include "SGMInterval.h"
#include "SGMTranslators.h"
#include "SGMTopology.h"
#include "SGMVector.h"
#include "SGMGeometry.h"
#include "SGMInterrogate.h"
#include "SGMIntersector.h"
#include "SGMTransform.h"
#include "SGMTopology.h"
#include "SGMDisplay.h"
#include "SGMMeasure.h"
#include "SGMEntityFunctions.h"
#include "SGMModify.h"
#include "SGMGraph.h"

#include "FileFunctions.h"
#include "EntityClasses.h"
#include "Intersectors.h"
#include "FacetToBRep.h"
#include "Primitive.h"
#include "Mathematics.h"
#include "Surface.h"

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <EntityFunctions.h>
#include <array>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal
{

bool TestSurface(SGM::Result                &rResult,
                 SGMInternal::surface const *pSurface,
                 SGM::Point2D         const &uv1)
    {
    if(rResult.GetDebugFlag()==4)
        {
        return false;
        }
    bool bAnswer=true;

    // Test to see if evaluate and inverse match.

    SGM::Point3D Pos,CPos;
    SGM::UnitVector3D Norm;
    SGM::Vector3D dU,dV,dUU,dUV,dVV;
    pSurface->Evaluate(uv1,&Pos,&dU,&dV,&Norm,&dUU,&dUV,&dVV);

    SGM::Point2D uv2=pSurface->Inverse(Pos,&CPos);
    if(SGM::NearEqual(uv1,uv2,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(Pos,CPos,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }

    // Test all the derivatives.

    double dx=SGM_FIT,dy=SGM_FIT;
    SGM::Vector3D dNU,dNV,dNUU,dNUV,dNVV;
    SGM::Point3D aMatrix[5][5];
    size_t Index1,Index2;
    for(Index1=0;Index1<5;++Index1)
        {
        double dX=uv1.m_u+dx*(Index1-2.0);
        for(Index2=0;Index2<5;++Index2)
            {
            double dY=uv1.m_v+dy*(Index2-2.0);
            SGM::Point2D uv3(dX,dY);
            SGM::Point3D GridPos;
            pSurface->Evaluate(uv3,&GridPos);
            aMatrix[Index1][Index2]=GridPos;
            }
        }
    SGM::PartialDerivatives<SGM::Point3D,SGM::Vector3D>(aMatrix,dx,dy,dNU,dNV,dNUU,dNUV,dNVV);

    int nContU=pSurface->UContinuity();
    int nContV=pSurface->VContinuity();

    if(0<nContU)
        {
        double dTol=std::max(1.0,dU.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dU,dNU,dTol)==false)
            {
            bAnswer=false;
            }
        }
    if(0<nContV)
        {
        double dTol=std::max(1.0,dV.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dV,dNV,dTol)==false)
            {
            bAnswer=false;
            }
        }
    if(1<nContU)
        {
        double dTol=std::max(1.0,dUU.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dUU,dNUU,dTol)==false)
            {
            bAnswer=false;
            }
        }
    if(0<nContU && 0<nContV)
        {
        double dTol=std::max(1.0,dUV.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dUV,dNUV,dTol)==false)
            {
            bAnswer=false;
            }
        }
    if(1<nContV)
        {
        double dTol=std::max(1.0,dVV.Magnitude())*SGM_FIT;
        if(SGM::NearEqual(dVV,dNVV,dTol)==false)
            {
            bAnswer=false;
            }
        }

    // Test parameter curves.
#if 0
    SGM::Interval2D const &Domain=pSurface->GetDomain();
    curve *pUParamLine=pSurface->UParamLine(rResult,Domain.m_UDomain.MidPoint());
    curve *pVParamLine=pSurface->VParamLine(rResult,Domain.m_VDomain.MidPoint());

    SGM::Point3D PosU,PosV;
    pUParamLine->Evaluate(pUParamLine->GetDomain().MidPoint(),&PosU);
    pVParamLine->Evaluate(pVParamLine->GetDomain().MidPoint(),&PosV);
    SGM::Point3D SurfPosU,SurfPosV;
    SGM::Point2D Uuv=pSurface->Inverse(PosU,&SurfPosU);
    SGM::Point2D Vuv=pSurface->Inverse(PosV,&SurfPosV);
    SGM::Point3D Zero(0,0,0);
    double dParamUTol=SGM_FIT*PosU.Distance(Zero);
    if(SGM::NearEqual(PosU,SurfPosU,dParamUTol)==false)
        {
        bAnswer=false;
        }
    double dParamVTol=SGM_FIT*PosV.Distance(Zero);
    if(SGM::NearEqual(PosV,SurfPosV,dParamVTol)==false)
        {
        bAnswer=false;
        }

    rResult.GetThing()->DeleteEntity(pUParamLine);
    rResult.GetThing()->DeleteEntity(pVParamLine);
#endif
    return bAnswer;
    }

bool TestCurve(SGM::Result              &rResult,
               SGMInternal::curve const *pCurve,
               double                    t1)
    {
    if(rResult.GetDebugFlag()==3)
        {
        return false;
        }
    SGM::Point3D Pos;
    SGM::Vector3D Vec1,Vec2;
    pCurve->Evaluate(t1,&Pos,&Vec1,&Vec2);
    SGM::Point3D ClosePos;
    double t2=pCurve->Inverse(Pos,&ClosePos);

    bool bAnswer=true;
    if(SGM::NearEqual(Pos,ClosePos,SGM_MIN_TOL)==false)
        {
        bAnswer=false;
        }
    if(SGM::NearEqual(t1,t2,SGM_MIN_TOL,false)==false)
        {
        bAnswer=false;
        }

    double h=SGM_MIN_TOL;
    SGM::Point3D Pos0,Pos1,Pos2,Pos3;
    pCurve->Evaluate(t1-2*h,&Pos0);
    pCurve->Evaluate(t1-h,&Pos1);
    pCurve->Evaluate(t1+h,&Pos2);
    pCurve->Evaluate(t1+2*h,&Pos3);

    SGM::Vector3D VecN1=SGM::FirstDerivative<SGM::Point3D,SGM::Vector3D>(Pos0,Pos1,Pos2,Pos3,h);
    SGM::Vector3D VecN2=SGM::SecondDerivative<SGM::Point3D,SGM::Vector3D>(Pos0,Pos1,Pos,Pos2,Pos3,h);

    double dDist=std::max(1.0,sqrt(Pos0.m_x*Pos0.m_x+Pos0.m_y*Pos0.m_y+Pos0.m_z*Pos0.m_z));
    if(0<pCurve->Continuity())
        {
        double dMag1=VecN1.Magnitude();
        double dTol1=std::max(dMag1,dDist)*SGM_FIT;
        if(SGM::NearEqual(Vec1,VecN1,dTol1)==false)
            {
            bAnswer=false;
            }
        }

    double dMag2=VecN2.Magnitude();
    double dTol2=std::max(dMag2,dDist)*SGM_FIT;
    if(1<pCurve->Continuity() && SGM::NearEqual(Vec2,VecN2,dTol2)==false)
        {
        bAnswer=false;
        }

    return bAnswer;
    }

    // for testing EntityVisitor and Box functions
    struct UnitTestGetBoxVisitor : EntityVisitor
        {
        SGM::Transform3D m_Transform3D;
    
        UnitTestGetBoxVisitor() = delete;

        explicit UnitTestGetBoxVisitor(SGM::Result &rResult, SGM::Transform3D const &transform3D) : 
            EntityVisitor(rResult), m_Transform3D(transform3D)
            {}

        void Visit(thing &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(assembly &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(attribute &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(body &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(complex &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(edge &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(face &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(reference &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(vertex &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(volume &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(line &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(circle &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(ellipse &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(hyperbola &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(parabola &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(hermite &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(NUBcurve &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(NURBcurve &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(PointCurve &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(TorusKnot &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(plane &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(cylinder &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(cone &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(sphere &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(torus &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(revolve &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(extrude &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(offset &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(NUBsurface &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        void Visit(NURBsurface &x) override
            { auto box = x.GetBox(*pResult); x.TransformBox(*pResult, m_Transform3D); x.ResetBox(*pResult); }

        };

    void PrintOrderedPoints(std::string name, buffer<unsigned> const &aIndexOrdered, std::vector<SGM::Point3D> const &aPoints)
        {
        std::cout.setf(std::ios::floatfield,std::ios::scientific);
        std::cout << std::setprecision(15) << name << std::endl;
        for (size_t i = 0; i < aPoints.size(); ++i)
            {
            SGM::Point3D const &Point = aPoints[aIndexOrdered[i]];
            std::cout << aIndexOrdered[i] << std::setw(23) << Point[0] << std::setw(23) << Point[1] << std::setw(23) << Point[2] << std::endl;
            }
        std::cout << std::endl;
        }

} // SGMInternal namespace


namespace SGM
{

static size_t iCallCount = 0;

double TestIntegrand(double x,void const *)
    {
    iCallCount++;
    return 4.0/(1.0+x*x);
    }

double TestIntegrand1DSinVoid(double x,void const *)
    {
    iCallCount++;
    return std::sin(x);
    }

double TestIntegrand1DSin(double x)
    {
    iCallCount++;
    return std::sin(x);
    }

double TestIntegrand1DSinCosVoid(double x,void const *)
    {
    iCallCount++;
    return x * std::sin(50*x) * std::cos(75*x);
    }

double TestIntegrand1DSinCos(double x)
    {
    iCallCount++;
    return x * std::sin(50*x) * std::cos(75*x);
    }

double TestIntegrand2D(SGM::Point2D const &uv,void const *)
    {
    iCallCount++;
    double x=uv.m_u;
    double y=uv.m_v;
    return x*x+4*y;
    }

bool RunInternalTest(SGM::Result &rResult,
                     size_t       nTestNumber)
    {
    bool bAnswer=true;

    if(nTestNumber==1) // Integration
        {
        SGM::Interval2D Domain2D(11,14,7,10);
        double dValue2D=SGMInternal::Integrate2D(TestIntegrand2D,Domain2D,nullptr,SGM_ZERO);
        if(SGM::NearEqual(dValue2D,1719,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        SGM::Point2D PosA(11,7),PosB(14,7),PosC(14,10),PosD(11,10);
        dValue2D=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosB,PosC,nullptr,SGM_ZERO);
        dValue2D+=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosC,PosD,nullptr,SGM_ZERO);
        if(SGM::NearEqual(dValue2D,1719,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        SGM::Point2D PosE(14,8.5),PosF(11,8.5);
        double dT1=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosB,PosE,nullptr,SGM_ZERO);
        double dT3=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosE,PosD,nullptr,SGM_ZERO);
        double dT2=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosD,PosC,PosE,nullptr,SGM_ZERO);
        dValue2D=dT1+dT2+dT3;
        if(SGM::NearEqual(dValue2D,1719,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        dT1=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosA,PosB,PosF,nullptr,SGM_ZERO);
        dT3=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosB,PosF,PosC,nullptr,SGM_ZERO);
        dT2=SGMInternal::IntegrateTriangle(TestIntegrand2D,PosD,PosC,PosF,nullptr,SGM_ZERO);
        dValue2D=dT1+dT2+dT3;
        if(SGM::NearEqual(dValue2D,1719,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        iCallCount = 0;
        SGM::Interval1D Domain(0.0,1.0);
        double dValue=SGMInternal::Integrate1D(TestIntegrand,Domain,nullptr,SGM_ZERO);
        if(SGM::NearEqual(dValue,SGM_PI,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        iCallCount = 0;
        SGM::Interval1D DomainSin(0.0,SGM_PI);
        dValue=SGMInternal::Integrate1D(TestIntegrand1DSinVoid,DomainSin,nullptr,SGM_ZERO);
        if(SGM::NearEqual(dValue,2.0,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        iCallCount = 0;
        SGM::Interval1D DomainSinCos(-1.0,1.0);
        dValue=SGMInternal::Integrate1D(TestIntegrand1DSinCosVoid,DomainSinCos,nullptr,1.e-15);
        if(SGM::NearEqual(dValue,0.033518732588154,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }

        SGM::Point3D Center(0,0,0);
        SGM::UnitVector3D Normal(0,0,1);
        SGM::Curve CurveID=SGM::CreateCircle(rResult,Center,Normal,1.0);

        SGM::Interval1D const &CurveDomain=SGM::GetCurveDomain(rResult,CurveID);
    
        dValue=SGM::FindCurveLength(rResult,CurveDomain,CurveID,SGM_MIN_TOL);
        if(SGM::NearEqual(dValue,SGM_TWO_PI,SGM_ZERO,false)==false)
            {
            bAnswer=false;
            }
        SGM::DeleteEntity(rResult,CurveID);

        SGM::Body BodyID=SGM::CreateSphere(rResult,Center,1.0);
        std::set<SGM::Face> sFaces;
        SGM::FindFaces(rResult,BodyID,sFaces);
        SGM::Face FaceID=*(sFaces.begin());
        double dArea=SGM::FindArea(rResult,FaceID);
        if(SGM::NearEqual(dArea,12.566370614359172953850573533118,SGM_MIN_TOL,false)==false)
            {
            bAnswer=false;
            }
        SGM::DeleteEntity(rResult,BodyID);
        }
    else if(nTestNumber==2) // Entity free intersections
        {
        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        SGM::Interval1D Domain1(-10,10),Domain2(-10,10);
        SGMInternal::IntersectLineAndLine(SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),Domain1,
                                          SGM::Point3D(0,0,1),SGM::UnitVector3D(0,0,-1),Domain2,
                                          SGM_MIN_TOL,aPoints,aTypes);
        if(aPoints.size()!=2)
            {
            bAnswer=false;
            }

        SGM::Interval1D Domain3(0,1);
        aPoints.clear();
        aTypes.clear();
        SGMInternal::IntersectLineAndPlane(SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),Domain3,
                                           SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),
                                           SGM_MIN_TOL,aPoints,aTypes);
        if(aPoints.size()!=1)
            {
            bAnswer=false;
            }

        SGM::Interval1D Domain4(-1,0);
        aPoints.clear();
        aTypes.clear();
        SGMInternal::IntersectLineAndPlane(SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),Domain4,
                                           SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),
                                           SGM_MIN_TOL,aPoints,aTypes);
        if(aPoints.size()!=1)
            {
            bAnswer=false;
            }

        SGM::Interval1D Domain5(-2,-1);
        aPoints.clear();
        aTypes.clear();
        SGMInternal::IntersectLineAndPlane(SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),Domain5,
                                           SGM::Point3D(0,0,0),SGM::UnitVector3D(0,0,1),
                                           SGM_MIN_TOL,aPoints,aTypes);
        if(!aPoints.empty())
            {
            bAnswer=false;
            }
        }
    else if(nTestNumber==3) // Entity Visitor/Accept interface
        {
        std::vector<SGMInternal::entity *> aEntities;
        SGM::Point3D thePoint3D(0.0, 0.0, 0.0);
        SGM::UnitVector3D theUnitVector3D(1.0, 0.0, 0.0);
        SGM::UnitVector3D otherUnitVector3D(0.0, 1.0, 0.0);
        std::vector<int> aIntVector(3, 1);
        std::vector<SGM::Point3D> aPoint3D = {{0, 0, 0},
                                              {1, 0, 0},
                                              {0, 2, 0},
                                              {0, 0, 3}};
        std::vector<SGM::Vector3D> aVector3D = {{1, 0, 0},
                                                {0, 1, 0},
                                                {0, 0, 1},
                                                {1, 1, 1}};
        std::vector<SGM::Point4D> aPoint4D = {{1, 0, 0, 1},
                                              {0, 1, 0, 1},
                                              {0, 0, 1, 1},
                                              {1, 1, 1, 1}};
        std::vector<double> aDoubleVector(4, 1.0);


        std::vector<std::string> aCheckStrings;
        SGM::CheckOptions checkOptions;
        std::map<SGMInternal::entity*,SGMInternal::entity*> replace_map;

        // create one of everything
        SGMInternal::thing *pThing = rResult.GetThing();

        // other random thing member functions
        try { pThing->ReplacePointers(replace_map); } catch (const std::logic_error&) {}

        auto *pAssembly = new SGMInternal::assembly(rResult);
        aEntities.push_back(pAssembly);
        auto *pCloneAssembly = pAssembly->Clone(rResult);
        pCloneAssembly->IsTopLevel();
        pCloneAssembly->Check(rResult,checkOptions, aCheckStrings, false);
        try { pCloneAssembly->ReplacePointers(replace_map); } catch (const std::logic_error&) {}

        auto *pAttribute = new SGMInternal::IntegerAttribute(rResult, "None", aIntVector);
        aEntities.push_back(pAttribute);
        auto *pCloneAttribute = pAttribute->Clone(rResult);
        pCloneAttribute->IsTopLevel();
        pCloneAttribute->Check(rResult,checkOptions, aCheckStrings, false);
        try { pCloneAttribute->ReplacePointers(replace_map); } catch (const std::logic_error&) {}

        std::vector<double> aDoubles(3,1.0);
        auto *pDoubleAttribute = new SGMInternal::DoubleAttribute(rResult,"DoubleAttributeTestName",aDoubles);
        aEntities.push_back(pDoubleAttribute);
        auto *pCloneDoubleAttribute = pDoubleAttribute->Clone(rResult);
        pCloneDoubleAttribute->IsTopLevel();
        pCloneDoubleAttribute->Check(rResult,checkOptions, aCheckStrings, false);
        try { pCloneAttribute->ReplacePointers(replace_map); } catch (const std::logic_error&) {}

        std::vector<char> aChars(3,'c');
        auto *pCharAttribute = new SGMInternal::CharAttribute(rResult,"CharAttributeTestName",aChars);
        aEntities.push_back(pCharAttribute);
        auto *pCloneCharAttribute = pCharAttribute->Clone(rResult);
        pCloneCharAttribute->IsTopLevel();
        pCloneCharAttribute->Check(rResult,checkOptions, aCheckStrings, false);
        try { pCloneAttribute->ReplacePointers(replace_map); } catch (const std::logic_error&) {}

        auto *pStringAttribute = new SGMInternal::StringAttribute(rResult,"StringAttributeTestName","string_test");
        aEntities.push_back(pStringAttribute);
        auto *pCloneStringAttribute = pStringAttribute->Clone(rResult);
        pCloneStringAttribute->IsTopLevel();
        pCloneStringAttribute->Check(rResult,checkOptions, aCheckStrings, false);
        try { pCloneAttribute->ReplacePointers(replace_map); } catch (const std::logic_error&) {}

        auto *pBody = new SGMInternal::body(rResult);
        aEntities.push_back(pBody);
        auto *pComplex = new SGMInternal::complex(rResult);
        aEntities.push_back(pComplex);
        auto *pEdge = new SGMInternal::edge(rResult);
        aEntities.push_back(pEdge);
        auto *pFace = new SGMInternal::face(rResult);
        aEntities.push_back(pFace);

        auto *pReference = new SGMInternal::reference(rResult);
        aEntities.push_back(pReference);
        auto *pCloneReference = pReference->Clone(rResult);
        pCloneReference->IsTopLevel();
        pCloneReference->Check(rResult,checkOptions, aCheckStrings, false);
        try { pCloneReference->ReplacePointers(replace_map); } catch (const std::logic_error&) {}

        auto *pVertex = new SGMInternal::vertex(rResult, thePoint3D);
        aEntities.push_back(pVertex);
        auto *pVolume = new SGMInternal::volume(rResult);
        aEntities.push_back(pVolume);
        auto *pLine = new SGMInternal::line(rResult, thePoint3D, thePoint3D);
        aEntities.push_back(pLine);
        auto *pCircle = new SGMInternal::circle(rResult, thePoint3D, theUnitVector3D, 1.0);
        aEntities.push_back(pCircle);
        auto *pEllipse = new SGMInternal::ellipse(rResult, thePoint3D, theUnitVector3D, otherUnitVector3D, 1.0, 2.0);
        aEntities.push_back(pEllipse);
        auto *pHyperbola = new SGMInternal::hyperbola(rResult, thePoint3D, theUnitVector3D, otherUnitVector3D, 1.0,
                                                      2.0);
        aEntities.push_back(pHyperbola);
        auto *pParabola = new SGMInternal::parabola(rResult, thePoint3D, theUnitVector3D, otherUnitVector3D, 1.0);
        aEntities.push_back(pParabola);
        auto *pHermite = new SGMInternal::hermite(rResult, aPoint3D, aVector3D, aDoubleVector);
        aEntities.push_back(pHermite);
        auto *pNUBcurve = new SGMInternal::NUBcurve(rResult, aPoint3D, aDoubleVector);
        aEntities.push_back(pNUBcurve);
        auto *pNURBcurve = new SGMInternal::NURBcurve(rResult, aPoint4D, aDoubleVector);
        aEntities.push_back(pNURBcurve);
        auto *pPointCurve = new SGMInternal::PointCurve(rResult, thePoint3D);
        aEntities.push_back(pPointCurve);
        auto *pTorusKnot = new SGMInternal::TorusKnot(rResult, thePoint3D, theUnitVector3D, otherUnitVector3D, 1.0, 3.0,
                                                      1, 3);
        aEntities.push_back(pTorusKnot);
        auto *pPlane = new SGMInternal::plane(rResult, thePoint3D, theUnitVector3D);
        aEntities.push_back(pPlane);
        auto *pCylinder = new SGMInternal::cylinder(rResult, thePoint3D, theUnitVector3D, 1.0);
        aEntities.push_back(pCylinder);
        auto *pCone = new SGMInternal::cone(rResult, thePoint3D, theUnitVector3D, 1.0, 45);
        aEntities.push_back(pCone);
        auto *pSphere = new SGMInternal::sphere(rResult, thePoint3D, 1.0);
        aEntities.push_back(pSphere);
        auto *pTorus = new SGMInternal::torus(rResult, thePoint3D, theUnitVector3D, 1.0, 2.0, false);
        aEntities.push_back(pTorus);
        auto *pRevolve = new SGMInternal::revolve(rResult, thePoint3D, theUnitVector3D, pLine);
        aEntities.push_back(pRevolve);
        auto *pExtrude = new SGMInternal::extrude(rResult, theUnitVector3D, pLine);
        aEntities.push_back(pExtrude);

        auto *pOtherPlane = new SGMInternal::plane(rResult, thePoint3D, theUnitVector3D);
        aEntities.push_back(pOtherPlane);
        auto *pOffset = new SGMInternal::offset(rResult, 1.0, pOtherPlane);
        aEntities.push_back(pOffset);

        pEdge->SetCurve(pLine);
        pEdge->SetStart(pVertex);
        pEdge->SetEnd(pVertex);
        pFace->SetSurface(pSphere);

        // create NUBsurface
        std::vector<double> aUKnots = {0, 0, 0, 1, 1, 1};
        std::vector<double> aVKnots = aUKnots;
        std::vector<std::vector<SGM::Point3D> > aaPoints3D;
        std::vector<SGM::Point3D> aPoints;
        aPoints.assign(3, SGM::Point3D(0, 0, 0));
        aaPoints3D.push_back(aPoints);
        aaPoints3D.push_back(aPoints);
        aaPoints3D.push_back(aPoints);
        aaPoints3D[0][0] = SGM::Point3D(0.0, 0.0, 1.0);
        aaPoints3D[0][1] = SGM::Point3D(0.0, 1.0, 0.0);
        aaPoints3D[0][2] = SGM::Point3D(0.0, 2.0, -1.0);
        aaPoints3D[1][0] = SGM::Point3D(1.0, 0.0, 0.0);
        aaPoints3D[1][1] = SGM::Point3D(1.0, 1.0, 0.0);
        aaPoints3D[1][2] = SGM::Point3D(1.0, 2.0, 0.0);
        aaPoints3D[2][0] = SGM::Point3D(2.0, 0.0, -1.0);
        aaPoints3D[2][1] = SGM::Point3D(2.0, 1.0, 0.0);
        aaPoints3D[2][2] = SGM::Point3D(2.0, 2.0, 1.0);
        auto *pNUBsurface = new SGMInternal::NUBsurface(rResult, aaPoints3D, aUKnots, aVKnots);
        aEntities.push_back(pNUBsurface);

        // create NURBsurface
        std::vector<std::vector<SGM::Point4D> > aaPoints4D;
        std::vector<SGM::Point4D> aPoints4D;
        aPoints4D.assign(3, SGM::Point4D(0, 0, 0, 1));
        aaPoints4D.push_back(aPoints4D);
        aaPoints4D.push_back(aPoints4D);
        aaPoints4D.push_back(aPoints4D);
        aaPoints4D[0][0] = SGM::Point4D(0.0, 0.0, 1.0, 1);
        aaPoints4D[0][1] = SGM::Point4D(0.0, 1.0, 0.0, 1);
        aaPoints4D[0][2] = SGM::Point4D(0.0, 2.0, -1.0, 1);
        aaPoints4D[1][0] = SGM::Point4D(1.0, 0.0, 0.0, 1);
        aaPoints4D[1][1] = SGM::Point4D(1.0, 1.0, 0.0, 1);
        aaPoints4D[1][2] = SGM::Point4D(1.0, 2.0, 0.0, 1);
        aaPoints4D[2][0] = SGM::Point4D(2.0, 0.0, -1.0, 1);
        aaPoints4D[2][1] = SGM::Point4D(2.0, 1.0, 0.0, 1);
        aaPoints4D[2][2] = SGM::Point4D(2.0, 2.0, 1.0, 1);
        auto *pNURBsurface = new SGMInternal::NURBsurface(rResult, aaPoints4D, aUKnots, aVKnots);
        aEntities.push_back(pNURBsurface);

        // call the visitor the base class Visitor class that does nothing
        SGMInternal::EntityVisitor basicVisitor;
        pThing->VisitEntities(basicVisitor);
        basicVisitor.Visit(*pThing);
        pThing->Accept(basicVisitor);

        // a transform that is not just a scale or translation
        SGM::Vector4D XAxis4D = {3, 2, 2, 7};
        SGM::Vector4D YAxis4D = {2, 3, 2, 7};
        SGM::Vector4D ZAxis4D = {2, 2, 3, 7};
        SGM::Vector4D Translate4D = {1, 2, 3, 4};
        SGM::Transform3D Deform(XAxis4D, YAxis4D, ZAxis4D, Translate4D);

        // call a visitor that gets bounding boxes and transforms the bounding boxes
        SGMInternal::UnitTestGetBoxVisitor boxVisitor(rResult, Deform);
        pThing->VisitEntities(boxVisitor);
        boxVisitor.Visit(*pThing);
        pThing->Accept(boxVisitor);

        // TODO: call transform on every entity
//        for (auto pEntity : aEntities)
//            {
//            if (pEntity->GetType() != SGM::AssemblyType && pEntity->GetType() != SGM::AttributeType)
//                SGMInternal::TransformEntity(rResult, Deform, pEntity);
//            }
        }
    else if(nTestNumber==4) // SortablePlane testing
        {
        std::vector<SGM::Point3D> aPoints1,aPoints2;
        aPoints1.emplace_back(SGM::Point3D(0,0,0));
        aPoints1.emplace_back(SGM::Point3D(1,0,0));
        aPoints1.emplace_back(SGM::Point3D(0,1,0));
        aPoints2.emplace_back(SGM::Point3D(0,0,1));
        aPoints2.emplace_back(SGM::Point3D(1,0,1));
        aPoints2.emplace_back(SGM::Point3D(0,1,1));

        SGMInternal::SortablePlane SP1(aPoints1);
        SGMInternal::SortablePlane SP2(aPoints2);

        SP1.SetTolerance(0.01);

//        bool bTest=SP1<SP2;
//        bTest=SP2<SP1;
//        bTest=SP1<SP1;
//        bTest=(SP1==SP2);

        SP1.Origin();
        SP1.Normal();
        SP1.Tolerance();

        SGM::Vector3D Offset;
        //bTest=SP1.Parallel(SP2,Offset,0.001);

        bAnswer=true;
        }
    else if(nTestNumber==5) // temporary offset surface testing
        {
        // this test can be removed once offset surface is completely implemented
        std::vector<SGMInternal::entity *> aEntities;
        SGM::Point3D thePoint3D(0.0, 0.0, 0.0);
        SGM::UnitVector3D theUnitVector3D(1.0, 0.0, 0.0);

        auto *pPlane = new SGMInternal::plane(rResult, thePoint3D, theUnitVector3D);
        aEntities.push_back(pPlane);
        auto *pOffset = new SGMInternal::offset(rResult, 1.0, pPlane);
        aEntities.push_back(pOffset);
        auto *pOffsetClone = pOffset->Clone(rResult);
        aEntities.push_back(pOffsetClone);

        rResult.GetThing()->SetConcurrentActive();
        rResult.GetThing()->SetConcurrentInactive();

        try {
            pOffset->WriteSGM(rResult, nullptr, SGM::TranslatorOptions());
        } catch (const std::logic_error&) {}

        try {
            SGM::Point2D thePoint2D(0.0, 0.0);
            pOffset->Evaluate(thePoint2D,&thePoint3D);
        } catch (const std::logic_error&) {}

        try {
            std::set<SGMInternal::entity*,SGMInternal::EntityCompare> sChildren;
            pOffset->FindAllChildren(sChildren);
        }  catch (const std::logic_error&) {}

        try {
            pOffset->Inverse(thePoint3D);
            }  catch (const std::logic_error&) {}

        try {
            SGM::Transform3D theTransform3D;
            pOffset->Transform(rResult,theTransform3D);
            }  catch (const std::logic_error&) {}

        try {
            pOffset->UParamLine(rResult, 0.0);
            }  catch (const std::logic_error&) {}
        try {
            pOffset->VParamLine(rResult, 0.0);
            }  catch (const std::logic_error&) {}

        try {
            pOffset->IsSame(pOffset,0.0);
            }  catch (const std::logic_error&) {}

        pOffset->GetSurface();

        auto *pOtherPlane = new SGMInternal::plane(rResult, thePoint3D, theUnitVector3D);
        aEntities.push_back(pOtherPlane);

        pOffset->SetSurface(pPlane);
        }
    else if(nTestNumber==6) // SnapToDomain testing
        {
        SGM::UnitVector3D ZAxis(0,0,1);
        SGMInternal::torus *pTorus=new SGMInternal::torus(rResult,SGM::Point3D(0,0,0),ZAxis,1.0,3.0,false);
        SGM::Point2D uv1(10,10),uv2(-10,-10);
        pTorus->SnapToDomain(uv1);
        pTorus->SnapToDomain(uv2);
        }
    else if(nTestNumber==7) // order points and sorting for complex testing
        {
        bAnswer=true;

        SGMInternal::PrintOrderedPoints("order points", {0}, {{0,0,0}}); // call once for coverage

        std::vector<SGM::Point3D> aPoints0 =
            {
                {0.0000000e+00, 4.5602101e-01, 2.5127900e-01},
                {0.0000000e+00, 1.6629200e-01, 5.0000000e-01},  // point[1] identical with point[4]
                {0.0000000e+00, 2.1484300e-01, 5.2078199e-01},
                {5.2347499e-01, 2.0895001e-02, 7.1533000e-01},
                {0.0000000e+00, 1.6629200e-01, 5.0000000e-01},
                {6.2853903e-01, 1.4633000e-02, 6.1653697e-01}
            };
        buffer<unsigned> aIndexOrdered0 = SGMInternal::OrderPointsLexicographical(aPoints0);
//    std::cout << std::endl;
//    for (int i = 0; i < aIndexOrdered0.size(); ++i)
//        std::cout << "aIndexOrdered0[" << i << "] = " << aIndexOrdered0[i] << std::endl;

        std::vector<SGM::Point3D> aPoints1 =
            {
                { 0.5,              0.5,              2.0},
                {-0.5,             -0.5,             -1.0},
                { 0.5,             -0.5,              2.0},
                { 0.5000000000001,  0.5000000000001,  2.0000000000001},
                {-0.5000000000001,  0.5000000000001, -1.0000000000001},
                {-0.5000000000001, -0.5000000000001, -1.0000000000001}
            };

        buffer<unsigned> aIndexOrdered1 = SGMInternal::OrderPointsZorder(aPoints1);

        if (aIndexOrdered1[0] != 5u ||
            aIndexOrdered1[1] != 1u ||
            aIndexOrdered1[2] != 4u ||
            aIndexOrdered1[3] != 2u ||
            aIndexOrdered1[4] != 0u ||
            aIndexOrdered1[5] != 3u)
            {
//            std::cout << std::endl << "Internal Test aIndexOrdered1 is wrong " << std::endl;
//            for (int i = 0; i < aIndexOrdered1.size(); ++i)
//                std::cout << "aIndexOrdered1[" << i << "] = " << aIndexOrdered1[i] << std::endl;
            bAnswer=false;
            }

        std::vector<SGM::Point3D> aPoints2 =
            {
                {-3000.0e-16,         -3000.0e-16,           5000.0e-16},
                {-3000.000000001e-16,  5000.000000001e-16,  -3000.000000001e-16},
                {-3000.000000001e-16, -3000.000000001e-16,   5000.000000001e-16},
                {-3000.0e-16,          5000.0e-16,          -3000.0e-16}
            };

        buffer<unsigned> aIndexOrdered2 = SGMInternal::OrderPointsLexicographical(aPoints2);
//        std::cout << std::endl;
//        for (int i = 0; i < aIndexOrdered2.size(); ++i)
//            std::cout << "aIndexOrdered2[" << i << "] = " << aIndexOrdered2[i] << std::endl;

        std::vector<SGM::Point3D> aPoints3 =
            {
                {     0.5,                     0.5,                    0.5},                    // 0
                {    -0.5,                    -0.5,                   -0.5},
                {    -0.5,                     0.5,                    0.5},                    // 2
                {     0.5,                    -0.5,                    0.5},
                {     0.5,                     0.5,                   -0.5},                    // 4
                {    -0.5,                    -0.5,                    0.5},
                {     0.5,                    -0.5,                   -0.5},                    // 6
                {    -0.5,                     0.5,                   -0.5},
                {   -10.0,                    20.0,                   20.0},                    // 8
                {    20.0,                   -10.0,                   20.0},
                {    20.0,                    20.0,                  -10.0},                    // 10
                { -3000.0,                 -3000.0,                  5000.0},
                {  5000.0,                 -3000.0,                 -3000.0},                   // 12
                { -3000.0,                  5000.0,                 -3000.0},
                {     0.5e-16,                 0.5e-16,                 0.5e-16},               // 14
                {    -0.5e-16,                -0.5e-16,                -0.5e-16},
                {    -0.5e-16,                 0.5e-16,                 0.5e-16},               // 16
                {     0.5e-16,                -0.5e-16,                 0.5e-16},
                {     0.5e-16,                 0.5e-16,                -0.5e-16},               // 18
                {    -0.5e-16,                -0.5e-16,                 0.5e-16},
                {     0.5e-16,                -0.5e-16,                -0.5e-16},               // 20
                {    -0.5e-16,                 0.5e-16,                -0.5e-16},
                {   -10.0e-16,                20.0e-16,                20.0e-16},               // 22
                {    20.0e-16,               -10.0e-16,                20.0e-16},
                {    20.0e-16,                20.0e-16,               -10.0e-16},               // 24
                { -3000.0e-16,             -3000.0e-16,              5000.0e-16},
                {  5000.0e-16,             -3000.0e-16,             -3000.0e-16},               // 26
                { -3000.0e-16,              5000.0e-16,             -3000.0e-16},
                {     0.5000000000001,         0.5000000000001,         0.5000000000001},       // 28
                {    -0.5000000000001,        -0.5000000000001,        -0.5000000000001},
                {    -0.5000000000001,         0.5000000000001,         0.5000000000001},       // 30
                {     0.5000000000001,        -0.5000000000001,         0.5000000000001},
                {     0.5000000000001,         0.5000000000001,        -0.5000000000001},       // 32
                {    -0.5000000000001,        -0.5000000000001,         0.5000000000001},
                {     0.5000000000001,        -0.5000000000001,        -0.5000000000001},       // 34
                {    -0.5000000000001,         0.5000000000001,        -0.5000000000001},
                {   -10.00000000001,          20.00000000001,          20.00000000001},         // 36
                {    20.00000000001,         -10.00000000001,          20.00000000001},
                {    20.00000000001,          20.00000000001,         -10.00000000001},         // 38
                { -3000.000000001,         -3000.000000001,          5000.000000001},
                {  5000.000000001,         -3000.000000001,         -3000.000000001},           // 40
                { -3000.000000001,          5000.000000001,         -3000.000000001},
                {     0.5000000000001e-16,     0.5000000000001e-16,     0.5000000000001e-16},   // 42
                {    -0.5000000000001e-16,    -0.5000000000001e-16,    -0.5000000000001e-16},
                {    -0.5000000000001e-16,     0.5000000000001e-16,     0.5000000000001e-16},   // 44
                {     0.5000000000001e-16,    -0.5000000000001e-16,     0.5000000000001e-16},
                {     0.5000000000001e-16,     0.5000000000001e-16,    -0.5000000000001e-16},   // 46
                {    -0.5000000000001e-16,    -0.5000000000001e-16,     0.5000000000001e-16},
                {     0.5000000000001e-16,    -0.5000000000001e-16,    -0.5000000000001e-16},   // 48
                {    -0.5000000000001e-16,     0.5000000000001e-16,    -0.5000000000001e-16},
                {   -10.00000000001e-16,      20.00000000001e-16,      20.00000000001e-16},     // 50
                {    20.00000000001e-16,     -10.00000000001e-16,      20.00000000001e-16},
                {    20.00000000001e-16,      20.00000000001e-16,     -10.00000000001e-16},     // 52
                { -3000.000000001e-16,     -3000.000000001e-16,      5000.000000001e-16},
                {  5000.000000001e-16,     -3000.000000001e-16,     -3000.000000001e-16},       // 54
                { -3000.000000001e-16,      5000.000000001e-16,     -3000.000000001e-16},
            };

        buffer<unsigned> aIndexOrdered3 = SGMInternal::OrderPointsZorder(aPoints3);

        // every consecutive pair in the sorted list should be close
        bool bExpectedAlmostEqual = true;
        for (size_t i = 0; i+1 < aIndexOrdered3.size(); i+=2)
            {
            bExpectedAlmostEqual = bExpectedAlmostEqual &&
                SGMInternal::AlmostEqual(aPoints3[aIndexOrdered3[i]],
                                         aPoints3[aIndexOrdered3[i + 1]],
                                         2.0 * std::numeric_limits<double>::epsilon());
            }
        if (!bExpectedAlmostEqual)
            {
//            std::cout << std::endl << "IndexOrdered3 is wrong" << std::endl;
//            for (int i = 0; i < aIndexOrdered3.size(); ++i)
//                std::cout << "aIndexOrdered3[" << i << "] = " << aIndexOrdered3[i] << std::endl;
            bAnswer=false;
            }

        std::vector<SGM::Point3D> aPoints4 =
            {
                {5.2093670000000, 0.2457350000000, 103.6632050000000},
                {5.2093670000003, 0.2854750000003, 103.6632050000003},
                {5.2093670000000, 0.2457350000000, 103.6632050000000},
                {5.2093670000001, 0.2854750000001, 103.6632050000001},
                {5.2093670000000, 0.2457350000000, 103.6632050000000},
                {5.2093670000002, 0.2854750000002, 103.6632050000002},
                {5.2093670000001, 0.2457350000001, 103.6632050000001},
                {5.2487370000001, 0.2384040000001, 103.6632050000001},
                {5.2487370000001, 0.2775250000001, 103.6632050000001},
                {5.2093670000000, 0.2854750000000, 103.6632050000000}
            };

            {
            buffer<unsigned> aIndexOrdered4 = SGMInternal::OrderPointsLexicographical(aPoints4);
            buffer<unsigned> aExpectedLexicographical4 = { 0, 2, 4, 9, 6, 3, 5, 1, 7, 8 };
            if (aIndexOrdered4 != aExpectedLexicographical4)
                {
//                std::cout << std::endl << "aPoints4 Lexicographical is wrong." << std::endl;
//                SGMInternal::PrintOrderedPoints("aPoints4 Lexicographical:", aIndexOrdered1, aPoints4);
                bAnswer=false;
                }
            }
            {
            buffer<unsigned> aIndexOrdered4 = SGMInternal::OrderPointsZorder(aPoints4);
            buffer<unsigned> aExpectedZorder4 = { 0, 2, 4, 6, 7, 9, 3, 5, 1, 8 };
            if (aIndexOrdered4 != aExpectedZorder4)
                {
//                std::cout << std::endl << "aPoints4 Z order is wrong." << std::endl;
//                SGMInternal::PrintOrderedPoints("aPoints4 Z order:", aIndexOrdered4, aPoints4);
                bAnswer=false;
                }
            }

        }

    return bAnswer;
    }

} // End of SGM namespace
#if 0
bool RunCPPTest(SGM::Result &rResult,
                     size_t       nTestNumber)
    {

    if(nTestNumber==27)
        {
        // Test the intersection of a line and NUBcurve.

        bool bAnswer=true;
    
        std::vector<SGM::Point3D> aPoints;
        size_t Index1;
        double d=0.2;
        for(Index1=0;Index1<100;++Index1)
            {
            aPoints.emplace_back(cos(Index1*d),sin(Index1*d),Index1*d*0.1);
            }
        SGMInternal::NUBcurve *pNUB=SGMInternal::CreateNUBCurve(rResult,aPoints);
        SGM::Point3D Pos0(1,0,0),Pos1(1,0,10);
        SGMInternal::line *pLine=new SGMInternal::line(rResult,Pos0,Pos1);
        std::vector<SGM::Point3D> aHits;
        std::vector<SGM::IntersectionType> aTypes;
        SGMInternal::IntersectCurves(rResult,pLine,pNUB,aHits,aTypes,nullptr,nullptr,SGM_FIT);

        size_t nHits=aHits.size();
        if(nHits!=4)
            {
            bAnswer=false;
            }
        for(Index1=0;Index1<nHits;++Index1)
            {
            SGM::Point3D const &Pos=aHits[Index1];
            SGM::Point3D CPos1,CPos2;
            pLine->Inverse(Pos,&CPos1);
            pNUB->Inverse(Pos,&CPos2);
            double dDist=CPos1.Distance(CPos2);
            if(SGM_FIT<dDist)
                {
                bAnswer=false;
                }
            }

        rResult.GetThing()->DeleteEntity(pNUB);
        rResult.GetThing()->DeleteEntity(pLine);

        return bAnswer;
        }

    if(nTestNumber==39)
        {
        bool bAnswer=true;

        std::vector<SGM::Point3D> aPoints1;
        aPoints1.emplace_back(-2,.5,0);
        aPoints1.emplace_back(-1,1.5,0);
        aPoints1.emplace_back(0,1,0);
        aPoints1.emplace_back(1,1.5,0);
        aPoints1.emplace_back(2,2,0);
        SGM::Curve CurveID = SGM::CreateNUBCurve(rResult, aPoints1);

        std::vector<SGM::Point3D> aPoints2;
        aPoints2.emplace_back(-2,0,.5);
        aPoints2.emplace_back(-1,0,1.5);
        aPoints2.emplace_back(0,0,1);
        aPoints2.emplace_back(1,0,1.5);
        aPoints2.emplace_back(2,0,2);
        SGM::Curve LeftCurveID = SGM::CreateNUBCurve(rResult, aPoints2);

        std::vector<SGM::Point3D> aPoints3;
        aPoints3.emplace_back(-2,0,-.5);
        aPoints3.emplace_back(-1,0,-1.5);
        aPoints3.emplace_back(0,0,-1);
        aPoints3.emplace_back(1,0,-1.5);
        aPoints3.emplace_back(2,0,-2);
        SGM::Curve RightCurveID = SGM::CreateNUBCurve(rResult, aPoints3);

        SGM::Point3D Origin(-1,0,0);
        SGM::UnitVector3D Axis(1,0,0);

        SGMInternal::curve* pCurve = (SGMInternal::curve *)rResult.GetThing()->FindEntity(CurveID.m_ID);
        SGMInternal::revolve *pRevolve=new SGMInternal::revolve(rResult,Origin,Axis,pCurve);

        SGM::Point3D pCurveStart;
        SGM::Point3D pCurveEnd;
        pCurve->Evaluate(pCurve->GetDomain().m_dMin, &pCurveStart);
        pCurve->Evaluate(pCurve->GetDomain().m_dMax, &pCurveEnd);

        SGM::Point3D StartCenter = Origin + ((pCurveStart - Origin) % Axis) * Axis;
        SGM::Point3D EndCenter = Origin + ((pCurveEnd - Origin) % Axis) * Axis;
        SGM::UnitVector3D XAxis = pCurveStart - StartCenter;
        double dRadiusStart = pCurveStart.Distance(StartCenter);
        double dRadiusEnd = pCurveEnd.Distance(EndCenter);

        SGMInternal::circle *pCircleStart=new SGMInternal::circle(rResult,StartCenter,-Axis,dRadiusStart,&XAxis);
        SGMInternal::circle *pCircleEnd=new SGMInternal::circle(rResult,EndCenter,Axis,dRadiusEnd,&XAxis);


        SGMInternal::body   *pBody=new SGMInternal::body(rResult); 
        SGMInternal::volume *pVolume=new SGMInternal::volume(rResult);

        SGMInternal::face *pRevolveFace=new SGMInternal::face(rResult);

        SGMInternal::edge *pEdgeBottom=new SGMInternal::edge(rResult);
        SGMInternal::edge *pEdgeTop=new SGMInternal::edge(rResult);
        SGMInternal::edge *pEdgeLeft=new SGMInternal::edge(rResult);
        SGMInternal::edge *pEdgeRight=new SGMInternal::edge(rResult);

        SGMInternal::vertex *pBottomLeft=new SGMInternal::vertex(rResult,SGM::Point3D(-2,0,.5));
        SGMInternal::vertex *pBottomRight=new SGMInternal::vertex(rResult,SGM::Point3D(-2,0,-.5));
        SGMInternal::vertex *pTopLeft=new SGMInternal::vertex(rResult,SGM::Point3D(2,0,2));
        SGMInternal::vertex *pTopRight=new SGMInternal::vertex(rResult,SGM::Point3D(2,0,-2));

        // Connect everything.

        pBody->AddVolume(pVolume);
        pVolume->AddFace(pRevolveFace);

        pRevolveFace->AddEdge(rResult,pEdgeBottom,SGM::FaceOnRightType);
        pRevolveFace->AddEdge(rResult,pEdgeTop,SGM::FaceOnRightType);
        pRevolveFace->AddEdge(rResult,pEdgeRight,SGM::FaceOnLeftType);
        pRevolveFace->AddEdge(rResult,pEdgeLeft,SGM::FaceOnRightType);

        pRevolveFace->SetSurface(pRevolve);
        pRevolveFace->SetSides(2);

        pEdgeBottom->SetStart(pBottomRight);
        pEdgeBottom->SetEnd(pBottomLeft);
        pEdgeRight->SetStart(pBottomRight);
        pEdgeRight->SetEnd(pTopRight);
        pEdgeTop->SetStart(pTopLeft);
        pEdgeTop->SetEnd(pTopRight);
        pEdgeLeft->SetStart(pBottomLeft);
        pEdgeLeft->SetEnd(pTopLeft);

        pEdgeBottom->SetCurve(pCircleStart);
        pEdgeTop->SetCurve(pCircleEnd);
        pEdgeLeft->SetCurve((SGMInternal::curve *)rResult.GetThing()->FindEntity(LeftCurveID.m_ID));
        pEdgeRight->SetCurve((SGMInternal::curve *)rResult.GetThing()->FindEntity(RightCurveID.m_ID));

        pEdgeBottom->SetDomain(rResult,SGM::Interval1D(SGM_PI/2.0, 3*SGM_PI/2.0));
        pEdgeTop->SetDomain(rResult,SGM::Interval1D(SGM_PI/2.0, 3*SGM_PI/2.0));
        pEdgeLeft->SetDomain(rResult,SGM::Interval1D(0,1));
        pEdgeRight->SetDomain(rResult,SGM::Interval1D(0,1));

        SGM::TranslatorOptions TranslatorOpts;
        SGM::SaveSTEP(rResult, "revolve_sheet.stp", rResult.GetThing()->GetID(),TranslatorOpts);
        /*
        SGM::CheckOptions Options;
        std::vector<std::string> CheckStrings;

        bAnswer = pRevolveFace->Check(rResult, Options, CheckStrings);
        if (!bAnswer) return bAnswer;

        bAnswer = pVolume->Check(rResult, Options, CheckStrings);
        if (!bAnswer) return bAnswer;

        bAnswer = pBody->Check(rResult, Options, CheckStrings);
        if (!bAnswer) return bAnswer;
        */
        return bAnswer;
        }
    
    if(nTestNumber==47)
        {
        bool bAnswer = true;

        SGM::Point3D Origin(1,1,-1.2);
        SGM::UnitVector3D Direction(0,-1,0);
        SGM::Point3D Center(1,0,0);
        SGM::UnitVector3D Axis(1,0,0);
        double dRadius = 1.5;
        double dTolerance = SGM_MIN_TOL;

        std::vector<SGM::Point3D> aPoints;
        std::vector<SGM::IntersectionType> aTypes;
        SGMInternal::IntersectLineAndCircle(Origin, Direction, SGM::Interval1D(-SGM_MAX, SGM_MAX), Center, Axis, dRadius, SGM_MIN_TOL, aPoints, aTypes);

        if (aPoints.size() != 2)
            return false;

        for (SGM::IntersectionType IType : aTypes )
            if (IType != SGM::PointType)
                return false;

        std::vector<SGM::Point3D> aExpected(2);
        aExpected[0] = SGM::Point3D(1,0.9,-1.2);
        aExpected[1] = SGM::Point3D(1,-0.9,-1.2);

        int found=0;
        for (SGM::Point3D PosExpected : aExpected)
            {
            for (SGM::Point3D PosComputed : aPoints)
                if (SGM::NearEqual(PosExpected, PosComputed, dTolerance))
                    found++;
            }
        return true;
        }

    if(nTestNumber==65)
        {
        std::vector<SGM::Point2D> aPoints;
        aPoints.push_back(SGM::Point2D(0,0));
        aPoints.push_back(SGM::Point2D(1,1));
        aPoints.push_back(SGM::Point2D(0,1));
        aPoints.push_back(SGM::Point2D(1,0));
        std::vector<unsigned int> aPolygon;
        aPolygon.push_back(0);
        aPolygon.push_back(1);
        aPolygon.push_back(2);
        aPolygon.push_back(3);
        std::vector<unsigned int> aTriangles;
        bool bAnswer=SGM::TriangulatePolygon(rResult,aPoints,aPolygon,aTriangles);
        int a=0;
        a*=1;
//        bAnswer = (found == 2);
//        return bAnswer;

        }

    return false;
    }


TEST(math_check, DISABLED_line_nub_surface_intersect)
    {
    bool bAnswer=true;

    SGMInternal::thing *pThing = SGMTesting::AcquireTestThing();
    SGM::Result rResult(pThing);

    std::vector<double> aUKnots,aVKnots;
    aUKnots.push_back(0.0);
    aUKnots.push_back(0.0);
    aUKnots.push_back(0.0);
    aUKnots.push_back(1.0);
    aUKnots.push_back(1.0);
    aUKnots.push_back(1.0);
    aVKnots=aUKnots;
    std::vector<std::vector<SGM::Point3D> > aaPoints;
    std::vector<SGM::Point3D> aPoints;
    aPoints.assign(3,SGM::Point3D(0,0,0));
    aaPoints.push_back(aPoints);
    aaPoints.push_back(aPoints);
    aaPoints.push_back(aPoints);
    aaPoints[0][0]=SGM::Point3D(0.0,0.0,1.0);
    aaPoints[0][1]=SGM::Point3D(0.0,1.0,0.0);
    aaPoints[0][2]=SGM::Point3D(0.0,2.0,-1.0);
    aaPoints[1][0]=SGM::Point3D(1.0,0.0,0.0);
    aaPoints[1][1]=SGM::Point3D(1.0,1.0,0.0);
    aaPoints[1][2]=SGM::Point3D(1.0,2.0,0.0);
    aaPoints[2][0]=SGM::Point3D(2.0,0.0,-1.0);
    aaPoints[2][1]=SGM::Point3D(2.0,1.0,0.0);
    aaPoints[2][2]=SGM::Point3D(2.0,2.0,1.0);
    SGM::Surface NUBSurfaceID=SGM::CreateNUBSurfaceFromControlPoints(rResult,aaPoints,aUKnots,aVKnots);

    // Test with a line that hits the saddle point.

    SGM::Point3D Pos0(0,0,0.0),Pos1(2,2,0.0);
    SGM::Curve LineID1=SGM::CreateLine(rResult,Pos0,Pos1-Pos0);

    std::vector<SGM::Point3D> aHits1;
    std::vector<SGM::IntersectionType> aTypes1;
    size_t nHits1=SGM::IntersectCurveAndSurface(rResult,LineID1,NUBSurfaceID,aHits1,aTypes1);

    if(nHits1!=1)
        {
        bAnswer=false;
        }
    else if(aTypes1[0]!=SGM::IntersectionType::TangentType)
        {
        bAnswer=false;
        }
    size_t Index1;
    for(Index1=0;Index1<nHits1;++Index1)
        {
        SGM::Point3D const &Pos=aHits1[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID1,Pos,&CPos1);
        SGM::SurfaceInverse(rResult,NUBSurfaceID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        if(SGM_ZERO<dDist)
            {
            bAnswer=false;
            }
        }
    SGM::DeleteEntity(rResult,LineID1);

    // Test with a line that hits two points.

    SGM::Point3D Pos2(0,0,0.5),Pos3(2,2,0.5);
    SGM::Curve LineID2=SGM::CreateLine(rResult,Pos2,Pos3-Pos2);

    std::vector<SGM::Point3D> aHits2;
    std::vector<SGM::IntersectionType> aTypes2;
    size_t nHits2=SGM::IntersectCurveAndSurface(rResult,LineID2,NUBSurfaceID,aHits2,aTypes2);

    if(nHits2!=2)
        {
        bAnswer=false;
        }
    for(Index1=0;Index1<nHits2;++Index1)
        {
        SGM::Point3D const &Pos=aHits2[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID2,Pos,&CPos1);
        SGM::SurfaceInverse(rResult,NUBSurfaceID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        if(SGM_ZERO<dDist)
            {
            bAnswer=false;
            }
        }
    SGM::DeleteEntity(rResult,LineID2);;

    // Test with a line that just misses the saddle but within tolernace.

    SGM::Point3D Pos4(2,0,0.0001),Pos5(0,2,0.0001);
    SGM::Curve LineID3=SGM::CreateLine(rResult,Pos4,Pos5-Pos4);

    std::vector<SGM::Point3D> aHits3;
    std::vector<SGM::IntersectionType> aTypes3;
    double dTestTol=0.001;
    size_t nHits3=SGM::IntersectCurveAndSurface(rResult,LineID3,NUBSurfaceID,aHits3,aTypes3,nullptr,nullptr,dTestTol);

    if(nHits3!=1)
        {
        bAnswer=false;
        }
    else if(aTypes3[0]!=SGM::IntersectionType::TangentType)
        {
        bAnswer=false;
        }
    for(Index1=0;Index1<nHits3;++Index1)
        {
        SGM::Point3D const &Pos=aHits3[Index1];
        SGM::Point3D CPos1,CPos2;
        SGM::CurveInverse(rResult,LineID3,Pos,&CPos1);
        SGM::SurfaceInverse(rResult,NUBSurfaceID,Pos,&CPos2);
        double dDist=CPos1.Distance(CPos2);
        if(dTestTol<dDist)
            {
            bAnswer=false;
            }
        }
    SGM::SaveSGM(rResult,"CoverageTest.sgm",SGM::Thing(),SGM::TranslatorOptions());
    SGM::DeleteEntity(rResult,LineID3);
    SGM::DeleteEntity(rResult,NUBSurfaceID);
    
    SGMTesting::ReleaseTestThing(pThing);

    EXPECT_TRUE(bAnswer);
    }

#endif

