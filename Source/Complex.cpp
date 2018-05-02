#include "SGMMathematics.h"
#include "SGMComplex.h"
#include "SGMTranslators.h"
#include "EntityClasses.h"
#include <algorithm>

SGM::Complex SGM::CreateTriangles(SGM::Result                     &rResult,
                                  std::vector<SGM::Point3D> const &aPoints,
                                  std::vector<size_t>       const &aTriangles)
    {
    if(aPoints.empty() || aTriangles.empty())
        {
        rResult.SetResult(SGM::ResultTypeInsufficientData);
        return SGM::Complex(0);
        }
    Impl::thing *pThing=rResult.GetThing();
    Impl::complex *pComplex=new Impl::complex(rResult,aPoints,aTriangles);
    pThing->AddTopLevelEntity(pComplex);
    return SGM::Complex(pComplex->GetID());
    }

size_t SGM::FindComponents(SGM::Result               &,//rResult,
                           SGM::Complex const        &,//ComplexID,
                           std::vector<SGM::Complex> &)//aComponents)
    {
    return 0;
    }

size_t SGM::FindBoundary(SGM::Result               &,//rResult,
                         SGM::Complex        const &,//ComplexID,
                         std::vector<SGM::Complex> &)//aBoundary)
    {
    return 0;
    }

size_t SGM::FindGenus(SGM::Result        &,//rResult,
                      SGM::Complex const &)//ComplexID)
    {
    return 0;
    }

size_t SGM::SplitWithPlane(SGM::Result               &,//rResult,
                           SGM::Complex        const &,//ComplexID,
                           SGM::Point3D        const &,//Point,
                           SGM::UnitVector3D   const &,//Normal,
                           std::vector<SGM::Complex> &)//aComponents)
    {
    return 0;
    }

size_t SGM::SplitWithSlices(SGM::Result                     &,//rResult,
                            SGM::Complex              const &,//ComplexID,
                            std::vector<SGM::Complex> const &,//aSlices,
                            std::vector<SGM::Complex>       &)//aComponents)
    {
    return 0;
    }

size_t SGM::SplitWithComplex(SGM::Result               &,//rResult,
                             SGM::Complex        const &,//ComplexID,
                             SGM::Complex        const &,//SliceID,
                             std::vector<SGM::Complex> &)//aComponents)
    {
    return 0;
    }

SGM::Complex SGM::CreateRectangle(SGM::Result        &rResult,
                                  SGM::Point2D const &Pos0,
                                  SGM::Point2D const &Pos1,
                                  bool                bFilled)
    {
    Impl::thing *pThing=rResult.GetThing();

    double dMinX=std::min(Pos0.m_u,Pos1.m_u);
    double dMaxX=std::max(Pos0.m_u,Pos1.m_u);
    double dMinY=std::min(Pos0.m_v,Pos1.m_v);
    double dMaxY=std::max(Pos0.m_v,Pos1.m_v);

    std::vector<SGM::Point3D> aPoints;
    aPoints.reserve(4);
    aPoints.emplace_back(dMinX,dMinY,0);
    aPoints.emplace_back(dMaxX,dMinY,0);
    aPoints.emplace_back(dMaxX,dMaxY,0);
    aPoints.emplace_back(dMinX,dMaxY,0);

    Impl::complex *pComplex;
    if(bFilled)
        {
        std::vector<size_t> aTriangles;
        aTriangles.reserve(6);
        aTriangles.push_back(0);
        aTriangles.push_back(1);
        aTriangles.push_back(3);
        aTriangles.push_back(3);
        aTriangles.push_back(1);
        aTriangles.push_back(2);
        pComplex=new Impl::complex(rResult,aPoints,aTriangles);
        }
    else
        {
        std::vector<size_t> aSegments;
        aSegments.reserve(8);
        aSegments.push_back(0);
        aSegments.push_back(1);
        aSegments.push_back(1);
        aSegments.push_back(2);
        aSegments.push_back(2);
        aSegments.push_back(3);
        aSegments.push_back(3);
        aSegments.push_back(0);
        pComplex=new Impl::complex(rResult,aSegments,aPoints);
        }

    pThing->AddTopLevelEntity(pComplex);
    return Complex(pComplex->GetID());
    }

SGM::Complex SGM::CreateSlice(SGM::Result             &,//rResult,
                    SGM::Complex      const &,//ComplexID,
                    SGM::Point3D      const &,//Point,
                    SGM::UnitVector3D const &,//Normal,
                    bool                     )//bLocal)
    {
    return Complex(0);
    }

SGM::Complex SGM::CreatePolygon(SGM::Result                     &,//rResult,
                      std::vector<Point3D> const &,//aPoints,
                      bool                             )//bFilled)
    {
    return Complex(0);
    }

namespace SGM { namespace Impl {

complex::complex(SGM::Result &rResult):
    entity(rResult,SGM::EntityType::ComplexType)
    {
    m_pThing = rResult.GetThing();
    m_Box.clear();
    }

complex::complex(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints):
    entity(rResult,SGM::EntityType::ComplexType),m_aPoints(aPoints)
    {
    m_pThing = rResult.GetThing();
    m_Box=SGM::FindBoundingBox3D(m_aPoints);
    }

complex::complex(SGM::Result                     &rResult,
                 std::vector<size_t>       const &aSegments,
                 std::vector<SGM::Point3D> const &aPoints):
    entity(rResult,SGM::EntityType::ComplexType),m_aPoints(aPoints),m_aSegments(aSegments)
    {
    m_pThing = rResult.GetThing();
    m_Box=SGM::FindBoundingBox3D(m_aPoints);
    }

complex::complex(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 std::vector<size_t>       const &aTriangles):
    entity(rResult,SGM::EntityType::ComplexType),m_aPoints(aPoints),m_aTriangles(aTriangles)
    {
    m_pThing = rResult.GetThing();
    m_Box=SGM::FindBoundingBox3D(m_aPoints);
    }

SGM::Interval3D const &complex::GetBox() const
    {
    if(m_Box.IsEmpty())
        {
        m_Box=SGM::FindBoundingBox3D(m_aPoints);
        }
    return m_Box;
    }

double complex::Area() const
    {
    size_t Index1;
    size_t nTriangles=m_aTriangles.size();
    double dArea=0;
    for(Index1=0;Index1<nTriangles;Index1+=3)
        {
        SGM::Point3D const &A=m_aPoints[Index1];
        SGM::Point3D const &B=m_aPoints[Index1+1];
        SGM::Point3D const &C=m_aPoints[Index1+2];
        dArea+=((A-B)*(C-B)).Magnitude();
        }
    return dArea*0.5;
    }

}}
