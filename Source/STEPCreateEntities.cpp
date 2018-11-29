
#include "SGMGraph.h"

#include "Curve.h"
#include "EntityFunctions.h"
#include "FileFunctions.h"
#include "Primitive.h"
#include "ReadFile.h"
#include "Surface.h"
#include "Topology.h"
#include "SGMTransform.h"

#include <numeric>

#include <iostream>

namespace SGMInternal
{

///////////////////////////////////////////////////////////////////////////////
//
// Functions called by CreateEntities
//
///////////////////////////////////////////////////////////////////////////////

// create a NUB or NURB curve

curve* CreateBSplineCurveFromSTEP(SGM::Result               &rResult,
                                  STEPLineDataMapType const &mSTEPData,
                                  STEPLineData        const &stepLineData)
    {
    std::vector<size_t> const &aIDs = stepLineData.m_aIDs;
    std::vector<int> const &aInts = stepLineData.m_aInts;
    std::vector<double> const &aDoubles = stepLineData.m_aDoubles;

    size_t nPoints = aIDs.size();
    size_t nKnots = aInts.size();

    std::vector<SGM::Point3D> aControlPoints;
    aControlPoints.reserve(nPoints);
    for (size_t Index1 = 0; Index1 < nPoints; ++Index1)
        {
        size_t nPointID = aIDs[Index1];
        std::vector<double> const &aPoint = mSTEPData.at(nPointID).m_aDoubles;
        aControlPoints.emplace_back(aPoint[0], aPoint[1], aPoint[2]);
        }
    size_t nMultiplicityKnots = (size_t)std::accumulate(aInts.begin(), aInts.end(), 0);
    std::vector<double> aKnots;
    aKnots.reserve(nMultiplicityKnots);
    for (size_t Index1 = 0; Index1 < nKnots; ++Index1)
        {
        double dKnot = aDoubles[Index1];
        size_t nMultiplicity = (size_t)aInts[Index1];
        aKnots.insert(aKnots.end(),nMultiplicity,dKnot); // push back N times
        }
    curve *pCurve = nullptr;
    if (stepLineData.m_bFlag)
        {
        // a NUB with no Weights
        assert(stepLineData.m_nSTEPTag == STEPTag::B_SPLINE_CURVE_WITH_KNOTS);
        pCurve = new NUBcurve(rResult, std::move(aControlPoints), std::move(aKnots));
        }
    else
        {
        // a NURB with Weights
        assert(stepLineData.m_nSTEPTag == STEPTag::BOUNDED_CURVE ||
               stepLineData.m_nSTEPTag == STEPTag::B_SPLINE_CURVE);
        std::vector<SGM::Point4D> aControlPoints4D;
        aControlPoints4D.reserve(nPoints);
        size_t Index2 = nKnots; // offset into the doubles array of STEPLineData
        for (size_t Index1 = 0; Index1 < nPoints; ++Index1,++Index2)
            {
            SGM::Point3D const &Pos = aControlPoints[Index1];
            aControlPoints4D.emplace_back(Pos.m_x, Pos.m_y, Pos.m_z, aDoubles[Index2]);
            }
        pCurve = new NURBcurve(rResult, std::move(aControlPoints4D), std::move(aKnots));
        }
        return pCurve;
    }

// Create a NUBsurface or a NURBsurface

surface* CreateBSplineSurfaceFromSTEP(SGM::Result               &rResult,
                                      STEPLineDataMapType const &mSTEPData,
                                      STEPLineData        const &stepLineData)
    {
    std::vector<size_t> const &aIDs = stepLineData.m_aIDs;
    std::vector<int> const &aInts = stepLineData.m_aInts;
    std::vector<double> const &aDoubles = stepLineData.m_aDoubles;
    std::vector<unsigned> const &aSizes = stepLineData.m_aSizes;

    int nDegreeU = aInts[0];
    int nDegreeV = aInts[1];
    unsigned nUKnots = aSizes[0];
    unsigned nVKnots = aSizes[1];
    size_t nCount = 2; // offset into aInts

    size_t Index1,Index2;
    size_t nMultiplicityKnots=0;
    size_t nEnd=nCount+nUKnots;
    for(Index1=nCount;Index1<nEnd;++Index1)
        {
        nMultiplicityKnots+=aInts[Index1];
        }
    std::vector<double> aUKnots;
    aUKnots.reserve(nMultiplicityKnots);
    for (Index1 = 0; Index1 < nUKnots; ++Index1)
        {
        double dKnot = aDoubles[Index1];
        size_t nMultiplicity = (size_t)aInts[nCount++];
        aUKnots.insert(aUKnots.end(),nMultiplicity,dKnot); // push back N times
        }

    nMultiplicityKnots=0;
    nEnd=nCount+nVKnots;
    for(Index1=nCount;Index1<nEnd;++Index1)
        {
        nMultiplicityKnots+=aInts[Index1];
        }
    std::vector<double> aVKnots;
    aVKnots.reserve(nMultiplicityKnots);
    for (Index1 = 0; Index1 < nVKnots; ++Index1)
        {
        size_t nMultiplicity = (size_t)aInts[nCount++];
        double dKnot = aDoubles[Index1 + nUKnots];
        aVKnots.insert(aVKnots.end(),nMultiplicity,dKnot); // push back N times
        }

    size_t nUControlPoints = aUKnots.size() - nDegreeU - 1;
    size_t nVControlPoints = aVKnots.size() - nDegreeV - 1;
    size_t nIDCount = 0;

    surface *pSurface = nullptr;

    if (stepLineData.m_nSTEPTag == STEPTag::B_SPLINE_SURFACE_WITH_KNOTS)
        {
        // create a NUBsurface (no weights)

        std::vector<std::vector<SGM::Point3D>> aaControlPoints;
        aaControlPoints.reserve(nUControlPoints);
        for (Index1 = 0; Index1 < nUControlPoints; ++Index1)
            {
            std::vector<SGM::Point3D> aControlPoints;
            aControlPoints.reserve(nVControlPoints);
            for (Index2 = 0; Index2 < nVControlPoints; ++Index2)
                {
                size_t nPointID = aIDs[nIDCount++];
                std::vector<double> const &aPoint = mSTEPData.at(nPointID).m_aDoubles;
                aControlPoints.emplace_back(aPoint[0], aPoint[1], aPoint[2]);
                }
            aaControlPoints.emplace_back(std::move(aControlPoints));
            }
        pSurface = new NUBsurface(rResult,aaControlPoints,aUKnots,aVKnots);
        }
    else
        {
        // create a NURBsurface with weights

        assert(stepLineData.m_nSTEPTag == STEPTag::BOUNDED_SURFACE  ||
               stepLineData.m_nSTEPTag == STEPTag::B_SPLINE_SURFACE);
        size_t nDoubleCount = nUKnots + nVKnots;
        std::vector<std::vector<SGM::Point4D>> aaControlPoints;
        aaControlPoints.reserve(nUControlPoints);

        for (Index1 = 0; Index1 < nUControlPoints; ++Index1)
            {
            std::vector<SGM::Point4D> aControlPoints;
            aControlPoints.reserve(nVControlPoints);
            for (Index2 = 0; Index2 < nVControlPoints; ++Index2)
                {
                double dWeight = aDoubles[nDoubleCount++];
                size_t nPointID = aIDs[nIDCount++];
                std::vector<double> const &aPoint = mSTEPData.at(nPointID).m_aDoubles;
                aControlPoints.emplace_back(aPoint[0], aPoint[1], aPoint[2], dWeight);
                }
            aaControlPoints.emplace_back(std::move(aControlPoints));
            }
        //TODO: overload NUBsurface constructor to take r-values and call std::move
        pSurface = new NURBsurface(rResult,std::move(aaControlPoints),std::move(aUKnots),std::move(aVKnots));
        }

    return pSurface;
    }

void GetAxesFromSTEP(STEPLineData        const &stepLineData,
                     STEPLineDataMapType const &mSTEPData,
                     SGM::Point3D              &Center,
                     SGM::UnitVector3D         &ZAxis,
                     SGM::UnitVector3D         &XAxis)
    {
    size_t nLength = stepLineData.m_aIDs.size();
    assert(nLength >= 2 && nLength <= 3);
    size_t nP = stepLineData.m_aIDs[0];
    size_t nN = stepLineData.m_aIDs[1];
    std::vector<double> const &aP = mSTEPData.at(nP).m_aDoubles;
    std::vector<double> const &aN = mSTEPData.at(nN).m_aDoubles;
    Center = SGM::Point3D(aP[0], aP[1], aP[2]);
    ZAxis = SGM::UnitVector3D(aN[0], aN[1], aN[2]);

    if (nLength == 2)
        {
        XAxis = ZAxis.Orthogonal();
        }
    else if (nLength == 3)
        {
        size_t nX = stepLineData.m_aIDs[2];
        std::vector<double> const &aX = mSTEPData.at(nX).m_aDoubles;
        XAxis = SGM::UnitVector3D(aX[0], aX[1], aX[2]);
        }
    else
        {
        throw std::runtime_error("AXIS is not 2D or 3D");
        }
    }

circle *CreateCircleFromSTEP(SGM::Result               &rResult,
                             const STEPLineData        &stepLineData,
                             const STEPLineDataMapType &mSTEPData)
    {
    size_t nAxis = stepLineData.m_aIDs[0];
    double dRadius = stepLineData.m_aDoubles[0];
    STEPLineData const &SLDA = mSTEPData.at(nAxis);
    SGM::Point3D Center;
    SGM::UnitVector3D ZAxis, XAxis;
    GetAxesFromSTEP(SLDA, mSTEPData, Center, ZAxis, XAxis);
    return new circle(rResult, Center, ZAxis, dRadius, &XAxis);
    }

cone *CreateConeFromSTEP(SGM::Result                &rResult,
                         const STEPLineData        &stepLineData,
                         const STEPLineDataMapType &mSTEPData)
    {
    size_t nAxis = stepLineData.m_aIDs[0];
    double dRadius = stepLineData.m_aDoubles[0];
    double dHalfAngle = stepLineData.m_aDoubles[1];
    STEPLineData const &SLDA = mSTEPData.at(nAxis);
    SGM::Point3D Center;
    SGM::UnitVector3D ZAxis, XAxis;
    GetAxesFromSTEP(SLDA, mSTEPData, Center, ZAxis, XAxis);
    ZAxis.Negate();
    return new cone(rResult, Center, ZAxis, dRadius, dHalfAngle, &XAxis);
    }

cylinder *CreateCylinderFromSTEP(SGM::Result               &rResult,
                                 const STEPLineData        &stepLineData,
                                 const STEPLineDataMapType &mSTEPData)
    {
    size_t nAxis = stepLineData.m_aIDs[0];
    double dRadius = stepLineData.m_aDoubles[0];
    STEPLineData const &SLDA = mSTEPData.at(nAxis);
    SGM::Point3D Center;
    SGM::UnitVector3D ZAxis, XAxis;
    GetAxesFromSTEP(SLDA, mSTEPData, Center, ZAxis, XAxis);
    return new cylinder(rResult, Center - ZAxis, Center + ZAxis, dRadius, &XAxis);
    }

torus *CreateTorusFromSTEP(SGM::Result         &rResult,
                           const STEPLineData  &stepLineData,
                           STEPLineDataMapType &mSTEPData,
                           bool                 bApple)
    {
    size_t nAxis = stepLineData.m_aIDs[0];
    double dMajor = stepLineData.m_aDoubles[0];
    double dMinor = stepLineData.m_aDoubles[1];
    STEPLineData const &SLDA = mSTEPData.at(nAxis);
    SGM::Point3D Center;
    SGM::UnitVector3D ZAxis, XAxis;
    GetAxesFromSTEP(SLDA, mSTEPData, Center, ZAxis, XAxis);
    return new torus(rResult, Center, ZAxis, dMinor, dMajor, bApple, &XAxis);
    }

ellipse *CreateEllipseFromSTEP(SGM::Result &rResult,
                               const STEPLineData &stepLineData,
                               const STEPLineDataMapType &mSTEPData)
    {
    size_t nAxis = stepLineData.m_aIDs[0];
    double dMajor = stepLineData.m_aDoubles[0];
    double dMinor = stepLineData.m_aDoubles[1];
    STEPLineData const &SLDA = mSTEPData.at(nAxis);
    SGM::Point3D Center;
    SGM::UnitVector3D ZAxis, XAxis;
    GetAxesFromSTEP(SLDA, mSTEPData, Center, ZAxis, XAxis);
    SGM::UnitVector3D YAxis = ZAxis * XAxis;
    return new ellipse(rResult, Center, XAxis, YAxis, dMajor, dMinor);
    }

line *CreateLineFromSTEP(SGM::Result &rResult,
                         const STEPLineData &stepLineData,
                         const STEPLineDataMapType &mSTEPData)
    {
    size_t nPos = stepLineData.m_aIDs[0];
    std::vector<double> const & aPos = mSTEPData.at(nPos).m_aDoubles;
    SGM::Point3D Origin(aPos[0], aPos[1], aPos[2]);
    size_t nVec = stepLineData.m_aIDs[1];
    size_t nDir = mSTEPData.at(nVec).m_aIDs[0];
    std::vector<double> const & aDir = mSTEPData.at(nDir).m_aDoubles;
    SGM::UnitVector3D Axis(aDir[0], aDir[1], aDir[2]);
    return new line(rResult, Origin, Axis);
    }

plane *CreatePlaneFromSTEP(SGM::Result &rResult,
                           const STEPLineData &stepLineData,
                           const STEPLineDataMapType &mSTEPData)
    {
    size_t nAxis = stepLineData.m_aIDs[0];
    STEPLineData const &SLDA = mSTEPData.at(nAxis);
    SGM::Point3D Origin;
    SGM::UnitVector3D ZAxis, XAxis;
    GetAxesFromSTEP(SLDA, mSTEPData, Origin, ZAxis, XAxis);
    SGM::UnitVector3D YAxis = ZAxis * XAxis;
    return new plane(rResult, Origin, XAxis, YAxis, ZAxis);
    }

sphere *CreateSphereFromSTEP(SGM::Result &rResult,
                             const STEPLineData &stepLineData,
                             const STEPLineDataMapType &mSTEPData)
    {
    size_t nAxis = stepLineData.m_aIDs[0];
    double dRadius = stepLineData.m_aDoubles[0];
    STEPLineData const &SLDA = mSTEPData.at(nAxis);
    SGM::Point3D Center;
    SGM::UnitVector3D ZAxis, XAxis;
    GetAxesFromSTEP(SLDA, mSTEPData, Center, ZAxis, XAxis);
    SGM::UnitVector3D YAxis = ZAxis * XAxis;
    return new sphere(rResult, Center, dRadius, &XAxis, &YAxis);
    }

extrude * CreateExtrudeFromSTEP(SGM::Result               &rResult,
                                const STEPLineData        &stepLineData,
                                const STEPLineDataMapType &mSTEPData)
    {
    size_t nVector = stepLineData.m_aIDs[1];
    STEPLineData const &SLDVector = mSTEPData.at(nVector);
    double dScale = SLDVector.m_aDoubles[0];
    size_t nDir = SLDVector.m_aIDs[0];
    std::vector<double> const & aDir = mSTEPData.at(nDir).m_aDoubles;
    SGM::UnitVector3D ZAxis(aDir[0], aDir[1], aDir[2]);
    if (dScale < 0)
        {
        ZAxis.Negate();
        }
    return new extrude(rResult, ZAxis, nullptr);
    }

revolve *CreateRevolveFromSTEP(SGM::Result               &rResult,
                               const STEPLineData        &stepLineData,
                               const STEPLineDataMapType &mSTEPData)
    {
    size_t nAxis = stepLineData.m_aIDs[1];
    STEPLineData const &SLDAxis = mSTEPData.at(nAxis);
    size_t nPos = SLDAxis.m_aIDs[0];
    size_t nDir = SLDAxis.m_aIDs[1];
    std::vector<double> const &aPos = mSTEPData.at(nPos).m_aDoubles;
    std::vector<double> const &aDir = mSTEPData.at(nDir).m_aDoubles;
    SGM::Point3D Pos(aPos[0], aPos[1], aPos[2]);
    SGM::UnitVector3D ZAxis(aDir[0], aDir[1], aDir[2]);
    return new revolve(rResult, Pos, ZAxis, nullptr);
    }

vertex *CreateVertexFromSTEP(SGM::Result &rResult,
                             const STEPLineData &stepLineData,
                             const STEPLineDataMapType &mSTEPData)
    {
    size_t nPos = stepLineData.m_aIDs[0];
    std::vector<double> const &aPos = mSTEPData.at(nPos).m_aDoubles;
    SGM::Point3D Pos(aPos[0], aPos[1], aPos[2]);
    return new vertex(rResult, Pos);
    }

// Connect the volumes to their bodies

void ConnectVolumesToBodies(STEPLineDataMapType const &mSTEPData,
                            IDEntityMapType     const &mIDToEntityMap,
                            std::set<entity *>        &sEntities,
                            std::vector<size_t>       &aBodies,
                            BodyToTransformMapType    &mBodyToTransforms)
    {
    size_t nBodies = aBodies.size();
    for (size_t Index1 = 0; Index1 < nBodies; ++Index1)
        {
        size_t nBodyID = aBodies[Index1];
        entity *pEntity = mIDToEntityMap.at(nBodyID);
        sEntities.insert(pEntity);

        body *pBody = dynamic_cast<body *>(pEntity);

        // VolumeID(s) ..., TransformID, JunkID

        STEPLineData const &SLD = mSTEPData.at(nBodyID);
        std::vector<size_t> const &aIDs = SLD.m_aIDs;
        size_t nID = aIDs.size();

        // next to last ID is the axes for the body
        size_t nTransformID = aIDs[nID-2];
        STEPLineData const &SLDA = mSTEPData.at(nTransformID);
        SGM::Point3D Center;
        SGM::UnitVector3D XAxis, ZAxis;
        GetAxesFromSTEP(SLDA, mSTEPData, Center, ZAxis, XAxis);

        if (!SGM::NearEqual(Center, SGM::Point3D(0.,0.,0.), SGM_MIN_TOL) &&
            !SGM::NearEqual(ZAxis, SGM::UnitVector3D(0.,0.,1.), SGM_MIN_TOL) &&
            !SGM::NearEqual(XAxis, SGM::UnitVector3D(1.,0.,0.), SGM_MIN_TOL) )
            {
            std::cout << "Body Axes" << std::endl;
            std::cout << "Center " << Center.m_x << " " << Center.m_y << " " << Center.m_z << std::endl;
            std::cout << "ZAxis " << ZAxis.m_x << " " << ZAxis.m_y << " " << ZAxis.m_z << std::endl;
            std::cout << "XAxis " << XAxis.m_x << " " << XAxis.m_y << " " << XAxis.m_z << std::endl;

            SGM::Transform3D BodyTransform(XAxis, ZAxis*XAxis, ZAxis, SGM::Vector3D(Center));
            mBodyToTransforms.emplace(std::pair<body*, SGM::Transform3D>(pBody, BodyTransform));
            }

        for (size_t Index2 = 0; Index2 < nID; ++Index2)
            {
            if (mIDToEntityMap.find(aIDs[Index2]) != mIDToEntityMap.end())
                {
                volume *pVolume = dynamic_cast<volume *>(mIDToEntityMap.at(aIDs[Index2]));
                if (pVolume)
                    {
                    pBody->AddVolume(pVolume);
                    }
                }
            }
        }
    }

void ConnectFacesAndEdgesToVolumes(SGM::Result               &rResult,
                                   STEPLineDataMapType const &mSTEPData,
                                   IDEntityMapType           &mIDToEntityMap,
                                   std::set<entity *>        &sEntities,
                                   std::vector<size_t> const &,//aFaces,
                                   std::vector<size_t> const &,//aEdges,
                                   std::vector<size_t>       &aVolumes)
    {
    size_t nVolumes = aVolumes.size();
    for (size_t Index1 = 0; Index1 < nVolumes; ++Index1)
        {
        size_t nVolumeID = aVolumes[Index1];
        entity *pEntity = mIDToEntityMap.at(nVolumeID);
        if (pEntity->IsTopLevel())
            {
            sEntities.insert(pEntity);
            }

        volume *pVolume = dynamic_cast<volume*>(pEntity);

        // ShellID(s) ...

        STEPLineData const& VolumeSTEPLineData = mSTEPData.at(nVolumeID);
        STEPTag nVolumeTag = VolumeSTEPLineData.m_nSTEPTag;
        std::vector<size_t> const &aVolumeChildIDs = VolumeSTEPLineData.m_aIDs;
        
        size_t nShells = aVolumeChildIDs.size();
        for (size_t Index2 = 0; Index2 < nShells; ++Index2)
            {
            // FaceID(s) ...
            
            if (nVolumeTag == STEPTag::GEOMETRIC_CURVE_SET)
                {
                
                // get type of child curve
                size_t nShellChildID = aVolumeChildIDs[Index2];

                curve *pCurve = nullptr;
                auto ShellChildIter = mIDToEntityMap.find(nShellChildID);
                if (ShellChildIter != mIDToEntityMap.end())
                    {
                    pCurve = dynamic_cast<curve *>(ShellChildIter->second);
                    }
                    
                if (pCurve == nullptr) // it was not in the ID -> entity map
                    {
                    STEPLineData const &CurveSTEPLineData = mSTEPData.at(nShellChildID);
                    STEPTag nCurveTag = CurveSTEPLineData.m_nSTEPTag;
                    if (nCurveTag == STEPTag::CARTESIAN_POINT)
                        {
                        body *pBody = pVolume->GetBody();
                        rResult.GetThing()->DeleteEntity(pBody);
                        rResult.GetThing()->DeleteEntity(pVolume);
                        sEntities.erase(pBody);
                        sEntities.erase(pVolume);
                        std::vector<double> const &aPoint = CurveSTEPLineData.m_aDoubles;
                        vertex *pVertex = new vertex(rResult, {aPoint[0], aPoint[1], aPoint[2]});
                        mIDToEntityMap[nVolumeID] = pVertex;
                        sEntities.insert(pVertex);
                        }
                    else
                        {
                        throw std::runtime_error("expected CARTESIAN_POINT inside GEOMETRIC_CURVE_SET");
                        }
                    }
                else if (pCurve->GetCurveType() == SGM::NUBCurveType)
                    {
                    NUBcurve *pNUB = dynamic_cast<NUBcurve *>(pCurve);
                    size_t nDegree = pNUB->GetDegree();
                    if (nDegree == 1)
                        {
                        // Deal with polylines.
                        std::vector<SGM::Point3D> const &aControlPoints = pNUB->GetControlPoints();
                        if (SGM::NearEqual(aControlPoints.front(), aControlPoints.back(), SGM_ZERO))
                            {
                            size_t nPoints = aControlPoints.size();
                            for (size_t Index3 = 1; Index3 < nPoints; ++Index3)
                                {
                                SGM::Point3D const &Pos0 = aControlPoints[Index3 - 1];
                                SGM::Point3D const &Pos1 = aControlPoints[Index3];
                                edge *pEdge = CreateEdge(rResult, Pos0, Pos1);
                                pVolume->AddEdge(pEdge);
                                }
                            rResult.GetThing()->DeleteEntity(pNUB);
                            }
                        else
                            {
                            throw;
                            }
                        }
                    }
                return;
                } // nVolumeTag == GEOMETRIC_CURVE_SET


            size_t nChildID = aVolumeChildIDs[Index2];
            STEPLineData const &VolumeChildSTEPLineData = mSTEPData.at(nChildID);
            STEPTag nChildTag = VolumeChildSTEPLineData.m_nSTEPTag;

            if (nChildTag == STEPTag::TRIMMED_CURVE)
                {
                for (size_t Index3 = 0; Index3 < nShells; ++Index3)
                    {
                    size_t mEdgeId = aVolumeChildIDs[Index3];
                    entity *pEdgeEntity = mIDToEntityMap.at(mEdgeId);
                    edge *pEdge = dynamic_cast<edge *>(pEdgeEntity);
                    pVolume->AddEdge(pEdge);
                    }
                }
            else
                {
                std::vector<size_t> aSubIDs = VolumeChildSTEPLineData.m_aIDs;
                int nSides = 1;
                bool bFlip = false;
                if (nChildTag == STEPTag::OPEN_SHELL)
                    {
                    nSides = 2;
                    }
                else if (nChildTag == STEPTag::ORIENTED_CLOSED_SHELL)
                    {
                    // #2094=ORIENTED_CLOSED_SHELL('',*,#2093,.F.);

                    size_t nShellID = aSubIDs[1];
                    STEPLineData const & SubSTEPLineData = mSTEPData.at(nShellID);
                    aSubIDs = SubSTEPLineData.m_aIDs;
                    if (SubSTEPLineData.m_bFlag == false)
                        {
                        bFlip = true;
                        }
                    }
                size_t nFaces = aSubIDs.size();
                for (size_t Index3 = 0; Index3 < nFaces; ++Index3)
                    {
                    size_t nSubID = aSubIDs[Index3];
                    entity *pFaceEntity = mIDToEntityMap.at(nSubID);
                    face *pFace = dynamic_cast<face *>(pFaceEntity);
                    pFace->SetSides(nSides);
                    if (bFlip)
                        {
                        pFace->SetFlipped(true);
                        }
                    pVolume->AddFace(pFace);
                    }
                }
            }
        }
    }

void ConnectEdgesAndSurfacesToFaces(SGM::Result               &rResult,
                                    STEPLineDataMapType const &mSTEPData,
                                    IDEntityMapType     const &mIDToEntityMap,
                                    std::set<entity *>        &sEntities,
                                    std::vector<size_t> const &,//aEdges,
                                    std::vector<size_t>       &aFaces)
    {
    size_t nFaces = aFaces.size();
    for (size_t Index1 = 0; Index1 < nFaces; ++Index1)
        {
        size_t nFaceID = aFaces[Index1];
        entity *pEntity = mIDToEntityMap.at(nFaceID);
        if (pEntity->IsTopLevel())
            {
            sEntities.insert(pEntity);
            }

        // LoopID(s) ..., SurfaceID, bFlag
        face *pFace = dynamic_cast<face*>(pEntity);

        STEPLineData const &SLD = mSTEPData.at(nFaceID);
        if (SLD.m_bFlag == false)
            {
            pFace->SetFlipped(true);
            }
        std::vector<size_t> const &aBoundIDs = SLD.m_aIDs;
        size_t nSurfaceID = aBoundIDs.back();
        surface *pSurface = dynamic_cast<surface *>(mIDToEntityMap.at(nSurfaceID));
        pFace->SetSurface(pSurface);
        switch (pSurface->GetSurfaceType())
            {
            case SGM::RevolveType:
                {
                STEPLineData const &SLDRevolve = mSTEPData.at(nSurfaceID);
                revolve *pRevolve = dynamic_cast<revolve*>(pSurface);
                curve *pCurve = dynamic_cast<curve *>(mIDToEntityMap.at(SLDRevolve.m_aIDs.front()));
                pRevolve->SetCurve(pCurve);
                break;
                }
            case SGM::ExtrudeType:
                {
                STEPLineData const &SLDExtrude = mSTEPData.at(nSurfaceID);
                extrude *pExtrude = dynamic_cast<extrude *>(pSurface);
                curve *pCurve = dynamic_cast<curve *>(mIDToEntityMap.at(SLDExtrude.m_aIDs.front()));
                pExtrude->SetCurve(pCurve);
                break;
                }
            default:
                {
                break;
                }
            }
        size_t nBounds = aBoundIDs.size() - 1;
        for (size_t Index2 = 0; Index2 < nBounds; ++Index2)
            {
            STEPLineData const &SLD2 = mSTEPData.at(aBoundIDs[Index2]);
            bool bLoopFlag = SLD2.m_bFlag;
            std::vector<size_t> const &aLoopIDs = SLD2.m_aIDs;
            size_t nLoopIDs = aLoopIDs.size();
            for (size_t Index3 = 0; Index3 < nLoopIDs; ++Index3)
                {
                STEPLineData const &SLD3 = mSTEPData.at(aLoopIDs[Index3]);
                std::vector<size_t> const &aCoedgeIDs = SLD3.m_aIDs;
                size_t nCoedgeIDs = aCoedgeIDs.size();
                std::set<size_t> sEdgeIDs, sDoubleSided;
                for (size_t Index4 = 0; Index4 < nCoedgeIDs; ++Index4)
                    {
                    size_t nCoedgeID = aCoedgeIDs[Index4];
                    STEPLineData const &SLD4 = mSTEPData.at(nCoedgeID);
                    size_t nEdgeID = SLD4.m_aIDs[2];
                    if (sEdgeIDs.find(nEdgeID) != sEdgeIDs.end())
                        {
                        sDoubleSided.insert(nEdgeID);
                        }
                    sEdgeIDs.insert(nEdgeID);
                    }
                for (size_t Index4 = 0; Index4 < nCoedgeIDs; ++Index4)
                    {
                    size_t nCoedgeID = aCoedgeIDs[Index4];
                    STEPLineData const &SLD4 = mSTEPData.at(nCoedgeID);
                    size_t nEdgeID = SLD4.m_aIDs[2];
                    SGM::EdgeSideType nEdgeSide = SGM::FaceOnBothSidesType;
                    edge *pEdge = dynamic_cast<edge *>(mIDToEntityMap.at(nEdgeID));
                    if (sDoubleSided.find(nEdgeID) == sDoubleSided.end())
                        {
                        bool bCoedgeFlag = SLD4.m_bFlag;
                        STEPLineData const &SLD5 = mSTEPData.at(nEdgeID);
                        bool bEdgeFlag = SLD5.m_bFlag;
                        nEdgeSide = SGM::FaceOnLeftType;
                        size_t nCount = 0;
                        if (bLoopFlag == false)
                            {
                            ++nCount;
                            }
                        if (bCoedgeFlag == false)
                            {
                            ++nCount;
                            }
                        if (bEdgeFlag == false)
                            {
                            ++nCount;
                            }
                        if (nCount % 2 == 1)
                            {
                            nEdgeSide = SGM::FaceOnRightType;
                            }
                        }
                    pFace->AddEdge(rResult, pEdge, nEdgeSide);
                    }
                }
            }
        }
    }
    
void ConnectCurvesAndVerticesToEdges(SGM::Result               &rResult,
                                     STEPLineDataMapType const &mSTEPData,
                                     IDEntityMapType     const &mIDToEntityMap,
                                     std::set<entity *>        &sEntities,
                                     std::vector<size_t> const &,//aFaces,
                                     std::vector<size_t> const &aEdges)
    {
    size_t nEdges = aEdges.size();
    for (size_t Index1 = 0; Index1 < nEdges; ++Index1)
        {
        size_t nEdgeID = aEdges[Index1];
        entity *pEntity = mIDToEntityMap.at(nEdgeID);
        if (pEntity->IsTopLevel())
            {
            sEntities.insert(pEntity);
            }

        // Start vertex ID, End vertex ID, Curve ID, bFlag
        edge *pEdge = dynamic_cast<edge*>(pEntity);

        STEPLineData const & SLD = mSTEPData.at(nEdgeID);
        if (SLD.m_aIDs.size() == 3) // EDGE_CURVE case
            {
            size_t nStartID = SLD.m_aIDs[0];
            size_t nEndID = SLD.m_aIDs[1];
            size_t nCurveID = SLD.m_aIDs[2];
            bool bEdgeFlag = SLD.m_bFlag;

            vertex *pStart = dynamic_cast<vertex *>(mIDToEntityMap.at(nStartID));
            vertex *pEnd = dynamic_cast<vertex *>(mIDToEntityMap.at(nEndID));
            curve *pCurve = dynamic_cast<curve *>(mIDToEntityMap.at(nCurveID));
            if (bEdgeFlag == false)
                {
                std::swap(pStart, pEnd);
                }
            if (pCurve->GetCurveType() == SGM::EntityType::CircleType)
                {
                //SGM::Point3D StartPos=pStart->GetPoint();
                double dStartParam = 0.0;//pCurve->Inverse(StartPos);
                SGM::Interval1D Domain(dStartParam, dStartParam + SGM_TWO_PI);
                pCurve->SetDomain(Domain);
                }
            pEdge->SetStart(pStart);
            pEdge->SetEnd(pEnd);
            pEdge->SetCurve(pCurve);
            SGM::Interval1D Domain = pEdge->GetDomain();
            if (Domain.IsEmpty())
                {
                Domain.m_dMax += pCurve->GetDomain().Length();
                pEdge->SetDomain(rResult, Domain);
                }
            }
        else // TRIMMED_CURVE case
            {
            size_t nCurveID = SLD.m_aIDs[0];
            bool bEdgeFlag = SLD.m_bFlag;

            double dStart = SLD.m_aDoubles[0];
            double dEnd = SLD.m_aDoubles[1];

            curve *pCurve = dynamic_cast<curve *>(mIDToEntityMap.at(nCurveID));
            if (bEdgeFlag == false)
                {
                std::swap(dStart, dEnd);
                }
            SGM::Point3D StartPos, EndPos;
            pCurve->Evaluate(dStart, &StartPos);
            pCurve->Evaluate(dEnd, &EndPos);

            vertex *pStart = new vertex(rResult, StartPos);
            vertex *pEnd = new vertex(rResult, EndPos);

            pEdge->SetStart(pStart);
            pEdge->SetEnd(pEnd);
            pEdge->SetCurve(pCurve);
            }
        }
    }

void ConnectSheetBodies(SGM::Result         &,//rResult,
                        std::vector<body *> &aSheetBodies)
    {
    // Make sheet bodies double sided.
    
    size_t nSheetBodies = aSheetBodies.size();
    for (size_t Index1 = 0; Index1 < nSheetBodies; ++Index1)
        {
        body *pBody = aSheetBodies[Index1];

        auto sBodyVolumes = pBody->GetVolumes();
        for (auto pVolume : sBodyVolumes)
            {
            auto sVolumeFaces = pVolume->GetFaces();
            for (auto pFace : sVolumeFaces)
                {
                pFace->SetSides(2); 
                }
            }
        }
    }

void ApplyBodyTransforms(SGM::Result &rResult,
                         BodyToTransformMapType &mBodyToTransforms)
    {
        for (auto BodyAndTransform : mBodyToTransforms)
        {
            body *pBody = BodyAndTransform.first;
            SGM::Transform3D &transform = BodyAndTransform.second;
            TransformEntity(rResult, transform, pBody);
        }
    }

typedef std::unordered_map<std::string, STEPTag> STEPTagMapType;
typedef std::unordered_map<size_t, STEPLineData> STEPLineDataMapType;
typedef std::unordered_map<size_t, entity *> IDEntityMapType;
typedef std::unordered_map<body *, SGM::Transform3D> BodyToTransformMapType;

// the SHAPE_REPRESENTATION data in STEP represents a node (assembly or part)
// in the assembly tree
struct STEPAssemblyNode
{
    STEPAssemblyNode(size_t SRID) :
        ShapeRepID(SRID),
        BrepID(0) {}
    size_t ShapeRepID;
    std::vector<size_t> aCDSRParents;
    std::vector<size_t> aCDSRChildren;
    size_t BrepID;
};

// the SHAPE_REPRESENTATION_RELATIONSHIP data ties a SHAPE_REPRESENTATION to a Brep
struct STEPShapeRepRelationData
{
    STEPShapeRepRelationData(size_t SRRid,
                             size_t SRid,
                             size_t brepid) :
        ShapeRepRelationID(SRRid),
        ShapeRepID(SRid),
        BrepID(brepid) {}
    size_t ShapeRepRelationID;
    size_t ShapeRepID;
    size_t BrepID;
};

// the CONTEXT_DEPENDENT_SHAPE_REPRESENTATION and its child REPRESENTATION_RELATIONSHIP
// represent a "link" between SHAPE_REPRESENTATIONs in the assembly tree
// and holds the transform that corresponds to the link
struct STEPContextDepShapeRepData
{
    STEPContextDepShapeRepData(size_t CDShapeRepID,
                                        size_t SRParentID,
                                        size_t SRChildID,
                                        size_t TransfID) :
        ContextDepShapeRepID(CDShapeRepID),
        ShapeRepParentID(SRParentID),
        ShapeRepChildID(SRChildID),
        TransformID(TransfID) {}
    size_t ContextDepShapeRepID;
    size_t ShapeRepParentID;
    size_t ShapeRepChildID;
    size_t TransformID;
};



void ReadAssemblyData(size_t                                             maxSTEPLineNumber,
                      STEPLineDataMapType                          const &mSTEPData,
                      std::map<size_t, STEPContextDepShapeRepData>       &mCDSRLinks,
                      std::vector<size_t>                                &aShapeRepIDs,
                      std::vector<STEPShapeRepRelationData>              &aSRRentities)
{
    for (size_t nID = 1; nID <= maxSTEPLineNumber; ++nID)
    {
        auto DataIter = mSTEPData.find(nID);
        if (DataIter == mSTEPData.end())
        {
            continue; // skip a STEP #ID that is not used
        }

        STEPLineData const &stepLineData = DataIter->second;

        if (stepLineData.m_nSTEPTag == STEPTag::CONTEXT_DEPENDENT_SHAPE_REPRESENTATION)
        {
            size_t RRid = stepLineData.m_aIDs[0];
            STEPLineData const &RRLineData = mSTEPData.at(RRid);

            mCDSRLinks.emplace(nID, STEPContextDepShapeRepData(nID, RRLineData.m_aIDs[0],
              RRLineData.m_aIDs[1], RRLineData.m_aIDs[2]));
        }
        else if (stepLineData.m_nSTEPTag == STEPTag::SHAPE_REPRESENTATION)
        {
            aShapeRepIDs.emplace_back(nID);
        }
        else if (stepLineData.m_nSTEPTag == STEPTag::SHAPE_REPRESENTATION_RELATIONSHIP)
        {
            aSRRentities.emplace_back(STEPShapeRepRelationData(nID, stepLineData.m_aIDs[0], stepLineData.m_aIDs[1]));
        }
    }
}

void InstantiateBodiesForLeafAssemblyNodes(SGM::Result &rResult,
                                           STEPAssemblyNode                             const &AsmParentNode,
                                           SGM::Transform3D                             const &ParentTransform,
                                           std::map<size_t, STEPAssemblyNode>           const &mAssemblyNodes,
                                           std::map<size_t, STEPContextDepShapeRepData> const &mCDSRLinks,
                                           STEPLineDataMapType                          const &mSTEPData,
                                           IDEntityMapType                              const &mIDToEntityMap,
                                           BodyToTransformMapType                             &mBodyToTransforms,
                                           std::set<entity *>                                 &sEntities)
{
    if (AsmParentNode.aCDSRChildren.size() == 0) // leaf node
    {
        size_t STEPBrepID = AsmParentNode.BrepID;
        entity *pEntity = mIDToEntityMap.at(STEPBrepID);
        body *pBody = dynamic_cast<body *>(pEntity);
        assert(pBody != nullptr);

        // copy the brep
        if (mBodyToTransforms.find(pBody) == mBodyToTransforms.end())
        {
            // reuse the already created body for the first instance
            mBodyToTransforms.emplace(pBody, ParentTransform);
        }
        else
        {
            entity *pCopy = CopyEntity(rResult, pEntity);
            body *pBody = dynamic_cast<body *>(pCopy);
            sEntities.emplace(pBody);
            mBodyToTransforms.emplace(pBody, ParentTransform);
        }
    }
    else
    {
        // keep traversing down to leaf nodes and accumulate transformations
        for (auto Child : AsmParentNode.aCDSRChildren)
        {
            // get the transform
            STEPContextDepShapeRepData const &CDSRChild = mCDSRLinks.at(Child);
            //size_t STEPTransformID = CDSRChild.TransformID;


            // parse the transforms
            STEPLineData const &TransformLineData = mSTEPData.at(CDSRChild.TransformID);
            size_t AxisID1 = TransformLineData.m_aIDs[0];
            STEPLineData STEPLineData1 = mSTEPData.at(AxisID1);
            size_t AxisID2 = TransformLineData.m_aIDs[1];
            STEPLineData STEPLineData2 = mSTEPData.at(AxisID2);

            SGM::Point3D Center1, Center2;
            SGM::UnitVector3D XAxis1, XAxis2;
            SGM::UnitVector3D ZAxis1, ZAxis2;
            GetAxesFromSTEP(STEPLineData1, mSTEPData, Center1, ZAxis1, XAxis1);
            GetAxesFromSTEP(STEPLineData2, mSTEPData, Center2, ZAxis2, XAxis2);

            SGM::Transform3D ChildTransform(XAxis1, SGM::UnitVector3D(ZAxis1*XAxis1), ZAxis1, Center1,
                                            XAxis2, SGM::UnitVector3D(ZAxis2*XAxis2), ZAxis2, Center2);

            // combine the transform with the parent transform
            SGM::Transform3D transform = ParentTransform * ChildTransform;

            //get the child node
            STEPAssemblyNode ChildNode = mAssemblyNodes.at(CDSRChild.ShapeRepChildID);

            InstantiateBodiesForLeafAssemblyNodes(rResult, ChildNode, transform, mAssemblyNodes, mCDSRLinks, mSTEPData, mIDToEntityMap, mBodyToTransforms, sEntities);
        }
    }
}



void FlattenAssemblies(SGM::Result &rResult,
                    size_t maxSTEPLineNumber,
                    STEPLineDataMapType const &mSTEPData,
                    IDEntityMapType const &mIDToEntityMap,
                    std::set<entity *> &sEntities)
{
    std::vector<STEPShapeRepRelationData> aSRRentities;
    std::map<size_t, STEPContextDepShapeRepData> mCDSRLinks;
    std::vector<size_t> aShapeRepIDs;
    BodyToTransformMapType mBodyToTransforms;

    // get line ids for CONTEXT_DEPENDENT_SHAPE_REPRESENTATION, SHAPE_DEFINITION_REPRESENTATION,
    // and SHAPE_REPRESENTATION_RELATIONSHIP
    ReadAssemblyData(maxSTEPLineNumber, mSTEPData, mCDSRLinks, aShapeRepIDs, aSRRentities);

    // create an assembly node for each shape rep and store for lookup by SHAPE_REPRESENTATION id
    std::map<size_t, STEPAssemblyNode> mAssemblyNodes;
    for (size_t SRID : aShapeRepIDs)
    {
        mAssemblyNodes.emplace(SRID, STEPAssemblyNode(SRID));
    }

    // add parent and child links in the assembly nodes
    for (auto Link : mCDSRLinks)
    {
        STEPContextDepShapeRepData CDSRData = Link.second;
        
        STEPAssemblyNode &SRParent = mAssemblyNodes.at(CDSRData.ShapeRepParentID);
        STEPAssemblyNode &SRChild = mAssemblyNodes.at(CDSRData.ShapeRepChildID);

        SRParent.aCDSRChildren.emplace_back(CDSRData.ContextDepShapeRepID);
        SRChild.aCDSRParents.emplace_back(CDSRData.ContextDepShapeRepID);
    }

    // add brep id to the assembly nodes
    for (auto SRR : aSRRentities)
    {
        STEPAssemblyNode &ShapeRep = mAssemblyNodes.at(SRR.ShapeRepID);
        assert(ShapeRep.aCDSRChildren.size() == 0);
        ShapeRep.BrepID = SRR.BrepID;
    }

    // recursively traverse the assembly to all leaf nodes
    for (auto Entry : mAssemblyNodes)
    {
        STEPAssemblyNode &AsmParentNode = Entry.second;
        if (AsmParentNode.aCDSRParents.size() == 0) // top level node
        {
            SGM::UnitVector3D XAxis(1.0, 0.0, 0.0);
            SGM::UnitVector3D YAxis(0.0, 1.0, 0.0);
            SGM::UnitVector3D ZAxis(0.0, 0.0, 1.0);
            SGM::Transform3D transform(XAxis, YAxis, ZAxis);

            InstantiateBodiesForLeafAssemblyNodes(rResult, AsmParentNode, transform, mAssemblyNodes, mCDSRLinks, mSTEPData, mIDToEntityMap, mBodyToTransforms, sEntities);
        }
    }

    // now apply all the assembly transforms
    ApplyBodyTransforms(rResult, mBodyToTransforms);
}

////////////////////////////////////////////////////////////////////////////////
//
// Create all entities (main function)
//
////////////////////////////////////////////////////////////////////////////////
void CreateEntities(SGM::Result &rResult,
                    size_t maxSTEPLineNumber,
                    STEPLineDataMapType &mSTEPData,
                    std::vector<entity *> &aEntities)
    {
    IDEntityMapType mIDToEntityMap;
    std::set<entity *> sEntities;
    std::vector<size_t> aBodies, aVolumes, aFaces, aEdges;
    std::vector<body *> aSheetBodies;
    BodyToTransformMapType mBodyToTransforms;

    // Create entities in a specific order by STEP #ID line number,
    // from 1 to maxSTEPLineNumber.

    for (size_t nID = 1; nID <= maxSTEPLineNumber; ++nID)
        {
        auto DataIter = mSTEPData.find(nID);
        if (DataIter == mSTEPData.end())
            {
            continue; // skip a STEP #ID that is not used
            }

        STEPLineData &stepLineData = DataIter->second;

        switch (stepLineData.m_nSTEPTag)
            {
            case STEPTag::ADVANCED_BREP_SHAPE_REPRESENTATION:
                {
                mIDToEntityMap[nID] = new body(rResult);
                aBodies.push_back(nID);
                break;
                }
            case STEPTag::ADVANCED_FACE:
                {
                mIDToEntityMap[nID] = new face(rResult);
                aFaces.push_back(nID);
                break;
                }
            case STEPTag::BOUNDED_CURVE:
            case STEPTag::B_SPLINE_CURVE:
            case STEPTag::B_SPLINE_CURVE_WITH_KNOTS:
                {
                mIDToEntityMap[nID] = CreateBSplineCurveFromSTEP(rResult, mSTEPData, stepLineData);
                break;
                }
            case STEPTag::BOUNDED_SURFACE:
            case STEPTag::B_SPLINE_SURFACE:
            case STEPTag::B_SPLINE_SURFACE_WITH_KNOTS:
                {
                mIDToEntityMap[nID] = CreateBSplineSurfaceFromSTEP(rResult, mSTEPData, stepLineData);
                break;
                }
            case STEPTag::BREP_WITH_VOIDS:
                {
                mIDToEntityMap[nID] = new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case STEPTag::CIRCLE:
                {
                mIDToEntityMap[nID] = CreateCircleFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            case STEPTag::CONICAL_SURFACE:
                {
                mIDToEntityMap[nID] = CreateConeFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            case STEPTag::CYLINDRICAL_SURFACE:
                {
                mIDToEntityMap[nID] = CreateCylinderFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            case STEPTag::DEGENERATE_TOROIDAL_SURFACE:
                {
                bool bApple = stepLineData.m_bFlag;
                mIDToEntityMap[nID] = CreateTorusFromSTEP(rResult, stepLineData, mSTEPData, bApple);
                break;
                }
            case STEPTag::EDGE_CURVE:
                {
                mIDToEntityMap[nID] = new edge(rResult);
                aEdges.push_back(nID);
                break;
                }
            case STEPTag::ELLIPSE:
                {
                mIDToEntityMap[nID] = CreateEllipseFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            case STEPTag::FACE_SURFACE:
                {
                mIDToEntityMap[nID] = new face(rResult);
                aFaces.push_back(nID);
                break;
                }
            case STEPTag::GEOMETRIC_CURVE_SET:
                {
                mIDToEntityMap[nID] = new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case STEPTag::GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION:
                {
                mIDToEntityMap[nID] = new body(rResult);
                aBodies.push_back(nID);
                break;
                }
            case STEPTag::LINE:
                {
                mIDToEntityMap[nID] = CreateLineFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            case STEPTag::MANIFOLD_SOLID_BREP:
                {
                mIDToEntityMap[nID] = new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case STEPTag::MANIFOLD_SURFACE_SHAPE_REPRESENTATION:
                {
                body *pBody = new body(rResult);
                aSheetBodies.push_back(pBody);
                mIDToEntityMap[nID] = pBody;
                aBodies.push_back(nID);
                break;
                }
            case STEPTag::PLANE:
                {
                mIDToEntityMap[nID] = CreatePlaneFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            case STEPTag::SHELL_BASED_SURFACE_MODEL:
                {
                mIDToEntityMap[nID] = new volume(rResult);
                aVolumes.push_back(nID);
                break;
                }
            case STEPTag::SPHERICAL_SURFACE:
                {
                mIDToEntityMap[nID] = CreateSphereFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            case STEPTag::SURFACE_OF_LINEAR_EXTRUSION:
                {
                mIDToEntityMap[nID] = CreateExtrudeFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            case STEPTag::SURFACE_OF_REVOLUTION:
                {
                mIDToEntityMap[nID] = CreateRevolveFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            case STEPTag::TOROIDAL_SURFACE:
                {
                bool bApple = true;
                mIDToEntityMap[nID] = CreateTorusFromSTEP(rResult, stepLineData, mSTEPData, bApple);
                break;
                }
            case STEPTag::TRIMMED_CURVE:
                {
                mIDToEntityMap[nID] = new edge(rResult);
                aEdges.push_back(nID);
                break;
                }
            case STEPTag::VERTEX_POINT:
                {
                mIDToEntityMap[nID] = CreateVertexFromSTEP(rResult, stepLineData, mSTEPData);
                break;
                }
            default:
                break;
            }
        }

    ConnectVolumesToBodies(mSTEPData,mIDToEntityMap,sEntities,aBodies,mBodyToTransforms);

    ConnectFacesAndEdgesToVolumes(rResult,mSTEPData,mIDToEntityMap,sEntities,aFaces,aEdges,aVolumes);

    ConnectEdgesAndSurfacesToFaces(rResult,mSTEPData,mIDToEntityMap,sEntities,aEdges,aFaces);

    ConnectCurvesAndVerticesToEdges(rResult,mSTEPData,mIDToEntityMap,sEntities,aFaces,aEdges);

    ConnectSheetBodies(rResult,aSheetBodies);

    ApplyBodyTransforms(rResult, mBodyToTransforms);

    FlattenAssemblies(rResult, maxSTEPLineNumber, mSTEPData, mIDToEntityMap, sEntities);

    // push set of entities into vector
    // TODO: could the caller just live with us returning the set? why copy to vector?
    aEntities.insert(aEntities.end(),sEntities.begin(),sEntities.end());

    } // CreateEntities

} // namespace SGMInternal