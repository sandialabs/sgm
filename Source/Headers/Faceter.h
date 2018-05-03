#ifndef FACETER_H
#define FACETER_H

#include "SGMDataClasses.h"
#include "EntityClasses.h"

class FacetOptions
    {
    public:

        FacetOptions():
            m_dFaceAngleTol(0.26179938779914943653855361527329),     // 15 degrees.
            m_dEdgeAngleTol(0.17453292519943295769236907684886),     // 10 degrees.
            m_dFreeEdgeAngleTol(0.08726646259971647884618453842443), //  5 degrees.
            m_dMaxLength(0),
            m_dCordHight(0),
            m_nMaxFacets(10000) {}

        double m_dFaceAngleTol;
        double m_dEdgeAngleTol;
        double m_dFreeEdgeAngleTol;
        double m_dMaxLength;
        double m_dCordHight;
        size_t m_nMaxFacets;
    };

void FacetCurve(curve               const *pCurve,
                SGM::Interval1D     const &Domain,
                FacetOptions        const &Options,
                std::vector<SGM::Point3D> &aPoints3D,
                std::vector<double>       *aParams=nullptr);

void FacetEdge(edge                const *pEdge,
               FacetOptions        const &Options,
               std::vector<SGM::Point3D> &aPoints3D);

void FacetFace(SGM::Result                    &rResult,
               face                     const *pFace,
               FacetOptions             const &Options,
               std::vector<SGM::Point2D>      &aPoints2D,
               std::vector<SGM::Point3D>      &aPoints3D,
               std::vector<SGM::UnitVector3D> &aNormals,
               std::vector<size_t>            &aTriangles,
               std::vector<entity *>          &aEntities);

size_t FacetFaceLoops(SGM::Result                       &rResult, 
                      face                        const *pFace,
                      FacetOptions                const &Options,
                      std::vector<SGM::Point2D>         &aPoints2D,
                      std::vector<SGM::Point3D>         &aPoints3D,
                      std::vector<std::vector<size_t> > &aaPolygons);

bool FlipTriangles(std::vector<SGM::Point2D> const &aPoints,
                   std::vector<size_t>             &aTriangles,
                   std::vector<size_t>             &aAdjacencies,
                   size_t                           nTri,
                   size_t                           nEdge);

void FixBackPointers(size_t                     nTri,
                     std::vector<size_t> const &aTriangles,
                     std::vector<size_t>       &aAdjacencies);

void DelaunayFlips(std::vector<SGM::Point2D> const &aPoints,
                   std::vector<size_t>             &aTriangles,
                   std::vector<size_t>             &aAdjacencies);

#endif // FACETER_H
