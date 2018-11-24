#ifndef FACETER_H
#define FACETER_H

#include "SGMVector.h"
#include "EntityClasses.h"

#define FACET_FACE_ANGLE_TOLERANCE 0.17453292519943295769236907684886 // 10 degrees.
#define FACET_EDGE_ANGLE_TOLERANCE 0.08726646259971647884618453842443 //  5 degrees.
#define FACET_HALF_TANGENT_OF_FACET_FACE_ANGLE 0.08816349035423248673554519343431 //0.5*tan(10)

namespace SGMInternal
{

class FacetOptions
    {
    public:

        FacetOptions():
            m_dFaceAngleTol(FACET_FACE_ANGLE_TOLERANCE), // 10 degrees.
            m_dEdgeAngleTol(FACET_EDGE_ANGLE_TOLERANCE), //  5 degrees.
            //m_dEdgeAngleTol(0.17453292519943295769236907684886),
            m_dMaxLength(0),
            m_dCordHeight(0),
            m_nMaxFacets(10000) {}

        double m_dFaceAngleTol;
        double m_dEdgeAngleTol;
        double m_dMaxLength;
        double m_dCordHeight;
        size_t m_nMaxFacets;
    };

void FacetCurve(curve               const *pCurve,
                SGM::Interval1D     const &Domain,
                FacetOptions        const &Options,
                std::vector<SGM::Point3D> &aPoints3D,
                std::vector<double>       &aParams);

void FacetEdge(SGM::Result               &rResult,
               edge                const *pEdge,
               FacetOptions        const &Options,
               std::vector<SGM::Point3D> &aPoints3D,
               std::vector<double>       &aParams);

void FacetFace(SGM::Result                    &rResult,
               face                     const *pFace,
               FacetOptions             const &Options,
               std::vector<SGM::Point2D>      &aPoints2D,
               std::vector<SGM::Point3D>      &aPoints3D,
               std::vector<SGM::UnitVector3D> &aNormals,
               std::vector<unsigned int>      &aTriangles);

bool FacetFaceLoops(SGM::Result                             &rResult, 
                    face                              const *pFace,
                    std::vector<SGM::Point2D>               &aPoints2D,
                    std::vector<SGM::Point3D>               &aPoints3D,
                    std::vector<std::vector<unsigned int> > &aaPolygons,
                    edge                                    *pInputEdge=nullptr,
                    std::vector<bool>                       *pImprintFlags=nullptr);

//bool FlipTriangles(std::vector<SGM::Point2D>      const &aPoints,
//                   std::vector<unsigned int>            &aTriangles,
//                   std::vector<unsigned int>            &aAdjacencies,
//                   unsigned int                          nTri,
//                   unsigned int                          nEdge,
//                   std::vector<SGM::Point3D>      const *pPoints3D=nullptr,
//                   std::vector<SGM::UnitVector3D> const *pNormals=nullptr,
//                   std::vector<size_t>            const *aTris=nullptr,
//                   SGM::BoxTree                         *Tree=nullptr);

//void FixBackPointers(unsigned int                     nTri,
//                     std::vector<unsigned int> const &aTriangles,
//                     std::vector<unsigned int>       &aAdjacencies);

void DelaunayFlips(std::vector<SGM::Point2D>      const &aPoints,
                   std::vector<unsigned int>            &aTriangles,
                   std::vector<unsigned int>            &aAdjacencies);


//  Devides all triangles into four triangles in the following way;
//  Points on edges are moved to the edges.
//       
//      / \
//     /___\
//    / \  /\
//   /___\/__\

void SubdivideFacets(face                const *pFace,
                     std::vector<SGM::Point3D> &aPoints3D,
                     std::vector<SGM::Point2D> &aPoints2D,
                     std::vector<unsigned int> &aTriangles,
                     std::vector<entity *>     &aEntities);

void InsertPoints(std::vector<SGM::Point2D> const &aInsertPoints,
                  std::vector<SGM::Point2D>       &aPoints,
                  std::vector<unsigned int>       &aTriangles);


//SGM::Interval3D TriangleBox(std::vector<SGM::Point2D> &aPoints,
//                            std::vector<unsigned int> &aTriangles,
//                            size_t                     nTri);

void SplitTriangleUpdateTree(SGM::Point2D        const &D,
                             std::vector<SGM::Point2D> &aPoints,
                             std::vector<unsigned int> &aTriangles,
                             size_t                     nHitTri,
                             std::vector<size_t> const &aTris,
                             SGM::BoxTree              &Tree);

void SplitEdgeUpdateTree(SGM::Point2D        const &D,
                         std::vector<SGM::Point2D> &aPoints,
                         std::vector<unsigned int> &aTriangles,
                         size_t                     nHitTri,
                         size_t                     nEdge1,
                         size_t                     nOther,
                         size_t                     nEdge2,
                         std::vector<size_t> const &aTris,
                         SGM::BoxTree              &Tree);

void SplitEdgeUpdateTree(SGM::Point2D        const &D,
                         std::vector<SGM::Point2D> &aPoints,
                         std::vector<unsigned int> &aTriangles,
                         size_t                     nHitTri,
                         size_t                     nEdge1,
                         std::vector<size_t> const &aTris,
                         SGM::BoxTree              &Tree);

}

#endif // FACETER_H
