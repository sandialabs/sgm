#ifndef SGM_TREE_H
#define SGM_TREE_H

#include "SGMDataClasses.h"
#include <vector>
#include <set>

namespace SGM
    {
    class TreeNode
        {
        public:

            size_t            m_nParent;
            SGM::Interval3D   m_Box;
            std::set<size_t>  m_sChildren;
            void       const *m_pData;
        };

    class BoxTree
        {
        public:

            BoxTree(size_t nReserve=0);

            void AddBox(SGM::Interval3D const &Box,
                        void            const *pData);

            void RemoveBox(SGM::Interval3D const &Box,
                           void            const *pData);

            void AddPoint(SGM::Point3D const &Point,
                          void         const *pData);

            void RemovePoint(SGM::Point3D const &Point,
                             void         const *pData);

            // Methods to find all data intersecting an object.

            size_t FindBox(SGM::Interval3D     const &Box,
                           std::vector<void const *> &aData);

            size_t FindPoint(SGM::Point3D        const &Center,
                             double                     dRadius,
                             std::vector<void const *> &aData);

            size_t FindLine(SGM::Point3D        const &Origin,
                            SGM::UnitVector3D   const &Axis,
                            double                     dRadius,
                            std::vector<void const *> &aData);

            size_t FindRay(SGM::Point3D        const &Origin,
                           SGM::UnitVector3D   const &Axis,
                           double                     dRadius,
                           std::vector<void const *> &aData);

            size_t FindSegment(SGM::Point3D        const &Start,
                               SGM::Point3D        const &End,
                               double                     dRadius,
                               std::vector<void const *> &aData);

            size_t FindPlane(SGM::Point3D        const &Origin,
                             SGM::UnitVector3D   const &Axis,
                             double                     dTolerance,
                             std::vector<void const *> &aData);

            size_t FindHalfSpace(SGM::Point3D        const &Origin,
                                 SGM::UnitVector3D   const &Axis,
                                 double                     dTolerance,
                                 std::vector<void const *> &aData);

            // Methods to find the first data intersecting an object.

            void *FindBox(SGM::Interval3D const &Box);

            void *FindPoint(SGM::Point3D const &Center,
                            double              dRadius);

            void *FindLine(SGM::Point3D      const &Origin,
                           SGM::UnitVector3D const &Axis,
                           double                   dRadius);

            void *FindRay(SGM::Point3D      const &Origin,
                          SGM::UnitVector3D const &Axis,
                          double                   dRadius);

            void *FindSegment(SGM::Point3D const &Start,
                              SGM::Point3D const &End,
                              double              dRadius);

            void *FindPlane(SGM::Point3D      const &Origin,
                            SGM::UnitVector3D const &Axis,
                            double                   dTolerance);

            void *FindHalfSpace(SGM::Point3D      const &Origin,
                                SGM::UnitVector3D const &Axis,
                                double                   dTolerance);

        private:

            void AddNode(size_t nID);

            void SplitLeaf(size_t nID);

            void Reinsert(size_t nID);

            size_t                     m_nRoot;
            std::vector<SGM::TreeNode> m_aData;
            std::vector<size_t>        m_aFree;
        };

    } // End of SGM namespace

#endif // SGM_TREE_H