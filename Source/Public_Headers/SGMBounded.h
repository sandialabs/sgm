#ifndef SGM_BOUNDED_H
#define SGM_BOUNDED_H

#include "SGMInterval.h"

namespace SGM {

    /**
     * A class for items having a bounding box in 3D (Interval3D)
     * implementing many functors (struct with an operator())
     * for modifying bounding boxes and comparing bounding boxes for sorting.
     *
     * This is also the base class of internal BoxTree::Node and BoxTree::Leaf.
     */
    struct Bounded
    {

        Interval3D *m_Bound; // Note that Bounded does not own m_Bound.

        /**
         * Functor that stretches the bounding box to include the given bounding box.
         */
        struct Stretch
        {
            Interval3D *m_bound;

            explicit Stretch(Interval3D *bound)
                    : m_bound(bound)
            {}

            void operator()(Bounded const *item)
            {
                m_bound->Stretch(*(item->m_Bound));
            }
        };

        /**
         * Functor for determining if one bounding box's first edge is less than a second bounding box's first edge
         * along a given axis in [0,1,2] (x,y,z coordinates).
         */
        struct FirstEdgeLess
        {
            std::size_t m_axis;

            explicit FirstEdgeLess(const std::size_t axis)
                    : m_axis(axis)
            {}

            bool operator()(Bounded *const bi1, Bounded *const bi2) const
            {
                if (m_axis == 0)
                    return bi1->m_Bound->m_XDomain.m_dMin < bi2->m_Bound->m_XDomain.m_dMin;
                if (m_axis == 1)
                    return bi1->m_Bound->m_YDomain.m_dMin < bi2->m_Bound->m_YDomain.m_dMin;
                return bi1->m_Bound->m_ZDomain.m_dMin < bi2->m_Bound->m_ZDomain.m_dMin;
            }
        };

        /**
         * Functor for determining if one bounding box's second edge is less than a second bounding box's second edge
         * along a given axis in [0,1,2] (x,y,z coordinates).
         */
        struct SecondEdgeLess
        {
            std::size_t m_axis;

            explicit SecondEdgeLess(const std::size_t axis)
                    : m_axis(axis)
            {}

            bool operator()(Bounded *const bi1, Bounded *const bi2) const
            {
                if (m_axis == 0)
                    return bi1->m_Bound->m_XDomain.m_dMax < bi2->m_Bound->m_XDomain.m_dMax;
                if (m_axis == 1)
                    return bi1->m_Bound->m_YDomain.m_dMax < bi2->m_Bound->m_YDomain.m_dMax;
                return bi1->m_Bound->m_ZDomain.m_dMax < bi2->m_Bound->m_ZDomain.m_dMax;
            }
        };

        /**
         * Functor for determining the squared distance between the centers of two bounding boxes.
         */
        struct CenterDistanceLess
        {
            Interval3D const *m_center;

            explicit CenterDistanceLess(Interval3D const *center)
                    : m_center(center)
            {}

            bool operator()(Bounded *const bi1, Bounded *const bi2) const
            {
                return bi1->m_Bound->SquaredDistanceFromCenters(*m_center) < bi2->m_Bound->SquaredDistanceFromCenters(
                        *m_center);
            }
        };

        /**
         * Functor that returns true if the difference in volume between bounding box 1 and the center bounding box is less
         * than difference in volume between bounding box 2 and the center bounding box.
         *
         * This functor is used to minimize the volume covered by a parent box in the tree, that is,
         * help minimize the dead space covered by the parent box that is not covered by children.
         */
        struct VolumeLess
        {
            double volume;

            explicit VolumeLess(const Interval3D *center)
                    : volume(center->Volume())
            {}

            bool operator()(Bounded *const bi1, Bounded *const bi2) const
            {
                return volume - bi1->m_Bound->Volume() < volume - bi2->m_Bound->Volume();
            }
        };

        struct OverlapLess
        {
            Interval3D const *m_bbox;

            explicit OverlapLess(Interval3D const *bbox)
                    : m_bbox(bbox)
            {}

            bool operator()(Bounded *const bi1, Bounded *const bi2) const
            {
                return bi1->m_Bound->IntersectingVolume(*m_bbox) < bi2->m_Bound->IntersectingVolume(*m_bbox);
            }
        };
    };
} // namespace SGM

#endif //SGM_BOUNDED_H
