#include <limits>
#include <string>
#include <gtest/gtest.h>

#include "SGMVector.h"
#include "SGMInterval.h"

using SGM::UnitVector3D;
using SGM::Interval3D;
using SGM::Ray3D;

TEST(interval_check, box_ray_intersections) {

    Interval3D bbox(2., 4., 1., 4., 0., 0.); // zero thickness in z-direction

    UnitVector3D unit_yy_slabs(4./5., 3./5., 0.);
    UnitVector3D unit_xx_slabs(1./sqrt(17), 4./sqrt(17), 0.0);
    UnitVector3D unit_xy_slabs(2./sqrt(20), 4./sqrt(20), 0.0);
    UnitVector3D unit_hit_vertex(1./sqrt(5), 2/sqrt(5), 0);
    UnitVector3D unit_parallel_x(1., 0., 0.);
    UnitVector3D unit_parallel_y(0., 1., 0.);
    UnitVector3D unit_parallel_z(0., 0., 1.);
    UnitVector3D unit_parallel_x_negative(-1., 0., 0.);
    UnitVector3D unit_parallel_y_negative(0., -1., 0.);
    UnitVector3D unit_parallel_z_negative(0., 0., -1.);

    Ray3D ray_start_outside_cross_yy({0.0,0.0,0.0}, unit_yy_slabs);
    Ray3D ray_start_outside_cross_xx({2.5,0.5,0.0}, unit_xx_slabs);
    Ray3D ray_start_outside_cross_xy({1.0,1.5,0.0}, unit_xy_slabs);
    Ray3D ray_start_outside_hit_vertex({1.0,2.0,0.0}, unit_hit_vertex);
    Ray3D ray_start_outside_parallel_x({1.0,4.0,0.0}, unit_parallel_x);
    Ray3D ray_start_outside_parallel_y({4.0,0.5,0.0}, unit_parallel_y);
    Ray3D ray_start_outside_parallel_z({2.0,1.0,-1.0}, unit_parallel_z);
    Ray3D ray_start_outside_parallel_x_negative({5.0,1.0,0.0}, unit_parallel_x_negative);
    Ray3D ray_start_outside_parallel_y_negative({2.0,5.0,0.0}, unit_parallel_y_negative);
    Ray3D ray_start_outside_parallel_z_negative({4.0,4.0,4.0}, unit_parallel_z_negative);
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_cross_yy));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_cross_xx));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_cross_xy));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_hit_vertex));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_parallel_x));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_parallel_y));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_parallel_z));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_parallel_x_negative));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_parallel_y_negative));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_outside_parallel_z_negative));

    // rays starting inside the box
    Ray3D ray_start_inside_cross_yy({3,2,0}, unit_yy_slabs);
    Ray3D ray_start_inside_cross_xx({3.0,2.5,0.0}, unit_xx_slabs);
    Ray3D ray_start_inside_cross_xy({2.5,3.75,0.0}, unit_xy_slabs);
    Ray3D ray_start_on_vertex({2.0,4.0,0.0}, unit_hit_vertex);
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_inside_cross_yy));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_inside_cross_xx));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_inside_cross_xy));
    EXPECT_TRUE(bbox.IntersectsRay(ray_start_on_vertex));

    // rays towards the box, but barely outside by tolerance
    Ray3D ray_misses_parallel_x({1.0,4.0+1.E12,0.0}, unit_parallel_x);
    Ray3D ray_misses_parallel_y({4.0+1.E12,0.5,0.0}, unit_parallel_y);
    Ray3D ray_misses_parallel_z({2.0,1.0+1.E12,-1.0}, unit_parallel_z);
    Ray3D ray_misses_parallel_x_negative({5.0,1.0-1.E12,0.0}, unit_parallel_x_negative);
    Ray3D ray_misses_parallel_y_negative({2.0-1.E12,5.0,0.0}, unit_parallel_y_negative);
    Ray3D ray_misses_parallel_z_negative({2.0,1.0-1.E12,1.0}, unit_parallel_z_negative);
    EXPECT_FALSE(bbox.IntersectsRay(ray_misses_parallel_x));
    EXPECT_FALSE(bbox.IntersectsRay(ray_misses_parallel_y));
    EXPECT_FALSE(bbox.IntersectsRay(ray_misses_parallel_z));
    EXPECT_FALSE(bbox.IntersectsRay(ray_misses_parallel_x_negative));
    EXPECT_FALSE(bbox.IntersectsRay(ray_misses_parallel_y_negative));
    EXPECT_FALSE(bbox.IntersectsRay(ray_misses_parallel_z_negative));

    // starting point of ray outside and direction away from the box
    Ray3D ray_outside_away_cross_yy({6,4.5,0}, unit_yy_slabs);
    Ray3D ray_outside_away_cross_xx({0.0,0.0,0.0}, unit_xx_slabs);
    Ray3D ray_outside_away_cross_xy({3.0,6.0,0.0}, unit_xy_slabs);
    Ray3D ray_outside_away_vertex({2.5,5.0,0.0}, unit_hit_vertex);
    Ray3D ray_outside_away_parallel_x({5.0,1.0,0.0}, unit_parallel_x);
    Ray3D ray_outside_away_parallel_y({2.0,5.0,0.0}, unit_parallel_y);
    Ray3D ray_outside_away_parallel_z({2.0,3.0,1.E12}, unit_parallel_z);
    Ray3D ray_outside_away_parallel_x_negative({1.0,1.0,0.0}, unit_parallel_x_negative);
    Ray3D ray_outside_away_parallel_y_negative({3.0,0.5,0.0}, unit_parallel_y_negative);
    Ray3D ray_outside_away_parallel_z_negative({2.0,3.0,-1.E12}, unit_parallel_z_negative);
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_cross_yy));
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_cross_xx));
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_cross_xy));
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_vertex));
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_parallel_x));
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_parallel_y));
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_parallel_z));
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_parallel_x_negative));
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_parallel_y_negative));
    EXPECT_FALSE(bbox.IntersectsRay(ray_outside_away_parallel_z_negative));
}

TEST(interval_check, box_line_intersections) {
    Interval3D bbox(2., 4., 1., 4., 0., 0.); // zero thickness in z-direction
    UnitVector3D unit_yy_slabs(4./5., 3./5., 0.);
    UnitVector3D unit_xx_slabs(1./sqrt(17), 4./sqrt(17), 0.0);
    UnitVector3D unit_xy_slabs(2./sqrt(20), 4./sqrt(20), 0.0);
    UnitVector3D unit_hit_vertex(1./sqrt(5), 2/sqrt(5), 0);
    UnitVector3D unit_parallel_x(1., 0., 0.);
    UnitVector3D unit_parallel_y(0., 1., 0.);
    UnitVector3D unit_parallel_z(0., 0., 1.);
    UnitVector3D unit_parallel_x_negative(-1., 0., 0.);
    UnitVector3D unit_parallel_y_negative(0., -1., 0.);
    UnitVector3D unit_parallel_z_negative(0., 0., -1.);

    // rays that start outside the box and go away from box
    Ray3D ray_outside_away_cross_yy({6,4.5,0}, unit_yy_slabs);
    Ray3D ray_outside_away_cross_xx({4.0,4.5,0.0}, unit_xx_slabs);
    Ray3D ray_outside_away_cross_xy({2.5,4.5,0.0}, unit_xy_slabs);
    Ray3D ray_outside_away_vertex({3.0,6.0,0.0}, unit_hit_vertex);
    Ray3D ray_outside_away_parallel_x({5.0,1.0,0.0}, unit_parallel_x);
    Ray3D ray_outside_away_parallel_y({2.0,5.0,0.0}, unit_parallel_y);
    Ray3D ray_outside_away_parallel_z({2.0,3.0,1.E12}, unit_parallel_z);
    Ray3D ray_outside_away_parallel_x_negative({1.0,1.0,0.0}, unit_parallel_x_negative);
    Ray3D ray_outside_away_parallel_y_negative({3.0,0.5,0.0}, unit_parallel_y_negative);
    Ray3D ray_outside_away_parallel_z_negative({2.0,3.0,-1.E12}, unit_parallel_z_negative);
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_cross_yy));
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_cross_xx));
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_cross_xy));
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_vertex));
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_parallel_x));
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_parallel_y));
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_parallel_z));
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_parallel_x_negative));
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_parallel_y_negative));
    EXPECT_TRUE(bbox.IntersectsLine(ray_outside_away_parallel_z_negative));

    // rays parallel towards the box, but barely outside by tolerance
    Ray3D ray_misses_parallel_x({1.0,4.0+1.E12,0.0}, unit_parallel_x);
    Ray3D ray_misses_parallel_y({4.0+1.E12,0.5,0.0}, unit_parallel_y);
    Ray3D ray_misses_parallel_z({2.0,1.0+1.E12,-1.0}, unit_parallel_z);
    Ray3D ray_misses_parallel_x_negative({5.0,1.0-1.E12,0.0}, unit_parallel_x_negative);
    Ray3D ray_misses_parallel_y_negative({2.0-1.E12,5.0,0.0}, unit_parallel_y_negative);
    Ray3D ray_misses_parallel_z_negative({2.0,1.0-1.E12,1.0}, unit_parallel_z_negative);
    EXPECT_FALSE(bbox.IntersectsLine(ray_misses_parallel_x));
    EXPECT_FALSE(bbox.IntersectsLine(ray_misses_parallel_y));
    EXPECT_FALSE(bbox.IntersectsLine(ray_misses_parallel_z));
    EXPECT_FALSE(bbox.IntersectsLine(ray_misses_parallel_x_negative));
    EXPECT_FALSE(bbox.IntersectsLine(ray_misses_parallel_y_negative));
    EXPECT_FALSE(bbox.IntersectsLine(ray_misses_parallel_z_negative));

    
}

TEST(interval_check, box_segment_intersections) {
    Interval3D bbox(2., 4., 1., 4., 0., 0.); // zero thickness in z-direction
    EXPECT_FALSE(bbox.IntersectsSegment({3.5,4.5,0.0}, {5.5,6.0,0.0}));
    EXPECT_FALSE(bbox.IntersectsSegment({5.5,6.0,0.0}, {3.5,4.5,0.0}));
    EXPECT_FALSE(bbox.IntersectsSegment({1.0,3.5,0.0}, {3.0,5.5,0.0}));

    // through diagonal or end-point on vertex
    EXPECT_TRUE(bbox.IntersectsSegment({3.0,3.0,0.0}, {5.0,5.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({5.0,5.0,0.0}, {3.0,3.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({4.0,4.0,0.0}, {5.0,5.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({3.0,3.0,0.0}, {1.0,5.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({1.0,5.0,0.0}, {3.0,3.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({2.0,4.0,0.0}, {1.0,5.0,0.0}));

    // parallel to x-direction
    EXPECT_TRUE(bbox.IntersectsSegment({3.0,3.0,0.0}, {5.0,3.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({5.0,3.0,0.0}, {3.0,3.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({3.0,3.0,0.0}, {3.5,3.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({2.0,3.0,0.0}, {3.0,3.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({3.0,3.0,0.0}, {2.0,3.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({1.0,2.0,0.0}, {2.0,2.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({2.0,2.0,0.0}, {1.0,2.0,0.0}));
    EXPECT_FALSE(bbox.IntersectsSegment({4.0+1.1e-12,2.0,0.0}, {5.0,2.0,0.0})); // barely outside
    EXPECT_FALSE(bbox.IntersectsSegment({1.0,2.0,0.0}, {2.0-1.1e-12,2.0,0.0})); // barely outside

    // parallel to y-direction
    EXPECT_TRUE(bbox.IntersectsSegment({2.0,2.0,0.0}, {2.0,3.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({2.0,3.0,0.0}, {2.0,2.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({4.0,2.0,0.0}, {4.0,3.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({4.0,3.0,0.0}, {4.0,2.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({4.0,1.0,0.0}, {4.0,0.0,0.0}));
    EXPECT_TRUE(bbox.IntersectsSegment({4.0,4.0,0.0}, {4.0,5.0,0.0}));

    EXPECT_FALSE(bbox.IntersectsSegment({4.0,4.0+1.1e-12,0.0}, {4.0,5.0,0.0})); // barely outside
    EXPECT_FALSE(bbox.IntersectsSegment({4.0,-1.0,0.0}, {4.0,1.0-1.1e-12,0.0})); // barely outside

    // parallel to z-direction
    EXPECT_TRUE(bbox.IntersectsSegment({2.0,4.0,0.0}, {2.0,4.0,1.0})); // corner
    EXPECT_TRUE(bbox.IntersectsSegment({2.0,4.0,-1.0}, {2.0,4.0,0.0})); // corner
    EXPECT_TRUE(bbox.IntersectsSegment({3.0,3.0,-1.0}, {3.0,3.0,1.0})); // interior
    EXPECT_FALSE(bbox.IntersectsSegment({3.0,3.0,-1.0}, {3.0,3.0,0.0-1.1e-12})); // barely outside
    EXPECT_FALSE(bbox.IntersectsSegment({3.0,3.0,0.0+1.1e-12}, {3.0,3.0,1.0})); // barely outside
}
