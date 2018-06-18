#include <string>
#include <gtest/gtest.h>

#include "SGMBoxTree.h"

using SGM::UnitVector3D;
using SGM::Interval3D;
using SGM::Point3D;

using SGM::Ray3D;
using SGM::BoxTree;

TEST(boxtree_check, intersect_single_item_tree)
{
    BoxTree boxTree;

    Point3D point0(3, 3, 0);
    boxTree.Insert(&point0, {3,3,3,3,0,0});
    Ray3D ray_start_outside_parallel_x({1.0,3.0,0.0}, {1., 0., 0.});
    auto container = boxTree.FindIntersectsLine(ray_start_outside_parallel_x);
    EXPECT_EQ(container.size(), 1);
    EXPECT_EQ(container[0].first, &point0);
}