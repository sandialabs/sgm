#include <chrono>
#include <random>
#include <ctime>
#include <string>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <functional>

#include "SGMBoxTree.h"

#define SGM_TIMER
#include "Timer.h"

#if 0
int main(int /*argc*/, char ** /*argv*/)
    {
    return 0;
    }
#endif

///////////////////////////////////////////////////////////////////////////////
//
// Example of timing the insertion of a large number of randomly
// sized bounding boxes in the SGM::BoxTree.
//
// Also queries of all boxes in the tree enclosed in a given box,
// and removing those boxes.
//
///////////////////////////////////////////////////////////////////////////////

struct LeafCounter {
	int count;
	bool bContinueVisiting;
	
	LeafCounter() : count(0), bContinueVisiting(true) {};
	
	void operator()(SGM::BoxTree::Leaf const *)
	{
		count++;
		//std::cout << "Visiting " << count << std::endl;
	}
};

std::string ToString(SGM::Interval3D const& box)
{
	std::stringstream ss("");
	ss << '[';
	ss << '(' << box.m_XDomain.m_dMin << ',' << box.m_XDomain.m_dMax << "),";
	ss << '(' << box.m_YDomain.m_dMin << ',' << box.m_YDomain.m_dMax << "),";
	ss << '(' << box.m_ZDomain.m_dMin << ',' << box.m_ZDomain.m_dMax << ")]";
	return ss.str();
}



void boxtree_timing()
{
    std::cout << std::endl << "*** Timing BoxTree Methods *** " << std::endl << std::flush;
    SGM::BoxTree tree;
    LeafCounter leafCounter;

    //std::random_device rd;  // using random seed
    //std::mt19937 mt(rd());

    std::mt19937 mt; // using default identical seed every runtime
    auto dist_origin = std::bind(std::uniform_real_distribution<double>(0.,900.), mt);
    auto dist_length = std::bind(std::uniform_real_distribution<double>(1.,100.), mt);

    SGM_TIMER_INITIALIZE();

    const size_t nodes = 1000000;
    const void * ptr = nullptr;
    SGM_TIMER_START("Insert nodes:");
    std::cout << "    Insert: " << nodes << " nodes " << std::flush;
    for (int i = 0; i<nodes; i++) {
        double xmin = dist_origin();
        double ymin = dist_origin();
        double zmin = dist_origin();
        double xlen = dist_length();
        double ylen = dist_length();
        double zlen = dist_length();
        tree.Insert(ptr, SGM::Interval3D(xmin, xmin+xlen, ymin, ymin+ylen, zmin, zmin+zlen));
        }
    SGM_TIMER_STOP();

    SGM::Interval3D bound(100.0, 300.0, 100.0, 400.0, 100.0, 500.0);

    SGM_TIMER_START("Visit all nodes");
    leafCounter = tree.Query(SGM::BoxTree::IsAny(), LeafCounter());
    std::cout << "    IsAny: " << leafCounter.count << " nodes visited (" << tree.Size() << " nodes in tree) ";
    SGM_TIMER_STOP();

    SGM_TIMER_START("Search in a box:");
    leafCounter = tree.Query(SGM::BoxTree::IsEnclosing(bound), LeafCounter());
    std::cout << "    IsEnclosing " << leafCounter.count << " nodes (" << tree.Size() << " nodes in tree) ";
    SGM_TIMER_STOP();

    SGM_TIMER_START("Remove enclosed in box:");
    tree.EraseEnclosed(bound);
    std::cout << "    EraseEnclosed " << ToString(bound) << " ";
    SGM_TIMER_STOP();

    SGM_TIMER_START("Search in a box: ");
    leafCounter = tree.Query(SGM::BoxTree::IsEnclosing(bound), LeafCounter());
    std::cout << "    IsEnclosing " << leafCounter.count << " nodes. (" << tree.Size() << " nodes in tree) ";
    SGM_TIMER_STOP();

    SGM_TIMER_SUM();
}

class ObjectConstructorDefault
{
public:
    double x;
    double y;
    double z;

    ObjectConstructorDefault() = default;

    double Product()
    {
        return x * y * z;
    }
};

class ObjectConstructorEmpty
{
public:
    double x;
    double y;
    double z;

    ObjectConstructorEmpty() {}

    double Product()
    {
        return x * y * z;
    }
};

/*
void constructor_timing()
{
    std::cout << std::endl <<"*** Timing Constructors of Trivial Objects *** " << std::endl << std::flush;

    SGM_TIMER_INITIALIZE();

    const size_t count = 10000000;
    double total_product = 0.0;

    SGM_TIMER_START("ObjectConstructorEmpty");
    for (size_t i = 0; i < count; ++i)
        {
        ObjectConstructorEmpty t;
        double product = t.Product();
        total_product += product;
        }
    SGM_TIMER_STOP();
    std::cout << "    total_product = " << total_product << std::endl << std::flush;

    SGM_TIMER_START("ObjectConstructorDefault");
    for (size_t i = 0; i < count; ++i)
        {
        ObjectConstructorDefault t;
        double product = t.Product();
        total_product += product;
        }
    std::cout << " total_product = " << total_product << std::endl << std::flush;
    SGM_TIMER_STOP();

    SGM_TIMER_SUM();
}

#ifdef SGM_SSE
inline double dot_vec2d_sse(double *Vec0, double *Vec1)
    {
    const int mask = 0x31;
    __m128d res = _mm_dp_pd(_mm_loadu_pd(&Vec0[0]), _mm_loadu_pd(&Vec1[0]), mask);
    return res[0];
    }
#endif

#ifdef SGM_SSE
inline double dot_vec3d_sse(double *Vec0, double *Vec1)
    {
    const int mask = 0x31;
    __m128d res = _mm_dp_pd(_mm_loadu_pd(&Vec0[0]), _mm_loadu_pd(&Vec1[0]), mask);
    return res[0] + Vec0[2]*Vec1[2];
    }
#endif

#ifdef SGM_SSE
void sse_timing()
    {
    std::cout << std::endl <<"*** Timing SSE instructions for vector operations *** " << std::endl << std::flush;
    SGM_TIMER_INITIALIZE();

//    double vec0[] = { 1.500000,10.250000};
//    double vec1[] = {-1.500000, 3.125000};
//    double result = dot_vec2d(vec0, vec1);
//    std::cout << "dot_vec2d = " << result << std::endl;
//    result = dot_vec2d_sse(vec0,vec1);
//    std::cout << "dot_vec2d_sse = " << result << std::endl;
//    std::cout << std::flush;

    std::mt19937 mt; // using default identical seed every runtime
    auto rand_value = std::bind(std::uniform_real_distribution<double>(-10.,10.0), mt);

    const int N = 50000000;
    double total;

/////////////////// 2D //////////////////////
//    std::vector<double> v0;
//    std::vector<double> v1;
//    v0.reserve(2*N);
//    v1.reserve(2*N);
//    total = 0.0;
//    for (int i = 0; i<N; ++i) {
//        v0.push_back(rand_value());
//        v0.push_back(rand_value());
//        v1.push_back(rand_value());
//        v1.push_back(rand_value());
//        }
//
//    SGM_TIMER_START("dot_vec2d");
//    for (int i = 0; i<N; ++i)
//        {
//        int j = 2*i;
//        total += dot_vec2d(&v0[j], &v1[j]);
//        }
//    SGM_TIMER_STOP();
//    std::cout << "    total dot_vec2d = " << total << std::endl;
//
//    total = 0.0;
//    SGM_TIMER_START("dot_vec2d_sse");
//    for (int i = 0; i<N; ++i)
//        {
//        int j = 2*i;
//        total += dot_vec2d_sse(&v0[j], &v1[j]);
//        }
//    SGM_TIMER_STOP();
//    std::cout << "    total dot_vec2d_sse = " << total << std::endl;

/////////////////// 3D //////////////////////

    std::vector<double> u0;
    std::vector<double> u1;
    u0.reserve(3*N);
    u1.reserve(3*N);
    total = 0.0;
    for (int i = 0; i<N; ++i) {
        u0.push_back(rand_value());
        u0.push_back(rand_value());
        u0.push_back(rand_value());
        u1.push_back(rand_value());
        u1.push_back(rand_value());
        u1.push_back(rand_value());
        }

    SGM_TIMER_START("dot_vec3d");
    for (int i = 0; i<N; ++i)
        {
        int j = 3*i;
        total += dot_vec3d(&u0[j], &u1[j]);
        }
    SGM_TIMER_STOP();
    std::cout << "    total dot_vec3d = " << total << std::endl;

    total = 0.0;
    SGM_TIMER_START("dot_vec3d_sse");
    for (int i = 0; i<N; ++i)
        {
        int j = 3*i;
        total += dot_vec3d_sse(&u0[j], &u1[j]);
        }
    SGM_TIMER_STOP();
    std::cout << "    total dot_vec3d_sse = " << total << std::endl;

    SGM_TIMER_SUM();
    }
#endif
 */

int main(int /*argc*/, char ** /*argv*/)
{
    //sse_timing();
    boxtree_timing();
    //constructor_timing();
	return 0;
}

