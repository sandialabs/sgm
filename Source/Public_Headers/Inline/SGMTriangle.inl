#ifndef SGM_TRIANGLE_INL
#define SGM_TRIANGLE_INL

#include <iostream>

namespace SGM
{

///////////////////////////////////////////////////////////////////////////////
//
//  Triangle methods
//
///////////////////////////////////////////////////////////////////////////////

inline double SignedArea(Point2D const &A,
                         Point2D const &B,
                         Point2D const &C)
    {
    return ((A.m_u * B.m_v - B.m_u * A.m_v) + (B.m_u * C.m_v - C.m_u * B.m_v) + (C.m_u * A.m_v - A.m_u * C.m_v)) * 0.5;
    }

inline Point2D CenterOfMass(Point2D const &A,
                            Point2D const &B,
                            Point2D const &C)
    {
    return {(A.m_u + B.m_u + C.m_u) / 3.0, (A.m_v + B.m_v + C.m_v) / 3.0};
    }

inline Point3D CenterOfMass(Point3D const &A,
                            Point3D const &B,
                            Point3D const &C)
    {
    return {(A.m_x + B.m_x + C.m_x) / 3.0, (A.m_y + B.m_y + C.m_y) / 3.0, (A.m_z + B.m_z + C.m_z) / 3.0};
    }


//inline double TriangleSign(SGM::Point2D const &A, SGM::Point2D const &B, SGM::Point2D const &C)
//    {
//    double sign = (A.m_u - C.m_u) * (B.m_v - C.m_v) - (B.m_u - C.m_u) * (A.m_v - C.m_v);
//    return std::abs(sign) < SGM_ZERO ? 0.0 : sign;
//    }

// NOTE: slowest version
//inline bool InTriangle(Point2D const &A,
//                Point2D const &B,
//                Point2D const &C,
//                Point2D const &D)
//    {
//    double TOLERANCE = 2*std::numeric_limits<double>::epsilon();
//    bool d_side_of_ab = TriangleSign(D, B, A) <= 0;
//    bool c_side_of_ab = TriangleSign(C, B, A) <= 0;
//    if (d_side_of_ab == c_side_of_ab)
//        {
//        bool d_side_of_ac = TriangleSign(D, C, A) <= 0;
//        bool b_side_of_ac = TriangleSign(B, C, A) <= 0;
//        if (d_side_of_ac == b_side_of_ac)
//            {
//            bool d_side_of_bc = TriangleSign(D, C, B) <= 0;
//            bool a_side_of_bc = TriangleSign(A, C, B) <= 0;
//            if (d_side_of_bc == a_side_of_bc)
//                {
//                return true;
//                }
//            }
//        }
//    return false;
//    }

// NOTE: slower version
//inline bool InTriangle(SGM::Point2D const &A, SGM::Point2D const &B, SGM::Point2D const &C, SGM::Point2D const &P)
//    {
//    double s1, s2, s3;
//    bool has_negative, has_positive;
//    s1 = TriangleSign(P, A, B);
//    s2 = TriangleSign(P, B, C);
//    s3 = TriangleSign(P, C, A);
//    has_negative = (s1 < 0) || (s2 < 0) || (s3 < 0);
//    has_positive = (s1 > 0) || (s2 > 0) || (s3 > 0);
//    return !(has_negative && has_positive);
//    }

inline double SnapSmallNegativeToZero(double d)
    {
    return d < 0 && d > SGM_ZERO_NEGATIVE ? 0 : d;
    }

// Based on barycentric coordinates (s,t)
inline bool InTriangle2DImplementation(double dU, double dV,
                                       double dUCA, double dVCA, double dUBA, double dVBA, double dD)
    {
    double s_p = SnapSmallNegativeToZero(dVCA * dU - dUCA * dV);
    if (dD > 0)
        {
        if (s_p >= 0)
            {
            double t_p = SnapSmallNegativeToZero(dUBA * dV - dVBA * dU);
            return (t_p >= 0) && (s_p + t_p) <= dD;
            }
        return false;
        }
    else
        {
        if (s_p <= 0)
            {
            double t_p = SnapSmallNegativeToZero(dUBA * dV - dVBA * dU);
            return (t_p <= 0) && (s_p + t_p) >= dD;
            }
        return false;
        }
    }

    // Cached triangle data for fast 2D point in 2D triangle tests.

class TriangleData2D
    {
    public:

    TriangleData2D(Point2D const &A,
                   Point2D const &B,
                   Point2D const &C);

    bool InTriangle(Point2D const &P) const;

    private:

    SGM::Point2D m_A;
    SGM::Point2D m_B;
    SGM::Point2D m_C;

    double m_dU_CA;
    double m_dV_CA;
    double m_dU_BA;
    double m_dV_BA;
    double m_dD;
    };

inline TriangleData2D::TriangleData2D(Point2D const &A,
                                      Point2D const &B,
                                      Point2D const &C) :
    m_A(A),
    m_B(B),
    m_C(C),
    m_dU_CA(C.m_u - A.m_u),
    m_dV_CA(C.m_v - A.m_v),
    m_dU_BA(B.m_u - A.m_u),
    m_dV_BA(B.m_v - A.m_v),
    m_dD(m_dU_BA * m_dV_CA - m_dV_BA * m_dU_CA)
    {}

inline bool TriangleData2D::InTriangle(Point2D const &P) const
    {
    double dU = P.m_u - m_A.m_u;
    double dV = P.m_v - m_A.m_v;
    return InTriangle2DImplementation(dU, dV, m_dU_CA, m_dV_CA, m_dU_BA, m_dV_BA, m_dD);
    }

// Based on barycentric coordinates (s,t)
inline bool InTriangle(SGM::Point2D const &A, SGM::Point2D const &B, SGM::Point2D const &C, SGM::Point2D const &P)
    {
    double dU = P.m_u - A.m_u;
    double dV = P.m_v - A.m_v;
    double dUCA = C.m_u - A.m_u;
    double dVCA = C.m_v - A.m_v;
    double dUBA = B.m_u - A.m_u;
    double dVBA = B.m_v - A.m_v;
    double dD = dUBA * dVCA - dVBA * dUCA;
    return InTriangle2DImplementation(dU,dV,dUCA,dVCA,dUBA,dVBA,dD);
}

} // namespace SGM

#endif //SGM_TRIANGLE_INL

