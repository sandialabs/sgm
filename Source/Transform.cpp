#include "SGMVector.h"
#include "SGMTransform.h"

///////////////////////////////////////////////////////////////////////////
//
//  Transform member functions and operators non-inline
//
///////////////////////////////////////////////////////////////////////////

namespace SGM {

    void Rotate(Point3D const &Origin,
                     UnitVector3D const &Axis,
                     double dAngle,
                     Transform3D &Trans)
    {
        UnitVector3D XAxis = Axis.Orthogonal();
        UnitVector3D YAxis = Axis * XAxis;
        Transform3D Trans0(XAxis, YAxis, Axis, Vector3D(Origin));
        Transform3D Trans1;
        Trans0.Inverse(Trans1);
        double dCos = cos(dAngle);
        double dSin = sin(dAngle);
        UnitVector3D ZAxis(0, 0, 1);
        UnitVector3D XRotate = XAxis * dCos + YAxis * dSin;
        UnitVector3D YRotate = ZAxis * XRotate;
        Transform3D Trans2(XRotate, YRotate, ZAxis);
        Trans = (Trans1 * Trans2) * Trans0;
    }

    void Transform3D::Inverse(Transform3D &Trans) const
    {
        double s0 = m_Matrix[0].m_x * m_Matrix[1].m_y - m_Matrix[1].m_x * m_Matrix[0].m_y;
        double s1 = m_Matrix[0].m_x * m_Matrix[1].m_z - m_Matrix[1].m_x * m_Matrix[0].m_z;
        double s2 = m_Matrix[0].m_x * m_Matrix[1].m_w - m_Matrix[1].m_x * m_Matrix[0].m_w;
        double s3 = m_Matrix[0].m_y * m_Matrix[1].m_z - m_Matrix[1].m_y * m_Matrix[0].m_z;
        double s4 = m_Matrix[0].m_y * m_Matrix[1].m_w - m_Matrix[1].m_y * m_Matrix[0].m_w;
        double s5 = m_Matrix[0].m_z * m_Matrix[1].m_w - m_Matrix[1].m_z * m_Matrix[0].m_w;

        double c5 = m_Matrix[2].m_z * m_Matrix[3].m_w - m_Matrix[3].m_z * m_Matrix[2].m_w;
        double c4 = m_Matrix[2].m_y * m_Matrix[3].m_w - m_Matrix[3].m_y * m_Matrix[2].m_w;
        double c3 = m_Matrix[2].m_y * m_Matrix[3].m_z - m_Matrix[3].m_y * m_Matrix[2].m_z;
        double c2 = m_Matrix[2].m_x * m_Matrix[3].m_w - m_Matrix[3].m_x * m_Matrix[2].m_w;
        double c1 = m_Matrix[2].m_x * m_Matrix[3].m_z - m_Matrix[3].m_x * m_Matrix[2].m_z;
        double c0 = m_Matrix[2].m_x * m_Matrix[3].m_y - m_Matrix[3].m_x * m_Matrix[2].m_y;

        // Note that a transformation matrix may never have a zero determinant.

        double invdet = 1.0 / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);

        Trans.m_Matrix[0].m_x = (m_Matrix[1].m_y * c5 - m_Matrix[1].m_z * c4 + m_Matrix[1].m_w * c3) * invdet;
        Trans.m_Matrix[0].m_y = (-m_Matrix[0].m_y * c5 + m_Matrix[0].m_z * c4 - m_Matrix[0].m_w * c3) * invdet;
        Trans.m_Matrix[0].m_z = (m_Matrix[3].m_y * s5 - m_Matrix[3].m_z * s4 + m_Matrix[3].m_w * s3) * invdet;
        Trans.m_Matrix[0].m_w = (-m_Matrix[2].m_y * s5 + m_Matrix[2].m_z * s4 - m_Matrix[2].m_w * s3) * invdet;

        Trans.m_Matrix[1].m_x = (-m_Matrix[1].m_x * c5 + m_Matrix[1].m_z * c2 - m_Matrix[1].m_w * c1) * invdet;
        Trans.m_Matrix[1].m_y = (m_Matrix[0].m_x * c5 - m_Matrix[0].m_z * c2 + m_Matrix[0].m_w * c1) * invdet;
        Trans.m_Matrix[1].m_z = (-m_Matrix[3].m_x * s5 + m_Matrix[3].m_z * s2 - m_Matrix[3].m_w * s1) * invdet;
        Trans.m_Matrix[1].m_w = (m_Matrix[2].m_x * s5 - m_Matrix[2].m_z * s2 + m_Matrix[2].m_w * s1) * invdet;

        Trans.m_Matrix[2].m_x = (m_Matrix[1].m_x * c4 - m_Matrix[1].m_y * c2 + m_Matrix[1].m_w * c0) * invdet;
        Trans.m_Matrix[2].m_y = (-m_Matrix[0].m_x * c4 + m_Matrix[0].m_y * c2 - m_Matrix[0].m_w * c0) * invdet;
        Trans.m_Matrix[2].m_z = (m_Matrix[3].m_x * s4 - m_Matrix[3].m_y * s2 + m_Matrix[3].m_w * s0) * invdet;
        Trans.m_Matrix[2].m_w = (-m_Matrix[2].m_x * s4 + m_Matrix[2].m_y * s2 - m_Matrix[2].m_w * s0) * invdet;

        Trans.m_Matrix[3].m_x = (-m_Matrix[1].m_x * c3 + m_Matrix[1].m_y * c1 - m_Matrix[1].m_z * c0) * invdet;
        Trans.m_Matrix[3].m_y = (m_Matrix[0].m_x * c3 - m_Matrix[0].m_y * c1 + m_Matrix[0].m_z * c0) * invdet;
        Trans.m_Matrix[3].m_z = (-m_Matrix[3].m_x * s3 + m_Matrix[3].m_y * s1 - m_Matrix[3].m_z * s0) * invdet;
        Trans.m_Matrix[3].m_w = (m_Matrix[2].m_x * s3 - m_Matrix[2].m_y * s1 + m_Matrix[2].m_z * s0) * invdet;
    }

    Transform3D operator*(Transform3D const &Trans0, Transform3D const &Trans1)
    {
        Vector4D const *Matrix0 = Trans0.GetData();
        Vector4D const &XAxis0 = Matrix0[0];
        Vector4D const &YAxis0 = Matrix0[1];
        Vector4D const &ZAxis0 = Matrix0[2];
        Vector4D const &WAxis0 = Matrix0[3];

        Vector4D const *Matrix1 = Trans1.GetData();
        Vector4D const &XAxis1 = Matrix1[0];
        Vector4D const &YAxis1 = Matrix1[1];
        Vector4D const &ZAxis1 = Matrix1[2];
        Vector4D const &WAxis1 = Matrix1[3];

        Vector4D X, Y, Z, W;

        X.m_x = XAxis0.m_x * XAxis1.m_x + YAxis0.m_x * XAxis1.m_y + ZAxis0.m_x * XAxis1.m_z + WAxis0.m_x * XAxis1.m_w;
        X.m_y = XAxis0.m_y * XAxis1.m_x + YAxis0.m_y * XAxis1.m_y + ZAxis0.m_y * XAxis1.m_z + WAxis0.m_y * XAxis1.m_w;
        X.m_z = XAxis0.m_z * XAxis1.m_x + YAxis0.m_z * XAxis1.m_y + ZAxis0.m_z * XAxis1.m_z + WAxis0.m_z * XAxis1.m_w;
        X.m_w = XAxis0.m_w * XAxis1.m_x + YAxis0.m_w * XAxis1.m_y + ZAxis0.m_w * XAxis1.m_z + WAxis0.m_w * XAxis1.m_w;

        Y.m_x = XAxis0.m_x * YAxis1.m_x + YAxis0.m_x * YAxis1.m_y + ZAxis0.m_x * YAxis1.m_z + WAxis0.m_x * YAxis1.m_w;
        Y.m_y = XAxis0.m_y * YAxis1.m_x + YAxis0.m_y * YAxis1.m_y + ZAxis0.m_y * YAxis1.m_z + WAxis0.m_y * YAxis1.m_w;
        Y.m_z = XAxis0.m_z * YAxis1.m_x + YAxis0.m_z * YAxis1.m_y + ZAxis0.m_z * YAxis1.m_z + WAxis0.m_z * YAxis1.m_w;
        Y.m_w = XAxis0.m_w * YAxis1.m_x + YAxis0.m_w * YAxis1.m_y + ZAxis0.m_w * YAxis1.m_z + WAxis0.m_w * YAxis1.m_w;

        Z.m_x = XAxis0.m_x * ZAxis1.m_x + YAxis0.m_x * ZAxis1.m_y + ZAxis0.m_x * ZAxis1.m_z + WAxis0.m_x * ZAxis1.m_w;
        Z.m_y = XAxis0.m_y * ZAxis1.m_x + YAxis0.m_y * ZAxis1.m_y + ZAxis0.m_y * ZAxis1.m_z + WAxis0.m_y * ZAxis1.m_w;
        Z.m_z = XAxis0.m_z * ZAxis1.m_x + YAxis0.m_z * ZAxis1.m_y + ZAxis0.m_z * ZAxis1.m_z + WAxis0.m_z * ZAxis1.m_w;
        Z.m_w = XAxis0.m_w * ZAxis1.m_x + YAxis0.m_w * ZAxis1.m_y + ZAxis0.m_w * ZAxis1.m_z + WAxis0.m_w * ZAxis1.m_w;

        W.m_x = XAxis0.m_x * WAxis1.m_x + YAxis0.m_x * WAxis1.m_y + ZAxis0.m_x * WAxis1.m_z + WAxis0.m_x * WAxis1.m_w;
        W.m_y = XAxis0.m_y * WAxis1.m_x + YAxis0.m_y * WAxis1.m_y + ZAxis0.m_y * WAxis1.m_z + WAxis0.m_y * WAxis1.m_w;
        W.m_z = XAxis0.m_z * WAxis1.m_x + YAxis0.m_z * WAxis1.m_y + ZAxis0.m_z * WAxis1.m_z + WAxis0.m_z * WAxis1.m_w;
        W.m_w = XAxis0.m_w * WAxis1.m_x + YAxis0.m_w * WAxis1.m_y + ZAxis0.m_w * WAxis1.m_z + WAxis0.m_w * WAxis1.m_w;

        return {X, Y, Z, W};
    }

    Point3D operator*(Transform3D const &Trans, Point3D const &Pos)
    {
        Vector4D const *Matrix = Trans.GetData();
        Vector4D const &XAxis = Matrix[0];
        Vector4D const &YAxis = Matrix[1];
        Vector4D const &ZAxis = Matrix[2];
        double x = Pos.m_x * XAxis.m_x + Pos.m_y * XAxis.m_y + Pos.m_z * XAxis.m_z + XAxis.m_w;
        double y = Pos.m_x * YAxis.m_x + Pos.m_y * YAxis.m_y + Pos.m_z * YAxis.m_z + YAxis.m_w;
        double z = Pos.m_x * ZAxis.m_x + Pos.m_y * ZAxis.m_y + Pos.m_z * ZAxis.m_z + ZAxis.m_w;
        return {x, y, z};
    }

    Vector3D operator*(Transform3D const &Trans, Vector3D const &Vec)
    {
        Vector4D const *Matrix = Trans.GetData();
        Vector4D const &XAxis = Matrix[0];
        Vector4D const &YAxis = Matrix[1];
        Vector4D const &ZAxis = Matrix[2];
        double x = Vec.m_x * XAxis.m_x + Vec.m_y * XAxis.m_y + Vec.m_z * XAxis.m_z;
        double y = Vec.m_x * YAxis.m_x + Vec.m_y * YAxis.m_y + Vec.m_z * YAxis.m_z;
        double z = Vec.m_x * ZAxis.m_x + Vec.m_y * ZAxis.m_y + Vec.m_z * ZAxis.m_z;
        return {x, y, z};
    }

    UnitVector3D operator*(Transform3D const &Trans, UnitVector3D const &UVec)
    {
        Vector4D const *Matrix = Trans.GetData();
        Vector4D const &XAxis = Matrix[0];
        Vector4D const &YAxis = Matrix[1];
        Vector4D const &ZAxis = Matrix[2];
        double x = UVec.m_x * XAxis.m_x + UVec.m_y * XAxis.m_y + UVec.m_z * XAxis.m_z;
        double y = UVec.m_x * YAxis.m_x + UVec.m_y * YAxis.m_y + UVec.m_z * YAxis.m_z;
        double z = UVec.m_x * ZAxis.m_x + UVec.m_y * ZAxis.m_y + UVec.m_z * ZAxis.m_z;
        return {x, y, z};
    }

} // namespace SGM