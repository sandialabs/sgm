#ifndef SGM_TRANSFORM_INL
#define SGM_TRANSFORM_INL

namespace SGM {

    inline Transform3D::Transform3D()
    {
        m_Matrix[0] = Vector4D(1, 0, 0, 0);
        m_Matrix[1] = Vector4D(0, 1, 0, 0);
        m_Matrix[2] = Vector4D(0, 0, 1, 0);
        m_Matrix[3] = Vector4D(0, 0, 0, 1);
    }

// Returns a transform that scales space by dS.

    inline Transform3D::Transform3D(double dS)
    {
        m_Matrix[0] = Vector4D(dS, 0, 0, 0);
        m_Matrix[1] = Vector4D(0, dS, 0, 0);
        m_Matrix[2] = Vector4D(0, 0, dS, 0);
        m_Matrix[3] = Vector4D(0, 0, 0, 1);
    }

// Returns a transform that translates space by the given vector.

    inline Transform3D::Transform3D(Vector3D
                                    const &Translate)
    {
        m_Matrix[0] = Vector4D(1, 0, 0, Translate.m_x);
        m_Matrix[1] = Vector4D(0, 1, 0, Translate.m_y);
        m_Matrix[2] = Vector4D(0, 0, 1, Translate.m_z);
        m_Matrix[3] = Vector4D(0, 0, 0, 1);
    }

// Returns a transform that scales by different amounts in each of
// the three coordinate axes.

    inline Transform3D::Transform3D(double dXScale,
                                    double dYScale,
                                    double dZScale)
    {
        m_Matrix[0] = Vector4D(dXScale, 0, 0, 0);
        m_Matrix[1] = Vector4D(0, dYScale, 0, 0);
        m_Matrix[2] = Vector4D(0, 0, dZScale, 0);
        m_Matrix[3] = Vector4D(0, 0, 0, 1);
    }

// Returns a transform that rotates space about the origin by
// moving the X, Y and Z axes to the given values.

    inline Transform3D::Transform3D(UnitVector3D const &XAxis,
                                    UnitVector3D const &YAxis,
                                    UnitVector3D const &ZAxis)
    {
        m_Matrix[0] = Vector4D(XAxis[0], YAxis[0], ZAxis[0], 0);
        m_Matrix[1] = Vector4D(XAxis[1], YAxis[1], ZAxis[1], 0);
        m_Matrix[2] = Vector4D(XAxis[2], YAxis[2], ZAxis[2], 0);
        m_Matrix[3] = Vector4D(0, 0, 0, 1);
    }

// Returns a transform that first rotates space and then translates.

    inline Transform3D::Transform3D(UnitVector3D const &XAxis,
                                    UnitVector3D const &YAxis,
                                    UnitVector3D const &ZAxis,
                                    Vector3D const &Translate
    )
    {
        m_Matrix[0] = Vector4D(XAxis[0], YAxis[0], ZAxis[0], Translate.m_x);
        m_Matrix[1] = Vector4D(XAxis[1], YAxis[1], ZAxis[1], Translate.m_y);
        m_Matrix[2] = Vector4D(XAxis[2], YAxis[2], ZAxis[2], Translate.m_z);
        m_Matrix[3] = Vector4D(0, 0, 0, 1);
    }

// Returns the four by four transform from all the data.

    inline Transform3D::Transform3D(Vector4D const &XAxis,
                                    Vector4D const &YAxis,
                                    Vector4D const &ZAxis,
                                    Vector4D const &Translate
    )
    {
        m_Matrix[0] = XAxis;
        m_Matrix[1] = YAxis;
        m_Matrix[2] = ZAxis;
        m_Matrix[3] = Translate;
    }

// Returns the transform from one set of axes to another.
    
    inline Transform3D::Transform3D(UnitVector3D const &XAxisFrom,
                                    UnitVector3D const &YAxisFrom,
                                    UnitVector3D const &ZAxisFrom,
                                    Point3D      const &CenterFrom,
                                    UnitVector3D const &XAxisTo,
                                    UnitVector3D const &YAxisTo,
                                    UnitVector3D const &ZAxisTo,
                                    Point3D      const &CenterTo)
    {
        SGM::Transform3D TransformFrom(XAxisFrom, YAxisFrom, ZAxisFrom, SGM::Vector3D(CenterFrom));
        SGM::Transform3D TransformTo(XAxisTo, YAxisTo, ZAxisTo, SGM::Vector3D(CenterTo));
        
        SGM::Transform3D T2Inverse;
        TransformFrom.Inverse(T2Inverse);

        SGM::Transform3D Trans = T2Inverse * TransformTo;

        for (size_t iIndex=0; iIndex<4; ++iIndex)
            m_Matrix[iIndex] = Trans.m_Matrix[iIndex];
    }

// Returns a tranform the rotates space in the right handed
// direction about the given axis by the given angle.

    inline Transform3D::Transform3D(SGM::Point3D      const &PointOnAxis,
                                    SGM::UnitVector3D const &Axis,
                                    double                   dAngle)
        {
        SGM::UnitVector3D XAxis=Axis.Orthogonal();
        SGM::UnitVector3D YAxis=Axis*XAxis;
        double dSin=sin(dAngle);
        double dCos=cos(dAngle);
        SGM::UnitVector3D XRot(dCos,dSin,0);
        SGM::UnitVector3D YRot(-dSin,dCos,0);
        SGM::UnitVector3D ZVec(0,0,1);
        SGM::Point3D Origin(0,0,0);
        SGM::Transform3D ToOriginRotated=SGM::Transform3D(XAxis,YAxis,Axis,PointOnAxis,XRot,YRot,ZVec,Origin);
        SGM::UnitVector3D XVec(1,0,0);
        SGM::UnitVector3D YVec(0,1,0);
        SGM::Transform3D ToAxisFromOrigin=SGM::Transform3D(XVec,YVec,ZVec,Origin,XAxis,YAxis,Axis,PointOnAxis);
        SGM::Transform3D Trans=ToOriginRotated*ToAxisFromOrigin;

        for (size_t iIndex=0; iIndex<4; ++iIndex)
            m_Matrix[iIndex] = Trans.m_Matrix[iIndex];
        }
            
    
    inline double SGM::Transform3D::Scale(SGM::UnitVector3D const &Direction) const
    {
        return ((*this)*Vector3D(Direction)).Magnitude();
    }

    inline double SGM::Transform3D::Scale() const
    {
        double dX=Scale(SGM::UnitVector3D(1.0,0.0,0.0));
        double dY=Scale(SGM::UnitVector3D(0.0,1.0,0.0));
        double dZ=Scale(SGM::UnitVector3D(0.0,0.0,1.0));
        if(SGM::NearEqual(dX,dY,SGM_ZERO,false) && SGM::NearEqual(dZ,dY,SGM_ZERO,false))
            {
            return dX;
            }
        return 0.0;
    }

} // namespace SGM

#endif //SGM_TRANSFORM_INL
