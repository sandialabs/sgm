#ifndef SGM_TRANSFORM_H
#define SGM_TRANSFORM_H

#include "sgm_export.h"

#include "SGMVector.h"

namespace SGM {

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Transformation classes
    //
    ///////////////////////////////////////////////////////////////////////////

    class SGM_EXPORT Transform3D
    {
    public:

        // Returns the identity transform.

        Transform3D();

        // Returns a transform that scales space by dS.

        explicit Transform3D(double dS);

        // Returns a transform that translates space by the given vector.

        explicit Transform3D(Vector3D const &Translate);

        // Returns a transform that scales by different amounts in each of
        // the three coordinate axes.

        Transform3D(double dXScale,
                    double dYScale,
                    double dZScale);

        // Returns a transform that rotates space about the origin by
        // moving the X, Y and Z axes to the given values.

        Transform3D(UnitVector3D const &XAxis,
                    UnitVector3D const &YAxis,
                    UnitVector3D const &ZAxis);

        // Returns a transform that first rotates space and then translates.

        Transform3D(UnitVector3D const &XAxis,
                    UnitVector3D const &YAxis,
                    UnitVector3D const &ZAxis,
                    Vector3D const &Translate);

        // Returns the four by four transform from all the data.

        Transform3D(Vector4D const &XAxis,
                    Vector4D const &YAxis,
                    Vector4D const &ZAxis,
                    Vector4D const &Translate);

        // Returns the transform from one set of axes to another.
        
        Transform3D(UnitVector3D const &XAxisFrom,
                    UnitVector3D const &YAxisFrom,
                    UnitVector3D const &ZAxisFrom,
                    Point3D      const &CenterFrom,
                    UnitVector3D const &XAxisTo,
                    UnitVector3D const &YAxisTo,
                    UnitVector3D const &ZAxisTo,
                    Point3D      const &CenterTo);
            
        // true if transformation does not include any non-uniform scale or rotation or shear or reflection

        bool IsScaleAndTranslate() const;

        // Returns a transform, Trans, such that this*Trans = Trans*this = Identity.

        void Inverse(Transform3D &Trans) const;

        // Returns the scale in a given direction.

        double Scale(UnitVector3D const &Direction) const;

        // Returns either zero or a uniform scale value.

        double Scale() const;

        // Get methods.

        Vector4D const *GetData() const
        { return m_Matrix; }

    private:

        Vector4D m_Matrix[4];
    };

    // Returns a transform, Trans, that rotates space about the given Origin and Axis
    // by the given aAngle in a right handed direction about the axis.

    SGM_EXPORT void Rotate(Point3D const &Origin,
                           UnitVector3D const &Axis,
                           double dAngle,
                           Transform3D &Trans);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  Vector and Point transformations
    //
    ///////////////////////////////////////////////////////////////////////////

    SGM_EXPORT Point3D operator*(Transform3D const &Trans,Point3D const &Pos);

    SGM_EXPORT Vector3D operator*(Transform3D const &Trans, Vector3D const &Vec);

    SGM_EXPORT UnitVector3D operator*(Transform3D const &Trans, UnitVector3D const &UVec);

    SGM_EXPORT Transform3D operator*(Transform3D const &Trans0, Transform3D const &Trans1);

} // End of SGM namespace

#include "Inline/SGMTransform.inl"

#endif //SGM_TRANSFORM_H
