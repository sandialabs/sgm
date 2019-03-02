#ifndef SIGNATURE_H
#define SIGNATURE_H

#include <limits>
#include <vector>

#include "SGMVector.h"

namespace SGMInternal
{

    class Signature
    {
    public:
        Signature(std::vector<SGM::Point3D> const &aPoints);

        bool Matches(Signature const &other, bool bIgnoreScale) const;

    public:
        SGM::Point3D Origin;
        SGM::UnitVector3D XAxis;
        SGM::UnitVector3D YAxis;
        SGM::UnitVector3D ZAxis;
        double Scale;
        std::vector<double> Xsequence;
        std::vector<double> Ysequence;
        std::vector<double> Zsequence;
        double Rubric;
    };

    inline Signature::Signature(std::vector<SGM::Point3D> const &aPoints)
    {
        FindLeastSquarePlane(aPoints, Origin, XAxis, YAxis, ZAxis);

        double Distance = 0;
        for (auto Pos : aPoints)
        {
            Distance += Pos.Distance(Origin);
        }
        double AvgDist = Distance / aPoints.size();
        Scale = 1.0 / AvgDist;

        for (auto Pos : aPoints)
        {
            Xsequence.emplace_back(((Pos - Origin) % XAxis) * Scale);
            Ysequence.emplace_back(((Pos - Origin) % YAxis) * Scale);
            Zsequence.emplace_back(((Pos - Origin) % ZAxis) * Scale);
            Distance += Pos.Distance(Origin);
        }
        std::sort(Xsequence.begin(), Xsequence.end());
        std::sort(Ysequence.begin(), Ysequence.end());
        std::sort(Zsequence.begin(), Zsequence.end());


        Rubric = 0;
        double rx = 0.0, ry = 0.0, rz = 0.0;
        for (size_t index = 0; index < aPoints.size() - 1; ++index)
        {
            rx += (Xsequence[index + 1] - Xsequence[index]);
            ry += (Ysequence[index + 1] - Ysequence[index]);
            rz += (Zsequence[index + 1] - Zsequence[index]);
        }
        Rubric = sqrt(rx / aPoints.size() * rx / aPoints.size() +
            ry / aPoints.size() * ry / aPoints.size() +
            rz / aPoints.size() * rz / aPoints.size());
    }
}

#endif // SIGNATURE_H
