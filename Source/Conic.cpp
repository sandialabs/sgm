#include "SGMVector.h"
#include "EntityClasses.h"
#include "Curve.h"
#include <utility>
#include <vector>
#include <cmath>

// A quadratic curve, or conic, in the plane is defined by
// the following equation
//
// AX^2+2Bxy+Cy^2+2Dx+2Ey+F=0
//
// (A,B,C,D,E,F) are called the conic parameters.
namespace SGMInternal
{
bool FindConicParameters(std::vector<SGM::Point3D> const &aPoints,
                         double                           dTolerance,
                         std::vector<double>             &aConicParams,
                         SGM::Point3D                    &Origin,
                         SGM::UnitVector3D               &XVec,
                         SGM::UnitVector3D               &YVec,
                         SGM::UnitVector3D               &ZVec)
    {
    // First find the least squares plane through the points.

    std::vector<SGM::Point2D> aPoints2D;
    if(SGM::FindLeastSquarePlane(aPoints,Origin,XVec,YVec,ZVec))
        {
        double dZDist=ProjectPointsToPlane(aPoints,Origin,XVec,YVec,ZVec,aPoints2D);
        if(dTolerance<=dZDist)
            {
            return false;
            }
        }
    else
        {
        return false;
        }

    std::vector<std::vector<double> > aaMatrix;
    aaMatrix.reserve(5);

    // A=1 -> B*(2xy)+C*(y^2)+D*(2x)+E*(2y)+F*(1)=(-X^2)

    size_t Index1;
    for(Index1=0;Index1<5;++Index1)
        {
        SGM::Point2D const &uv=aPoints2D[Index1];
        std::vector<double> aMatrix;
        aMatrix.reserve(6);
        aMatrix.push_back(2.0*uv.m_u*uv.m_v);
        aMatrix.push_back(uv.m_v*uv.m_v);
        aMatrix.push_back(2.0*uv.m_u);
        aMatrix.push_back(2.0*uv.m_v);
        aMatrix.push_back(1.0);
        aMatrix.push_back(-uv.m_u*uv.m_u);
        aaMatrix.push_back(aMatrix);
        }
    if(!SGM::LinearSolve(aaMatrix))
        {
        return false;
        }
    aConicParams.reserve(6);
    aConicParams.push_back(1.0);
    for(Index1=0;Index1<5;++Index1)
        {
        aConicParams.push_back(aaMatrix[Index1].back());
        }
    return true;
    }

curve *CheckForLine(SGM::Result                     &rResult,
                    std::vector<SGM::Point3D> const &aPoints,
                    double                           dTolerance)
    {
    size_t Index1;
    double dTol=dTolerance*dTolerance;
    SGM::Point3D Origin=aPoints[0];
    SGM::Vector3D XVec=aPoints[1]-aPoints[0];
    for(Index1=0;Index1<5;++Index1)
        {
        SGM::Point3D const &Pos=aPoints[Index1];
        double dDist=(Origin+((Pos-Origin)%XVec)*XVec).DistanceSquared(Pos);
        if(dTol<=dDist)
            {
            return nullptr;
            }
        }
    return new line(rResult,Origin,XVec);
    }

parabola *FindParabola(SGM::Result                     &rResult,
                       SGM::Point3D                    &Origin,
                       SGM::UnitVector3D               &XAxis,
                       SGM::UnitVector3D               &YAxis,
                       std::vector<SGM::Point3D> const &aPoints)
    {
    std::vector<SGM::Point2D> aXY;
    aXY.reserve(5);
    for(auto Pos : aPoints)
        {
        SGM::Vector3D Vec=Pos-Origin;
        aXY.push_back(SGM::Point2D(Vec%XAxis,Vec%YAxis));
        }
    
    // Find y=a*x^2+b*x+c both ways xy and yx.
     
    std::vector<std::vector<double> > aaMatrix;
    aaMatrix.reserve(3);
    size_t Index1;
    for(Index1=0;Index1<3;++Index1)
        {
        std::vector<double> aMatrix;
        aMatrix.reserve(4);
        aMatrix.push_back(aXY[Index1].m_u*aXY[Index1].m_u);
        aMatrix.push_back(aXY[Index1].m_u);
        aMatrix.push_back(1.0);
        aMatrix.push_back(aXY[Index1].m_v);
        aaMatrix.push_back(aMatrix);
        }
    SGM::LinearSolve(aaMatrix);
    double a1=aaMatrix[0].back();
    double b1=aaMatrix[1].back();
    double c1=aaMatrix[2].back();

    aaMatrix.clear();
    for(Index1=0;Index1<3;++Index1)
        {
        std::vector<double> aMatrix;
        aMatrix.reserve(4);
        aMatrix.push_back(aXY[Index1].m_v*aXY[Index1].m_v);
        aMatrix.push_back(aXY[Index1].m_v);
        aMatrix.push_back(1.0);
        aMatrix.push_back(aXY[Index1].m_u);
        aaMatrix.push_back(aMatrix);
        }
    SGM::LinearSolve(aaMatrix);
    double a2=aaMatrix[0].back();
    double b2=aaMatrix[1].back();
    double c2=aaMatrix[2].back();

    // Check the fit of each equation.

    double dFit1=0,dFit2=0;
    for(auto XY : aXY)
        {
        double y1=a1*XY.m_u*XY.m_u+b1*XY.m_u+c1;
        double y2=a2*XY.m_v*XY.m_v+b2*XY.m_v+c2;
        dFit1+=fabs(y1-XY.m_v);
        dFit2+=fabs(y2-XY.m_u);
        }

    // Find the center.
    // Parabola f(t)=at^2
    // find y=ax^2+bx+c -> y'=2ax+b -> x=-b/2a

    SGM::Point3D Center;
    if(dFit1<dFit2)
        {
        double x=-b1/(2.0*a1);
        double y=x*(a1*x+b1)+c1;
        Center=Origin+x*XAxis+y*YAxis;
        return new parabola(rResult,Center,XAxis,YAxis,a1);
        }
    else
        {
        std::swap(XAxis,YAxis);
        double x=-b2/(2.0*a2);
        double y=x*(a2*x+b2)+c2;
        Center=Origin+x*XAxis+y*YAxis;
        return new parabola(rResult,Center,XAxis,YAxis,a2);
        }
    }

hyperbola *FindHyperbola(SGM::Result                     &rResult,
                         SGM::Point3D                    &Center,
                         SGM::UnitVector3D               &XAxis,
                         SGM::UnitVector3D               &YAxis,
                         double                           dTolerance,
                         std::vector<SGM::Point3D> const &aPoints)
    {
    // Hyperbola f(t)=a*sqrt(1+t^2/b^2)
    // x^2/a^2-y^2/b^2=1 -> an*x^2+bn*y^2=1, where an=1/a^2, bn=1/b^2
    // a=sqrt(1/an) b=sqrt(1/bn)

    bool bQuad1=false,bQuad2=false,bQuad3=false,bQuad4=false;
    std::vector<SGM::Point2D> aXY;
    aXY.reserve(5);
    for(auto Pos : aPoints)
        {
        SGM::Vector3D Vec=Pos-Center;
        double x=Vec%XAxis;
        double y=Vec%YAxis;
        aXY.push_back(SGM::Point2D(x,y));
        if(x<-dTolerance)
            {
            if(y<-dTolerance)
                {
                bQuad3=true;
                }
            else if(dTolerance<y)
                {
                bQuad2=true;
                }
            }
        else if(dTolerance<x)
            {
            if(y<-dTolerance)
                {
                bQuad4=true;
                }
            else if(dTolerance<y)
                {
                bQuad1=true;
                }
            }
        }

    // Note that all three branches were tested by moving the points to different
    // quads at the top of this function.

    size_t Index1;
    if(bQuad1 && bQuad4)
        {
        std::swap(XAxis,YAxis);
        }
    else if(bQuad2 && bQuad3)
        {
        std::swap(XAxis,YAxis);
        YAxis.Negate();
        for(Index1=0;Index1<3;++Index1)
            {
            SGM::Point2D xy=aXY[Index1];
            aXY[Index1]=SGM::Point2D(xy.m_u,-xy.m_v);
            }
        }
    else if(bQuad3 && bQuad4)
        {
        XAxis.Negate();
        YAxis.Negate();
        for(Index1=0;Index1<3;++Index1)
            {
            SGM::Point2D xy=aXY[Index1];
            aXY[Index1]=SGM::Point2D(-xy.m_v,-xy.m_u);
            }
        }
    
    // Find two points that do not match in x or y.

    SGM::Point2D xy1=aXY[0],xy2;
    double dDiff=0;
    for(auto xy : aXY)
        {
        double dx=fabs(xy.m_u-xy1.m_u);
        double dy=fabs(xy.m_v-xy1.m_v);
        double dD=std::min(dx,dy);
        if(dDiff<dD)
            {
            dDiff=dD;
            xy2=xy;
            }
        }
    std::vector<SGM::Point2D> aXY2;
    aXY2.push_back(xy1);
    aXY2.push_back(xy2);

    // Find an*x^2+bn*y^2=1
     
    std::vector<std::vector<double> > aaMatrix;
    aaMatrix.reserve(2);
    for(Index1=0;Index1<2;++Index1)
        {
        std::vector<double> aMatrix;
        aMatrix.reserve(3);
        aMatrix.push_back(aXY2[Index1].m_u*aXY2[Index1].m_u);
        aMatrix.push_back(aXY2[Index1].m_v*aXY2[Index1].m_v);
        aMatrix.push_back(1.0);
        aaMatrix.push_back(aMatrix);
        }
    SGM::LinearSolve(aaMatrix);
    double a=sqrt(fabs(1.0/aaMatrix[0].back()));
    double b=sqrt(fabs(1.0/aaMatrix[1].back()));
    return new hyperbola(rResult,Center,XAxis,YAxis,a,b);
    }

curve *FindConic(SGM::Result                     &rResult,
                 std::vector<SGM::Point3D> const &aPoints,
                 double                           dTolerance)
    {
    size_t Index1;
    if(aPoints.size()<5)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeInsufficientData);
        return nullptr;
        }
    SGM::Point3D Origin;
    SGM::UnitVector3D XVec,YVec,ZVec;
    std::vector<double> aParams;
    if(FindConicParameters(aPoints,dTolerance,aParams,Origin,XVec,YVec,ZVec))
        {
        double A=aParams[0];
        double B=aParams[1];
        double C=aParams[2];
        double D=aParams[3];
        double E=aParams[4];
        double F=aParams[5];
        
        //          | A B D |        | A B |
        // aaMatrix=| B C E |  aaMat=| B C |
        //          | E E F |

        double aaMatrix[3][3],aaMat[2][2];
        aaMat[0][0]=aaMatrix[0][0]=A;
        aaMat[0][1]=aaMatrix[0][1]=B;
        aaMatrix[0][2]=D;
        aaMat[1][0]=aaMatrix[1][0]=B;
        aaMat[1][1]=aaMatrix[1][1]=C;
        aaMatrix[1][2]=E;
        aaMatrix[2][0]=D;
        aaMatrix[2][1]=E;
        aaMatrix[2][2]=F;
        double determinat=SGM::Determinate3D(aaMatrix);
        double discriminant=SGM::Determinate2D(aaMat);
        if(SGM_MIN_TOL<fabs(determinat))
            {
            // Axis angle t=atan(2B/(A-C))/2

            double dAngle=atan(2.0*B/(A-C))*0.5;
            double dCos=cos(dAngle);
            double dSin=sin(dAngle);
            SGM::UnitVector3D XAxis=dCos*YVec-dSin*XVec;
            SGM::UnitVector3D YAxis=dCos*XVec+dSin*YVec;

            if(fabs(discriminant)<SGM_MIN_TOL)
                {
                return FindParabola(rResult,Origin,XAxis,YAxis,aPoints);
                }
            else
                {
                // Center=-(1/discriminant)(| D B |,| A D |)
                //                          | E C | | B E |
            
                aaMat[0][0]=D;
                aaMat[0][1]=B;
                aaMat[1][0]=E;
                aaMat[1][1]=C;
                double u=-SGM::Determinate2D(aaMat)/discriminant;
                aaMat[0][0]=A;
                aaMat[0][1]=D;
                aaMat[1][0]=B;
                aaMat[1][1]=E;
                double v=-SGM::Determinate2D(aaMat)/discriminant;
                SGM::Point3D Center=Origin+XVec*u+YVec*v;
                std::vector<double> aRoots;
                SGM::Quadratic(1.0,-A-C,discriminant,aRoots);
                double r=determinat/discriminant;
                double a=sqrt(fabs(r/aRoots[0]));
                double b=sqrt(fabs(r/aRoots[1]));

                std::vector<double> ax,ay;
                ax.reserve(5);
                ay.reserve(5);
                for(Index1=0;Index1<5;++Index1)
                    {
                    SGM::Vector3D Vec=aPoints[Index1]-Center;
                    ax.push_back(Vec%XAxis);
                    ay.push_back(Vec%YAxis);
                    }

                if(discriminant<0)
                    {
                    return FindHyperbola(rResult,Center,XAxis,YAxis,dTolerance,aPoints);
                    }
                else 
                    {
                    // Ellipse f(t)=(a*cos(t),b*sin(t))
                    // x^2/a^2+y^2/b^2=1

                    double dTest1=0,dTest2=0;
                    for(Index1=0;Index1<5;++Index1)
                        {
                        double x=ax[Index1];
                        double y=ay[Index1];
                        dTest1+=fabs(x*x/(a*a)+y*y/(b*b)-1);
                        dTest2+=fabs(x*x/(b*b)+y*y/(a*a)-1);
                        }
                    if(dTest2<dTest1)
                        {
                        std::swap(a,b);
                        }
                    if(SGM::NearEqual(a,b,dTolerance,false))
                        {
                        // Circle

                        double dRadius=(a+b)*0.5;
                        return new circle(rResult,Center,ZVec,dRadius,&XAxis);
                        }
                    else
                        {
                        // Ellipse

                        return new ellipse(rResult,Center,XAxis,YAxis,a,b);
                        }
                    }
                }
            }
        else
            {
            return CheckForLine(rResult,aPoints,dTolerance);
            }
        }
    return CheckForLine(rResult,aPoints,dTolerance);
    }
}