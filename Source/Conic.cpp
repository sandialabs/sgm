#include "SGMDataClasses.h"
#include "EntityClasses.h"
#include "Curve.h"
#include <vector>
#include <cmath>

// A quadratic curve, or conic, in the plane is defined by
// the following equation
//
// AX^2+2Bxy+Cy^2+2Dx+2Ey+F=0
//
// (A,B,C,D,E,F) are called the conic parameters.

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

    size_t nPoints=aPoints.size();
    size_t Index1;
    for(Index1=0;Index1<nPoints;++Index1)
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
    if(SGM::LinearSolve(aaMatrix)==false)
        {
        return false;
        }
    aConicParams.reserve(6);
    aConicParams.push_back(1.0);
    for(Index1=0;Index1<nPoints;++Index1)
        {
        aConicParams.push_back(aaMatrix[Index1].back());
        }
    return true;
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
                // Parabola f(t)=at^2
                // find y=ax^2+bx+c -> y'=2ax+b -> x=-b/2a

                SGM::Vector3D Vec0=aPoints[0]-Origin;
                SGM::Point2D Pos0(Vec0%XAxis,Vec0%YAxis);
                SGM::Vector3D Vec1=aPoints[1]-Origin;
                SGM::Point2D Pos1(Vec1%XAxis,Vec1%YAxis);
                SGM::Vector3D Vec2=aPoints[2]-Origin;
                SGM::Point2D Pos2(Vec2%XAxis,Vec2%YAxis);

                std::vector<std::vector<double> > aaEquations;
                aaEquations.reserve(3);
                std::vector<double> aEquation;
                aEquation.reserve(4);
                aEquation.push_back(Pos0.m_u*Pos0.m_u);
                aEquation.push_back(Pos0.m_u);
                aEquation.push_back(1.0);
                aEquation.push_back(Pos0.m_v);
                aaEquations.push_back(aEquation);
                aEquation.clear();
                aEquation.push_back(Pos1.m_u*Pos1.m_u);
                aEquation.push_back(Pos1.m_u);
                aEquation.push_back(1.0);
                aEquation.push_back(Pos1.m_v);
                aaEquations.push_back(aEquation);
                aEquation.clear();
                aEquation.push_back(Pos2.m_u*Pos2.m_u);
                aEquation.push_back(Pos2.m_u);
                aEquation.push_back(1.0);
                aEquation.push_back(Pos2.m_v);
                aaEquations.push_back(aEquation);
                SGM::LinearSolve(aaEquations);
                double a=aaEquations[0].back();
                double b=aaEquations[1].back();
                double c=aaEquations[2].back();
                double x=-b/(2.0*a);
                double y=x*(a*x+b)+c;
                SGM::Point3D Center=Origin+x*XAxis+y*YAxis;
                if(a<0)
                    {
                    a=-a;
                    YAxis.Negate();
                    }
                return new parabola(rResult,Center,XAxis,YAxis,a);
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
                    // Hyperbola f(t)=a*sqrt(1+t^2/b^2)
                    // x^2/a^2-y^2/b^2=1

                    bool bLow=false,bHigh=false;
                    double dTest1=0,dTest2=0;
                    for(Index1=0;Index1<5;++Index1)
                        {
                        double x=ax[Index1];
                        if(dTolerance<x)
                            {
                            bHigh=true;
                            }
                        else if(x<-dTolerance)
                            {
                            bLow=true;
                            }
                        double y=ay[Index1];
                        dTest1+=fabs(x*x/(a*a)-y*y/(b*b)-1);
                        dTest2+=fabs(x*x/(b*b)-y*y/(a*a)-1);
                        }
                    if(bLow && bHigh)
                        {
                        return nullptr; // Not in one sheet.
                        }
                    if(dTest2<dTest1)
                        {
                        std::swap(a,b);
                        }
                    if(bLow)
                        {
                        YAxis.Negate();
                        }
                    return new hyperbola(rResult,Center,YAxis,XAxis,a,b);
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
            // Check for a single line.

            double dTol=dTolerance*dTolerance;
            for(Index1=0;Index1<5;++Index1)
                {
                SGM::Point3D const &Pos=aPoints[Index1];
                double dDist=(Origin+((Pos-Origin)%XVec)*XVec).DistanceSquared(Pos);
                if(dTol<=dDist)
                    {
                    return nullptr;
                    }
                }
            return new line(rResult,Origin,XVec,1.0);
            }
        }
    return nullptr;
    }
