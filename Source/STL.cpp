#include "SGMVector.h"
#include "SGMTranslators.h"
#include "EntityClasses.h"
#include "Topology.h"
#include <string>

// Lets us use fprintf
#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

namespace SGMInternal
{
void SaveSTL(SGM::Result                  &rResult,
             std::string            const &FileName,
             entity                       *pEntity,
             SGM::TranslatorOptions const &Options)
    {
    ////////////////////////////////////////////////////
    //
    //  What an STL text file looks like.
    //
    //  solid SGM
    //     facet normal -1.000000 0.000000 0.000000
    //        outer loop
    //           vertex 0.000000 0.000000 10.000000
    //           vertex 0.000000 10.000000 0.000000
    //           vertex 0.000000 0.000000 0.000000
    //        endloop
    //     endfacet 
    //  endsolid 
    //
    ////////////////////////////////////////////////////

    // Open the file.

    FILE *pFile=nullptr;
    if(Options.m_bBinary)
        {
        pFile = fopen(FileName.c_str(),"wb");
        }
    else
        {
        pFile = fopen(FileName.c_str(),"wt");
        }
    if(pFile==nullptr)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        return;
        }

    // Write out the complexes.

    std::set<complex *,EntityCompare> sComplexes;
    FindComplexes(rResult,pEntity,sComplexes);
    std::set<complex *,EntityCompare>::iterator ComplexIter=sComplexes.begin();
    while(ComplexIter!=sComplexes.end())
        {
        complex *pComplex=*ComplexIter;
        fprintf(pFile,"solid Complex %lu\n",pComplex->GetID());
        std::vector<SGM::Point3D> const &aPoints=pComplex->GetPoints();
        std::vector<unsigned int> const &aTriangles=pComplex->GetTriangles();
        size_t Index1;
        size_t nTriangles=aTriangles.size();
        for(Index1=0;Index1<nTriangles;Index1+=3)
            {
            unsigned int a=aTriangles[Index1];
            unsigned int b=aTriangles[Index1+1];
            unsigned int c=aTriangles[Index1+2];
            SGM::Point3D const &A=aPoints[a];
            SGM::Point3D const &B=aPoints[b];
            SGM::Point3D const &C=aPoints[c];
            SGM::UnitVector3D Norm=(B-A)*(C-A);
            fprintf(pFile,"   facet normal %lf %lf %lf\n",Norm.m_x,Norm.m_y,Norm.m_z);
            fprintf(pFile,"      outer loop\n");
            fprintf(pFile,"         vertex %lf %lf %lf\n",A.m_x,A.m_y,A.m_z);
            fprintf(pFile,"         vertex %lf %lf %lf\n",B.m_x,B.m_y,B.m_z);
            fprintf(pFile,"         vertex %lf %lf %lf\n",C.m_x,C.m_y,C.m_z);
            fprintf(pFile,"      endloop\n");
            fprintf(pFile,"   endfacet\n");
            }
        fprintf(pFile,"endsolid Complex %lu\n",pComplex->GetID());
        ++ComplexIter;
        }

    // Write out the faces.

    std::set<face *,EntityCompare> sFaces;
    FindFaces(rResult,pEntity,sFaces);
    std::set<face *,EntityCompare>::iterator FaceIter=sFaces.begin();
    while(FaceIter!=sFaces.end())
        {
        if(Options.m_b2D==false)
            {
            face *pFace=*FaceIter;
            //pFace=(face *)rResult.GetThing()->FindEntity(57);
            fprintf(pFile,"solid Face %lu\n",pFace->GetID());
            std::vector<SGM::Point3D> const &aPoints=pFace->GetPoints3D(rResult);
            std::vector<unsigned int> const &aTriangles=pFace->GetTriangles(rResult);
            size_t Index1;
            size_t nTriangles=aTriangles.size();
            for(Index1=0;Index1<nTriangles;Index1+=3)
                {
                unsigned int a=aTriangles[Index1];
                unsigned int b=aTriangles[Index1+1];
                unsigned int c=aTriangles[Index1+2];
                SGM::Point3D const &A=aPoints[a];
                SGM::Point3D const &B=aPoints[b];
                SGM::Point3D const &C=aPoints[c];
                SGM::UnitVector3D Norm=(B-A)*(C-A);
                fprintf(pFile,"   facet normal %lf %lf %lf\n",Norm.m_x,Norm.m_y,Norm.m_z);
                fprintf(pFile,"      outer loop\n");
                fprintf(pFile,"         vertex %lf %lf %lf\n",A.m_x,A.m_y,A.m_z);
                fprintf(pFile,"         vertex %lf %lf %lf\n",B.m_x,B.m_y,B.m_z);
                fprintf(pFile,"         vertex %lf %lf %lf\n",C.m_x,C.m_y,C.m_z);
                fprintf(pFile,"      endloop\n");
                fprintf(pFile,"   endfacet\n");
                }
            fprintf(pFile,"endsolid Face %lu\n",pFace->GetID());
            ++FaceIter;
            }
        else
            {
            face *pFace=*FaceIter;
            //pFace=(face *)rResult.GetThing()->FindEntity(57);
            fprintf(pFile,"solid Face %lu\n",pFace->GetID());
            std::vector<SGM::Point2D> const &aPoints=pFace->GetPoints2D(rResult);
            std::vector<unsigned int> const &aTriangles=pFace->GetTriangles(rResult);
            size_t Index1;
            size_t nTriangles=aTriangles.size();
            for(Index1=0;Index1<nTriangles;Index1+=3)
                {
                unsigned int a=aTriangles[Index1];
                unsigned int b=aTriangles[Index1+1];
                unsigned int c=aTriangles[Index1+2];
                SGM::Point2D const &A=aPoints[a];
                SGM::Point2D const &B=aPoints[b];
                SGM::Point2D const &C=aPoints[c];
                SGM::UnitVector3D Norm(0,0,1);
                fprintf(pFile,"   facet normal %lf %lf %lf\n",Norm.m_x,Norm.m_y,Norm.m_z);
                fprintf(pFile,"      outer loop \n");
                fprintf(pFile,"         vertex %lf %lf %lf\n",A.m_u,A.m_v,0.0);
                fprintf(pFile,"         vertex %lf %lf %lf\n",B.m_u,B.m_v,0.0);
                fprintf(pFile,"         vertex %lf %lf %lf\n",C.m_u,C.m_v,0.0);
                fprintf(pFile,"      endloop\n");
                fprintf(pFile,"   endfacet\n");
                }
            fprintf(pFile,"endsolid Face %lu\n",pFace->GetID());
            ++FaceIter;
            }
        }

    fclose(pFile);
    }
}