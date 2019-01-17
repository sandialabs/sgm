#include "SGMVector.h"
#include "SGMTranslators.h"
#include "EntityClasses.h"
#include "ReadFile.h"
#include "Topology.h"

#include <iostream>
#include <fstream>
#include <future>
#include <string>

//#ifdef SGM_MULTITHREADED
#include "SGMThreadPool.h"
//#endif

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
    auto ComplexIter=sComplexes.begin();
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
    auto FaceIter=sFaces.begin();
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

inline void ParseSTLVertex(std::string &Line, std::vector<SGM::Point3D> &aPoints)
    {
    // here we just ignore "facet normal", "outer loop", "endloop", "endfacet"
    char *pos = const_cast<char *>(FindWord(Line.c_str(), "vertex", 6));
    if (pos != nullptr)
        {
        double x = std::strtod(pos, &pos);
        assert(errno != ERANGE);
        if (x == 0.0) x = 0.0; // convert negative zero to positive zero
        double y = std::strtod(pos, &pos);
        assert(errno != ERANGE);
        if (y == 0.0) y = 0.0; // convert negative zero to positive zero
        double z = std::strtod(pos, &pos);
        assert(errno != ERANGE);
        if (z == 0.0) z = 0.0; // convert negative zero to positive zero
        aPoints.emplace_back(x, y, z);
        }
    }

complex* ParseSTLCreateComplex(SGM::Result &rResult,
                        bool bMerge,
                        std::vector<SGM::Point3D> &aPoints)
    {
    complex *pComplex = nullptr;
    if (bMerge)
        {
        // Merge the points and construct triangles indices.
        pComplex = new complex(rResult, aPoints, numeric_tolerance<float>::relative);
        }
    else
        {
        // triangles are just the range [0, N_points).
        std::vector<unsigned> aTriangles(aPoints.size());
        std::iota(aTriangles.begin(), aTriangles.end(), 0);
        pComplex = new complex(rResult, aPoints, std::move(aTriangles));
        }
    return pComplex;
    }

void ParseSTLTextSerial(SGM::Result &rResult,
                        std::string const &FileName,
                        std::vector<entity *> &aEntities,
                        bool bMerge)
    {
    std::ifstream file(FileName, std::ios::in);
    if (!file.is_open())
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        rResult.SetMessage("Could not open " + FileName);
        return;
        }
    std::string Line;
    while (std::getline(file, Line))
        {
        if (FindWord(Line.c_str(), "solid", 5) != nullptr)
            {
            std::vector<SGM::Point3D> aPoints;

            while (std::getline(file, Line) && FindWord(Line.c_str(), "end solid", 9) == nullptr)
                {
                ParseSTLVertex(Line, aPoints);
                }
            aEntities.push_back(ParseSTLCreateComplex(rResult,bMerge,aPoints));
            }
        }
    file.close();
    }

bool ParseSTLStreamChunk(StringLinesChunk          *p_aChunkLines,
                         std::vector<SGM::Point3D> *p_aPoints)
    {
    for (std::string *pLine : *p_aChunkLines)
        {
        // If we are we at the end, notify the job.
        if (pLine->empty() || FindWord(pLine->c_str(), "end solid", 9) != nullptr)
            return true;
        ParseSTLVertex(*pLine, *p_aPoints);
        }
    return false;  // Notify job we are NOT at the end
    }

void QueueSTLParseChunks(std::ifstream &inputFileStream,
                         std::vector<StringLinesChunk*> &aChunkLines,
                         std::vector<SGM::Point3D>      &aPoints,
                         SGM::ThreadPool                &threadPool,
                         std::vector<std::future<bool>> &futures)
    {
    const size_t NUM_CHUNKS = aChunkLines.size();
    assert(NUM_CHUNKS > 0);
    const size_t CHUNK_SIZE = aChunkLines[0]->size();

    // read and queue a number of chunks
    // too many and we will use too much memory
    for (size_t k = 0; k < NUM_CHUNKS; ++k)
        {
        bool isAtEnd = false;
        StringLinesChunk & aStringLinesChunk = *aChunkLines[k];

        for (size_t iLine = 0; iLine < CHUNK_SIZE; ++iLine)
            {
            // read line from stream
            std::string &Line = *aStringLinesChunk[iLine];
            isAtEnd = !(std::getline(inputFileStream, Line));
            if (isAtEnd)
                {
                // clear the string to signal the end of strings and jobs
                Line.clear();
                break;
                }
            }

        // add the chunk of strings task to the queue
        futures.emplace_back(threadPool.enqueue(std::bind(ParseSTLStreamChunk,
                                                          &aStringLinesChunk,
                                                          &aPoints)));
        // check if no more jobs
        if (isAtEnd || !inputFileStream.good())
            break;
        }
    }


// Returns true if the end of the "solid" was reached
bool SyncSTLParseChunks(std::vector<std::future<bool>> &futures,
                        std::vector<SGM::Point3D> &aPoints)
    {
    bool isAtEnd = false;

    // sync up results and if needed consolidate results of the jobs
    for (auto &&future: futures)
        {
        // sync up with the job
        future.wait();
        isAtEnd = isAtEnd || future.get();
        }
    futures.clear(); // ready to queue other jobs on the futures

    return isAtEnd;
    }


void CreateSTLParseChunks(size_t nNumChunks,
                          size_t nChunkLines,
                          size_t nStringReserve,
                          std::vector<StringLinesChunk*> &aChunkLines)
    {
    aChunkLines.reserve(nNumChunks);
    for (size_t i = 0; i < nNumChunks; ++i)
        {
        auto *pStringLinesChunk = new StringLinesChunk();
        pStringLinesChunk->reserve(nChunkLines);
        for (size_t j = 0; j < nChunkLines; ++j)
            {
            auto *pLine = new std::string();
            pLine->reserve(nStringReserve);
            pStringLinesChunk->push_back(pLine);
            }
        aChunkLines.push_back(pStringLinesChunk);
        }
    }

void DestroySTLParseChunks(std::vector<StringLinesChunk*> &aChunkLines)
    {
    // free the chunks of strings
    for (StringLinesChunk *pStringLinesChunk : aChunkLines)
        {
        for (std::string *pLine : *pStringLinesChunk)
            delete pLine;
        delete pStringLinesChunk;
        }
    }

void ParseSTLTextConcurrent(SGM::Result                  &rResult,
                            std::string            const &FileName,
                            std::vector<entity*>         &aEntities,
                            bool                          bMerge)
    {
    const size_t STRING_RESERVE = 63;
    const size_t CHUNK_SIZE = 2048;
    const size_t NUM_CHUNKS = 1;
    const size_t NUM_PARSE_THREADS = 1; // IMPORTANT: only one shared vector of points

    // make chunks of strings
    std::vector<StringLinesChunk *> aChunkLines;
    CreateSTLParseChunks(NUM_CHUNKS, CHUNK_SIZE, STRING_RESERVE, aChunkLines);

    // storage for all the "vertex" points in the file
    std::vector<SGM::Point3D> aPoints;

    rResult.GetThing()->SetConcurrentActive();

    SGM::ThreadPool threadPool(NUM_PARSE_THREADS);

    // array of jobs (chunks), each result returned in a future object
    std::vector<std::future<bool>> futures;

    std::ifstream inputFileStream(FileName, std::ios::in);
    if (!inputFileStream.is_open())
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        rResult.SetMessage("Could not open " + FileName);
        return;
        }

    std::string line;
    while (std::getline(inputFileStream, line))
        {
        bool isAtEnd = false;
        if (FindWord(line.c_str(), "solid", 5) != nullptr)
            {
            while (!isAtEnd)
                {
                // read lines from file and queue jobs (futures) of chunks into the worker pool
                QueueSTLParseChunks(inputFileStream,
                                    aChunkLines,
                                    aPoints,
                                    threadPool,
                                    futures);

                // wait for jobs to complete, vertex points inserted into the main point vector
                isAtEnd = SyncSTLParseChunks(futures, aPoints);
                }
            }
        }

    inputFileStream.close();
    DestroySTLParseChunks(aChunkLines);

    rResult.GetThing()->SetConcurrentInactive();

    aEntities.push_back(ParseSTLCreateComplex(rResult,bMerge,aPoints));
    }

} // namespace SGMInternal