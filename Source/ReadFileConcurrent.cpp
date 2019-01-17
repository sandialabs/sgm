#ifdef SGM_MULTITHREADED

#include "SGMVector.h"
#include "SGMTranslators.h"
#include "SGMThreadPool.h"
#include "SGMGraph.h"

#include "Curve.h"
#include "EntityFunctions.h"
#include "FileFunctions.h"
#include "Primitive.h"
#include "ReadFile.h"
#include "Surface.h"
#include "Topology.h"

#include <utility>
#include <string>
#include <algorithm>
#include <cstring>
#include <fstream>

#ifdef _MSC_VER
__pragma(warning(disable: 4996 ))
#endif

//#define SGM_PROFILE_READER

#ifdef SGM_PROFILE_READER
#include <iostream>
#endif


///////////////////////////////////////////////////////////////////////////////
//
// Multithreaded parser for STEP data reader
//
// 1. Reads a chunk (a multiple of STEP lines ending with ';')
// 2. Send chunks to thread workers to be parsed
// 3. Syncs chunks and adds their data to the map (Line number->STEP data)
//
///////////////////////////////////////////////////////////////////////////////


namespace SGMInternal
{

STEPLineChunk ParseSTEPStreamChunk(STEPTagMapType const &mSTEPTagMap,
                                   bool bScan,
                                   StringLinesChunk &aStringLinesChunk,
                                   STEPLineChunk &aSTEPLineChunk)
    {
    for (size_t iLine = 0; iLine < aStringLinesChunk.size(); ++iLine)
        {
        std::string *pLine = aStringLinesChunk[iLine];
        STEPLine *stepLine = aSTEPLineChunk[iLine];

        // are we at the end of the file?
        if (pLine->empty())
            {
            // the result must signal the end for when it gets synced
            stepLine->m_nLineNumber = std::numeric_limits<size_t>::max();
            stepLine->m_sTag.assign("END OF FILE");
            break;
            }
        ProcessSTEPLine(mSTEPTagMap, *pLine, *stepLine, bScan);
        }
    return aSTEPLineChunk;
    }

void CreateParseChunks(size_t nNumChunks,
                       size_t nChunkSize,
                       size_t nStringReserve,
                       std::vector<StringLinesChunk *> &aChunkLines,
                       std::vector<STEPLineChunk *> &aChunkSTEPLines)
    {
    aChunkLines.reserve(nNumChunks);
    for (size_t i = 0; i < nNumChunks; ++i)
        {
        auto *pStringLinesChunk = new StringLinesChunk();
        pStringLinesChunk->reserve(nChunkSize);
        for (size_t j = 0; j < nChunkSize; ++j)
            {
            auto *pLine = new std::string();
            pLine->reserve(nStringReserve);
            pStringLinesChunk->push_back(pLine);
            }
        aChunkLines.push_back(pStringLinesChunk);
        }

    aChunkSTEPLines.reserve(nNumChunks);
    for (size_t i = 0; i < nNumChunks; ++i)
        {
        auto *pSTEPLineChunk = new STEPLineChunk();
        pSTEPLineChunk->reserve(nChunkSize);
        for (size_t j = 0; j < nChunkSize; ++j)
            {
            pSTEPLineChunk->push_back(new STEPLine());
            }
        aChunkSTEPLines.push_back(pSTEPLineChunk);
        }
    }

void DestroyParseChunks(size_t nNumChunks,
                        size_t nChunkSize,
                        std::vector<StringLinesChunk *> &aChunkLines,
                        std::vector<STEPLineChunk *> &aChunkSTEPLines)
    {
    // free the line memory in the chunks
    for (size_t i = 0; i < nNumChunks; ++i)
        {
        StringLinesChunk &aStringLinesChunk = *aChunkLines[i];
        STEPLineChunk &aSTEPLineChunk = *aChunkSTEPLines[i];
        for (size_t j = 0; j < nChunkSize; ++j)
            {
            delete aStringLinesChunk[j];
            delete aSTEPLineChunk[j];
            }
        delete aChunkLines[i];
        delete aChunkSTEPLines[i];
        }
    }

void QueueParseChunks(std::ifstream &inputFileStream,
                      bool bScan,
                      STEPTagMapType const &mSTEPTagMap,
                      std::vector<StringLinesChunk *> &aChunkLines,
                      std::vector<STEPLineChunk *> &aChunkSTEPLines,
                      SGM::ThreadPool &threadPool,
                      std::vector<std::future<STEPLineChunk>> &futures)
    {
    const size_t NUM_CHUNKS = aChunkLines.size();
    assert(NUM_CHUNKS > 0);
    const size_t CHUNK_SIZE = aChunkLines[0]->size();

    // read and queue a number of chunks
    // too many and we will use too much memory
    for (size_t k = 0; k < NUM_CHUNKS; ++k)
        {
        StringLinesChunk & aStringLinesChunk = *aChunkLines[k];

        for (size_t iLine = 0; iLine < CHUNK_SIZE; ++iLine)
            {
            // post the next string and STEPLine structure to use into the chunk
            std::string *pLine = aStringLinesChunk[iLine];

            // read line from stream
            if (!std::getline(inputFileStream, *pLine, ';'))
                {
                // last line of stream was iLine-1,
                // clearing this one will signal end of file to the job
                pLine->clear();
                break;
                }
            }

        STEPLineChunk & aSTEPLineChunk = *aChunkSTEPLines[k];

        // add chunk task to the queue
        futures.emplace_back(threadPool.enqueue(std::bind(ParseSTEPStreamChunk,
                                                          mSTEPTagMap,
                                                          bScan,
                                                          aStringLinesChunk,
                                                          aSTEPLineChunk)));
        // if no more input stream
        if (!inputFileStream.good())
            return; // done looping over chunks

        }
    }

// return value is the max STEPLineNumber seen in these chunks

size_t SyncParseChunks(SGM::Result &rResult,
                       std::vector<std::string> &aLog,
                       std::vector<std::future<STEPLineChunk>> &futures,
                       STEPLineDataMapType &mSTEPData)
    {
    size_t maxSTEPLineNumber = 0;
    size_t nSTEPLineNumber;

    // sync up results and put the results in the map of (lineNumber -> STEPLineData)
    for (auto &&future: futures)
        {
        // get chunk result of the job
        future.wait();
        STEPLineChunk stepLineChunk = future.get();
        for (size_t iLine = 0; iLine < stepLineChunk.size(); ++iLine)
            {
            STEPLine *pSTEPLine = stepLineChunk[iLine];

            // if we are at the end of the file
            if (pSTEPLine->m_nLineNumber == std::numeric_limits<size_t>::max())
                {
                assert(pSTEPLine->m_sTag == "END OF FILE");
                return maxSTEPLineNumber;
                }

            // move the result into the map
            nSTEPLineNumber = MoveSTEPLineIntoMap(rResult, aLog, *pSTEPLine, mSTEPData);
            maxSTEPLineNumber = std::max(maxSTEPLineNumber, nSTEPLineNumber);

            // reset the structure for use again
            pSTEPLine->clear();
            }
        }
    futures.clear(); // ready to queue other jobs on the futures

    return maxSTEPLineNumber;
    }



// Parallel version of ParseSTEPStreamSerial
// return value is the max STEPLineNumber (#ID)

size_t ParseSTEPStreamConcurrent(SGM::Result &rResult,
                                 SGM::TranslatorOptions const &Options,
                                 std::vector <std::string> &aLog,
                                 std::ifstream &inputFileStream,
                                 STEPTagMapType const &mSTEPTagMap,
                                 STEPLineDataMapType &mSTEPData)
    {
    const size_t STRING_RESERVE = 4096 - 32;
    const size_t CHUNK_SIZE = 1024;
    const size_t NUM_CHUNKS = 8;
    const size_t NUM_PARSE_THREADS = 2;

    // make a stack of string and a stack of STEPLine for all the chunks
    std::vector < StringLinesChunk * > aChunkLines;
    std::vector < STEPLineChunk * > aChunkSTEPLines;

    CreateParseChunks(NUM_CHUNKS, CHUNK_SIZE, STRING_RESERVE, aChunkLines, aChunkSTEPLines);

    SGM::ThreadPool threadPool(NUM_PARSE_THREADS);

    // array of jobs (chunks), each result returned in a future object
    std::vector <std::future<STEPLineChunk>> futures;

    size_t maxSTEPLineNumber = 0;
    size_t maxChunkSTEPLineNumber;

    // until the stream reaches end-of-file or fails
    while (inputFileStream.good())
        {
        // read lines from file and queue jobs (futures) of chunks into the thread pool
        QueueParseChunks(inputFileStream,
                         Options.m_bScan,
                         mSTEPTagMap,
                         aChunkLines,
                         aChunkSTEPLines,
                         threadPool,
                         futures);

        // wait for jobs (chunks) to complete, copy results (#ID->STEPLineData) to the map, clear futures
        maxChunkSTEPLineNumber = SyncParseChunks(rResult, aLog, futures, mSTEPData);

        maxSTEPLineNumber = std::max(maxSTEPLineNumber, maxChunkSTEPLineNumber);
        }

    // free up all chunks of line (string) and STEPLine
    DestroyParseChunks(NUM_CHUNKS, CHUNK_SIZE, aChunkLines, aChunkSTEPLines);

    return maxSTEPLineNumber;
    }


} // namespace SGMInternal

#endif // SGM_MULTITHREADED