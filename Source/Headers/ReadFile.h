#ifndef SGM_READFILE_H
#define SGM_READFILE_H

#include "STEP.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <unordered_map>
#include <vector>

#if !defined( _MSC_VER ) || _MSC_VER >= 1900
#define NOEXCEPT noexcept
#define NOEXCEPT_ARGS(ARGS) noexcept((ARGS))
#else
#define NOEXCEPT
#define NOEXCEPT_ARGS(ARGS)
#endif

namespace SGMInternal {

class entity;

///////////////////////////////////////////////////////////////////////////////
//
// Types we use for parsing
//
///////////////////////////////////////////////////////////////////////////////

// Holds data arrays read from a step line:
//  - enum STEPTag
//  - array of #IDs
//  - array of doubles
//  - array of ints
//  - array of unsigned

struct STEPLineData
    {
    STEPLineData() :
            m_nSTEPTag(STEPTag::NULL_NONE_INVALID),
            m_aIDs(),
            m_aDoubles(),
            m_aInts(),
            m_aSizes(),
            m_aStrings(),
            m_bFlag(true)
    {}

    explicit STEPLineData(STEPTag nSTEPTag) :
            m_nSTEPTag(nSTEPTag),
            m_aIDs(),
            m_aDoubles(),
            m_aInts(),
            m_aSizes(),
            m_aStrings(),
            m_bFlag(true)
    {}

    STEPLineData(STEPLineData const &) = default;

    STEPLineData(STEPLineData&& other) : /* noexcept */
            m_nSTEPTag(other.m_nSTEPTag),
            m_aIDs(std::move(other.m_aIDs)),
            m_aDoubles(std::move(other.m_aDoubles)),
            m_aInts(std::move(other.m_aInts)),
            m_aSizes(std::move(other.m_aSizes)),
            m_aStrings(std::move(other.m_aStrings)),
            m_bFlag(other.m_bFlag)
        {
            other.m_nSTEPTag = STEPTag::NULL_NONE_INVALID;
            other.m_bFlag = true;
        }

    STEPLineData& operator=(const STEPLineData&) = default;

    STEPLineData& operator=(STEPLineData && other) NOEXCEPT
        {
        if (&other == this)
            return *this;
        m_nSTEPTag = other.m_nSTEPTag;
        other.m_nSTEPTag = STEPTag::NULL_NONE_INVALID;
        m_aIDs = std::move(other.m_aIDs);
        m_aDoubles = std::move(other.m_aDoubles);
        m_aInts = std::move(m_aInts);
        m_aSizes = std::move(other.m_aSizes);
        m_aStrings = std::move(other.m_aStrings);
        m_bFlag = other.m_bFlag;
        other.m_bFlag = true;
        return *this;
        }

    ~STEPLineData() = default;

    void clear()
        {
            m_nSTEPTag = STEPTag::NULL_NONE_INVALID;
            m_aIDs.clear();
            m_aDoubles.clear();
            m_aInts.clear();
            m_aSizes.clear();
            m_aStrings.clear();
            m_bFlag=true;
        }

    STEPTag m_nSTEPTag;
    std::vector <size_t> m_aIDs;
    std::vector<double> m_aDoubles;
    std::vector<int> m_aInts;
    std::vector<unsigned> m_aSizes;
    std::vector<std::string> m_aStrings;
    bool m_bFlag;
    };

// A line number (#ID), a string Tag, and a STEPLineData object.

struct STEPLine
    {
    STEPLine() :
            m_nLineNumber(0),
            m_sTag(),
            m_STEPLineData(STEPTag::NULL_NONE_INVALID)
    {}


    explicit STEPLine(STEPTag nSTEPTag) :
            m_nLineNumber(0),
            m_sTag(),
            m_STEPLineData(nSTEPTag)
    { m_sTag.reserve(256 - 32); }

    STEPLine(const STEPLine &) = default;

    STEPLine(STEPLine&& other) NOEXCEPT :
        m_nLineNumber(other.m_nLineNumber), m_sTag(std::move(other.m_sTag)), m_STEPLineData(std::move(other.m_STEPLineData))
        {
            other.m_nLineNumber = 0;
        }

    STEPLine& operator=(const STEPLine& other) = default;

    STEPLine& operator=(STEPLine && other) NOEXCEPT
        {
        if (&other == this)
			return *this;
        m_nLineNumber = other.m_nLineNumber;
        other.m_nLineNumber = 0;
        m_sTag = std::move(other.m_sTag);
        m_STEPLineData = std::move(other.m_STEPLineData);
        return *this;
        }

    ~STEPLine() = default;

    void clear()
        {
            m_nLineNumber = 0;
            m_sTag.clear();
            m_STEPLineData.clear();
        }

    size_t m_nLineNumber;
    std::string m_sTag;
    STEPLineData m_STEPLineData;
    };

typedef std::unordered_map<std::string, STEPTag> STEPTagMapType;
typedef std::unordered_map<size_t, STEPLineData> STEPLineDataMapType;
typedef std::unordered_map<size_t, entity *> IDEntityMapType;
typedef std::unordered_map<body *, SGM::Transform3D> BodyToTransformMapType;

typedef std::vector<std::string *> StringLinesChunk;
typedef std::vector<STEPLine *> STEPLineChunk;

///////////////////////////////////////////////////////////////////////////////
//
// Declarations: functions called by serial and concurrent readers
//
///////////////////////////////////////////////////////////////////////////////

#ifdef SGM_MULTITHREADED
size_t ParseSTEPStreamConcurrent(SGM::Result &rResult,
                                 SGM::TranslatorOptions const &Options,
                                 std::vector <std::string> &aLog,
                                 std::ifstream &inputFileStream,
                                 STEPTagMapType const &mSTEPTagMap,
                                 STEPLineDataMapType &mSTEPData);
#endif

void CreateEntities(SGM::Result           &rResult,
                    size_t                 maxSTEPLineNumber,
                    STEPLineDataMapType   &mSTEPData,
                    std::vector<entity *> &aEntities);

void ProcessSTEPLine(STEPTagMapType const &mSTEPTagMap,
                     std::string &sLine,
                     STEPLine &stepLine,
                     bool bScan);

///////////////////////////////////////////////////////////////////////////////
//
// Parsing functions
//
///////////////////////////////////////////////////////////////////////////////

// return one after the first occurrence of the character in the string
// the character must exist

inline const char* SkipChar(char const *pos, int character)
    {
    pos = std::strchr(pos,character);
    assert(pos != nullptr);
    return ++pos;
    }

// return one after last character of the word string of the given length
// the word must exist

inline const char* SkipWord(char const *pos, char const *word, size_t length)
    {
    pos = std::strstr(pos,word);
    assert(pos != nullptr);
    return (pos+length);
    }

// return one after last character of the word string of the given length
// or nullptr if the word is not found

inline const char* FindWord(char const *pos, char const *word, size_t length)
    {
    pos = std::strstr(pos,word);
    if (pos != nullptr)
        return (pos+length);
    return nullptr;
    }

// read an unsigned index at the string position
// return one past the end of the parsed number
inline const char* ReadIndex(char const *pString, size_t *index)
    {
    char * end;
    *index = std::strtoul(pString, &end, 10);
    assert(errno != ERANGE);
    return end;
    }

// append a ID index read from the string to the vector
// return one past the end of the parsed number

inline const char* AppendIndex(char const *pString,
                               std::vector<size_t> &aIDs)
    {
    char * end;
    size_t n = std::strtoul(pString, &end, 10);
    assert(errno != ERANGE);
    aIDs.push_back(n);
    return end;
    }

inline const char* AppendInt(char const *pString,
                             std::vector<int> &aInts)
    {
    char * end;
    long i = std::strtol(pString, &end, 10); // only long parsing is available
    assert(errno != ERANGE);
    assert(i <= std::numeric_limits<int>::max());
    assert(i >= std::numeric_limits<int>::min());
    aInts.push_back((int)i);
    return end;
    }

inline const char* AppendDouble(char const *pString,
                                std::vector<double> &aDoubles)
    {
    char * end;
    double d = std::strtod(pString, &end);
    assert(errno != ERANGE);
    aDoubles.push_back(d);
    return end;
    }

// find a single index, must use '#', and add it to aIndices.
// return position one after the last digit

inline const char* FindIndex(char const   *pos,
                             std::vector<size_t> &aIndices)
    {
    // CIRCLE ( 'NONE', #12021, 3.885554089709762700 ) ;
    // FACE_SURFACE('',([...]),#3914,.T.); <--- used for only the '#' after the ([...])
    return AppendIndex(SkipChar(pos,'#'),aIndices);
    }

// an array of multiple indices '#' '*' that end with closing parenthesis ')'.
// return the position at the closing parenthesis

inline const char* FindIndicesGroup(char const          *pLineAfterStepTag,
                                    std::vector<size_t> &aIndices)
    {
    // B_SPLINE_CURVE( 2, ( #94249, #94250, #94251 ), [...]
    // FACE_SURFACE('',(#3912,#3913),#3914,.T.); <---only the first two inside the ()
    // ADVANCED_FACE('PartBody',(#27845),#27609,.F.) ; <---only the first one inside the ()
    // EDGE_LOOP ( 'NONE', ( #4088, #4089, #4090, #4091 ) ) ;
    static const char key[] = "#*)";
    const char *pos;
    pos = std::strpbrk(pLineAfterStepTag, key);
    while (pos != nullptr)
        {
        if (*pos == '#')
            {
            pos = AppendIndex(++pos,aIndices);
            }
        else if (*pos == '*')
            {
            aIndices.push_back(0);
            ++pos;
            }
        else if (*pos == ')')
            {
            return pos;
            }
        pos = strpbrk(pos, key);
        }
    throw std::runtime_error(pLineAfterStepTag);
    }

// a two dimensional array of indices '#' that start and end with double parenthesis
// return the position after the closing parenthesis

// ((#3246,#3247),(#3250,#3251),(#3254,#3255),(#3286,#3287)),

inline const char* FindIndicesMatrix(char const          *pos,
                                     std::vector<size_t> &aIndices,
                                     size_t              *nRows,
                                     size_t              *nColumns)
    {
    static const char NEXT_ROW[] = ",)";
    static const char NEXT_COL[] = "#)";

    size_t nColumnCount = 0;
    *nRows = 0;
    *nColumns = 0;

    pos = SkipChar(pos,'(');

    while (pos != nullptr)              // START outer loop
        {
        pos = SkipChar(pos,'(');
        pos = std::strpbrk(pos, NEXT_COL);
        while (pos != nullptr)          // START inner loop
            {
            if (*pos == '#')
                {
                pos = AppendIndex(++pos,aIndices);
                ++nColumnCount;
                }
            else if (*pos == ')')
                {
                ++pos;
                if (*nColumns == 0)
                    *nColumns = nColumnCount;
                else if (*nColumns != nColumnCount)
                    throw std::runtime_error("different number of columns in indices matrix");
                break;
                }
            pos = strpbrk(pos, NEXT_COL);
            }
        pos = std::strpbrk(pos, NEXT_ROW);
        nColumnCount = 0;
        ++*nRows;
        assert(pos != nullptr);
        if (*pos == ',')
            continue;
        else if (*pos == ')')
            return ++pos;
        }
    throw std::runtime_error("unable to end indices matrix");
    }


// find all indices '#' or '*' on the string no matter how arranged between (#,(#)), all the way to end of string
// return one past the position of the last number found

inline const char* FindIndicesAll(char const          *pLineAfterStepTag,
                                  std::vector<size_t> &aIndices)
    {
    //TRIMMED_CURVE($,#2697,(#12677,PARAMETER_VALUE(22.336)),(#12678,PARAMETER_VALUE(247.663)),.T.,.PARAMETER.);
    static const char key[] = "#*";
    const char *pos;
    const char *last = nullptr;
    pos = std::strpbrk(pLineAfterStepTag, key);
    while (pos != nullptr)
        {
        if (*pos == '#')
            {
            pos = AppendIndex(++pos,aIndices);
            last = pos;
            }
        else if (*pos == '*')
            {
            aIndices.push_back(0);
            ++pos;
            last = pos;
            }
        pos = strpbrk(pos, key);
        }
    return last;
    }

// find vector of indices followed by a bool flag
// return position one after the closing '.'

inline const char * FindIndicesAndFlag(char const *pLineAfterStepTag,
                                       std::vector<size_t> &aIndices,
                                       bool       &bFlag)
    {
    static const char key[] = "#*.";
    const char *pos;
    pos = std::strpbrk(pLineAfterStepTag, key);
    while (pos != nullptr)
        {
        if (*pos == '#')
            {
            pos = AppendIndex(++pos,aIndices);
            }
        else if (*pos == '*')
            {
            aIndices.push_back(0);
            ++pos;
            }
        else if (*pos == '.')
            {
            ++pos;
            if(*pos=='T')
                {
                bFlag=true;
                return pos+2;
                }
            else if(*pos=='F')
                {
                bFlag=false;
                return pos+2;
                }
            }
        pos = strpbrk(pos, key);
        }
    throw std::runtime_error(pLineAfterStepTag);
    }

inline const char* FindFlag(char const *pos,
                            bool       &bFlag)
    {
    pos = SkipChar(pos, '.');
    if(*pos=='T')
        {
        bFlag=true;
        return pos+2;
        }
    else if(*pos=='F')
        {
        bFlag=false;
        return pos+2;
        }
    throw std::runtime_error("could not find T/F flag");
    }

// get the next int after a ','

inline const char* FindInt(char const *pos,
                           std::vector<int> &aInts)
    {
    return AppendInt(SkipChar(pos,','),aInts);
    }

// get the next double after a ','

inline const char* FindDouble(char const *pos,
                              std::vector<double> &aData)
    {
    return AppendDouble(SkipChar(pos,','),aData);
    }

inline const char * FindIndexAndDouble(char const *pos,
                                       std::vector<size_t> &aIndices,
                                       std::vector<double> &aDoubles)
    {
    return FindDouble(FindIndex(pos, aIndices), aDoubles);
    }

inline void FindDoubleVector3(char const * pos,
                              std::vector<double> &aData)
    {
    // " ( 'NONE',  ( 81.01848140189099500, 21.68962376674730000, -3.860665940705227900 ) ) "
    // This is the most heavily called Find function;
    char * end;
    aData.push_back(std::strtod(SkipChar(SkipChar(pos,'('),'('), &end));
    assert(errno != ERANGE);
    aData.push_back(std::strtod(SkipChar(end,','), &end));
    assert(errno != ERANGE);
    aData.push_back(std::strtod(SkipChar(end,','), nullptr));
    assert(errno != ERANGE);
    }

inline void FindVector(char const * pos,
                       double *vec)
    {
    // " ( 'NONE',  ( 81.01848140189099500, 21.68962376674730000, -3.860665940705227900 ) ) "
    // This is the most heavily called Find function;
    char * end;
    vec[0] = std::strtod(SkipChar(SkipChar(pos,'('),'('), &end);
    assert(errno != ERANGE);
    vec[1] = std::strtod(SkipChar(end,','), &end);
    assert(errno != ERANGE);
    vec[2] = std::strtod(SkipChar(end,','), nullptr);
    assert(errno != ERANGE);
    }

// find all double PARAMETER_VALUE(x.xxx) on the string
// return position one after the last double value, i.e. the close parenthesis

inline const char* FindParameters(char const   *pLineAfterStepTag,
                                  std::vector<double> &aData)
    {
    //TRIMMED_CURVE($,#2697,(#12677,PARAMETER_VALUE(22.336)),(#12678,PARAMETER_VALUE(247.663)),.T.,.PARAMETER.);
    static const char key[] = "PARAMETER_VALUE";
    const char *last = nullptr;

    const char *pos = SkipWord(pLineAfterStepTag,key,15);
    pos = SkipChar(pos,'(');
    while (pos != nullptr)
        {
        last = AppendDouble(pos,aData);
        pos = FindWord(last,key,15);
        if (pos != nullptr)
            {
            pos = SkipChar(pos, '(');
            }
        }
    return last;
    }


// find all doubles between first opening parenthesis '('
// and the next closing parenthesis ')'
// return the position after the closing parenthesis

inline const char* FindDoubleVector(char const   *pos,
                             std::vector<double> &aData)
    {
    static const char separator[] = ",)";

    pos = SkipChar(pos, '(');
    pos = AppendDouble(pos,aData);

    pos = std::strpbrk(pos, separator);
    while (pos != nullptr)
        {
        if (*pos == ',')
            {
            pos = AppendDouble(++pos,aData);
            }
        else if (*pos == ')')
            {
            ++pos;
            return pos;
            }
        pos = strpbrk(pos, separator);
        }
    throw std::runtime_error("no closing ) separator");
    }

// find all integers between first opening parenthesis '('
// and the next closing parenthesis ')'
// return the position after the closing parenthesis

inline const char* FindIntVector(char const   *pos,
                          std::vector<int> &aInts)
    {
    static const char separator[] = ",)";

    pos = SkipChar(pos, '(');
    pos = AppendInt(pos,aInts);

    pos = std::strpbrk(pos, separator);
    while (pos != nullptr)
        {
        if (*pos == ',')
            {
            pos = AppendInt(++pos,aInts);
            }
        else if (*pos == ')')
            {
            ++pos;
            return pos;
            }
        pos = strpbrk(pos, separator);
        }
    throw std::runtime_error("no closing ) separator");
    }

// find a STEP tag
// return pointer to string one past the end of first STEP tag

inline const char *FindStepTag(const char *pString, std::string &sTag)
    {
    static const char WHITE_SPACE[] = " \n\r\t";
    static const char WHITE_SPACE_PAREN[] = " (\n\r\t";

    // skip to first non-white space
    pString += std::strspn(pString, WHITE_SPACE);

    // stop at the next following whitespace or open parenthesis
    size_t end = std::strcspn(pString, WHITE_SPACE_PAREN);

    // Try again if we did not find a token before a parenthesis, for example,
    // #103697 =( BOUNDED_CURVE ( )  B_SPLINE_CURVE ( 3, (
    // #2465=(B_SPLINE_CURVE(3,(#4345,#4346,#4347,#4348,#4349,#4350,#4351)
    if (end == 0)
        {
        ++pString;
        pString += std::strspn(pString, WHITE_SPACE);
        end = std::strcspn(pString, WHITE_SPACE_PAREN);
        if (end < 1)
            {
            throw std::runtime_error(pString); // no token was found
            }
        }
    sTag.assign(pString, end); // assign the token to our m_sTag string
    return pString + end;      // return position after the tag
    }


// Return the STEP Line number, unless the line is not a STEP Line or the tag is unrecognized then return 0.

inline size_t MoveSTEPLineIntoMap(SGM::Result &rResult,
                                  std::vector<std::string> &aLog,
                                  STEPLine &stepLine,
                                  STEPLineDataMapType &mSTEPData)
    {
    // if there was a line number (#ID) and a recognized tag
    // add the data parsed to the Map (#ID -> STEPLineData)

    if (stepLine.m_nLineNumber == 0)
        {
        return 0;
        }
    else if (stepLine.m_STEPLineData.m_nSTEPTag == STEPTag::NULL_NONE_INVALID)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeUnknownType);
        aLog.push_back("Unknown STEP Tag " + stepLine.m_sTag);
        return 0;
        }
    else
        {
        // Cast stepLine into an r-value reference---in effect, clearing out the
        // stepLine member data object as it is copied into the map
        mSTEPData.emplace(stepLine.m_nLineNumber, std::move(stepLine.m_STEPLineData));
        return stepLine.m_nLineNumber;
        }
    }

inline const char *FindSingleQuotedString(const char *pString, std::string &sQuotedString)
{
    std::string sInput(pString);
    size_t first = sInput.find('\'');
    size_t end = sInput.find('\'', ++first);
    sQuotedString = sInput.substr(first, end-first);
    pString += ++end;
    return pString;


    //static const char SINGLE_QUOTE[] = "\'";
    //// skip to first quote
    //size_t first = std::strspn(pString, SINGLE_QUOTE);
    //pString += first;

    //// stop at the next following quote
    //size_t end = std::strcspn(pString, SINGLE_QUOTE);

    //sQuotedString.assign(pString, end); // assign the token to our m_sTag string
    //return pString + end;      // return position after the tag
}

} // namespace SGMInternal

#endif //SGM_READFILE_H
