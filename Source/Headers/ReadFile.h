#ifndef SGM_READFILE_H
#define SGM_READFILE_H

#include <cerrno>
#include <cstdlib>
#include <unordered_map>
#include <vector>

namespace SGMInternal {

class entity;

///////////////////////////////////////////////////////////////////////////////
//
// Types we use for parsing
//
///////////////////////////////////////////////////////////////////////////////

class STEPLineData
{
public:

    // TODO: add a single m_Double and m_ID member data for many Types
    STEPLineData() : m_nType(0), m_aIDs(), m_aDoubles(), m_aInts(), m_bFlag(true)
    {}

//    STEPLineData(const STEPLineData &other) = default;
//
//    STEPLineData(STEPLineData &&other) :
//            m_nType(other.m_nType),
//            m_aIDs(std::move(other.m_aIDs)),
//            m_aDoubles(std::move(other.m_aDoubles)),
//            m_aInts(std::move(other.m_aInts)),
//            m_bFlag(other.m_bFlag)
//    {}

    void clear()
    {
        // reset it to match exactly the default constructor
        m_nType = 0;
        m_aIDs.clear();
        m_aDoubles.clear();
        m_aInts.clear();
        m_bFlag = true;
    }

    size_t m_nType;
    std::vector <size_t> m_aIDs;
    std::vector<double> m_aDoubles;
    std::vector<int> m_aInts;
    bool m_bFlag;
};

typedef std::unordered_map <std::string, size_t> STEPTagMapType;
typedef std::unordered_map <size_t, STEPLineData> STEPLineDataMapType;
typedef std::unordered_map<size_t, entity *> IDEntityMapType;


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
    std::strstr(pos,word);
    assert(pos != nullptr);
    return (pos+length);
    }

// return one after last character of the word string of the given length
// or nullptr if the word is not found

inline const char* FindWord(char const *pos, char const *word, size_t length)
    {
    std::strstr(pos,word);
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

inline void FindDoubleVector3(char const *pos,
                              std::vector<double> &aData)
    {
    // This is the most heavily called Find function;
    // collapsing makes it a little faster.
    AppendDouble(SkipChar(
        AppendDouble(SkipChar(
            AppendDouble(SkipChar(SkipChar(pos,'('),
                '('),aData),
            ','),aData),
        ','),aData);
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
    const char separator[] = ",)";

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
    const char separator[] = ",)";

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

}

#endif //SGM_READFILE_H
