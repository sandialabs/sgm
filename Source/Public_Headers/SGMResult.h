#ifndef SGM_RESULT_H
#define SGM_RESULT_H

#define SGM_TIMING_RAY_FIRE

#include "SGMEnums.h"
#include "SGMEntityClasses.h"

#include <memory>
#include <string>
#include <vector>

#ifdef SGM_TIMING_RAY_FIRE
#include <map>
#endif

#include "sgm_export.h"

namespace SGMInternal
{
class thing;
}

namespace SGM
{
class Result
    {
    public:

        SGM_EXPORT Result() : m_nType(ResultType::ResultTypeOK),m_pThing(nullptr),m_bLog(false),m_nDebugFlag(0) {}

        SGM_EXPORT explicit Result(SGMInternal::thing *pThing) :
                m_nType(ResultType::ResultTypeOK), m_pThing(pThing),
                m_bLog(false),m_nDebugFlag(0)
        {}

        SGM_EXPORT void SetResult(ResultType nType);

        SGM_EXPORT void SetMessage(std::string const &sMessage) { m_sMessage += sMessage; }

        SGM_EXPORT void ClearMessage() { m_sMessage.clear(); m_nType = ResultType::ResultTypeOK; }

        SGM_EXPORT ResultType GetResult() const {return m_nType;}

        SGM_EXPORT std::string const &Message() const {return m_sMessage;}

        SGM_EXPORT SGMInternal::thing *GetThing() const {return m_pThing;}

        SGM_EXPORT void SetLog(bool bTurnOn) {m_bLog=bTurnOn;}

        SGM_EXPORT bool GetLog() const {return m_bLog;}

        SGM_EXPORT std::vector<Entity> const &GetLogEntities1() const {return m_aLogEntities1;}

        SGM_EXPORT std::vector<Entity> const &GetLogEntities2() const {return m_aLogEntities2;}

        SGM_EXPORT std::vector<LogType> const &GetLogEntries() const {return m_aLogEntries;}

        SGM_EXPORT void ClearLog() {m_aLogEntities1.clear(); m_aLogEntities2.clear(); m_aLogEntries.clear();}

        SGM_EXPORT void AddLog(Entity const &EntityID1,
                               Entity const &EntityID2,
                               LogType       nLogEntry) 
            {
            m_aLogEntities1.push_back(EntityID1);
            m_aLogEntities2.push_back(EntityID2);
            m_aLogEntries.push_back(nLogEntry);
            }

        //////////////////////////////////////////////////////////////////////
        //
        // For internal use only.
        //
        //////////////////////////////////////////////////////////////////////

        SGM_EXPORT void SetDebugFlag(size_t nFlag) {m_nDebugFlag=nFlag;}

        SGM_EXPORT size_t GetDebugFlag() {return m_nDebugFlag;}

        SGM_EXPORT void SetDebugData(std::vector<double> const &aData) {m_aDebugData=aData;}

        SGM_EXPORT std::vector<double> GetDebugData() {return m_aDebugData;}

#ifdef SGM_TIMING_RAY_FIRE
        SGM_EXPORT void IncrementIntersectLineAndEntityCount(SGM::EntityType EntityType);

        SGM_EXPORT void PrintIntersectLineAndEntityCount() const;
#endif

        // Debug flags
        // 0 turned off. (Default)
        // 1 make bad face facets.
        // 2 make no face facets.
        // 3 return bad curve tests.
        // 4 return bad surface tests.
        // 5 make blocks with bad topology connections.
        // 6 set the ray fire direction to m_aDebugData[0-2].

    private:

        ResultType           m_nType;
        std::string          m_sMessage;
        SGMInternal::thing  *m_pThing;
        bool                 m_bLog;
        std::vector<Entity>  m_aLogEntities1;
        std::vector<Entity>  m_aLogEntities2;
        std::vector<LogType> m_aLogEntries;
        size_t               m_nDebugFlag;
        std::vector<double>  m_aDebugData;

#ifdef SGM_TIMING_RAY_FIRE
    std::map<SGM::EntityType,size_t> m_mIntersectLineAndEntityCount;
#endif

};

} // End of SGM namespace

#endif // End of SGM_RESULT_H
