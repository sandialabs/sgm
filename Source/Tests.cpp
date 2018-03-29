#include "SGMChecker.h"
#include "SGMPrimatives.h"
#include "SGMComplex.h"
#include "SGMMathematics.h"
#include "SGMTranslators.h"
#include "SGMDataClasses.h"
#include "SGMQuery.h"
#include "SGMTree.h"
#include "FileFunctions.h"
#include "EntityClasses.h"
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <algorithm>

class TestCommand
    {
    public:

        std::string              m_sOutput;
        std::string              m_sCommand;
        std::vector<double>      m_aDoubles;
        std::vector<std::string> m_aStrings;
        std::vector<std::string> m_aVariables;
    };

size_t ParseVariable(std::map<std::string,std::vector<size_t> > &mVariableMap,
                     std::string                          const &sVariable)
    {
    // Find the variable name and index.
    
    size_t nLength=sVariable.length();
    char const *str=sVariable.c_str();
    size_t Index1,Index2;
    bool bFound=false;
    for(Index1=0;Index1<nLength;++Index1)
        {
        if(str[Index1]=='[')
            {
            bFound=true;
            break;
            }
        }
    if(bFound)
        {
        std::string sIndex=str+Index1+1;
        int nIndex;
        std::stringstream(sIndex) >> nIndex;
        std::string sVariableName;
        for(Index2=0;Index2<Index1;++Index2)
            {
            sVariableName+=str[Index2];
            }
        return mVariableMap[sVariableName][nIndex];
        }
    else
        {
        return mVariableMap[sVariable][0];
        }
    }

bool RunCreateBlock(SGM::Result &rResult,std::map<std::string,std::vector<size_t> > &mVariableMap,std::string const &,TestCommand &TestData)
    {
    SGM::Point3D Pos0(TestData.m_aDoubles[0],TestData.m_aDoubles[1],TestData.m_aDoubles[2]);
    SGM::Point3D Pos1(TestData.m_aDoubles[3],TestData.m_aDoubles[4],TestData.m_aDoubles[5]);
    SGM::Body BodyID=SGM::CreateBlock(rResult,Pos0,Pos1);
    mVariableMap[TestData.m_sOutput].push_back(BodyID.m_ID);
    return true;
    }

bool RunSaveSTEP(SGM::Result &rResult,std::map<std::string,std::vector<size_t> > &mVariableMap,std::string const &,TestCommand &TestData)
    {
    SGM::Entity Ent(mVariableMap[TestData.m_aVariables[0]][0]);
    SGM::TranslatorOptions Options;
    SGM::SaveSTEP(rResult,TestData.m_aStrings[0],Ent,Options);
    return true;
    }

bool RunSaveSTL(SGM::Result &rResult,std::map<std::string,std::vector<size_t> > &mVariableMap,std::string const &sDir,TestCommand &TestData)
    {
    size_t nEnt=ParseVariable(mVariableMap,TestData.m_aVariables[0]);
    SGM::Entity Ent(nEnt);
    SGM::TranslatorOptions Options;
    std::string sFullPath=sDir+"/"+TestData.m_aStrings[0];
    SGM::SaveSTL(rResult,sFullPath,Ent,Options);
    return true;
    }

bool RunReadFile(SGM::Result &rResult,std::map<std::string,std::vector<size_t> > &mVariableMap,std::string const &sDir,TestCommand &TestData)
    {
    std::vector<SGM::Entity> aEntities;
    std::vector<std::string> aLog;
    SGM::TranslatorOptions Options;
    std::string sFullPath=sDir+"/"+TestData.m_aStrings[0];
    size_t nEnts=SGM::ReadFile(rResult,sFullPath,aEntities,aLog,Options);
    std::vector<size_t> aEnts;
    aEnts.reserve(nEnts);
    size_t Index1;
    for(Index1=0;Index1<nEnts;++Index1)
        {
        aEnts.push_back(aEntities[Index1].m_ID);
        }
    mVariableMap[TestData.m_aVariables[0]]=aEnts;
    return true;
    }

bool RunCompareFiles(SGM::Result &rResult,std::map<std::string,std::vector<size_t> > &,std::string const &sDir,TestCommand &TestData)
    {
    std::string sFullPath0=sDir+"/"+TestData.m_aStrings[0];
    std::string sFullPath1=sDir+"/"+TestData.m_aStrings[1];
    return SGM::CompareFiles(rResult,sFullPath0,sFullPath1);
    }

bool RunCompareSizes(SGM::Result &,std::map<std::string,std::vector<size_t> > &mVariableMap,std::string const &,TestCommand &TestData)
    {
    std::vector<size_t> aSizes;
    size_t Index1;
    size_t nDoubles=TestData.m_aDoubles.size();
    for(Index1=0;Index1<nDoubles;++Index1)
        {
        aSizes.push_back((size_t)TestData.m_aDoubles[Index1]);
        }
    size_t nVariables=TestData.m_aVariables.size();
    for(Index1=0;Index1<nVariables;++Index1)
        {
        aSizes.push_back(ParseVariable(mVariableMap,TestData.m_aVariables[Index1]));
        }
    return aSizes[0]==aSizes[1];
    }

bool RunFindCloseFaces(SGM::Result &rResult,std::map<std::string,std::vector<size_t> > &mVariableMap,std::string const &,TestCommand &TestData)
    {
    std::vector<SGM::Face> aFaces;
    size_t nEnt=ParseVariable(mVariableMap,TestData.m_aVariables[0]);
    SGM::Entity Ent(nEnt);
    SGM::Point3D Pos(TestData.m_aDoubles[0],TestData.m_aDoubles[1],TestData.m_aDoubles[2]);
    double dDist=TestData.m_aDoubles[3];
    size_t nEnts=SGM::FindCloseFaces(rResult,Pos,Ent,dDist,aFaces);

    std::vector<size_t> aEnts;
    aEnts.reserve(nEnts);
    size_t Index1;
    for(Index1=0;Index1<nEnts;++Index1)
        {
        aEnts.push_back(aFaces[Index1].m_ID);
        }
    mVariableMap[TestData.m_aVariables[0]]=aEnts;
    mVariableMap[TestData.m_sOutput].push_back(nEnts);

    return true;
    }

bool RunFindCloseEdges(SGM::Result &rResult,std::map<std::string,std::vector<size_t> > &mVariableMap,std::string const &,TestCommand &TestData)
    {
    std::vector<SGM::Edge> aEdges;
    size_t nEnt=ParseVariable(mVariableMap,TestData.m_aVariables[0]);
    SGM::Entity Ent(nEnt);
    SGM::Point3D Pos(TestData.m_aDoubles[0],TestData.m_aDoubles[1],TestData.m_aDoubles[2]);
    double dDist=TestData.m_aDoubles[3];
    size_t nEnts=SGM::FindCloseEdges(rResult,Pos,Ent,dDist,aEdges);

    std::vector<size_t> aEnts;
    aEnts.reserve(nEnts);
    size_t Index1;
    for(Index1=0;Index1<nEnts;++Index1)
        {
        aEnts.push_back(aEdges[Index1].m_ID);
        }
    mVariableMap[TestData.m_aVariables[0]]=aEnts;
    mVariableMap[TestData.m_sOutput].push_back(nEnts);

    return true;
    }

typedef bool (*SGMFunction)(SGM::Result &,
                            std::map<std::string,std::vector<size_t> > &,
                            std::string const &,
                            TestCommand &); // function pointer type

void CreateFunctionMap(std::map<std::string,SGMFunction> &mFunctionMap)
    {
    mFunctionMap["CreateBlock"]=RunCreateBlock;
    mFunctionMap["SaveSTEP"]=RunSaveSTEP;
    mFunctionMap["SaveSTL"]=RunSaveSTL;
    mFunctionMap["ReadFile"]=RunReadFile;
    mFunctionMap["CompareFiles"]=RunCompareFiles;
    mFunctionMap["CompareSizes"]=RunCompareSizes;
    mFunctionMap["FindCloseFaces"]=RunFindCloseFaces;
    mFunctionMap["FindCloseEdges"]=RunFindCloseEdges;
    }

void FindOutput(std::string const &sFileLine,
                std::string       &sOutput,
                std::string       &sRight)
    {
    size_t nLength=sFileLine.length();
    size_t Index1,Index2;
    for(Index1=0;Index1<nLength;++Index1)
        {
        if(sFileLine.c_str()[Index1]=='=')
            {
            for(Index2=0;Index2<Index1;++Index2)
                {
                sOutput+=sFileLine.c_str()[Index2];
                }
            for(Index2=Index1+1;Index2<nLength;++Index2)
                {
                sRight+=sFileLine.c_str()[Index2];
                }
            return;
            }
        }
    sRight=sFileLine;
    }

void FindCommand(std::string const &sFileLine,
                 std::string       &sCommand,
                 std::string       &sRight)
    {
    size_t nLength=sFileLine.length();
    size_t Index1,Index2;
    for(Index1=0;Index1<nLength;++Index1)
        {
        char c=sFileLine.c_str()[Index1];
        if(c=='(' || c==';')
            {
            for(Index2=0;Index2<Index1;++Index2)
                {
                sCommand+=sFileLine.c_str()[Index2];
                }
            for(Index2=Index1+1;Index2<nLength;++Index2)
                {
                sRight+=sFileLine.c_str()[Index2];
                }
            return;
            }
        }
    }

bool FindFirstArgument(std::string const &sFileLine,
                       std::string       &sLeft,
                       std::string       &sRight)
    {
    size_t nLength=sFileLine.length();
    size_t Index1,Index2;
    for(Index1=0;Index1<nLength;++Index1)
        {
        if(sFileLine.c_str()[Index1]==',')
            {
            for(Index2=0;Index2<Index1;++Index2)
                {
                sLeft+=sFileLine.c_str()[Index2];
                }
            for(Index2=Index1+1;Index2<nLength;++Index2)
                {
                sRight+=sFileLine.c_str()[Index2];
                }
            return true;
            }
        if(sFileLine.c_str()[Index1]==')')
            {
            for(Index2=0;Index2<Index1;++Index2)
                {
                sLeft+=sFileLine.c_str()[Index2];
                }
            return true;
            }
        }
    return false;
    }

size_t FindArguments(std::string        const &sFileLine,
                     std::vector<std::string> &aArguments)
    {
    std::string sLeft,sRight,sString=sFileLine;
    bool bFound=true;
    while(bFound)
        {
        bFound=FindFirstArgument(sString,sLeft,sRight);
        if(bFound)
            {
            sString=sRight;
            sRight.clear();
            aArguments.push_back(sLeft);
            sLeft.clear();
            }
        }
    return aArguments.size();
    }

bool ParseLine(std::string const &sFileLine,
               TestCommand       &LineData)
    {
    if(sFileLine.empty() || sFileLine.c_str()[0]=='/')
        {
        return false;
        }

    // Anything in front of an '=' is the sOutput.
    // Anything after an '=' is the sCommand or if no '=' exists then the first 
    // up to the first (.
    // Break things up by ',' and ')' if they have a '"' then they are a string
    // else they are a double.

    std::string sRight,sArgs;
    FindOutput(sFileLine,LineData.m_sOutput,sRight);
    FindCommand(sRight,LineData.m_sCommand,sArgs);
    std::vector<std::string> aArguments;
    size_t nArguments=FindArguments(sArgs,aArguments);
    size_t Index1,Index2;
    for(Index1=0;Index1<nArguments;++Index1)
        {
        char c=aArguments[Index1].c_str()[0];
        if(c==-109 || c==-108)  // String
            {
            size_t nLength=aArguments[Index1].length()-1;
            std::string sString;
            for(Index2=1;Index2<nLength;++Index2)
                {
                sString+=aArguments[Index1].c_str()[Index2];
                }
            LineData.m_aStrings.push_back(sString);
            }
        else if(c<'A' && c!='#') // Double
            {
            double dData;
            std::stringstream(aArguments[Index1]) >> dData;
            LineData.m_aDoubles.push_back(dData);
            }
        else // Variable
            {
            LineData.m_aVariables.push_back(aArguments[Index1]);
            }
        }
    return true;
    }

bool RunFileLine(SGM::Result                                &rResult,
                 std::map<std::string,SGMFunction>          &mFunctionMap,
                 std::string                          const &sTestDirectory,
                 std::map<std::string,std::vector<size_t> > &mVariableMap,
                 std::string                          const &sFileLine,
                 FILE                                       *pOutputFilep)
    {
    TestCommand LineData;
    if(ParseLine(sFileLine,LineData))
        {
        std::map<std::string,SGMFunction>::const_iterator iter=mFunctionMap.find(LineData.m_sCommand);
        if(iter==mFunctionMap.end())
            {
            rResult.SetResult(SGM::ResultType::ResultTypeUnknownCommand);
            rResult.SetMessage(LineData.m_sCommand);
            }
        if((*(iter->second))(rResult,mVariableMap,sTestDirectory,LineData)==false)
            {
            return false;
            }
        }

    pOutputFilep;
    return true;
    }

bool RunTestFile(SGM::Result                       &rResult,
                 std::map<std::string,SGMFunction> &mFunctionMap,
                 std::string                 const &sTestDirectory,
                 std::string                 const &sFileName,
                 FILE                              *pTestFile,
                 FILE                              *pOutputFile)
    {
    std::map<std::string,std::vector<size_t> > mVariableMap;
    bool bFound=true;
    bool bPassed=true;
    while(bFound)
        {
        std::string sFileLine;
        bFound=ReadFileLine(pTestFile,sFileLine);
        if(bFound)
            {
            if(RunFileLine(rResult,mFunctionMap,sTestDirectory,mVariableMap,sFileLine,pOutputFile)==false)
                {
                bPassed=false;
                }
            }
        }
    if(bPassed)
        {
        fprintf(pOutputFile,"Passed  \"%s\"\n",sFileName.c_str());
        }
    else
        {
        fprintf(pOutputFile,"Failed  \"%s\"\n",sFileName.c_str());
        }
    return bPassed;
    }

bool SGM::RunTestFile(SGM::Result       &rResult,
                      std::string const &sTestDirectory,
                      std::string const &sTestFileName,
                      std::string const &sOutputFileName)
    {
    std::map<std::string,SGMFunction> mFunctionMap;
    CreateFunctionMap(mFunctionMap);
    FILE *pOutputFile;
    fopen_s(&pOutputFile,sOutputFileName.c_str(),"w");
    if(pOutputFile==NULL)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        rResult.SetMessage(sOutputFileName);
        return false;
        }
    FILE *pTestFile;
    std::string sFullPathName=sTestDirectory+"/"+sTestFileName;
    fopen_s(&pTestFile,sFullPathName.c_str(),"rt");
    if(pTestFile==NULL)
        {
        rResult.SetResult(SGM::ResultType::ResultTypeFileOpen);
        rResult.SetMessage(sFullPathName);
        return false;
        }
    bool bAnswer=RunTestFile(rResult,mFunctionMap,sTestDirectory,sTestFileName,pTestFile,pOutputFile);
    fclose(pOutputFile);
    return bAnswer;
    }

void SGM::RunTestDirectory(SGM::Result       &rResult,
                           std::string const &sTestDirectory,
                           std::string const &sOutputFileName)
    {
    ///////////////////////////////////////////////////////////////////////////
    //
    //  Temp code to test STEP read.
    //
    ///////////////////////////////////////////////////////////////////////////

    //std::vector<Entity> aEntities;
    //std::vector<std::string> aLog;
    //SGM::TranslatorOptions Options;
    //SGM::ReadFile(rResult,sTestDirectory+"/Sphere.stp",aEntities,aLog,Options);

    ///////////////////////////////////////////////////////////////////////////
    //
    //  End of Temp code to test STEP read.
    //
    ///////////////////////////////////////////////////////////////////////////

    std::vector<std::string> aFileNames;
    ReadDirectory(sTestDirectory,aFileNames);

    FILE *pOutputFile;
    fopen_s(&pOutputFile,sOutputFileName.c_str(),"w");
    std::map<std::string,SGMFunction> mFunctionMap;
    CreateFunctionMap(mFunctionMap);

    size_t nFiles=aFileNames.size();
    size_t nPassed=0,nFailed=0;
    size_t Index1;
    for(Index1=0;Index1<nFiles;++Index1)
        {
        if(aFileNames[Index1].c_str()[0]!='.')
            {
            std::string sExtension;
            FindFileExtension(aFileNames[Index1],sExtension);
            if(sExtension=="txt")
                {
                std::string FullPath=sTestDirectory;
                FullPath+="/";
                FullPath+=aFileNames[Index1];
                FILE *pTestFile;
                fopen_s(&pTestFile,FullPath.c_str(),"rt");
                if(RunTestFile(rResult,mFunctionMap,sTestDirectory,aFileNames[Index1],pTestFile,pOutputFile))
                    {
                    ++nPassed;
                    }
                else
                    {
                    ++nFailed;
                    }
                fclose(pTestFile);
                }
            }
        }
    fprintf(pOutputFile,"\n%ld Passed %ld Failed\n",nPassed,nFailed);
    fclose(pOutputFile);
    }

bool SGM::CompareFiles(SGM::Result       &rResult,
                       std::string const &sFile1,
                       std::string const &sFile2)
    {
    // Find the file types.

    std::string Ext1,Ext2;
    FindFileExtension(sFile1,Ext1);
    FindFileExtension(sFile2,Ext2);
    if(Ext1!=Ext2)
        {
        return false;
        }

    // Open the files.

    FILE *pFile1,*pFile2;
    fopen_s(&pFile1,sFile1.c_str(),"rt");
    fopen_s(&pFile2,sFile2.c_str(),"rt");

    // Compare the files.

    bool bAnswer=false;
    if(Ext1=="stl")
        {
        bAnswer=true;
        SGM::TranslatorOptions Options;
        std::vector<std::string> aLog;
        std::vector<SGM::Entity> aEntities1,aEntities2;
        size_t nEntities1=ReadFile(rResult,sFile1,aEntities1,aLog,Options);
        size_t nEntities2=ReadFile(rResult,sFile2,aEntities2,aLog,Options);
        thing *pThing=rResult.GetThing();
        std::vector<double> aAreas1,aAreas2;
        aAreas1.reserve(nEntities1);
        aAreas2.reserve(nEntities2);
        size_t Index1;
        for(Index1=0;Index1<nEntities1;++Index1)
            {
            complex *pComplex=(complex *)(pThing->FindEntity(aEntities1[Index1].m_ID));
            aAreas1.push_back(pComplex->Area());
            delete pComplex;
            }
        for(Index1=0;Index1<nEntities1;++Index1)
            {
            complex *pComplex=(complex *)(pThing->FindEntity(aEntities2[Index1].m_ID));
            aAreas2.push_back(pComplex->Area());
            delete pComplex;
            }
        if(nEntities1==nEntities2)
            {
            std::sort(aAreas1.begin(),aAreas1.end());
            std::sort(aAreas2.begin(),aAreas2.end());
            bAnswer=true;
            for(Index1=0;Index1<nEntities1;++Index1)
                {
                double dArea1=aAreas1[Index1];
                double dArea2=aAreas1[Index1];
                if(SGM::NearEqual(dArea1,dArea2,0.01,true)==false)
                    {
                    bAnswer=false;
                    break;
                    }
                }
            }
        }
    else if(Ext1=="spt")
        {
        ReadToString(pFile1,"Data;");
        ReadToString(pFile2,"Data;");
        bAnswer=true;
        while(bAnswer)
            {
            char data1,data2;
            fread(&data1,1,1,pFile1);
            fread(&data2,1,1,pFile2);
            if(data1!=data2)
                {
                bAnswer=false;
                }
            }
        }

    fclose(pFile1);
    fclose(pFile2);
    return bAnswer;
    }

bool SGM::RunCPPTest(SGM::Result       &rResult,
                     size_t             nTestNumber,
                     std::string const &sOutputFileName)
    {
    rResult;

    FILE *pOutputFile;
    fopen_s(&pOutputFile,sOutputFileName.c_str(),"w");

    if(nTestNumber==1)
        {
        // Test the quartic equation.

        bool bAnswer=true;

        // 2*(x-1)(x-2)(x-3)(x-4) -> 2*x^4-20*x^3+70*x^2-100*x+48 Four roots

        std::vector<double> aRoots;
        size_t nRoots=SGM::Quartic(2,-20,70,-100,48,aRoots);
        if( nRoots!=4 || 
            SGM_ZERO<fabs(aRoots[0]-1) || 
            SGM_ZERO<fabs(aRoots[1]-2) ||
            SGM_ZERO<fabs(aRoots[2]-3) ||
            SGM_ZERO<fabs(aRoots[3]-4))
            {
            bAnswer=false;
            }

        // (x-1)(x-2)(x-3)(x-3) -> x^4-9*x^3+29*x^2-39*x+18 Three roots, one double

        aRoots.clear();
        nRoots=SGM::Quartic(1,-9,29,-39,18,aRoots);
        if( nRoots!=3 || 
            SGM_ZERO<fabs(aRoots[0]-1) || 
            SGM_ZERO<fabs(aRoots[1]-2) ||
            SGM_ZERO<fabs(aRoots[2]-3))
            {
            bAnswer=false;
            }

        // (x-1)(x-2)(x-2)(x-2) -> x^4-7*x^3+18*x^2-20*x+8 Two roots, one triple

        aRoots.clear();
        nRoots=SGM::Quartic(1,-7,18,-20,8,aRoots);
        if( nRoots!=2 || 
            SGM_ZERO<fabs(aRoots[0]-1) || 
            SGM_ZERO<fabs(aRoots[1]-2))
            {
            bAnswer=false;
            }

        // (x-1)(x-1)(x-2)(x-2) -> x^4-6*x^3+13*x^2-12*x+4 Two double roots

        aRoots.clear();
        nRoots=nRoots=SGM::Quartic(1,-6,13,-12,4,aRoots);
        if( nRoots!=2 || 
            SGM_ZERO<fabs(aRoots[0]-1) || 
            SGM_ZERO<fabs(aRoots[1]-2))
            {
            bAnswer=false;
            }

        // (x-1)(x-2)(x^2+1) -> x^4-3*x^3+3*x^2-3*x+2 Two roots

        aRoots.clear();
        nRoots=nRoots=SGM::Quartic(1,-3,3,-3,2,aRoots);
        if( nRoots!=2 || 
            SGM_ZERO<fabs(aRoots[0]-1) || 
            SGM_ZERO<fabs(aRoots[1]-2))
            {
            bAnswer=false;
            }

        // (x-1)(x-1)(x^2+1) -> x^4-2*x^3+2*x^2-2*x+1 One double root.

        aRoots.clear();
        nRoots=nRoots=SGM::Quartic(1,-2,2,-2,1,aRoots);
        if( nRoots!=1 || 
            SGM_ZERO<fabs(aRoots[0]-1))
            {
            bAnswer=false;
            }

        // (x-1)(x-1)(x-1)(x-1) -> x^4-4*x^3+6*x^2-4*x+1 One quadruple root.

        aRoots.clear();
        nRoots=nRoots=SGM::Quartic(1,-4,6,-4,1,aRoots);
        if( nRoots!=1 || 
            SGM_ZERO<fabs(aRoots[0]-1))
            {
            bAnswer=false;
            }

        // (x^2+1)(x^2+1) -> x^4+2*x^2+1 No roots.

        aRoots.clear();
        nRoots=nRoots=SGM::Quartic(1,0,2,0,1,aRoots);
        if( nRoots!=0 )
            {
            bAnswer=false;
            }

        fclose(pOutputFile);
        return bAnswer;
        }

    if(nTestNumber==2)
        {
        // Test the cubic equation.

        bool bAnswer=true;

        // 2*(x-1)*(x-2)*(x-3)=0 -> 2*x^3-12*x^2+22*x-12=0 Three roots

        std::vector<double> aRoots;
        size_t nRoots=SGM::Cubic(2,-12,22,-12,aRoots);
        if( nRoots!=3 || 
            SGM_ZERO<fabs(aRoots[0]-1) || 
            SGM_ZERO<fabs(aRoots[1]-2) ||
            SGM_ZERO<fabs(aRoots[2]-3))
            {
            bAnswer=false;
            }

        // (x-1)*(x-2)*(x-2)=0 -> x^3-5*x^2+8*x-4=0 Two roots, one double

        aRoots.clear();
        nRoots=SGM::Cubic(1,-5,8,-4,aRoots);
        if( nRoots!=2 || 
            SGM_ZERO<fabs(aRoots[0]-1) || 
            SGM_ZERO<fabs(aRoots[1]-2))
            {
            bAnswer=false;
            }

        // (x-1)*(x^2+1)=0 -> x^3-x^2+x-1=0 One root

        aRoots.clear();
        nRoots=SGM::Cubic(1,-1,1,-1,aRoots);
        if( nRoots!=1 || 
            SGM_ZERO<fabs(aRoots[0]-1))
            {
            bAnswer=false;
            }

        // (x-1)*(x-1)*(x-1)=0 -> x^3-x^2+x-1=0 One triple root

        aRoots.clear();
        nRoots=SGM::Cubic(1,-3,3,-1,aRoots);
        if( nRoots!=1 || 
            SGM_ZERO<fabs(aRoots[0]-1))
            {
            bAnswer=false;
            }

        // (x-1)*(x-2)=0 -> x^2-3*x+2=0 Two roots and degenerate

        aRoots.clear();
        nRoots=SGM::Cubic(0,1,-3,2,aRoots);
        if( nRoots!=2 || 
            SGM_ZERO<fabs(aRoots[0]-1) || 
            SGM_ZERO<fabs(aRoots[1]-2))
            {
            bAnswer=false;
            }

        fclose(pOutputFile);
        return bAnswer;
        }

    fclose(pOutputFile);
    return false;
    }

bool SGM::CompareSizes(size_t nSize1,size_t nSize2)
    {
    return nSize1==nSize2;
    }