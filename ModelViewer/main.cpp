#include <QApplication>
#include <QSettings>
#include <QSurfaceFormat>
#include <QFont>
#include <iostream>

#include "MainWindow.hpp"
#include "SGMGraphicsWidget.hpp"

#include "SGMEntityFunctions.h"
#include "SGMPrimitives.h"
#include "SGMTranslators.h"
#include "SGMComplex.h"

std::string GetExecutableName(const char *path_name)
{
    // strip off any path to get basename of file
    // find the last '\' or '/' characters
    const char * pos = strrchr(path_name,'/');
    if (pos == nullptr)
        {
        pos = strrchr(path_name,'\\');
        pos = pos == nullptr ? path_name : ++pos;
        }
    else
        {
        ++pos;
        }
    return std::string(pos);

}

#ifdef SGM_PRODUCT_VERSION
#define SGM_STR(s) #s
#define SGM_MAKE_VERSION_STRING(s) SGM_STR(s)
#else
#define SGM_MAKE_VERSION_STRING(s)
#endif

void PrintVersion()
    {
        std::cerr << "SGM version " << std::string(SGM_MAKE_VERSION_STRING(SGM_PRODUCT_VERSION)) << std::endl;
    }

// print short usage message

void PrintUsage(std::string sExecutableName)
    {
    std::cerr << "usage: " << std::endl;
    std::cerr << "    " << sExecutableName << " [-c] [FILE [FILE ...]]" << std::endl;
    }

// print longer help message

void PrintHelp(std::string sExecutableName)
    {
    PrintVersion();
    PrintUsage(sExecutableName);
    std::cerr << std::endl;
    std::cerr << "     -c --cover      read STL file and output an STL file with coordinate aligned planar gaps covered" << std::endl;
    }

// cover open surfaces on an STL file

void CoverSTL(int argc, char **argv)
    {
    if (argc != 4)
        {
        std::cerr << "error: unable to determine input and output file names" << std::endl;
        PrintUsage(GetExecutableName(argv[0]));
        exit(1);
        }

    SGMInternal::thing *pThing = SGM::CreateThing();
    SGM::Result rResult(pThing);
    std::string InputSTLFile(argv[2]);
    std::vector<SGM::Entity> aEntities;
    std::vector<std::string> aLog;
    SGM::TranslatorOptions Options;
    Options.m_bMerge = true;
    SGM::ReadFile(rResult, InputSTLFile, aEntities, aLog, Options);
    auto resultType = rResult.GetResult();
    if (resultType != SGM::ResultTypeOK)
        {
        std::cerr << rResult.Message() << std::endl;
        exit(1);
        }
    std::vector<SGM::Complex> *aComplexes = (std::vector<SGM::Complex> *) &aEntities;
    SGM::Complex ComplexID = aComplexes->front();
    if (aComplexes->size() > 1)
        {
        ComplexID = SGM::MergeComplexes(rResult, *aComplexes);
        }
    //SGM::Complex MergedComplexID=SGM::MergePoints(rResult, ComplexID, SGM_MIN_TOL);
    //SGM::DeleteEntity(rResult,ComplexID);
    //SGM::Complex CoverID = SGM::CoverComplex(rResult, MergedComplexID);
    //aParts.push_back(MergedComplexID);
    //aParts.push_back(CoverID);
    //SGM::MergeComplexes(rResult, aParts);
    //for (auto &iPart : aParts)
    //    {
    //    SGM::DeleteEntity(rResult,iPart); // delete original entity
    //    }
    SGM::Complex CoverID = SGM::CoverComplex(rResult, ComplexID);
    std::vector<SGM::Complex> aParts;
    aParts.push_back(ComplexID);
    aParts.push_back(CoverID);
    SGM::Complex AnswerID=SGM::MergeComplexes(rResult, aParts);
    SGM::DeleteEntity(rResult,CoverID); 
    SGM::DeleteEntity(rResult,ComplexID); 
    std::string OutputSTLFile(argv[3]);

    SGM::SaveSTL(rResult, OutputSTLFile, AnswerID, Options);
    }

// return false if we are not done processing and should go on to the QtApp

bool RunFromCommandLine(int argc, char **argv)
    {
    std::string sExecutableName = GetExecutableName(argv[0]);

    // did the user supply arguments?
    if (argc == 1)
        {
        return false;
        }

    // Are there options present.
    if (0 == strncmp(argv[1], "-", 1))
        {
        // Is it help or version?
        for (int i = 1; i < argc; ++i)
            {
            if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
                {
                PrintHelp(sExecutableName);
                exit(0);
                }
            else
                {
                if (strcmp(argv[i], "--version") == 0 || strcmp(argv[i], "-v") == 0)
                    {
                    PrintVersion();
                    exit(0);
                    }
                }
            }

        // Is the first one cover?
        if (strcmp(argv[1], "--cover") == 0 || strcmp(argv[1], "-c") == 0)
            {
            CoverSTL(argc, argv);
            }
        else
            {
            // unrecognized options
            PrintUsage(sExecutableName);
            }
        // we are done processing
        return true;
        }

    else
        {
        // there are no options
        // there are file arguments but we will let the QtApp take care of it
        return false;
        }
    }

int main(int argc, char **argv)
    {

    // Run from command line arguments.

    if (RunFromCommandLine(argc,argv))
        {
        // we are finished
        exit(0);
        }

    // Setup some defaults for the settings file
    QApplication::setOrganizationName("Sandia");
    QApplication::setApplicationName("SGM-Model-Viewer");
    QSettings::setDefaultFormat(QSettings::IniFormat);

    // The surface format should be set before we instantiate the
    // QApplication
    QSurfaceFormat::setDefaultFormat(SGMGraphicsWidget::default_format());

    QApplication app(argc, argv);

    // Setup the main window
    MainWindow mw;
    mw.show();

#if defined (__linux__)
    QFont new_font = app.font();
    new_font.setPointSize( 14 ); //your option
    //new_font.setWeight( int ** ); //your option
    app.setFont( new_font );     // Run the application
#endif

    return QApplication::exec();
    }

