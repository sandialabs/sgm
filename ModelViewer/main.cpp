#include <QApplication>
#include <QSettings>
#include <QSurfaceFormat>
#include <QFont>

#include "MainWindow.hpp"
#include "SGMGraphicsWidget.hpp"

void RunFromCommandLine(int argc, char **argv)
    {
    std::string Arg1(argv[1]);
    if(Arg1=="-Cover" || Arg1=="-cover")
        {
        fprintf(stdout,"This is a test\n");
        }
    else if(Arg1=="-h" || Arg1=="-help" || Arg1=="-H" || Arg1=="-Help")
        {
        fprintf(stdout,"This is help\n");
        }
    }

int main(int argc, char **argv)
    {
    if(1<argc)
        {
        // Run from command line arguments.

        RunFromCommandLine(argc,argv);
        return 0;
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

