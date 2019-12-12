#include <unistd.h>


#include <QSplashScreen>


#include "../include/jur_gui_segmentation/JURGUIMainWindow.h"
#include "../include/jur_gui_segmentation/JURGUIUtilities.hpp"


int main(int argc, char **argv)
{
    QApplication a(argc, argv);

    // Splash Window
    QSplashScreen* splash = new QSplashScreen;
    splash->setPixmap( QPixmap( ":/Images/images/img_splash.jpg" ) );
    splash->show();
    splash->repaint();
    JURGUI::JURGUIUtilities::displayInCenter( splash );

    // Main Window
    JURGUI::JURGUIMainWindow w;
    w.setWindowTitle( "Segmentation Tool" );

    // Sleep
    sleep(2);

    // Switch To MainWindow
    w.init( argc, argv );
    w.show();
    JURGUI::JURGUIUtilities::displayInCenter( &w );
    splash->finish( &w );

    // Delete Pointer
    delete splash;

    return a.exec();
}
