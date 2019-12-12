#include "../include/jur_gui_segmentation/JURGUIMainWindow.h"


JURGUI::JURGUIMainWindow::JURGUIMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::JURGUIMainWindow)
{
    ui->setupUi(this);

    this->frm_2d = new JURGUI2DWindow();

    /// Disabling 3D button because we don't have anything for it yet.
    this->ui->pBtn_3d->setDisabled(true);

    connect( this->frm_2d, SIGNAL( backToMainFormSignal() ), this, SLOT( backHere() ) );
}


void JURGUI::JURGUIMainWindow::init(int _argc, char** _argv)
{
    this->mode = " ";

    /// It is just for Staring ROS Node along with running program
    this->frm_2d->init( _argc, _argv, true );
}


JURGUI::JURGUIMainWindow::~JURGUIMainWindow()
{
    // Below line causes error
    // Because it is done automatically
//    delete this->frm_2d;

    // Closing JUROpenCVNode
    this->frm_2d->closeRosNode();

    delete ui;
}


void JURGUI::JURGUIMainWindow::on_pBtn_2d_clicked(void)
{
    this->mode = "2d";

    this->frm_2d->show();
    JURGUI::JURGUIUtilities::displayInCenter( this->frm_2d );

    /// Initialize form 2D witout starting ROS Node again
    /// If it is first time we start ROS node in init
    /// if it is a return from back also we keep ROS node alive
    /// we don't need to start it again
    this->frm_2d->init( false );

    this->close();
}


void JURGUI::JURGUIMainWindow::on_pBtn_3d_clicked(void)
{
    this->mode = "3d";

    /// Add Required Code For Showing 3D Window.
    /// We don't have any thing for this mode so it is set as desiabled.

//    this->close();
}


void JURGUI::JURGUIMainWindow::backHere(void)
{
    if ( this->mode.compare( "2d" ) == 0 )
        this->frm_2d->close();

    // Add Code for 3D mode.

    this->show();
}


void JURGUI::JURGUIMainWindow::on_pBtn_exit_clicked(void)
{
    this->close();
}


void JURGUI::JURGUIMainWindow::on_actionAbout_triggered(void)
{
    Ui_about_dialog about;
    QDialog d;

    about.setupUi( &d );
    d.setWindowTitle( "About" );
    JURGUI::JURGUIUtilities::displayInCenter( &d );

    d.exec();
}
