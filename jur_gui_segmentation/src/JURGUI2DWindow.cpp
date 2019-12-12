#include "../include/jur_gui_segmentation/JURGUI2DWindow.h"
#include "JURGUICVSWrapper.h"

JURGUI::JURGUI2DWindow::JURGUI2DWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::JURGUI2DWindow)
{
    ui->setupUi(this);

    connect( this, SIGNAL( imageIsLoaded(bool) ), this->ui->gV, SLOT( setDrawingFlag(bool) ) );

    connect( &this->ros_node, SIGNAL( imageReady (QImage) ), this, SLOT( onRecieveFinalImage(QImage) ) );
}


JURGUI::JURGUI2DWindow::~JURGUI2DWindow()
{
    delete ui;
}


void JURGUI::JURGUI2DWindow::init(int _argc, char** _argv, bool _ros_flag)
{
    /// Initializing Defualt Config for this from
    if ( !_ros_flag )
    {
        this->cur_ind      = -1;
        this->cur_poly_ind = -1;
        this->zoom_factor  = 1.0;

        this->color = Qt::red;
        this->pen_size = 1;

        this->ui->gV->setColor( this->color    );
        this->ui->gV->setSize ( this->pen_size );

        this->cur_file_list.clear();
        this->clearPolyVec();
        this->ui->lstV->clear();
        this->ui->lstPol->clear();

        this->cv_param1 = 0;
        this->cv_param2 = 0;

        this->ui->pTE_log->clear();
        this->intImages.clear();

        this->loadTrainData = false;
        this->loadedImg     = false;
        this->sanityCheck   = false;
        this->sampleImg     = false;

        this->opencv_method = "mean";  /// Default Method, mean stands for MeanShift

        this->setEnableEnv(false);

        Q_EMIT this->imageIsLoaded(false);

        this->updateDisplay();

        this->ui->comBox_sel_methods->addItem( "Rand Index (RI)" );
        this->ui->comBox_sel_methods->addItem( "Jaccard" );
        this->ui->comBox_sel_methods->addItem( "Mutual Information (MI)" );
        this->ui->comBox_sel_methods->setCurrentIndex(0);
        this->ui->comBox_sel_methods->setSizeAdjustPolicy( QComboBox::AdjustToContents );
    }
    /// This is just for starting ROSNode, not setting any config
    else
        if ( !this->ros_node.init( _argc, _argv ) )
        {
            ROS_ERROR("JURGUI::JURGUI2DWindow:: Node is not initialized.");

            return;
        }
}


void JURGUI::JURGUI2DWindow::on_pBtn_load_clicked(void)
{
    QStringList tmpcur_file_list;

    tmpcur_file_list = QFileDialog::getOpenFileNames( this,
                                             tr("Open Images"),
                                             QString(QDir::homePath() + "/Pictures"),
                                             tr("Image Files (*.png *.jpg *.jpeg *.bmp)") );

    // Keep Current dir of user for next openning, it is not neccessry

    if ( !tmpcur_file_list.empty() )
    {
        this->cur_file_list.clear();
        this->ui->lstV->clear();

        this->cur_file_list = tmpcur_file_list;
        tmpcur_file_list.clear();

        this->intImages.clear();

        this->cur_ind = 0;

        for ( int i = 0 ; i < this->cur_file_list.size() ; i++ )
        {
            QFileInfo fi( this->cur_file_list.at(i) );

            this->ui->lstV->addItem( fi.baseName() );

            if ( this->loadTrainData )
                this->intImages.append( this->readIntImages( QString ( fi.absolutePath() + "/lbl_" + fi.baseName() + ".txt") )  );
        }

        this->clearPolyVec();
        this->polys_vec.resize( this->cur_file_list.size() );

        this->loadedImg = true;
        this->setEnableEnv(true);

        this->ui->lstV->setCurrentRow( this->cur_ind );

        Q_EMIT this->imageIsLoaded( true );

        this->updateDisplay();

    }
    else if ( tmpcur_file_list.isEmpty() && this->cur_file_list.isEmpty() )
    {
        Q_EMIT this->imageIsLoaded(false);

        this->loadedImg = false;
        this->setEnableEnv(false);
    }
}


void JURGUI::JURGUI2DWindow::updateDisplay(bool _sel_list_poly)
{
    if ( this->cur_ind == -1 )
    {
        QPixmap img( ":/Images/images/img_no_image.jpg" );

        QGraphicsScene* scene = new QGraphicsScene;
        scene->addPixmap( img );

        this->ui->gV->setScene( scene );
        this->ui->gV->show();
        this->ui->gV->adjustFit();

        this->ui->gVROS->setScene( scene );
        this->ui->gVROS->show();
        this->ui->gVROS->fitInView( this->ui->gVROS->scene()->itemsBoundingRect(), Qt::IgnoreAspectRatio );
    }
    else if ( this->cur_ind > -1 )
    {
        QPixmap img( this->cur_file_list.at( this->cur_ind ) );

        QGraphicsScene* scene = new QGraphicsScene;
        scene->addPixmap( img );

        // Adjust Pen
        QPen cusPen;
        cusPen.setWidth( this->pen_size );
        cusPen.setCapStyle( Qt::RoundCap );
        cusPen.setJoinStyle( Qt::RoundJoin );
        cusPen.setColor( this->color );

        QPen selPolyPen;
        selPolyPen.setWidth( this->pen_size );
        selPolyPen.setStyle(Qt::DotLine);
        selPolyPen.setCapStyle( Qt::RoundCap );
        selPolyPen.setJoinStyle( Qt::RoundJoin );
        selPolyPen.setColor( this->color );

        // Updating Polygon List
        if ( !_sel_list_poly )
        {
            this->ui->lstPol->clear();

            for ( int i = 0 ; i < this->polys_vec.at( this->cur_ind ).size() ; i++ )
            {
                QListWidgetItem* item = new QListWidgetItem;
                this->ui->lstPol->addItem( item );

                QLabel* label = new QLabel(QString( "Polygon No." ) + QString::number( i+1 ));

                QPushButton* but = new QPushButton();
                but->setIcon( QIcon( ":/Icons/images/ico_delete.png" ) );
                but->setIconSize( QSize(32,32) );
                but->setMaximumSize( 50, 50 );
                but->setFlat( true );

                QHBoxLayout *layout= new QHBoxLayout();
                layout->addWidget( label );
                layout->addWidget( but );

                QWidget* widget = new QWidget();
                widget->setLayout( layout );

                item->setSizeHint( widget->sizeHint() );

                this->ui->lstPol->setItemWidget( item, widget );

                connect( but, SIGNAL( clicked() ), this, SLOT( onDelBtnClicked() ) );
            }
        }

        double rad = 5;
        int num = 1;
        for ( int i = 0 ; i < this->polys_vec.at( this->cur_ind ).size() ; i++ )
        {
            QPolygonF tmp_polyf = this->polys_vec.at(this->cur_ind).at(i);

            // Drawing lines
            for ( int j = 0 ; j < tmp_polyf.size() ; j++ )
            {
                if ( j + 1 >= tmp_polyf.size() )
                {
                    QLineF tmp_linef( tmp_polyf.at(j), tmp_polyf.at(0) );

                    if ( i == this->cur_poly_ind )
                        scene->addLine( tmp_linef, selPolyPen );
                    else
                        scene->addLine( tmp_linef, cusPen );

                    break;
                }
                else
                {
                    QLineF tmp_linef( tmp_polyf.at(j), tmp_polyf.at(j+1) );

                    if ( i == this->cur_poly_ind )
                        scene->addLine( tmp_linef, selPolyPen );
                    else
                        scene->addLine( tmp_linef, cusPen );
                }
            }

            // Draw Vertexes
            for ( int j = 0 ;  j < tmp_polyf.size(); j++ )
            {
                int x = tmp_polyf.at(j).x();
                int y = tmp_polyf.at(j).y();

                if ( i == this->cur_poly_ind )
                    scene->addEllipse( x-rad, y-rad, rad*2.0, rad*2.0, selPolyPen );
                else
                    scene->addEllipse( x-rad, y-rad, rad*2.0, rad*2.0, cusPen );
            }

            // Drawing Number of Polygons
            QPointF tmp_rec = this->calcCenterOfPoly( tmp_polyf );

            QFont tmpFont( "Courier New" );
            tmpFont.setBold( true );
            tmpFont.setPixelSize( this->pen_size * 8 );

            QGraphicsTextItem* io = new QGraphicsTextItem();
            io->setParent( this->ui->gV->scene() );
            io->setFont( tmpFont );
            io->setPos( tmp_rec );
            io->setPlainText( QString::number( num ) );

            if ( i == this->cur_poly_ind )
                io->setDefaultTextColor( this->color );
            else
                io->setDefaultTextColor( this->color );

            scene->addItem( io );

            num++;
        }

        this->ui->gV->setScene( scene );
        this->ui->gV->show();
    }
}



/// Updating Mouse position in statuebar
void JURGUI::JURGUI2DWindow::on_gV_mousePosUpdate(QPointF _pos)
{
    if ( this->cur_ind != -1 )
        /// Mouse is not in Scence
        if ( int( _pos.x() ) != -1 && int( _pos.y() ) != -1 )
            this->statusBar()->showMessage( "(" + QString::number( int( _pos.x() ) ) + "," + QString::number( int( _pos.y() ) ) + ")" );
        /// Mouse Leave
        else
            this->statusBar()->showMessage( QString(" ") );
    /// Image is not loaded
    else
        this->statusBar()->showMessage( QString(" ") );
}


// Next Button
void JURGUI::JURGUI2DWindow::on_pBtn_next_clicked(void)
{
    if ( this->cur_ind != this->cur_file_list.size() - 1 )
        this->cur_ind++;
    else
        this->cur_ind = 0;

    this->ui->lstV->setCurrentRow( this->cur_ind );

    this->ui->lstPol->clear();

    this->cur_poly_ind = -1;

    this->updateDisplay();

    this->ui->gV->adjustFit();

    this->ui->gVROS->scene()->clear();
}


// Previous Button
void JURGUI::JURGUI2DWindow::on_pBtn_previous_clicked(void)
{
    if ( this->cur_ind != 0 )
        this->cur_ind--;
    else
        this->cur_ind = this->cur_file_list.size() - 1;

    this->ui->lstV->setCurrentRow( this->cur_ind );

    this->ui->lstPol->clear();

    this->cur_poly_ind = -1;

    this->updateDisplay();

    this->ui->gV->adjustFit();

    this->ui->gVROS->scene()->clear();
}


// Click in images list
void JURGUI::JURGUI2DWindow::on_lstV_currentRowChanged(int _row)
{
    if ( this->cur_ind == _row )
        return;

    this->cur_ind      = _row;

    this->ui->lstPol->clear();

    this->cur_poly_ind = -1;

    this->updateDisplay();

    this->ui->gV->adjustFit();

    this->ui->gVROS->scene()->clear();
}


/// Set Enable/Disable for Widgets in Ui
void JURGUI::JURGUI2DWindow::setEnableEnv(bool _flag)
{
    if ( this->loadedImg )
    {
        this->ui->pBtn_extract->setEnabled  ( !this->ui->chBox_load_train->isChecked() );
        this->ui->pBtn_color->setEnabled    ( !this->ui->chBox_load_train->isChecked() );
        this->ui->sBox_size->setEnabled     ( !this->ui->chBox_load_train->isChecked() );
        this->ui->pBtn_train_seg->setEnabled(  this->ui->chBox_load_train->isChecked() );
        this->ui->chBox_sample->setEnabled  (  this->ui->chBox_load_train->isChecked() );
        this->ui->pBtn_expr->setEnabled     (  this->ui->chBox_load_train->isChecked() );
    }
    else
    {
        this->ui->pBtn_extract->setEnabled  ( _flag );
        this->ui->pBtn_color->setEnabled    ( _flag );
        this->ui->sBox_size->setEnabled     ( _flag );
        this->ui->pBtn_train_seg->setEnabled( _flag );
        this->ui->chBox_sample->setEnabled  ( _flag );
        this->ui->pBtn_expr->setEnabled     ( _flag );
    }

    this->ui->pBtn_next->setEnabled         ( _flag );
    this->ui->pBtn_previous->setEnabled     ( _flag );
    this->ui->pBtn_zoom_in->setEnabled      ( _flag );
    this->ui->pBtn_zoom_out->setEnabled     ( _flag );
    this->ui->rdBtnMean->setEnabled         ( _flag );
    this->ui->rdBtnWatershed->setEnabled    ( _flag );
    this->ui->pBtn_req->setEnabled          ( _flag );
    this->ui->pBtn_fit->setEnabled          ( _flag );
    this->ui->pBtn_eval->setEnabled         ( _flag );
    this->ui->pBtn_setting->setEnabled      ( _flag );
    this->ui->comBox_sel_methods->setEnabled( _flag );
    this->ui->chBox_sanity->setEnabled      ( _flag );
}


// Zoom-in
void JURGUI::JURGUI2DWindow::on_pBtn_zoom_in_clicked(void)
{
    this->zoom_factor = 2.0;

    this->ui->gV->scale( this->zoom_factor, this->zoom_factor );
}


// Zoom-out
void JURGUI::JURGUI2DWindow::on_pBtn_zoom_out_clicked(void)
{
    this->zoom_factor = 0.5;

    this->ui->gV->scale( this->zoom_factor, this->zoom_factor );
}


// Fit Button
void JURGUI::JURGUI2DWindow::on_pBtn_fit_clicked(void)
{
    this->zoom_factor = 1.0;

    this->ui->gV->adjustFit();
}


// Color selection
void JURGUI::JURGUI2DWindow::on_pBtn_color_clicked(void)
{
    this->color = QColorDialog::getColor();

    this->ui->gV->setColor( this->color );
    /// Just Changing Color
    /// but we need to redraw all things
    this->updateDisplay();
}


void JURGUI::JURGUI2DWindow::on_sBox_size_valueChanged(int _val)
{
    this->pen_size = _val;

    this->ui->gV->setSize( this->pen_size );

    /// Just Change pen Size
    /// But we need to redraw all thing
    this->updateDisplay();
}


/// Clearing all polygon for all images
void JURGUI::JURGUI2DWindow::clearPolyVec(void)
{
    for ( int i = 0 ; i < this->polys_vec.size() ; i++ )
    {
        QVector<QPolygonF> tmp_vec_poly = this->polys_vec.at(i);
        tmp_vec_poly.clear();
    }

    this->polys_vec.clear();
}


/// Calc Center of polygon
/// The formula is from Wikipedia
QPointF JURGUI::JURGUI2DWindow::calcCenterOfPoly(QPolygonF _poly)
{
    QPointF tmp_out;

    float area = 0.0;

    for ( int i = 0 ; i < _poly.size() - 1  ; i++ )
        area = area + ( _poly.at(i).x() * _poly.at(i+1).y() ) - ( _poly.at(i+1).x() * _poly.at(i).y() );

    area = 0.5 * area;

    float tmp_val = 0.0;
    for ( int i = 0 ; i < _poly.size() - 1 ; i++ )
        tmp_val = tmp_val + ( ( _poly.at(i).x() + _poly.at(i+1).x() ) * ( _poly.at(i).x() * _poly.at(i+1).y() - _poly.at(i+1).x() * _poly.at(i).y() ) );

    tmp_out.setX( ( 1 / ( 6 * area ) ) * tmp_val );

    tmp_val = 0.0;
    for ( int i = 0 ; i < _poly.size() - 1 ; i++ )
        tmp_val = tmp_val + ( ( _poly.at(i).y() + _poly.at(i+1).y() ) * ( _poly.at(i).x() * _poly.at(i+1).y() - _poly.at(i+1).x() * _poly.at(i).y() ) );

    tmp_out.setY( ( 1 / ( 6 * area ) ) * tmp_val );

    return tmp_out;
}


cv::Mat JURGUI::JURGUI2DWindow::getIntImageFromHandySeg(void)
{
    QImage img( this->cur_file_list.at( this->cur_ind ) );

    cv::Mat res( img.size().height(), img.size().width(), CV_8U );

    QVector<QPolygonF> tmp_vec_poly = this->polys_vec.at( this->cur_ind );

    for ( int i = 0 ; i < tmp_vec_poly.size() ; i++ )
    {
        QPolygonF tmp_polyf = tmp_vec_poly.at(i);

        QPainterPath path;
        path.addPolygon( tmp_polyf );

        QPixmap img( this->cur_file_list.at(this->cur_ind) );
        QImage source = img.toImage();

        QImage cutout( source.size(), QImage::Format_ARGB32_Premultiplied );
        cutout.fill( Qt::black );
        QPainter p( &cutout );
        p.setClipPath( path );
        p.drawImage( 0, 0, source );
        p.end();

        cv::Mat tmp_cv4 = JURGUITOOL::qImageToCvMat( cutout.convertToFormat( QImage::Format_RGB32 ), true ) ;
        cv::Mat tmp_cv3;
        cvtColor( tmp_cv4, tmp_cv3, CV_BGRA2BGR );

        for ( int k = 0 ; k < tmp_cv3.rows ; k++ )
            for ( int j = 0 ; j < tmp_cv3.cols ; j++ )
            {
                cv::Vec3b tmp_vec = tmp_cv3.at<cv::Vec3b>(k,j);

                if ( tmp_vec[0] != 0 && tmp_vec[1] != 0 && tmp_vec[2] != 0 )
                    res.at<uchar>(k,j) = i+1;
            }
    }

    return res;
}


cv::Mat JURGUI::JURGUI2DWindow::getIntImageFromOpenCV(bool _flag)
{
    cv::Mat tmp_cv3;

    if ( !_flag )
    {
        cv::Mat tmp_cv4 = JURGUITOOL::qImageToCvMat( this->openCV_img.convertToFormat( QImage::Format_RGB32 ), true ) ;
        cv::cvtColor( tmp_cv4, tmp_cv3, CV_BGRA2BGR );

    }
    else
    {
        tmp_cv3 = this->mean_res;
    }

    std::vector<cv::Vec3b> tmpCols;
    cv::Vec3b curColor;
    int num = 1;

    cv::Mat res( tmp_cv3.rows, tmp_cv3.cols, CV_8U);

    for ( int i = 0 ; i < tmp_cv3.rows ; i++ )
        for ( int j = 0 ; j < tmp_cv3.cols ; j++ )
        {
            curColor = tmp_cv3.at<cv::Vec3b>(i,j);

            /// Check Color for avoiding repeat
            if ( checkRepColor(tmpCols,curColor) )
                continue;
            else
            {
                tmpCols.push_back(curColor);

                if ( curColor[0] == 0 && curColor[1] == 0 && curColor[2] == 0 )
                    continue;
            }

            /// Extract in image 1 with current color
            cv::Mat exImg1 = extractWithColor(tmp_cv3,curColor);

            for ( int k = 0 ; k < exImg1.rows ; k++ )
                for ( int l = 0 ; l < exImg1.cols ; l++ )
                {
                    cv::Vec3b tmp_vec = exImg1.at<cv::Vec3b>(k,l);

                    if ( tmp_vec[0] != 0 && tmp_vec[1] != 0 && tmp_vec[2] != 0 )
                        res.at<uchar>(k,l) = num;
                }

            num++;
        }

    return res;
}


bool JURGUI::JURGUI2DWindow::checkRepColor(std::vector<cv::Vec3b> _vec, cv::Vec3b _col)
{
    for ( int i = 0 ; i < _vec.size() ; i++ )
        if ( _vec[i] == _col )
            return true;

    return false;
}


cv::Mat JURGUI::JURGUI2DWindow::extractWithColor(cv::Mat _image, cv::Vec3b _col)
{
    cv::Mat image = _image.clone();
    cv::Vec3b tmp_zero_val;
    tmp_zero_val[0] = 0;
    tmp_zero_val[1] = 0;
    tmp_zero_val[2] = 0;

    for ( int i = 0 ; i < image.rows ; i++ )
        for ( int j = 0 ; j < image.cols ; j++ )
            if ( image.at<cv::Vec3b>(i,j) != _col )
                image.at<cv::Vec3b>(i,j) = tmp_zero_val;

    return image;
}


void JURGUI::JURGUI2DWindow::setLog(char _ch, QString _text)
{
    if ( _ch == 'i' )
    {
        QPalette palette;
        palette.setColor( QPalette::Text, Qt::green );

        this->ui->pTE_log->setPalette( palette );
        this->ui->pTE_log->appendPlainText( QString("<INFO> " + _text) );
    }
    else if ( _ch == 'e' )
    {
        QPalette palette;
        palette.setColor( QPalette::Text, Qt::red );

        this->ui->pTE_log->setPalette( palette );
        this->ui->pTE_log->appendPlainText( QString("<ERROR> " + _text) );
    }
}


cv::Mat JURGUI::JURGUI2DWindow::readIntImages(QString adrs)
{
    QImage img( this->cur_file_list.at( this->cur_ind ) );

    cv::Mat res( img.size().height(), img.size().width(), CV_8U );

    QFile file( adrs );

    if ( !file.exists() )
    {
        this->setLog( 'i', QString( adrs + "is not exist!" ) );

        return res;
    }

    file.open( QIODevice::ReadOnly );

    QTextStream textStream( &file );

    int row = 0;

    while ( !textStream.atEnd() )
    {
        QString     line = textStream.readLine();

        QStringList vals = line.split(",");

        for ( int col = 0 ; col < res.cols ; col++ )
            res.at<uchar>(row,col) = vals.at(col).toInt();

        row++;
    }

    file.close();

    return res;
}


int JURGUI::JURGUI2DWindow::genRandNum(int _low, int _high)
{
    return qrand() % ((_high + 1) - _low) + _low;
}


void JURGUI::JURGUI2DWindow::startExpr(int _in)
{
    QPolygonF res_points;

    this->setLog( 'i', QString("Expriment is starting!") );
    this->openCLConfig();

    QFileInfo fi( this->cur_file_list.at( this->cur_ind) );
    cv::Mat source = cv::imread( fi.absoluteFilePath().toStdString().c_str() );
    cv::Mat source_conv;
    cvtColor( source , source_conv , CV_BGR2BGRA );

    cv::Mat res_mean, ref, cors;

    cv::ocl::oclMat ocl_image;
    ocl_image.upload( source_conv );

    double resRi, resJacc;
    QVector<double> resMi;

    QFile file( this->experVal.name );
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    for ( int i = this->experVal.sstart; i <= this->experVal.send; i = i + this->experVal.sstep )
    //for ( int i = 5 ; i <= 10 ; i += 5 )
    {
        cv::ocl::meanShiftSegmentation( ocl_image, res_mean ,  i, this->experVal.cbw, this->experVal.size );
        this->mean_res = res_mean;

        //this->openCV_img = JURGUIFormatConvertROSOpenCV::cVMatToQImage( res_mean );

        ref  = this->intImages.at( this->cur_ind );

        cors = this->getIntImageFromOpenCV(true);

        if ( this->experVal.ri )
        {
            resRi = JUREVAL::JURGUIEvalMethods::orginalRI(ref,cors);
            out<<this->experVal.cbw<<";"<<i<<";"<<resRi<<"\n";
            res_points.push_back( QPoint(i, resRi));
        }
        else if ( this->experVal.jacc )
        {
            resJacc = JUREVAL::JURGUIEvalMethods::jaccard(ref,cors);
            out<<this->experVal.cbw<<";"<<i<<";"<<resJacc<<"\n";
            res_points.push_back( QPoint(i, resJacc));
        }
        else if ( this->experVal.mi )
        {
            resMi = JUREVAL::JURGUIEvalMethods::MI(ref,cors);
            out<<this->experVal.cbw<<";"<<i<<";"<<resMi.at(0)<<"\n";
        }
    }

    file.close();

    this->plotWidg.drawRes( res_points );

    this->setLog( 'i', QString("Expriment is finished") );
}


void JURGUI::JURGUI2DWindow::openCLConfig(void)
{
    /// Device Info
    cv::ocl::DevicesInfo dev_info;
    cv::ocl::getOpenCLDevices( dev_info );

    /// Platform Info
    cv::ocl::PlatformsInfo plat_info;
    cv::ocl::getOpenCLPlatforms( plat_info );

    cv::ocl::setDevice( dev_info[0] );
}


void JURGUI::JURGUI2DWindow::setMeanSColBW(int _val)
{
    this->cv_param1 = _val;
}


void JURGUI::JURGUI2DWindow::setMeanSSpaBW(int _val)
{
    this->cv_param2 = _val;
}


void JURGUI::JURGUI2DWindow::setExprName(QString _name)
{
    this->experVal.name = _name;
}


void JURGUI::JURGUI2DWindow::setExprsbw(int _val)
{
    this->experVal.sbw = _val;
}


void JURGUI::JURGUI2DWindow::setExprcbw(int _val)
{
    this->experVal.cbw = _val;
}


void JURGUI::JURGUI2DWindow::setExprcstart(int _val)
{
    this->experVal.sstart = _val;
}


void JURGUI::JURGUI2DWindow::setExprcend(int _val)
{
    this->experVal.send = _val;
}


void JURGUI::JURGUI2DWindow::setExprcstep(int _val)
{
    this->experVal.sstep = _val;
}


void JURGUI::JURGUI2DWindow::setExprsize(int _val)
{
    this->experVal.size = _val;
}


void JURGUI::JURGUI2DWindow::setExprri(bool _state)
{
    this->experVal.ri = _state;
}


void JURGUI::JURGUI2DWindow::setExprmi(bool _state)
{
    this->experVal.mi = _state;
}


void JURGUI::JURGUI2DWindow::setExprjacc(bool _state)
{
    this->experVal.jacc = _state;
}


/// Closing JUROpenCVNode
void JURGUI::JURGUI2DWindow::closeRosNode(void)
{
    this->ros_node.sendMsg( QString("shutdown"), QImage( 1, 1, QImage::Format_Mono ) );
}


void JURGUI::JURGUI2DWindow::on_pBtn_back_clicked(void)
{
    /// Reset All Things ( It Is Done In Init )
    Q_EMIT this->backToMainFormSignal();
}


/// Extract all Segmented part
void JURGUI::JURGUI2DWindow::on_pBtn_extract_clicked(void)
{
    QVector<QPolygonF> tmp_vec_poly = this->polys_vec.at( this->cur_ind );

    QPixmap img2( this->cur_file_list.at(this->cur_ind) );
    QImage source2 = img2.toImage();
    cv::Mat final( source2.height(), source2.width(), CV_16U, cv::Scalar(0) );

    std::string pathTemp;

    for ( int i = 0 ; i < tmp_vec_poly.size() ; i++ )
    {
        QPolygonF tmp_polyf = tmp_vec_poly.at(i);

        QPainterPath path;
        path.addPolygon( tmp_polyf );

        QPixmap img( this->cur_file_list.at(this->cur_ind) );
        QImage source = img.toImage();

        QImage cutout( source.size(), QImage::Format_ARGB32_Premultiplied );
        cutout.fill( Qt::black );
        QPainter p( &cutout );
        p.setClipPath( path );
        p.drawImage( 0, 0, source );
        p.end();

        QFileInfo fi( this->cur_file_list.at(this->cur_ind) );

        cutout.save( QString( QDir::homePath() + "/Pictures/" + fi.baseName() + "_" + QString::number(i) + ".jpg" ) );

        pathTemp = QString( QDir::homePath() + "/Pictures/" + fi.baseName() + "_" + QString::number(i) ).toStdString();

        for ( int k = 0 ; k < final.rows ; ++k )
            for ( int l = 0 ; l < final.cols ; ++l )
                if ( source.pixel(k,l) != Qt::black )
                    final.at<ushort>(k,l) = i+1;
    }

    cv::FileStorage test;
    cv::FileStorage write( pathTemp, cv::FileStorage::WRITE);
    write<<"img"<<final;
    final.release();
}


void JURGUI::JURGUI2DWindow::on_pBtn_eval_clicked(void)
{
    cv::Mat ref, cors;
    double res1, res2;

    if ( !this->loadTrainData )
        // Int image from our image
        ref = this->getIntImageFromHandySeg();
    else
        ref = this->intImages.at( this->cur_ind );

    if ( !this->sanityCheck && !this->sampleImg )
        cors = this->getIntImageFromOpenCV();
    else if ( this->sanityCheck )
    {
        this->setLog( 'i', QString( "It is a Sanity Check" ) );

        cors = ref;
    }
    else if ( !this->sanityCheck && this->sampleImg )
    {
        QString fileName = QFileDialog::getOpenFileName( this,
                                                 tr("Open paired sample image"),
                                                 QString(QDir::homePath() + "/Pictures"),
                                                 tr("Image Files (*.png *.jpg *.jpeg *.bmp)") );

        if ( fileName.isEmpty() )
        {
            this->setLog( 'i', QString( "You don't select any image!" ) );

            return;
        }

        QFileInfo fi( fileName );

        cors = this->readIntImages( QString ( fi.absolutePath() + "/lbl_" + fi.baseName() + ".txt") );

        this->onRecieveFinalImage( QImage( fileName ) );
    }
    else
    {
        return;
    }

    this->setLog( 'i', QString( "Current Evaluation Method is: " + this->ui->comBox_sel_methods->currentText() ) );

    switch ( this->ui->comBox_sel_methods->currentIndex() )
    {
        case 0: // RI
            res1 = JUREVAL::JURGUIEvalMethods::orginalRI(ref,cors);
            this->setLog( 'i', QString( "Orginal RI: " + QString::number( res1 ) ) );

            //res2 = JUREVAL::JURGUIEvalMethods::secRI(ref,cors);
            //this->setLog( 'i', QString( "Second RI: " + QString::number( res2 ) ) );
        break;

        case 1: // Jaccard
            res1 = JUREVAL::JURGUIEvalMethods::jaccard(ref,cors);
            this->setLog( 'i', QString( "Jaccard: " + QString::number( res1 ) ) );
        break;

        case 2: // MI
            QVector<double> resM = JUREVAL::JURGUIEvalMethods::MI(ref,cors);
            this->setLog( 'i', QString( "Mutual Information (MI): " + QString::number( resM.at(0) ) ) );
            this->setLog( 'i', QString( "Anteropy ref: " + QString::number( resM.at(1) ) ) );
            this->setLog( 'i', QString( "Anteropy cors: " + QString::number( resM.at(2) ) ) );
        break;
    }
}


/// Send Image to JUROpenCVNode for applying segmentation
void JURGUI::JURGUI2DWindow::on_pBtn_req_clicked()
{    
    QImage img( this->cur_file_list.at( this->cur_ind ) );

    this->setLog( 'i' , QString(" Method: " + this->opencv_method) );

    this->ros_node.sendMsg( this->opencv_method, img, this->cv_param1, this->cv_param2 );
}


void JURGUI::JURGUI2DWindow::on_pBtn_train_seg_clicked(void)
{
    cv::Mat img = this->intImages.at( this->cur_ind );
    QImage resImg(img.cols, img.rows, QImage::Format_RGB888);

    double min, max;
    cv::Point min_loc, max_loc;

    cv::minMaxLoc(img, &min, &max, &min_loc, &max_loc);

    // Random Nums
    QTime time = QTime::currentTime();
    qsrand((uint)time.msec());

    QVector<QRgb> colors;
    QRgb col;
    for ( int i = 0 ; i < int(max) ; i++ )
    {
        do
        {
            col = qRgb(  this->genRandNum(0,255), this->genRandNum(0,255), this->genRandNum(0,255) );
        }while( !this->distColor(colors,col) );

        colors.append( col );
    }

    for ( int i = 0 ; i < img.cols ; i++ )
        for ( int j = 0 ; j < img.rows ; j++ )
        {
            QRgb col = colors.at( (int)img.at<uchar>(j,i) - 1 );
            resImg.setPixel( i,j, col);
        }

    QPixmap piximg = QPixmap::fromImage( resImg );

    QGraphicsScene* scene = new QGraphicsScene;
    scene->addPixmap( piximg );

    this->ui->gV->setScene( scene );
    this->ui->gV->show();
    this->ui->gV->adjustFit();
}


bool JURGUI::JURGUI2DWindow::distColor(QVector<QRgb> _list, QRgb _col)
{
    double min = DBL_MAX, res;
    for ( int i = 0 ; i < _list.size() ; i++ )
    {
        res = sqrt( pow( qGreen(_col) - qGreen(_list.at(i)), 2 ) + pow( qRed(_col) - qRed(_list.at(i)), 2 ) + pow( qBlue(_col) - qBlue(_list.at(i)) , 2 ) );
        if ( res < min )
            min = res;
    }

    if ( min > 100.0 )
        return true;
    else
        return false;
}

void JURGUI::JURGUI2DWindow::on_pBtn_expr_clicked(void)
{
    Ui_expriment_dialog expr;
    QDialog d;

    expr.setupUi( &d );
    d.setWindowTitle( "Setting" );
    JURGUI::JURGUIUtilities::displayInCenter( &d );

    connect ( expr.txtname       , SIGNAL( textChanged (QString) ), this, SLOT( setExprName  (QString) ) );
    connect ( expr.chBox_jacc    , SIGNAL( clicked     (bool)    ), this, SLOT( setExprjacc  (bool)    ) );
    connect ( expr.chBox_mi      , SIGNAL( clicked     (bool)    ), this, SLOT( setExprmi    (bool)    ) );
    connect ( expr.chBox_ri      , SIGNAL( clicked     (bool)    ), this, SLOT( setExprri    (bool)    ) );
    connect ( expr.sBox_end_col  , SIGNAL( valueChanged(int)     ), this, SLOT( setExprcend  (int)     ) );
    connect ( expr.sBox_mean_col , SIGNAL( valueChanged(int)     ), this, SLOT( setExprsize  (int)     ) );
    connect ( expr.sBox_sbw      , SIGNAL( valueChanged(int)     ), this, SLOT( setExprsbw   (int)     ) );
    connect ( expr.sBox_start_col, SIGNAL( valueChanged(int)     ), this, SLOT( setExprcstart(int)     ) );
    connect ( expr.sBox_step_col , SIGNAL( valueChanged(int)     ), this, SLOT( setExprcstep (int)     ) );

    connect ( &d, SIGNAL( finished(int) ), this, SLOT( startExpr(int) ) );

    d.exec();
}


/// Select MeanShift
void JURGUI::JURGUI2DWindow::on_rdBtnMean_clicked(bool)
{
    this->opencv_method = "mean";

    this->ui->pBtn_setting->setEnabled( true );
}


/// Select Watershed
void JURGUI::JURGUI2DWindow::on_rdBtnWatershed_clicked(bool)
{
    this->opencv_method = "watershed";

    this->ui->pBtn_setting->setEnabled( false );
}


/// Meanshift Setting Window
void JURGUI::JURGUI2DWindow::on_pBtn_setting_clicked(void)
{
    Ui_mean_set_dialog mean_setting;
    QDialog d;

    mean_setting.setupUi( &d );
    d.setWindowTitle( "Mean Shift Setting");
    JURGUI::JURGUIUtilities::displayInCenter( &d );

    mean_setting.sB_color->setValue  ( this->cv_param1 );
    mean_setting.sB_spatial->setValue( this->cv_param2 );

    connect ( mean_setting.sB_color  , SIGNAL( valueChanged(int) ), this, SLOT( setMeanSColBW(int) ) );
    connect ( mean_setting.sB_spatial, SIGNAL( valueChanged(int) ), this, SLOT( setMeanSSpaBW(int) ) );

    d.exec();
}


void JURGUI::JURGUI2DWindow::on_pBtn_clear_clicked()
{
    this->ui->pTE_log->clear();

    //QPolygonF test;
    //test<<QPointF(1,2)<<QPointF(3,4)<<QPointF(5,6);

    //this->plotWidg.drawRes( test );
    cv::Mat test(10,10,CV_32F);
    for ( int i = 0 ; i < test.rows ; i++ )
        for ( int j = 0 ; j < test.cols ; j++ )
            test.at<float>(i,j) = i*j;

    //std::cerr<<row<<"    "<<col<<std::endl;
    std::cerr<<"\\\\\\\\\\\\\\\\\\\\\\ ORGINAL \\\\\\\\\\\\\\\\\\\\\\\\\\"<<std::endl;
    std::cerr<<test<<std::endl;
    std::cerr<<"/////////////////////////////////////////////////////\n";

    JURTOOL::JURGUICVSWrapper::writeCSV("/home/vahid/kuni5.txt", test);
    cv::Mat getmat;
    int row, col;
    JURTOOL::JURGUICVSWrapper::readCSV("/home/vahid/kuni5.txt", getmat, row, col);

    std::cerr<<"\\\\\\\\\\\\\\\\\\\\\\ hhhh \\\\\\\\\\\\\\\\\\\\\\\\\\"<<std::endl;
    std::cerr<<row<<"  "<<col<<std::endl;
    std::cerr<<getmat<<std::endl;
    std::cerr<<"////////////////////////////////////////////////////";
}


/// About window
void JURGUI::JURGUI2DWindow::on_actionAbout_triggered()
{
    Ui_about_dialog about;
    QDialog d;

    about.setupUi( &d );
    d.setWindowTitle( "About" );
    JURGUI::JURGUIUtilities::displayInCenter( &d );

    d.exec();
}


/// Drawing polygon is finished
void JURGUI::JURGUI2DWindow::on_gV_polyIsReady(QPolygonF _poly)
{
    /// This Remove is because first and last vertexs are same
    _poly.remove( _poly.size() - 1 );

    QVector<QPolygonF> tmp_vec_poly = this->polys_vec.at( this->cur_ind );
    tmp_vec_poly.push_back( _poly );

    this->polys_vec.replace( this->cur_ind, tmp_vec_poly );

    this->ui->lstPol->clear();

    /// It is possible just to add this new polygn to all staffs
    /// but it not important to redraw or just adding new one
    this->updateDisplay();
}


void JURGUI::JURGUI2DWindow::on_lstPol_currentRowChanged(int _ind)
{
    if ( this->cur_poly_ind == _ind )
        return;

    this->cur_poly_ind = _ind;

    /// It is possible just to change color of selected polygon
    /// but it not important to redraw or just adding new one
    this->updateDisplay(true);
}


void JURGUI::JURGUI2DWindow::on_chBox_load_train_clicked(bool _flag)
{
    if ( _flag )
    {
        this->setLog( 'i', QString("You set load train data as True") );

        this->loadTrainData = _flag;
    }
    else
    {
        this->setLog( 'i', QString("You set load train data as False") );

        this->loadTrainData = _flag;
        this->sampleImg = _flag;

        this->ui->chBox_sample->setChecked( _flag );
    }

    if ( this->loadedImg )
    {
        this->ui->pBtn_extract->setEnabled  ( !_flag );
        this->ui->sBox_size->setEnabled     ( !_flag );
        this->ui->pBtn_color->setEnabled    ( !_flag );
        this->ui->pBtn_train_seg->setEnabled(  _flag );
        this->ui->chBox_sample->setEnabled  (  _flag );
    }
}


void JURGUI::JURGUI2DWindow::on_chBox_sanity_clicked(bool _flag)
{
    this->sanityCheck = _flag;
}


void JURGUI::JURGUI2DWindow::on_chBox_sample_clicked(bool _flag)
{
    this->sampleImg = _flag;
}


void JURGUI::JURGUI2DWindow::onDelBtnClicked()
{
    int tmpIndex = -1;

    QListWidgetItem *runningItem;
    QWidget *clickedWidget = qobject_cast<QWidget *>(sender()->parent());

    for ( int i = 0; i < this->ui->lstPol->count() ; i++ )
    {
        runningItem = this->ui->lstPol->item(i);
        QWidget *widget = this->ui->lstPol->itemWidget(runningItem);

        if ( clickedWidget == widget )
        {
            tmpIndex = i;
            break;
        }
    }

    if ( tmpIndex != -1 )
    {
        this->ui->lstPol->removeItemWidget(runningItem);

        QVector<QPolygonF> tmp_vec_poly = this->polys_vec.at( this->cur_ind );
        tmp_vec_poly.remove(tmpIndex);
        this->polys_vec.replace( this->cur_ind, tmp_vec_poly );
    }

    this->updateDisplay();
}


void JURGUI::JURGUI2DWindow::on_gV_mouseSingleClick(QPointF _cur_click_pos)
{
    if ( QGraphicsItem *item = this->ui->gV->scene()->itemAt( _cur_click_pos ) )
    {
        if ( item->type() == QGraphicsEllipseItem::Type )
        {
            qreal minDist = std::numeric_limits<float>::max();

            int tmpI = 0, tmpJ = 0;
            for ( int i = 0 ; i < this->polys_vec.at(this->cur_ind).size() ; i++ )
            {
                QPolygonF tmp_polyf = this->polys_vec.at(this->cur_ind).at(i);

                for ( int j = 0 ; j < tmp_polyf.size() ; j++ )
                {
                    int x = tmp_polyf.at(j).x();
                    int y = tmp_polyf.at(j).y();

                    qreal tmp_dist = qSqrt( qPow( _cur_click_pos.x() - x , 2 ) + qPow( _cur_click_pos.y() - y , 2 ) );

                    if (  tmp_dist < minDist )
                    {
                        minDist = tmp_dist;
                        tmpI = i;
                        tmpJ = j;
                    }
                }
            }

            this->ind_first = tmpI;
            this->ind_sec   = tmpJ;

            this->ui->gV->setEditFlag( true );
        }
    }
}


void JURGUI::JURGUI2DWindow::on_gV_newPoseVert(QPointF _new_pos)
{
    QVector<QPolygonF> tmp_vec_poly = this->polys_vec.at( this->cur_ind );

    QPolygonF tmp_polyf = tmp_vec_poly.at( this->ind_first );

    tmp_polyf.replace( this->ind_sec , _new_pos );

    tmp_vec_poly.replace( this->ind_first , tmp_polyf );

    this->polys_vec.replace( this->cur_ind, tmp_vec_poly );

    this->updateDisplay(true);
}


void JURGUI::JURGUI2DWindow::onRecieveFinalImage(QImage _img)
{
    QPixmap piximg = QPixmap::fromImage( _img );

    this->openCV_img = _img;

    QGraphicsScene* scene = new QGraphicsScene;
    scene->addPixmap( piximg );

    this->ui->gVROS->setScene( scene );
    this->ui->gVROS->show();
    this->ui->gVROS->fitInView( scene->itemsBoundingRect(), Qt::IgnoreAspectRatio );
}


void JURGUI::JURGUI2DWindow::on_gV_emitResizeEvent(void)
{
    if ( this->zoom_factor == 1.0 )
        this->ui->gV->adjustFit();

    //this->ui->gVROS->fitInView( this->ui->gVROS->scene()->itemsBoundingRect(), Qt::IgnoreAspectRatio );
}


void JURGUI::JURGUI2DWindow::on_pBtn_quit_clicked(void)
{
    this->close();
}


