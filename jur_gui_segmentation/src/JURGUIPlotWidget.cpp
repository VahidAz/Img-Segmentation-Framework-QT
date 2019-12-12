#include "JURGUIPlotWidget.h"
#include <iostream>


JURGUI::JURGUIPlotWidget::JURGUIPlotWidget(QWidget *parent) :
    QWidget(parent)
{
    ui = new Ui::JURGUIPlotWidget;
    ui->setupUi(this);

    qPlot = new QwtPlot( this );

    //QHBoxLayout *layout = new QHBoxLayout;
    //layout->addWidget( qPlot );
    //this->setLayout( layout );

    this->ui->horizontalLayout->addWidget( qPlot );

    connect ( this->ui->pushButton, SIGNAL( clicked() ), this, SLOT( savePlot()) );
}


JURGUI::JURGUIPlotWidget::~JURGUIPlotWidget()
{
    delete ui;
}


void JURGUI::JURGUIPlotWidget::drawRes(QPolygonF _in)
{
    this->points = _in;
    this->draw();
}


void JURGUI::JURGUIPlotWidget::savePlot()
{
    QPixmap pixmap = QPixmap::grabWidget(this->qPlot);
    pixmap.save(QString("/home/vahid/resPlot"), "JPG");
}


void JURGUI::JURGUIPlotWidget::draw(void)
{
    this->qPlot->setTitle( "Expriment Result" );
    this->qPlot->setCanvasBackground( Qt::white );

    this->qPlot->setAxisTitle( QwtPlot::yLeft, "Spatial BW");
    this->qPlot->setAxisTitle( QwtPlot::xBottom, "Result");
    qPlot->axisScaleEngine(QwtPlot::yLeft)->setAttribute(QwtScaleEngine::Floating,true);
    this->qPlot->setAxisAutoScale(QwtPlot::yLeft);
    this->qPlot->insertLegend( new QwtLegend() );

    //QRectF brect = this->points.boundingRect();

    QwtPlotGrid *grid = new QwtPlotGrid();
    grid->setMajPen(QPen( Qt::gray, 2 ));
    grid->attach( this->qPlot );

    QwtPlotCurve *curve = new QwtPlotCurve();
    curve->setTitle( "Expriment Result" );
    QPen cusPen;
    cusPen.setWidth( 6 );
    cusPen.setColor( Qt::blue );
    curve->setPen( cusPen );
    curve->setRenderHint( QwtPlotItem::RenderAntialiased, true );

    QwtSymbol *symbol = new QwtSymbol( QwtSymbol::Ellipse, QBrush( Qt::yellow ), QPen( Qt::red, 2 ), QSize( 8, 8 ) );
    curve->setSymbol( symbol );

    curve->setSamples( this->points );

    curve->attach( qPlot );

    this->show();
}
