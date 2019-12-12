#include "../include/jur_gui_segmentation/JURGUISubClassQGV.h"


JURGUI::JURGUISubClassQGV::JURGUISubClassQGV(QWidget *parent) :
    QGraphicsView(parent)
{
    this->init();
}


void JURGUI::JURGUISubClassQGV::init(void)
{
    this->setMouseTracking(true);

    this->start_polygon = false;
    this->drawing_flag  = false;
    this->edit_flag     = false;
}


bool JURGUI::JURGUISubClassQGV::checkDClickIsOnImage(QPointF _cur_click_pos)
{
    if ( QGraphicsItem *item = this->scene()->itemAt( _cur_click_pos ) )
    {
        if ( item->type() == QGraphicsPixmapItem::Type )
            return true;
        else
            return false;
    }
    else
        return false;
}


void JURGUI::JURGUISubClassQGV::mouseMoveEvent(QMouseEvent* _m_ev)
{
    this->mouse_cur_pos.setX( _m_ev->pos().x() );
    this->mouse_cur_pos.setY( _m_ev->pos().y() );

    // Just sending mouse position to the scence
    QPointF tmp_pos = this->mapToScene( _m_ev->pos() );
    if ( this->drawing_flag && tmp_pos.x() >= 0 && tmp_pos.y() >=0 )
        Q_EMIT this->mousePosUpdate( tmp_pos );
    else
    {
        tmp_pos.setX(-1);
        tmp_pos.setY(-1);

        Q_EMIT this->mousePosUpdate(tmp_pos);
    }

    // If it is on drawing polygons
    if ( this->start_polygon )
        this->viewport()->update();

    // If it is on editing polygons
    if ( this->edit_flag && this->checkDClickIsOnImage( this->mapToScene( _m_ev->pos() ) ) )
        Q_EMIT this->newPoseVert( this->mapToScene( _m_ev->pos() ) );
}


void JURGUI::JURGUISubClassQGV::mousePressEvent(QMouseEvent* _m_ev)
{
    if ( _m_ev->button() == Qt::LeftButton && this->start_polygon &&
         this->checkDClickIsOnImage( this->mapToScene( _m_ev->pos() ) ) )
    {
        QPoint tmp_point( _m_ev->x(), _m_ev->y() );

        this->cur_poly.push_back( tmp_point );

        this->viewport()->update();
    }
    else if ( _m_ev->button() == Qt::LeftButton && !this->start_polygon && this->drawing_flag )
        Q_EMIT this->mouseSingleClick( this->mapToScene( _m_ev->pos() ) );
}


void JURGUI::JURGUISubClassQGV::mouseDoubleClickEvent(QMouseEvent* _m_ev)
{
    // If image is loaded
    if ( !this->start_polygon && this->drawing_flag )
    {
        if ( this->checkDClickIsOnImage( this->mapToScene( _m_ev->pos() ) ) )
        {
            this->start_polygon = true;
            this->cur_poly.clear();

            QPoint tmp_point( _m_ev->x(), _m_ev->y() );

            this->cur_poly.push_back( tmp_point );

            this->viewport()->update();
        }
    }
    else if ( this->start_polygon && this->checkDClickIsOnImage( this->mapToScene( _m_ev->pos() ) ) )
    {
        this->cur_poly.push_back( this->cur_poly.at(0) );

        this->start_polygon = false;

        QPolygon  tmp_poly( this->cur_poly );
        QPolygonF tmp_poly_f = this->mapToScene( tmp_poly );

        this->cur_poly.clear();

        Q_EMIT this->polyIsReady( tmp_poly_f );
    }
}


void JURGUI::JURGUISubClassQGV::mouseReleaseEvent(QMouseEvent* _m_ev)
{
    this->edit_flag = false;

    this->setMouseTracking(true);
}


void JURGUI::JURGUISubClassQGV::paintEvent(QPaintEvent* _p_ev)
{
    // it is possible to limit this
    QGraphicsView::paintEvent( _p_ev );

    // Adjust Pen
    QPen cusPen;
    cusPen.setWidth( this->pen_size );
    cusPen.setColor( this->pen_color );
    cusPen.setStyle( Qt::DashLine );

    // Painter
    QPainter painter( this->viewport() );
    painter.setPen(cusPen);

    // Draw Points
    for ( int i = 0 ; i < this->cur_poly.size() ; i++ )
        painter.drawPoint( this->cur_poly.at(i) );

    // Draw Line Between Constant Points
    if ( this->cur_poly.size() > 1 )
        for ( int i = 0 ; i < this->cur_poly.size() ; i++ )
            if ( i + 1 >= this->cur_poly.size() )
                break;
            else
            {
                QLineF tmpl(this->cur_poly.at(i), this->cur_poly.at(i+1));
                painter.drawLine( tmpl );
            }

    // Draw Line to Current Mouse Position
    if ( this->cur_poly.size() > 0 && this->start_polygon )
    {
        QLineF tmpl(this->cur_poly.at(this->cur_poly.size() -1), this->mouse_cur_pos);
        painter.drawLine( tmpl);
    }
}


// For changing size of GraphicWidget
void JURGUI::JURGUISubClassQGV::resizeEvent(QResizeEvent *_ev)
{
    Q_EMIT this->emitResizeEvent();
}


// Fit image in view
void JURGUI::JURGUISubClassQGV::adjustFit(void)
{
    this->fitInView( this->scene()->itemsBoundingRect(), Qt::IgnoreAspectRatio );
}


void JURGUI::JURGUISubClassQGV::setSize(int _size)
{
    this->pen_size = _size;
}


void JURGUI::JURGUISubClassQGV::setColor(QColor _color)
{
    this->pen_color = _color;
}


void JURGUI::JURGUISubClassQGV::setEditFlag(bool _flag)
{
    this->edit_flag = _flag;

    this->setMouseTracking( !_flag );
}


void JURGUI::JURGUISubClassQGV::setDrawingFlag(bool _flag)
{
    this->drawing_flag = _flag;
}


// Setting NULL in Statuebar in form
void JURGUI::JURGUISubClassQGV::leaveEvent(QEvent* _ev)
{
    QPointF tmp_pos;

    tmp_pos.setX(-1);
    tmp_pos.setY(-1);

    Q_EMIT this->mousePosUpdate( tmp_pos );
}
