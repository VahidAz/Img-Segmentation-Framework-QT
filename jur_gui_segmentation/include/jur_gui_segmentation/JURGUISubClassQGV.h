/**
 * @addtogroup JURGUI
 * @{
 * @defgroup JURGUI_JURGUISubClassQGV JURGUISubClassQGV
 * @{
 *
 * @file  jur_gui_segmentation/include/jur_gui_segmentation/JURGUISubClassQGV.h
 * @class JURGUISubClassQGV
 *
 * @brief Subclass of QGeraphivsView to handel handy segmentation.
 *
 * This class is a subclass from QGraphicsView to add some
 * properties for image segmentatuion with human.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JURGUISUBCLASSQGV_H
#define JURGUISUBCLASSQGV_H


#include <QObject>
#include <QGraphicsView>
#include <QMouseEvent>
#include <QEvent>
#include <QDebug>
#include <QPen>
#include <QStyle>
#include <QGraphicsPixmapItem>


namespace  JURGUI
{
    class JURGUISubClassQGV : public QGraphicsView
    {
        Q_OBJECT

        public:
            explicit JURGUISubClassQGV(QWidget *parent = 0);

            void mouseMoveEvent       (QMouseEvent*);
            void mousePressEvent      (QMouseEvent*);
            void mouseDoubleClickEvent(QMouseEvent*);
            void mouseReleaseEvent    (QMouseEvent*);

            void leaveEvent (QEvent*);
            void paintEvent (QPaintEvent*);
            void resizeEvent(QResizeEvent*);

            void adjustFit  (void);
            void setSize    (int);
            void setColor   (QColor);
            void setEditFlag(bool);

            QPoint mouse_cur_pos;

        private Q_SLOTS:
            void setDrawingFlag(bool);

        Q_SIGNALS:
            void mousePosUpdate  (QPointF);
            void polyIsReady     (QPolygonF);
            void mouseSingleClick(QPointF);
            void newPoseVert     (QPointF);
            void changeEditMode  (bool);
            void emitResizeEvent (void);

        private:
            void init(void);

            bool start_polygon,
                 drawing_flag,
                 edit_flag;

            int pen_size;

            QVector<QPoint> cur_poly;
            QColor pen_color;

            bool checkDClickIsOnImage(QPointF);
    };
}


#endif // JURGUISUBCLASSQGV_H


/**
 * @}
 * @}
 */

