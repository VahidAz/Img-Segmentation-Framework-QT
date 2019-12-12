/**
 * @addtogroup JURGUI
 * @{
 * @defgroup JURGUI_JURGUIPlotWidget JURGUIPlotWidget
 * @{
 *
 * @file  jur_gui_segmentation/include/jur_gui_segmentation/JURGUIPlotWidget.h
 * @class JURGUIPlotWidget
 *
 * @brief This class is for drawing chart from result of expriments.
 *
 * This class handels drawing chart from result of expriments.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JURGUIPLOTWIDGET_H
#define JURGUIPLOTWIDGET_H


#include <QWidget>
#include <QHBoxLayout>
#include <QObject>
#include <QBuffer>
#include <QFile>


#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_grid.h>
#include <qwt/qwt_legend.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_plot_magnifier.h>
#include <qwt/qwt_plot_panner.h>
#include <qwt/qwt_plot_picker.h>
#include <qwt/qwt_picker_machine.h>
#include <qwt/qwt_scale_engine.h>


#include "ui_JURGUIPlotWidget.h"


namespace JURGUI
{
    namespace Ui
    {
        class JURGUIPlotWidget;
    }

    class JURGUIPlotWidget : public QWidget
    {
        Q_OBJECT

        public:
            explicit JURGUIPlotWidget(QWidget *parent = 0);
            ~JURGUIPlotWidget();

        void drawRes(QPolygonF);

        private Q_SLOTS:
            void savePlot();

        private:
            Ui::JURGUIPlotWidget *ui;

            QwtPlot   *qPlot;
            QPolygonF points;

            void draw(void);
    };
}


#endif // JURGUIPLOTWIDGET_H


/**
 * @}
 * @}
 */
