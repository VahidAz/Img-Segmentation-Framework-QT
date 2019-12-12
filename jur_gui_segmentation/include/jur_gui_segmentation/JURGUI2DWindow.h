/**
 * @addtogroup JURGUI
 * @{
 * @defgroup JURGUI_JURGUI2DWindow JURGUI2DWindow
 * @{
 *
 * @file  jur_gui_segmentation/include/jur_gui_segmentation/JURGUI2DWindow.h
 * @class JJURGUI2DWindow
 *
 * @brief GUI of 2D Segmentation.
 *
 * This class is for handeling all function in 2D segmentation form.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JURGUI2DWINDOW_H
#define JURGUI2DWindow_H

#include <limits>
#include <float.h>

#include <QObject>
#include <QMainWindow>
#include <QFileDialog>
#include <QFileInfo>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsView>
#include <QDebug>
#include <QHBoxLayout>
#include <qmath.h>
#include <QColorDialog>
#include <QTime>
#include <QRgb>


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/ocl/ocl.hpp"


#include "qwt/qwt.h"
#include "omp.h"


#include "JURGUIROSNode.h"
#include "JURGUIUtilities.hpp"
#include "JURGUIEvalMethods.hpp"
#include "JURGUIPlotWidget.h"


#include "ui_JURGUI2DWindow.h"
#include "ui_JURGUIAboutWindow.h"
#include "ui_JURGUIMeanShiftSetting.h"
#include "ui_JURGUIExpriment.h"


namespace JURGUI
{
    namespace Ui
    {
        class JURGUI2DWindow;
    }

    class JURGUI2DWindow : public QMainWindow
    {
        Q_OBJECT

        public:
            explicit JURGUI2DWindow(QWidget *parent = 0);
                    ~JURGUI2DWindow();

            void init        (int argc=0,char** argv=NULL,bool _ros_flag=false);
            void closeRosNode(void);

        private Q_SLOTS:
            void on_pBtn_load_clicked    (void);

            void on_pBtn_next_clicked    (void);
            void on_pBtn_previous_clicked(void);
            void on_pBtn_back_clicked    (void);
            void on_pBtn_quit_clicked    (void);

            void on_pBtn_extract_clicked  (void);
            void on_pBtn_eval_clicked     (void);
            void on_pBtn_req_clicked      (void);
            void on_pBtn_train_seg_clicked(void);
            void on_pBtn_expr_clicked     (void);

            void on_pBtn_zoom_in_clicked  (void);
            void on_pBtn_zoom_out_clicked (void);
            void on_pBtn_fit_clicked      (void);
            void on_pBtn_color_clicked    (void);
            void on_sBox_size_valueChanged(int);

            void on_rdBtnMean_clicked     (bool);
            void on_rdBtnWatershed_clicked(bool);

            void on_pBtn_setting_clicked(void);

            void on_pBtn_clear_clicked(void);

            void on_actionAbout_triggered(void);

            void on_gV_mousePosUpdate  (QPointF);
            void on_gV_polyIsReady     (QPolygonF);
            void on_gV_mouseSingleClick(QPointF);
            void on_gV_newPoseVert     (QPointF);
            void on_gV_emitResizeEvent (       );

            void on_lstV_currentRowChanged  (int);
            void on_lstPol_currentRowChanged(int);

            void on_chBox_load_train_clicked(bool);
            void on_chBox_sanity_clicked    (bool);
            void on_chBox_sample_clicked    (bool);

            void onDelBtnClicked(void);

            void onRecieveFinalImage(QImage);

            void setMeanSColBW(int);
            void setMeanSSpaBW(int);

            void startExpr    (int);
            void setExprName  (QString);
            void setExprsbw   (int);
            void setExprcbw   (int);
            void setExprcstart(int);
            void setExprcend  (int);
            void setExprcstep (int);
            void setExprsize  (int);
            void setExprri    (bool);
            void setExprmi    (bool);
            void setExprjacc  (bool);

        Q_SIGNALS:
            void backToMainFormSignal(void);
            void imageIsLoaded       (bool);

        private:
            Ui::JURGUI2DWindow *ui;

            QStringList                   cur_file_list;
            QVector< QVector<QPolygonF> > polys_vec;
            QString                       opencv_method, absul_path;
            QColor                        color;
            QImage                        openCV_img;

            QVector<cv::Mat> intImages;
            cv::Mat mean_res;

            JURGUI::JURGUIROSNode ros_node;

            int    cur_ind,
                   cur_poly_ind,
                   pen_size,
                   ind_first,
                   ind_sec,
                   cv_param1,
                   cv_param2;

            double zoom_factor;

            bool loadTrainData,
                 loadedImg,
                 sanityCheck,
                 sampleImg;

            void     updateDisplay          (bool _sel_list_poly = false);
            void     setEnableEnv           (bool);
            void     clearPolyVec           (void);
            QPointF  calcCenterOfPoly       (QPolygonF);
            cv::Mat  getIntImageFromHandySeg(void);
            cv::Mat  getIntImageFromOpenCV  (bool flag=false);
            bool     checkRepColor          (std::vector<cv::Vec3b>,cv::Vec3b);
            cv::Mat  extractWithColor       (cv::Mat,cv::Vec3b);
            void     setLog                 (char,QString);
            cv::Mat  readIntImages          (QString);
            int      genRandNum             (int,int);
            void     openCLConfig           (void);
            bool     distColor              (QVector<QRgb>,QRgb);

            struct stexprval
            {
                QString name;

                bool ri,
                     jacc,
                     mi;

                int cbw,
                    sbw,
                    sstart,
                    sstep,
                    send,
                    size;
            };

            stexprval experVal;

            JURGUI::JURGUIPlotWidget plotWidg;
    };
}


#endif // JURGUI2DWindow_H


/**
 *@}
 *@}
 */
