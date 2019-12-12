/**
 * @addtogroup JURGUI
 * @{
 * @defgroup JURGUI_JURGUIPlotWidget JURGUIMainWindow
 * @{
 *
 * @file  jur_gui_segmentation/include/jur_gui_segmentation/JURGUIMainWindow.h
 * @class JURGUIMainWindow
 *
 * @brief This is the Main form in program.
 *
 * This class is to handel main from in GUI.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JURGUIMAINWINDOW_H
#define JURGUIMAINWINDOW_H


#include <QObject>
#include <QMainWindow>


#include "JURGUI2DWindow.h"
#include "JURGUIUtilities.hpp"

#include "JURGUICVSWrapper.h"


#include "ui_JURGUIMainWindow.h"
#include "ui_JURGUIAboutWindow.h"


namespace JURGUI
{
    namespace Ui
    {
        class JURGUIMainWindow;
    }

    class JURGUIMainWindow : public QMainWindow
    {
        Q_OBJECT

        public:
            explicit JURGUIMainWindow(QWidget* parent = 0);
                    ~JURGUIMainWindow();

            void init(int,char**);

        private Q_SLOTS:
            void on_pBtn_2d_clicked      (void);
            void on_pBtn_3d_clicked      (void);
            void on_pBtn_exit_clicked    (void);
            void on_actionAbout_triggered(void);

            void backHere(void);

        private:
            Ui::JURGUIMainWindow* ui;

            JURGUI2DWindow* frm_2d;

            QString mode;
    };
}


#endif // JURGUIMainWindow_H


/**
 * @}
 * @}
 */
