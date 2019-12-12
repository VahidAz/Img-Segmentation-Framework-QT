/**
 * @addtogroup JURGUI
 * @{
 * @defgroup JURGUI_JURGUIUtilities JURGUIUtilities
 * @{
 *
 * @file  jur_gui_segmentation/include/jur_gui_segmentation/JURGUIUtilities.hpp
 * @class JURGUIUtilities
 *
 * @brief This class is for adjusting window in center of screen.
 *
 * This class is for adjusting window in center of screen.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JURGUIUTILITIES_H
#define JURGUIUTILITIES_H


#include <QWidget>
#include <QDesktopWidget>
#include <QApplication>


namespace JURGUI
{

    class JURGUIUtilities
    {
        public:
            JURGUIUtilities();

            static void displayInCenter(QWidget*);
    };

}


#endif // JURGUIUTILITIES_H


/**
 * @}
 * @}
 */
