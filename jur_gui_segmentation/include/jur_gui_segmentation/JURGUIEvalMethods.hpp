/**
 * @addtogroup JURGUI
 * @{
 * @defgroup JURGUI_JURGUIEvalMethods.hpp JURGUIEvalMethods.hpp
 * @{
 *
 * @file  jur_gui_segmentation/include/jur_gui_segmentation/JURGUIEvalMethods.hpp
 * @class JURGUIEvalMethods.hpp
 *
 * @brief Evalution Methods for evaluating segmentation result.
 *
 * This class is implementation of some evalution methods for image segmentation.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JURGUIEVALMETHODS_H
#define JURGUIEVALMETHODS_H


#include <stdint-gcc.h>
#include <iomanip>
#include <math.h>


#include <QHash>
#include <QVector>


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"


#include "omp.h"


typedef long long int lint;
typedef long double ldouble;


namespace JUREVAL
{
    class JURGUIEvalMethods
    {
        public:
            JURGUIEvalMethods();

            static double orginalRI  (cv::Mat,cv::Mat);
            static double jaccard    (cv::Mat,cv::Mat);
            static QVector<double> MI(cv::Mat,cv::Mat);

        private:
            static ldouble jointProbabilityDist(QList<lint>,QList<lint>,lint);
    };
}


#endif // JURGUIEVALMETHODS_H


/**
 *@}
 *@}
 */
