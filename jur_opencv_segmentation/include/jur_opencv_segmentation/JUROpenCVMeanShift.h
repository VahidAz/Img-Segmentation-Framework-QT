/**
 * @addtogroup JUROEPNCV
 * @{
 * @defgroup JUROEPNCV_JUROpenCVMeanShift JUROpenCVMeanShift
 * @{
 *
 * @file  jur_opencv_segmentation/include/jur_opencv_segmentation/JUROpenCVMeanShift.h
 * @class JUROpenCVMeanShift
 *
 * @brief A Node for applying OpenCV's MeanShift.
 *
 * This Node is for applying MeanShift Method from OpenCV.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JUROPENCVMEANSHIFT_H
#define JUROPENCVMEANSHIFT_H


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ocl/ocl.hpp"


namespace JUROPENCV
{
    class JUROpenCVMeanShift
    {
        public:
            JUROpenCVMeanShift();

            static cv::Mat applyMeanShift(cv::Mat,int,int,int);

        private:
            static void configOpenCl(void);
    };
}


#endif // JUR_OPENCV_MEANSHIFT_H


/**
 * @}
 * @}
 */
