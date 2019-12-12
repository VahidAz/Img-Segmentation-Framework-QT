/**
 * @addtogroup JUROEPNCV
 * @{
 * @defgroup JUROEPNCV_JUROpenCVWatershed JUROpenCVWatershed
 * @{
 *
 * @file  jur_opencv_segmentation/include/jur_opencv_segmentation/JUROpenCVWatershed.h
 * @class JUROpenCVWatershed
 *
 * @brief A Node for applying OpenCV's Watershed.
 *
 * This Node is for applying Watershed Method from OpenCV.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JUROPENCVWATERSHED_H
#define JUROPENCVWATERSHED_H


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ocl/ocl.hpp"
#include "opencv2/opencv.hpp"


namespace JUROPENCV
{
    class JUROpenCVWatershed
    {
        public:
            JUROpenCVWatershed();

            static cv::Mat applyWatershed(cv::Mat);
    };
}


#endif // JUROPENCVWATERSHED_H


/**
 * @}
 * @}
 */
