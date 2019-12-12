/**
 * @addtogroup JUROEPNCV
 * @{
 * @defgroup JUROEPNCV_JUROpenCVSegmentation JUROpenCVSegmentation
 * @{
 *
 * @file  jur_opencv_segmentation/include/jur_opencv_segmentation/JUROpenCVSegmentation.h
 * @class JUROpenCVSegmentation
 *
 * @brief A Node for applying built'in OpenCV segmentation methods.
 *
 * This Node is for getting image via ros message
 * and apply desired segmentation method and in final
 * send result to sender node.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JUROPENCVSEGMENTATION_H
#define JUROPENCVSEGMENTATION_H


#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"


#include "jur_gui_segmentation/jur_segmentation_service.h"


#include "jur_opencv_segmentation/JUROpenCVMeanShift.h"
#include "JUROpenCVWatershed.h"


namespace JUROPENCV
{
    class JUROpenCVSegmentation
    {
        public:
            JUROpenCVSegmentation();

            bool getMsg(jur_gui_segmentation::jur_segmentation_service::Request&,
                        jur_gui_segmentation::jur_segmentation_service::Response&);

        private:
            ros::NodeHandle    node;
            ros::ServiceServer service;
            JUROPENCV::JUROpenCVMeanShift mean_shift;
            JUROPENCV::JUROpenCVWatershed watershed;

            cv::Mat            convertToCVMat(sensor_msgs::Image,std::string);
            sensor_msgs::Image createImageMsg(cv::Mat,std::string);
     };

}


#endif // JUROPENCVSEGMENTATION_H


/**
 * @}
 * @}
 */
