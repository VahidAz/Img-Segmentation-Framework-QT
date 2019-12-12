#include "jur_opencv_segmentation/JUROpenCVSegmentation.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "jur_opencv_segmentation");

    JUROPENCV::JUROpenCVSegmentation jur_opencv_segmentation;

    ros::spinOnce();

    try
    {
        ros::spin();
    }
    catch(std::runtime_error& e)
    {
        return -1;
    }

    return 0;
}

