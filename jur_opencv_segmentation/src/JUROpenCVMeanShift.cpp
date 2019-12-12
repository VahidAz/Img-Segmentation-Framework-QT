#include "jur_opencv_segmentation/JUROpenCVMeanShift.h"


JUROPENCV::JUROpenCVMeanShift::JUROpenCVMeanShift()
{
    this->configOpenCl();
}


void JUROPENCV::JUROpenCVMeanShift::configOpenCl(void)
{
    // Device Info
    cv::ocl::DevicesInfo dev_info;
    cv::ocl::getOpenCLDevices( dev_info );

    // Platform Info
    cv::ocl::PlatformsInfo plat_info;
    cv::ocl::getOpenCLPlatforms( plat_info );

    cv::ocl::setDevice( dev_info[0] );
}


cv::Mat JUROPENCV::JUROpenCVMeanShift::applyMeanShift(cv::Mat _img, int _sp, int _sr, int _mean_size)
{
    // Applying MeanShift
    cv::ocl::oclMat ocl_image;
    cv::Mat res_mean;

    // Converting to CV_8UC4
    cv::Mat _img_conv;
    cvtColor( _img , _img_conv , CV_BGR2BGRA );

    ocl_image.upload(_img_conv.clone());

    cv::ocl::meanShiftSegmentation( ocl_image, res_mean , _sp , _sr , _mean_size );

    return res_mean.clone();
}
