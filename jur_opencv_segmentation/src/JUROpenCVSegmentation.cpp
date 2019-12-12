#include "jur_opencv_segmentation/JUROpenCVSegmentation.h"


JUROPENCV::JUROpenCVSegmentation::JUROpenCVSegmentation()
{
    this->service =  this->node.advertiseService("jur_segmentation_interaction", &JUROpenCVSegmentation::getMsg, this);
}


bool JUROPENCV::JUROpenCVSegmentation::getMsg(jur_gui_segmentation::jur_segmentation_service::Request& _req,
                                              jur_gui_segmentation::jur_segmentation_service::Response& _res)
{
    // MeanShift
    if ( strcmp( _req.reqstr.c_str(), "mean" ) == 0 )
    {
        cv::Mat mean_shift_res = this->mean_shift.applyMeanShift( this->convertToCVMat( _req.reqimg, sensor_msgs::image_encodings::TYPE_8UC4 ), _req.reqparm1, _req.reqparm2, 2000 );

        _res.resimg = this->createImageMsg( mean_shift_res, sensor_msgs::image_encodings::TYPE_8UC4 );
    }
    // Watershed
    else if ( strcmp( _req.reqstr.c_str(), "watershed" ) == 0 )
    {
        cv::Mat watershed_res = this->watershed.applyWatershed( this->convertToCVMat( _req.reqimg, sensor_msgs::image_encodings::TYPE_8UC4 ) );

        _res.resimg = this->createImageMsg( watershed_res, sensor_msgs::image_encodings::TYPE_8UC3 );
    }
    // ShutDown
    else if ( strcmp( _req.reqstr.c_str(), "shutdown" )  == 0 )
    {
        if ( ros::isStarted() )
        {
            ros::shutdown();
            ros::waitForShutdown();
        }
    }

    return true;
}


cv::Mat JUROPENCV::JUROpenCVSegmentation::convertToCVMat(sensor_msgs::Image _img, std::string _type)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy( _img, _type );
    }
    catch ( cv_bridge::Exception& e )
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());

        //return;
    }

    cv::Mat tmp_mat = cv_ptr->image;

    return tmp_mat;
}


sensor_msgs::Image JUROPENCV::JUROpenCVSegmentation::createImageMsg(cv::Mat _img, std::string _type)
{
    ros::Time time = ros::Time::now();

    cv_bridge::CvImage cvi;
    cvi.header.stamp    = time;
    cvi.header.frame_id = "image";
    cvi.encoding        = _type;
    cvi.image           = _img;

    sensor_msgs::Image img_msg;
    cvi.toImageMsg(img_msg);

    return img_msg;
}
