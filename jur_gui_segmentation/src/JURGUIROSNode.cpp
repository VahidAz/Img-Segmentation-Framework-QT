#include "../include/jur_gui_segmentation/JURGUIROSNode.h"


JURGUI::JURGUIROSNode::JURGUIROSNode()
{
}


JURGUI::JURGUIROSNode::~JURGUIROSNode()
{
    if ( ros::isStarted() )
    {
        ros::shutdown();
        ros::waitForShutdown();
    }

    wait();
}


bool JURGUI::JURGUIROSNode::init(int _argc, char** _argv)
{
    ros::init( _argc, _argv, "jur_gui_segmentation" );

    if ( !ros::master::check() )
        return false;

    ros::start();
    ros::NodeHandle n;

    client = n.serviceClient<jur_gui_segmentation::jur_segmentation_service>("jur_segmentation_interaction");

    start();

    return true;
}


void JURGUI::JURGUIROSNode::sendMsg(QString _arg, QImage _img, int _param1, int _param2)
{
    std::stringstream ss;
    ss << _arg.toStdString();
    srv.request.reqstr = ss.str();

    srv.request.reqparm1 = _param1;
    srv.request.reqparm2 = _param2;

    // Check availability of service
    if ( !client.exists() )
    {
        ROS_ERROR("JURGUI::JURGUIROSNode:: Service is not exist!!!");

        return;
    }

    // Shutdown
    if ( _arg.compare( QString("shutdown") ) == 0 )
    {
        this->client.call(srv);

        return;
    }

    // Convert Qimage to OpenCV Image
    srv.request.reqimg = this->createImageMsg( JURGUITOOL::qImageToCvMat( _img, true ) );

    // Calling Service (Sending image and reciving result)
    if ( client.call(srv) )
    {
        cv::Mat img;

        // MeanShift (Used type is 8UC4)
        if ( _arg.compare( QString("mean") ) == 0 )
             img = this->convertToCVMat( srv.response.resimg, sensor_msgs::image_encodings::TYPE_8UC4 ).clone();
        // Watershed (Used type is 8UC3)
        else if ( _arg.compare( QString("watershed") ) == 0 )
             img = this->convertToCVMat( srv.response.resimg, sensor_msgs::image_encodings::TYPE_8UC3 ).clone();

        // Converting to RGB
        cv::Mat _img_res;
        cv::cvtColor(img, _img_res, CV_BGR2RGB);

        // Send result to form for presenting
        Q_EMIT this->imageReady( JURGUITOOL::cVMatToQImage( _img_res ) );
    }
    else
        ROS_ERROR("JURGUI::JURGUIROSNode:: There is not any responce from service.");
}


sensor_msgs::Image JURGUI::JURGUIROSNode::createImageMsg(cv::Mat _img)
{
    ros::Time time = ros::Time::now();

    cv_bridge::CvImage cvi;
    cvi.header.stamp    = time;
    cvi.header.frame_id = "image";
    cvi.encoding        = sensor_msgs::image_encodings::TYPE_8UC4;
    cvi.image           = _img;

    sensor_msgs::Image img_msg;
    cvi.toImageMsg(img_msg);

    return img_msg;
}


cv::Mat JURGUI::JURGUIROSNode::convertToCVMat(sensor_msgs::Image _img, std::string _type)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(_img, _type);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("JURGUI::JURGUIROSNode:: cv_bridge exception: %s", e.what());

        //return;
    }

    cv::Mat tmp_mat = cv_ptr->image;

    return tmp_mat;
}
