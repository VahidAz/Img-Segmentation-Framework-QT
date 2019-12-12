/**
 * @addtogroup JURGUI
 * @{
 * @defgroup JURGUI_JURGUIROSNode JURGUIROSNode
 * @{
 *
 * @file  jur_gui_segmentation/include/jur_gui_segmentation/JURGUIROSNode.h
 * @class JURGUIROSNode
 *
 * @brief This class is for managing send/recieve between two nodes.
 *
 * This class manages connection between two nodes and also applies
 * required conversion on data.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JURGUIROSNODE_H
#define JURGUIROSNODE_H


#include <QImage>
#include <QThread>
#include <QDebug>
#include <QObject>


#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"


// This is for avoiding from a strange namespace error that
// qmake have for creating moc file in making phase
// This causes this header is passed by qmake
// when qmake starts to creating moc files
#ifndef Q_MOC_RUN
    #include "jur_gui_segmentation/jur_segmentation_service.h"
#endif


#include "JURGUIFormatConvertROSOpenCV.hpp"


namespace JURGUI
{
    class JURGUIROSNode: public QThread
    {
        Q_OBJECT

        public:
            JURGUIROSNode();

            virtual ~JURGUIROSNode();

            bool init   (int,char**);
            void sendMsg(QString,QImage, int _param1=0, int _param2=0);

        Q_SIGNALS:
            void imageReady (QImage);

        private:
            ros::ServiceClient                             client;
            jur_gui_segmentation::jur_segmentation_service srv;

            sensor_msgs::Image createImageMsg(cv::Mat);
            cv::Mat            convertToCVMat(sensor_msgs::Image,std::string);
    };
}


#endif // JURGUIROSNode_H


/**
 * @}
 * @}
 */
