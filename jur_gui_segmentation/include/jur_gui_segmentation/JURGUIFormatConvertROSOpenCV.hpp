/**
 * @addtogroup JURGUITOOL
 * @{
 * @defgroup JURGUITOOL_JURGUISubClassQGV JURGUISubClassQGV
 * @{
 *
 * @file jur_gui_segmentation/include/jur_gui_segmentation/JURGUISubClassQGV.h
 * @class JURGUISubClassQGV
 *
 * @brief Convert between QImage and CV::Mat.
 *
 * This class is for converting image type from QImage to CV::Mat
 * and vice versa.
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 18.09.2014
 *
 */


#ifndef JURGUIFORMATCONVERTROSOPENCV_H
#define JURGUIFORMATCONVERTROSOPENCV_H


#include <QImage>
#include <QPixmap>
#include <QDebug>


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"


namespace JURGUITOOL
{
    inline cv::Mat qImageToCvMat(const QImage &_in_image, bool _in_clone_image_data = true)
    {
        switch ( _in_image.format() )
        {
            // 8-bit, 4 channel
            case QImage::Format_RGB32:
            {
                cv::Mat mat( _in_image.height(), _in_image.width(), CV_8UC4, const_cast<uchar*>(_in_image.bits()), _in_image.bytesPerLine() );

                return (_in_clone_image_data ? mat.clone() : mat);
            }

            // 8-bit, 3 channel
            case QImage::Format_RGB888:
            {
                if ( !_in_clone_image_data )
                    qWarning() << "QImageToCvMat() - Conversion requires cloning since we use a temporary QImage";

                QImage swapped = _in_image.rgbSwapped();

                return cv::Mat( swapped.height(), swapped.width(), CV_8UC3, const_cast<uchar*>(swapped.bits()), swapped.bytesPerLine() ).clone();
            }

            // 8-bit, 1 channel
            case QImage::Format_Indexed8:
            {
                cv::Mat mat( _in_image.height(), _in_image.width(), CV_8UC1, const_cast<uchar*>(_in_image.bits()), _in_image.bytesPerLine() );

                return (_in_clone_image_data ? mat.clone() : mat);
            }

            default:
                qWarning() << "QImageToCvMat() - QImage format not handled in switch:" << _in_image.format();

            break;
        }

        return cv::Mat();
    }

    inline QImage cVMatToQImage(const cv::Mat3b &_in_image)
    {
        QImage dest(_in_image.cols, _in_image.rows, QImage::Format_ARGB32);

        for (int y = 0; y < _in_image.rows; ++y)
        {
            const cv::Vec3b *_in_imagerow = _in_image[y];
            QRgb *destrow = (QRgb*)dest.scanLine(y);

            for (int x = 0; x < _in_image.cols; ++x)
                destrow[x] = qRgba(_in_imagerow[x][2], _in_imagerow[x][1], _in_imagerow[x][0], 255);
        }

        return dest;
    }
}


#endif // JURGUIFORMATCONVERTROSOPENCV


/**
 * @}
 * @}
 */
