#include "jur_opencv_segmentation/JUROpenCVWatershed.h"


JUROPENCV::JUROpenCVWatershed::JUROpenCVWatershed()
{
}


cv::Mat JUROPENCV::JUROpenCVWatershed::applyWatershed(cv::Mat _img)
{
    cv::Mat bw;
    cv::cvtColor( _img, bw, CV_BGR2GRAY );

    cv::threshold( bw, bw, 40, 255, CV_THRESH_BINARY );

    cv::Mat dist;
    cv::distanceTransform(bw, dist, CV_DIST_L2, 3 );
    cv::normalize( dist, dist, 0, 1., cv::NORM_MINMAX );

    cv::threshold( dist, dist, .5, 1., CV_THRESH_BINARY );

    cv::Mat dist_8u;
    dist.convertTo( dist_8u, CV_8U );

    // Find total markers
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
    int ncomp = contours.size();

    cv::Mat markers = cv::Mat::zeros( dist.size(), CV_32SC1 );
    for ( int i = 0; i < ncomp; i++ )
        cv::drawContours(markers, contours, i, cv::Scalar::all(i+1), -1);

    cv::circle( markers, cv::Point(5,5), 3, CV_RGB(255,255,255), -1 );

    cvtColor( _img, _img, CV_BGRA2BGR );
    cv::watershed( _img, markers );

    // Generate random colors
    std::vector<cv::Vec3b> colors;
    for ( int i = 0; i < ncomp; i++ )
    {
        int b = cv::theRNG().uniform(0, 255);
        int g = cv::theRNG().uniform(0, 255);
        int r = cv::theRNG().uniform(0, 255);

        colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
    }

    // Create the result image
    cv::Mat dst = cv::Mat::zeros( markers.size(), CV_8UC3 );

    // Fill labeled objects with random colors
    for ( int i = 0; i < markers.rows; i++ )
    {
        for ( int j = 0; j < markers.cols; j++ )
        {
            int index = markers.at<int>(i,j);
            if (index > 0 && index <= ncomp)
                dst.at<cv::Vec3b>(i,j) = colors[index-1];
            else
                dst.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
        }
    }

    return dst;
}

