/**
 * @addtogroup JURGUITOOL
 * @{
 * @defgroup JURGUITOOL_JURGUICVSWRAPER JURGURCVSWRAPER
 * @{
 *
 * @file jur_gui_segmentation/include/jur_gui_segmentation/JURGUICVSWrapper.h
 * @class JURGUICSVWrapper
 *
 * @brief Read/Wrire cv::Mat in txt fiel
 *
 * Jacobs University Bremen\n
 * Project: JUR Segmenattion Tool\n
 *
 * @author Vahid Azizi (v.azizi@jacobs-university.de)
 * @date 20.09.2014
 *
 */

/**
 * Writing => JURTOOL::JURGUICVSWrapper::writeCSV(const char* _name filename, cv::Mat _mat matrix)
 * @param filename, yout matrix
 *
 *
 * Reading => JURTOOL::JURGUICVSWrapper::readCSV(const char* filename, cv::Mat matrix, int& row, int& col)
 * @param filename, returnning matrix, returning row, returning column
 *
 * Supported Types: CV_8U, CV_8S, CV_16U, CV_16S, CV_32F, CV_64F
 */

#ifndef JURGUICVSWRAPPER_H
#define JURGUICVSWRAPPER_H


#include <iostream>


#include <QFile>
#include <QTextStream>
#include <QString>
#include <QStringList>


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"


namespace JURTOOL
{
    class JURGUICVSWrapper
    {
        public:
            static void writeCSV(const char* _name ,cv::Mat _mat)
            {
                std::string s = _name;

                QFile file( QString::fromStdString(s) );
                file.open(QIODevice::WriteOnly | QIODevice::Text);
                QTextStream out(&file);

                out<<_mat.rows<<":"<<_mat.cols<<"\n";
                out<<_mat.type()<<"\n";

                switch ( _mat.type() )
                {
                    case 0: //8U
                    {
                        uchar* ref = _mat.data;

                        size_t step = _mat.step / sizeof(uchar);

                        for ( int i = 0 ; i < _mat.rows ; i++ )
                        {
                            for ( int j = 0 ; j < _mat.cols ; j++ )
                                out<<ushort(ref[i * step + j])<<";";

                            out<<"\n";
                        }

                        break;
                    }
                    case 1: //8S
                    {
                        char *ref = (char*)_mat.data;

                        size_t step = _mat.step / sizeof(char);

                        for ( int i = 0 ; i < _mat.rows ; i++ )
                        {
                            for ( int j = 0 ; j < _mat.cols ; j++ )
                                out<<char(ref[i * step + j])<<";";

                            out<<"\n";
                        }

                        break;
                    }
                    case 2://16U
                    {
                        ushort* ref = (ushort*)_mat.data;

                        size_t step = _mat.step / sizeof(ushort);

                        for ( int i = 0 ; i < _mat.rows ; i++ )
                        {
                            for ( int j = 0 ; j < _mat.cols ; j++ )
                                out<<ushort(ref[i * step + j])<<";";

                            out<<"\n";
                        }

                        break;
                    }
                    case 3://16S
                    {
                        short* ref = (short*)_mat.data;

                        size_t step = _mat.step / sizeof(short);

                        for ( int i = 0 ; i < _mat.rows ; i++ )
                        {
                            for ( int j = 0 ; j < _mat.cols ; j++ )
                                out<<short(ref[i * step + j])<<";";

                            out<<"\n";
                        }

                        break;
                    }
                    case 4://32S
                    {
                        int* ref = (int*)_mat.data;

                        size_t step = _mat.step / sizeof(int);

                        for ( int i = 0 ; i < _mat.rows ; i++ )
                        {
                            for ( int j = 0 ; j < _mat.cols ; j++ )
                                out<<int(ref[i * step + j])<<";";

                            out<<"\n";
                        }

                        break;
                    }
                    case 5://32F
                    {
                        float* ref = (float*)_mat.data;

                        size_t step = _mat.step / sizeof(float);

                        for ( int i = 0 ; i < _mat.rows ; i++ )
                        {
                            for ( int j = 0 ; j < _mat.cols ; j++ )
                                out<<float(ref[i * step + j])<<";";

                            out<<"\n";
                        }

                        break;
                    }
                    case 6://64F
                    {
                        double* ref = (double*)_mat.data;

                        size_t step = _mat.step / sizeof(double);

                        for ( int i = 0 ; i < _mat.rows ; i++ )
                        {
                            for ( int j = 0 ; j < _mat.cols ; j++ )
                                out<<double(ref[i * step + j])<<";";

                            out<<"\n";
                        }

                        break;
                    }
                    default:
                        break;
                }

                file.close();
            }

            // Read
            static void readCSV (const char* _name, cv::Mat& _mat, int& _row, int& _col)
            {
                std::string s = _name;

                QFile file( QString::fromStdString(s) );
                if ( !file.open(QIODevice::ReadOnly | QIODevice::Text) )
                {
                    std::cerr<<" File is Not Available! "<<std::endl;
                    return;
                }

                QTextStream in(&file);

                QString tmp = in.readLine();
                QStringList tmpList = tmp.split(":");

                _row = tmpList.at(0).toInt();
                _col = tmpList.at(1).toInt();

                int type = in.readLine().toInt();
                cv::Mat res(_row,_col,type);

                int rowCont = 0;

                switch ( type )
                {
                    case 0: //8U
                    {
                        while ( !in.atEnd() )
                        {
                            tmp = in.readLine();
                            tmp = tmp.simplified();
                            tmpList = tmp.split(";");

                            for ( int i = 0 ; i < tmpList.size() ; i++ )
                                res.at<uchar>(rowCont,i) = tmpList.at(i).toUShort();

                            rowCont++;
                        }

                        break;
                    }
                    case 1: //8S
                    {
                        while ( !in.atEnd() )
                        {
                            tmp = in.readLine();
                            tmp = tmp.simplified();
                            tmpList = tmp.split(";");

                            for ( int i = 0 ; i < tmpList.size() ; i++ )
                                res.at<char>(rowCont,i) = tmpList.at(i).toShort();

                            rowCont++;
                        }

                        break;
                    }
                    case 2://16U
                    {
                        while ( !in.atEnd() )
                        {
                            tmp = in.readLine();
                            tmp = tmp.simplified();
                            tmpList = tmp.split(";");

                            for ( int i = 0 ; i < tmpList.size() ; i++ )
                                res.at<ushort>(rowCont,i) = tmpList.at(i).toUShort();

                            rowCont++;
                        }

                        break;
                    }
                    case 3://16S
                    {
                        while ( !in.atEnd() )
                        {
                            tmp = in.readLine();
                            tmp = tmp.simplified();
                            tmpList = tmp.split(";");

                            for ( int i = 0 ; i < tmpList.size() ; i++ )
                                res.at<short>(rowCont,i) = tmpList.at(i).toShort();

                            rowCont++;
                        }

                        break;
                    }
                    case 4://32S
                    {
                        while ( !in.atEnd() )
                        {
                            tmp = in.readLine();
                            tmp = tmp.simplified();
                            tmpList = tmp.split(";");

                            for ( int i = 0 ; i < tmpList.size() ; i++ )
                                res.at<int>(rowCont,i) = tmpList.at(i).toInt();

                            rowCont++;
                        }

                        break;
                    }
                    case 5://32F
                    {
                        while ( !in.atEnd() )
                        {
                            tmp = in.readLine();
                            tmp = tmp.simplified();
                            tmpList = tmp.split(";");

                            for ( int i = 0 ; i < tmpList.size() ; i++ )
                                res.at<float>(rowCont,i) = tmpList.at(i).toFloat();

                            rowCont++;
                        }

                        break;
                    }
                    case 6://64F
                    {
                        while ( !in.atEnd() )
                        {
                            tmp = in.readLine();
                            tmp = tmp.simplified();
                            tmpList = tmp.split(";");

                            for ( int i = 0 ; i < tmpList.size() ; i++ )
                            {
                                std::cerr<<tmpList.at(i).toDouble()<<std::endl;
                                res.at<double>(rowCont,i) = tmpList.at(i).toDouble();
                            }

                            rowCont++;
                        }

                        break;
                    }
                    default:
                        break;
                }

                _mat = res;

                file.close();
            }
    };//class
}// namespace


#endif // JURGUICVSWRAPPER_H


/**
 *@}
 *@}
 */
