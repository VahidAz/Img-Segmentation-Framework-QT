#include "JURGUIEvalMethods.hpp"


JUREVAL::JURGUIEvalMethods::JURGUIEvalMethods()
{
}


double JUREVAL::JURGUIEvalMethods::orginalRI(cv::Mat _ref, cv::Mat _cors)
{
    lint num_val = 0;
    lint n = _ref.cols * _ref.rows;

    const uint8_t* ref = (uint8_t*)_ref.data;
    const uint8_t* cor = (uint8_t*)_cors.data;

    #pragma omp parallel for reduction(+:num_val)
    for ( int i = 0 ; i < _ref.cols * _ref.rows  ; i++ )
        for ( int j = i + 1 ; j < _ref.cols * _ref.rows  ; j++ )
            if ( ( int(ref[i]) == int(ref[j]) && int(cor[i]) == int(cor[j]) ) ||
                 ( int(ref[i]) != int(ref[j]) && int(cor[i]) != int(cor[j]) )
                 )
                num_val++;


    lint      m = n * ( n - 1 );
    ldouble   k = m / 2.0;
    ldouble   l = 1 / k;
    double    res = l * num_val;

    return res;
}


double JUREVAL::JURGUIEvalMethods::jaccard(cv::Mat _ref, cv::Mat _cors)
{
    /// OLD Implementation
    /*
    double    min, max;
    cv::Point min_loc, max_loc;

    cv::minMaxLoc(_ref, &min, &max, &min_loc, &max_loc);
    int maxRef = max;

    cv::minMaxLoc(_cors, &min, &max, &min_loc, &max_loc);
    int maxCor = max;

    if ( maxRef >= maxCor )
        max = maxRef;
    else
        max = maxCor;

    cv::Mat scorePerClass = cv::Mat::zeros( max, 1, CV_64F);
    cv::Mat hasLabels( max, 1, CV_8U, cv::Scalar(1) );

    long int numPos,
             numCom,
             numUni;

    for ( int i = 1 ; i <= max ; i++ )
    {
        numPos = 0;
        numCom = 0;
        numUni = 0;

        /// Count Number of Current Class (i) in ref
        for ( int j = 0 ; j < _ref.rows ; j++ )
            for ( int k = 0 ; k < _ref.cols ; k++ )
                if ( int( _ref.at<uchar>(j,k) ) == i )
                    numPos++;

        /// Count Number of Current Class that is same in two image
        for ( int j = 0 ; j < _ref.rows ; j++ )
            for ( int k = 0 ; k < _ref.cols ; k++ )
                if ( int( _ref.at<uchar>(j,k) ) == i && int( _cors.at<uchar>(j,k) ) == i )
                    numCom++;

        /// Union
        for ( int j = 0 ; j < _ref.rows ; j++ )
            for ( int k = 0 ; k < _ref.cols ; k++ )
                if ( int( _ref.at<uchar>(j,k) ) == i || int( _cors.at<uchar>(j,k) ) == i )
                    numUni++;

        if ( numPos == 0 || numUni == 0 )
        {
            hasLabels.at<uchar>(i-1,0) = 0;
            continue;
        }

        scorePerClass.at<float>(i-1,0) = numCom/(double)numUni;
    }

    int numP = 0;
    float sum = 0;
    for ( int i = 0 ; i < scorePerClass.rows ; i++ )
        if ( int( hasLabels.at<uchar>(i,0) ) == 1 )
        {
            sum = sum + scorePerClass.at<float>(i,0);
            numP++;
        }

    return sum/(double)numP;
    */

    lint n11 = 0,
         n01 = 0,
         n10 = 0,
         n00 = 0;
    lint n = _ref.cols * _ref.rows;

    const uint8_t* ref = (uint8_t*)_ref.data;
    const uint8_t* cor = (uint8_t*)_cors.data;

    #pragma omp parallel for reduction(+:n11,n00,n10,n01)
    for ( int i = 0 ; i < n  ; i++ )
       for ( int j = i + 1 ; j < n  ; j++ )
           if ( ref[i] == ref[j] && cor[i] == cor[j] )
               n11++;
           else if ( ref[i] != ref[j] && cor[i] != cor[j] )
               n00++;
           else if ( ref[i] == ref[j] && cor[i] != cor[j] )
               n10++;
           else if ( ref[i] != ref[j] && cor[i] == cor[j] )
               n01++;

    lint sum = n11 + n01 + n10;

    ldouble div = n11/(ldouble)sum;

    /// [0 1]
    return div;
}


QVector<double> JUREVAL::JURGUIEvalMethods::MI(cv::Mat _ref, cv::Mat _cors)
{
    double    min, max;
    cv::Point min_loc, max_loc;

    cv::minMaxLoc(_ref, &min, &max, &min_loc, &max_loc);
    int maxRef = max;

    cv::minMaxLoc(_cors, &min, &max, &min_loc, &max_loc);
    int maxCor = max;

    lint n = _ref.rows * _ref.cols;

    /// Clustring Ref
    const uint8_t* ref = (uint8_t*)_ref.data;
    QHash<QString, lint> refClus;

    for ( int i = 1 ; i <= maxRef ; i++ )
        for ( lint j = 0 ; j < _ref.rows * _ref.cols ; j++ )
            if ( int(ref[j]) == i )
                refClus.insertMulti( QString::number(i), j );

    /// Clustring Cor
    const uint8_t* cor = (uint8_t*)_cors.data;
    QHash<QString,lint> corClus;

    for ( int i = 1 ; i <= maxCor ; i++ )
        for ( lint j = 0 ; j < n ; j++ )
            if ( int(cor[j]) == i )
                corClus.insertMulti( QString::number(i), j );

    // Calculate
    double res = 0,
           hx = 0,
           hy = 0;

    #pragma omp parallel for reduction(+:res)
    for ( int i = 1 ; i <= maxRef ; i++ )
        for ( int j = 1 ; j <= maxCor ; j++ )
        {
            QList<lint> refList = refClus.values( QString::number(i) );
            QList<lint> corList = corClus.values( QString::number(j) );

            ldouble jointProDist = JUREVAL::JURGUIEvalMethods::jointProbabilityDist( refList, corList, n );
            double px = refList.size() /(ldouble)n;
            double py = corList.size() /(ldouble)n;

            if ( jointProDist > 0 )
                res = res + ( jointProDist * log2( jointProDist / ( px * py ) ) );
            else
                res = res + 0;
        }

    for ( int i = 0 ; i <= maxRef ; i++ )
    {
        QList<lint> refList = refClus.values( QString::number(i) );
        double px = refList.size() /(ldouble)n;

        if ( px > 0 )
            hx = hx + ( px * log2(px) );
    }

    for ( int i = 0 ; i <= maxCor ; i++ )
    {
        QList<lint> corList = corClus.values( QString::number(i) );
        double py = corList.size() /(ldouble)n;

        if ( py > 0 )
            hy = hy + ( py * log2(py) );
    }

    QVector<double> ans;
    ans.push_back( res );
    ans.push_back( hx ); //ref
    ans.push_back( hy ); //cors

    return ans;
}


ldouble JUREVAL::JURGUIEvalMethods::jointProbabilityDist(QList<lint> _ref, QList<lint> _cor, lint _num)
{
    long int numInters = 0;

    for ( lint i = 0 ; i < _ref.size() ; i++ )
            if ( _cor.contains( _ref.at(i) ) )
                numInters++;

    return numInters/(ldouble)_num;
}

