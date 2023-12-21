#include <ros/ros.h>
#include "ros/console.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>


using namespace cv;
using namespace std;

Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

/// Function header
void thresh_callback(int, void* );


void threshfunction(Mat src_gray,int thresh)
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cout << thresh<<endl;
    /// Detect edges using Threshold
    threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }


    /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
    }

    /// Show in a window
    namedWindow( "Contours", WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );
}


/** @function main */
int main( int argc, char** argv )
{
    /// Load source image and convert it to gray

    Mat map = imread( argv[1], 1 );
    src = imread( argv[1], 1 );
    /// keep only red pixel
    blur( src, src, Size(3,3) );
    for(int i=0;i<src.rows;i++)
    {
        for(int j=0;j<src.cols;j++)
        {
            if(src.at<Vec3b>(i,j).val[2] < 150 || src.at<Vec3b>(i,j).val[1] > 50 || src.at<Vec3b>(i,j).val[0] > 50){
                src.at<Vec3b>(i,j).val[0] = 255;
                src.at<Vec3b>(i,j).val[1] = 255;
                src.at<Vec3b>(i,j).val[2] = 255;
            }
        }
    }

    /// Convert image to gray and blur it
    cvtColor( src, src_gray, CV_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );
    threshfunction(src_gray,100);

    /// Create Window
    String source_window = "Source";
    namedWindow( source_window, WINDOW_AUTOSIZE );
    imshow( source_window, src );

    //createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
    //thresh_callback( 0, 0 );

    namedWindow( "map", WINDOW_AUTOSIZE );
    imshow( "map", map );

    waitKey(0);
    return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cout << thresh<<endl;
    /// Detect edges using Threshold
    threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
    /// Find contours
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }


    /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
    }

    /// Show in a window
    namedWindow( "Contours", WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );
}

