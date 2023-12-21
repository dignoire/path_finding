#include <ros/ros.h>
#include "ros/console.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace cv;
using namespace std;


/** @function main */
int main( int argc, char** argv )
{
    /// Load source image and convert it to gray

    Mat src = imread( argv[1], 1 );
    Mat map = imread( argv[1], 1 );
    
    /// Create Window
    String source_window = "Source";
    namedWindow( source_window, WINDOW_AUTOSIZE );
    imshow( source_window, src );
    
    resize(src, map, Size(100,7), 0, 0, CV_INTER_LINEAR);

    namedWindow( "map", WINDOW_AUTOSIZE );
    imshow( "map", map );

    waitKey(0);
    return(0);
}

