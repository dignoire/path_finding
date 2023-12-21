#include <vector>
#include <limits>
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
#include "path_finding/Path.h"
#include <geometry_msgs/Point32.h>

using namespace std;
using namespace cv;

int mouseStep = 0;
int rate = 100;
vector<Point> final_path;

/*** VERTEX ***/
class Vertex
{
public:
    Vertex(int y,int x, int col, int row);
    Vertex(int y,int x);
    int _x;
    int _y;
    int _row;
    int _col;
    bool _free;
    int _distRoot;
    int _distDest;
    int _total;
    bool _done;
    Vertex* _father;
};
Vertex::Vertex(int y,int x,int col,int row): _x(x), _y(y), _free(free), _row(row), _col(col){_total = 0; _free=true;_done=false;}
Vertex::Vertex(int y,int x): _x(x), _y(y){_free=true; _total=0;_done=false;}


/*** A* ***/
class AStar
{
public:
    AStar(Mat map,Mat mapProcessing);
    Mat _map;
    Mat _mapProcessing;
    Vertex* _root;
    Vertex* _dest;
    vector<vector<Vertex> > _cells;
    //Vertex** _cells;
    int _step;
    int _nbCols;
    int _nbRows;
    int _idxDest,_idxRoot,_distance;
    vector<Vertex*> _path;
    void nearestVertex(Vertex,bool,int*,int*);
    int distance(Vertex,Vertex);
    bool loop();
    void decomposition(int,int);
    void mouseEvents( int event, int x, int y);
    int computeWeights(int c,int r, int dist);
    vector<Point> spline(vector<Vertex> path);

};

AStar::AStar(Mat map,Mat mapProcessing): _map(map), _mapProcessing(mapProcessing){
    _step = 20;
    decomposition(2*_step,_step);
    _nbCols = floor(map.cols/(2*_step));
    _nbRows = floor(map.rows/_step);
    const int x = floor(map.cols/(2*_step));
    const int y = floor(map.rows/_step);
}

int AStar::computeWeights(int c,int r, int dist)
{
    _cells[c][r]._distDest = abs(_cells[c][r]._row - _dest->_row) + abs(_cells[c][r]._col - _dest->_col);
    _cells[c][r]._distRoot = dist + 1;//abs(_cells[c][r]._row - _root->_row) + abs(_cells[c][r]._col - _root->_col);
    _cells[c][r]._total = _cells[c][r]._distRoot + _cells[c][r]._distDest;

    circle(_map, Point(_cells[c][r]._y,_cells[c][r]._x),_step/4,CV_RGB( 150, 250, 150 ), _step/4);
    // std::cout << "--- Point ---" << '\n';
    // std::cout << "_distDest: " << _cells[c][r]._distDest<<'\n';
    // std::cout << "_distRoot: " << _cells[c][r]._distRoot<<'\n';
    // std::cout << "_total: " << _cells[c][r]._total<<'\n';
    return _cells[c][r]._total;
    //return 0;
}

void AStar::decomposition(int decomposition_x, int decomposition_y){
    int dec_x = decomposition_x;
    int dec_y = decomposition_y;

    for(int i=0;i<_mapProcessing.cols;i=(i+dec_x)){
        vector<Vertex> col;
        //std::cout << "----- x: " << i<< '\n';
        for(int j=0;j<_mapProcessing.rows;j=(j+dec_y)){
            //std::cout << "----- y: " << j<< '\n';
            bool isObstacle = false;
            int k,l;
            for(k=0;k<dec_x;k++){
                for(l=0;l<dec_y;l++){
                    if(_mapProcessing.at<uchar>(j+l,i+k) == 0){
                        isObstacle = true;
                    }
                }
            }
            Vertex tmp = Vertex(floor(i+dec_x/2),floor(j+dec_y/2),i/dec_x,j/dec_y);
            if(isObstacle){
                rectangle(_map, Point(i,j),Point(i+k,j+l),CV_RGB( 250, 100, 100 ), 1, 8, 0);
                circle( _map, Point(floor(i+dec_x/2),floor(j+dec_y/2)), 3, CV_RGB( 250, 100, 100 ), 2);
                tmp._free = false;
            }
            else{
                rectangle(_map, Point(i,j),Point(i+k,j+l),CV_RGB( 100, 200, 100 ), 1, 8, 0);
                circle( _map, Point(floor(i+dec_x/2),floor(j+dec_y/2)), 3, CV_RGB( 100, 200, 100 ), 2);
            }

            col.push_back(tmp);
        }
        _cells.push_back(col);
    }
    /*for(int i=0;i<_cells.size();i++){
        for(int j=0;j<_cells[i].size();j++){
            cout<<i<< " "<<j<<endl;
            std::cout << _cells[i][j]._col <<" "<<_cells[i][j]._row<< '\n';
        }
    }*/
    imshow( "A*", _map );
    std::cout << "init done" << '\n';
}

bool AStar::loop()
{
    bool stopCondition = false;
    bool findOne = false;
    // std::cout << "--------------------------" << '\n';
    for(int i=0;i<_cells.size();i++){
        for(int j=0;j<_cells[i].size();j++){
            if(_cells[i][j]._total == _distance && _cells[i][j]._done == false){
                // std::cout << "total == distance == " << _distance<< '\n';
                if(_cells[i][j]._distDest == 0){
                    stopCondition = true;
                    return stopCondition;
                }
                if( (i-1)>=0 ){ // gauche
                    if(_cells[i-1][j]._total == 0  && _cells[i-1][j]._free){
                        computeWeights(_cells[i-1][j]._col,_cells[i-1][j]._row, _cells[i][j]._distRoot);
                        _cells[i-1][j]._father = &_cells[i][j];
                    }
                }
                if( (i+1)<_cells.size() ){ //droite
                    if(_cells[i+1][j]._total == 0 && _cells[i+1][j]._free){
                        computeWeights(_cells[i+1][j]._col,_cells[i+1][j]._row, _cells[i][j]._distRoot);
                        _cells[i+1][j]._father = &_cells[i][j];
                    }
                }
                if( (j-1)>=0 ){ // haut
                    if(_cells[i][j-1]._total == 0 && _cells[i][j-1]._free){
                        computeWeights(_cells[i][j-1]._col,_cells[i][j-1]._row, _cells[i][j]._distRoot);
                        _cells[i][j-1]._father = &_cells[i][j];
                    }
                }
                if( (j+1)<_cells[i].size() ){ // bas
                    if(_cells[i][j+1]._total == 0 && _cells[i][j+1]._free){
                        computeWeights(_cells[i][j+1]._col,_cells[i][j+1]._row, _cells[i][j]._distRoot);
                        _cells[i][j+1]._father = &_cells[i][j];
                    }
                }
                _cells[i][j]._done = true;
                findOne = true;
                imshow( "A*", _map );
                waitKey(10);
            }
        }
    }
    if(!findOne){
        _distance++;
        // std::cout << "-- distance: " << _distance<< '\n';
    }
    imshow( "A*", _map );

    return stopCondition;
}

void AStar::nearestVertex(Vertex v, bool inside, int *col_v, int *row_v)
{
    int dist = 2*_mapProcessing.cols;
    int indexVertex = -1;
    int c,r;
    for(int col=0;col<_nbCols;col++){
        for(int row=0;row<_nbRows;row++){
            int d_tmp =  distance(v, _cells[col][row]);
            if(!inside){
                if(d_tmp < dist && _step < d_tmp){
                    dist = d_tmp;
                    c = col;
                    r = row;
                }
            }
            else{
                if(d_tmp < dist ){
                    dist = d_tmp;
                    c = col;
                    r = row;
                }
            }
        }
    }
    *col_v = c;
    *row_v = r;
    return;
}

int AStar::distance(Vertex v1,Vertex v2)
{
    int d = (int)sqrt(pow(v1._x-v2._x , 2) + pow(v1._y-v2._y , 2));
    return d;
}

vector<Point> AStar::spline(vector<Vertex> path)
{
    vector<Point>  bezierPoints;
    for(int i=0;i<path.size()-1;i++)
    {
        bezierPoints.push_back(Point(path[i]._x,path[i]._y));
        int x = (path[i]._x+path[i+1]._x)/2;
        int y = (path[i]._y+path[i+1]._y)/2;
        bezierPoints.push_back(Point(x,y));
    }
    bezierPoints.push_back(Point(path[path.size()-1]._x,path[path.size()-1]._y));

    double size = 50;
    vector<Point>  pathBezier;
    if(path.size() < 3) return pathBezier;
    for(int i=0;i<size;i++)
    {
        Point p0 = Point(bezierPoints[0].x,bezierPoints[0].y);
        Point p1 = Point(bezierPoints[1].x,bezierPoints[1].y);
        double t = (i / size);
        float x = (1-t)*p0.x + t * p1.x;
        float y = (1-t)*p0.y + t * p1.y;
        pathBezier.push_back(Point(round(y),round(x)));
    }
    for(int b=1;b<bezierPoints.size()-2;b=b+2){
        Point p0 = Point(bezierPoints[b].x,bezierPoints[b].y);
        Point p1 = Point(bezierPoints[b+1].x,bezierPoints[b+1].y);
        Point p2 = Point(bezierPoints[b+2].x,bezierPoints[b+2].y);

        for(int i=0;i<=size;i++)
        {
            double t = (i / size);
            //float x = (pow(1-t,2))*p0.x + 2*(1-t)*t*p1.x + (t*t) * p2.x;
            //float y = (pow(1-t,2))*p0.y + 2*(1-t)*t*p1.y + (t*t) * p2.y;
            float x = (pow(1-t,3))*p0.x + 3*pow((1-t),2)*t*p1.x + 3*(1-t)*t*t*p1.x + (t*t*t) * p2.x;
            float y = (pow(1-t,3))*p0.y + 3*pow((1-t),2)*t*p1.y + 3*(1-t)*t*t*p1.y + (t*t*t) * p2.y;
            pathBezier.push_back(Point(round(y),round(x)));
        }

    }
    for(int i=0;i<size;i++)
    {
        Point p0 = Point(bezierPoints[bezierPoints.size()-2].x,bezierPoints[bezierPoints.size()-2].y);
        Point p1 = Point(bezierPoints[bezierPoints.size()-1].x,bezierPoints[bezierPoints.size()-1].y);
        double t = (i / size);
        float x = (1-t)*p0.x + t * p1.x;
        float y = (1-t)*p0.y + t * p1.y;
        pathBezier.push_back(Point(round(y),round(x)));
    }
    for(int i=0;i<pathBezier.size()-1;i++){
        line(_map,pathBezier[i],pathBezier[i+1],CV_RGB( 250, 50, 50 ),3);
    }
    cout << "spline path size: " << pathBezier.size() << endl;
    return pathBezier;
}

bool sendPath(path_finding::Path::Request  &req, path_finding::Path::Response &res)
{
    for(int i=0;i<final_path.size();i++){
        geometry_msgs::Point32 tmp;
        tmp.x = final_path[i].x;
        tmp.y = final_path[i].y;
        res.path.push_back(tmp);
    }
    return true;
}

void AStar::mouseEvents(int event, int x, int y)
{
    if( ( event == EVENT_LBUTTONDOWN ) ) {
        cout << x <<", " << y << endl;
        if(mouseStep == 0){
            mouseStep++;
            int c;
            int r;
            nearestVertex(Vertex(x,y),true,&c,&r);
            _root = &_cells[c][r];
            circle( _map, Point(_root->_y,_root->_x), 10, CV_RGB( 10, 10, 10 ), 5);
            imshow( "A*", _map );
            cout <<"row: "<<_root->_row<<" col: "<<_root->_col<<endl;
        }
        else if(mouseStep == 1){
            mouseStep++;
            int c;
            int r;
            nearestVertex(Vertex(x,y),true,&c,&r);
            _dest = &_cells[c][r];
            circle( _map, Point(_dest->_y,_dest->_x), 10, CV_RGB( 10, 10, 10 ), 5);
            imshow( "A*", _map );
            cout <<"row: "<<_dest->_row<<" col: "<<_dest->_col<<endl;

            _distance = computeWeights(_root->_col,_root->_row, -1);
            cout<<"distance: "<<_distance<<endl;
        }
    }
}
/**********************************************************/

void mouseEventsStatic( int event, int x, int y, int flags, void* astar)
{
    static_cast<AStar*>(astar)->mouseEvents(event,x,y);
}


/*** MAIN ***/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "AStar");
    ros::start();
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("path", sendPath);

    srand (time(NULL));

    Mat map;
    Mat mapProcessing;
    if (argc >= 2){
        map = imread(argv[1], IMREAD_COLOR);
        cvtColor(map,mapProcessing,CV_RGB2GRAY);
        threshold(mapProcessing,mapProcessing, 120, 255, CV_THRESH_BINARY);
        Mat element5 = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
        Mat element15 = getStructuringElement(MORPH_ELLIPSE,Size(20,20));
        //dilate(mapProcessing,mapProcessing,element5);
        //erode(mapProcessing,mapProcessing,element15);
    }
    else{
        cout<<"ERROR need argv[1]: map.png for example"<<endl;
    }

    if(! map.data )
    {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }
    namedWindow( "A*", WINDOW_AUTOSIZE );
    //cvNamedWindow( "Map processing window", WINDOW_AUTOSIZE );

    startWindowThread();

    imshow( "A*", map );
    //imshow( "Map processing window", mapProcessing );

    waitKey(100);

    AStar astar(map,mapProcessing);
    setMouseCallback("A*", mouseEventsStatic, &astar);

    ros::Rate loop_rate(rate);

    bool stop = false;
    while(ros::ok()){
        if(mouseStep == 2 && !stop){
            stop = astar.loop();
            if(stop){
                cout << "path found" <<endl;
                astar._path.push_back(astar._dest);
                Vertex* tmp1 = astar._dest->_father;
                for(int i=0;i<astar._distance;i++){
                    astar._path.push_back(tmp1);
                    Vertex* tmp2 = tmp1->_father;
                    tmp1 = tmp2;
                }
                for(int i=0;i<astar._path.size()-1;i++){
                    Point a = Point(astar._path[i]->_y,astar._path[i]->_x);
                    Point b = Point(astar._path[i+1]->_y,astar._path[i+1]->_x);
                    line(astar._map,a,b ,CV_RGB( 100, 100, 200 ),3);
                }
                vector<Vertex> path;
                for(int i=0;i<astar._path.size()-1;i++){
                    path.push_back(*astar._path[i]);
                }
                final_path = astar.spline(path);
            }
        }

        imshow( "A*", astar._map );
        ros::spinOnce();
        loop_rate.sleep();
        waitKey(10);
    }
    destroyWindow("A*");
    return 0;
}
