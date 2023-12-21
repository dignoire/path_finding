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
#include "Dijkstra.cpp"
//#include "suivi_chemin.cpp"
#include "path_finding/Path.h"
#include <geometry_msgs/Point32.h>





using namespace cv;
using namespace std;

int mouseStep = 0;
int rate = 100;
vector<Point> final_path;


/*** VERTEX ***/
class Square
{
public:
    Square(int x,int y);
    Square(int x,int y,int width, int heigh);
    int _x;
    int _y;
    int _width;
    int _heigh;
    int _status;
    vector< vector <int> > _neighborsIdx;
};

Square::Square(int x,int y): _x(x), _y(y){}
Square::Square(int x,int y,int width, int heigh): _x(x), _y(y), _width(width), _heigh(heigh)
{
    _status = -1;
}

/**********************************************************/


/*** Quadtree ***/
class Quadtree
{
public:
    Quadtree(Mat map,Mat mapProcessing);
    void loop();
    Mat _map;
    Mat _mapProcessing;
    Vertex* _root;
    Vertex* _dest;
    int _dijkstraEnd;
    void connectAllVertices();
    void mouseEvents( int event, int x, int y);
    vector<Square> _squares;
    vector<Vertex> _vertices;
    Dijkstra* _dijkstra;
    //Suivi* _suivi;
    bool addGoals();
    void smoothing(vector<Vertex> *path);
    vector<Point> spline(vector<Vertex> path);

private:
    bool test(Square*,int,vector<Square>*tmp);
    int distance(Vertex, Vertex);

};

Quadtree::Quadtree(Mat map, Mat mapProcessing): _map(map), _mapProcessing(mapProcessing)
{
    _squares.push_back(Square(0,0,_map.rows,_map.cols));
    _dijkstra = new Dijkstra();
    _dijkstraEnd = 0;
}

void Quadtree::loop()
{
    vector<Square> tmp;
    cout << "--------------"<<endl;
    for(int i=0;i<_squares.size();i++)
    {
        if(_squares[i]._status == -1)
        {
            bool b = test(&_squares[i],i,&tmp);
        }
    }

    for(int i=0;i<tmp.size();i++)
    {
        _squares.push_back(tmp[i]);
    }
    cout << "nb squares: " << _squares.size() << endl;
    tmp.clear();
}

bool Quadtree::test(Square *v, int idx, vector<Square> *tmp)
{
    int status = -1;
    for(int i=v->_x;i<v->_x+v->_width;i++)
    {
        for(int j=v->_y;j<v->_y+v->_heigh;j++)
        {
            int value = _mapProcessing.at<uchar>(i,j);
            if(status == -1)
                status = value;
            if(value != status)
            {
                Square v1 = Square(v->_x,v->_y,ceil(v->_width/2),ceil(v->_heigh/2));
                Square v2 = Square(v->_x + floor(v->_width/2),v->_y,ceil(v->_width/2),ceil(v->_heigh/2));
                Square v3 = Square(v->_x,v->_y + floor(v->_heigh/2),ceil(v->_width/2),ceil(v->_heigh/2));
                Square v4 = Square(v->_x + floor(v->_width/2),v->_y + floor(v->_heigh/2),ceil(v->_width/2),ceil(v->_heigh/2));
                tmp->push_back(v1);
                tmp->push_back(v2);
                tmp->push_back(v3);
                tmp->push_back(v4);
                v->_status = -2;
                return true;
            }
        }
    }
    v->_status = status;
    return false;
}

int Quadtree::distance(Vertex v1,Vertex v2)
{
    int d = (int)sqrt(pow(v1._x-v2._x , 2) + pow(v1._y-v2._y , 2));
    return d;
}

void Quadtree::connectAllVertices()
{
    int N = _vertices.size();
    for(int i=0;i<_vertices.size();i++)
    {
        int w1 = ceil(_squares[N-1-i]._width/2);
        int h1 = ceil(_squares[N-1-i]._heigh/2);

        for(int j=0;j<_vertices.size();j++)
        {
            if(i == j || _vertices[i].checkNeighbors(j) ) continue;

            int w2 = ceil(_squares[N-1-j]._width/2);
            int h2 = ceil(_squares[N-1-j]._heigh/2);

            int dx = abs(_vertices[i]._x - _vertices[j]._x);
            int dy = abs(_vertices[i]._y - _vertices[j]._y);

            int d = (int)sqrt(dx*dx + dy*dy);

            if( (dx<5+w1+w2) && (dy<5+h1+h2) ){
                vector<int> tmpj;
                tmpj.push_back(j);
                tmpj.push_back(d);
                vector<int> tmpi;
                tmpi.push_back(i);
                tmpi.push_back(d);
                _vertices[i]._neighborsIdx.push_back(tmpj);
                _vertices[j]._neighborsIdx.push_back(tmpi);
                Point a = Point(_vertices[i]._y,_vertices[i]._x);
                Point b = Point(_vertices[j]._y,_vertices[j]._x);
                //line(_map,a,b ,CV_RGB( 10, 10, 10 ),1);
            }
        }
    }
}

bool Quadtree::addGoals()
{
    int dist = _map.rows;
    int idx;
    for(int i=0;i<_vertices.size();i++)
    {
        int d = distance(_vertices[i],*_root);
        if(d < dist)
        {
            dist = d;
            idx = i;
        }
    }

    LineIterator it(_mapProcessing, Point(_vertices[idx]._y,_vertices[idx]._x), Point(_root->_y,_root->_x), 8);
    for(int i = 0; i < it.count; i++, ++it)
    {
        int val = _mapProcessing.at<uchar>(it.pos());
        if(val == 0) return false;

    }

    vector<int> tmproot;
    tmproot.push_back(idx);
    tmproot.push_back(dist);
    _root->_neighborsIdx.push_back(tmproot);
    vector<int> tmpi;
    tmpi.push_back(_vertices.size());
    tmpi.push_back(dist);
    _vertices[idx]._neighborsIdx.push_back(tmpi);
    _vertices.push_back(*_root);
    line(_map,Point(_root->_y,_root->_x), Point(_vertices[idx]._y,_vertices[idx]._x),CV_RGB( 10, 10, 10 ),1);

    dist = _map.rows;
    for(int i=0;i<_vertices.size();i++)
    {
        int d = distance(_vertices[i],*_dest);
        if(d < dist)
        {
            dist = d;
            idx = i;
        }
    }

    LineIterator it2(_mapProcessing, Point(_vertices[idx]._y,_vertices[idx]._x), Point(_dest->_y,_dest->_x), 8);
    for(int i = 0; i < it2.count; i++, ++it2)
    {
        int val = _mapProcessing.at<uchar>(it2.pos());
        if(val == 0) return false;

    }

    vector<int> tmpdest;
    tmpdest.push_back(idx);
    tmpdest.push_back(dist);
    _dest->_neighborsIdx.push_back(tmpdest);
    tmpi.clear();
    tmpi.push_back(_vertices.size());
    tmpi.push_back(dist);
    _vertices[idx]._neighborsIdx.push_back(tmpi);
    _vertices.push_back(*_dest);
    line(_map,Point(_dest->_y,_dest->_x), Point(_vertices[idx]._y,_vertices[idx]._x),CV_RGB( 10, 10, 10 ),1);

    return true;
}

void Quadtree::smoothing(vector<Vertex> *path)
{
    Vertex tmp = Vertex(0,0);
    for(int i=0;i<path->size()-2;i++){
        for(int j=path->size()-1;j>=i+2;j--){
            bool connect = true;
            LineIterator it(_mapProcessing, Point(path->at(i)._y,path->at(i)._x), Point(path->at(j)._y,path->at(j)._x), 8);
            for(int t = 0; t < it.count; t++, ++it)
            {
                int val = _mapProcessing.at<uchar>(it.pos());
                if(val == 0) connect = false;
            }
            if(connect == true){
                for(int k=j-1;k>i;k--){
                    path->erase(path->begin()+ k);
                }
                break;
            }
        }
    }
    cout << "smooth path size: " << path->size() << endl;
}

vector<Point> Quadtree::spline(vector<Vertex> path)
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

void Quadtree::mouseEvents(int event, int x, int y)
{
    if( ( event == EVENT_LBUTTONDOWN ) ) {
        cout << x <<", " << y << endl;
        if(mouseStep == 0){
            _root = new Vertex(y,x);
            circle( _map, Point(_root->_y,_root->_x), 10, CV_RGB( 10, 10, 10 ), 5);
            mouseStep++;
        }
        else if(mouseStep == 1){
            _dest = new Vertex(y,x);
            circle( _map, Point(_dest->_y,_dest->_x), 10, CV_RGB( 10, 10, 10 ), 5);
            mouseStep++;
            //rate = 5000;
        }

    }
}
/**********************************************************/

void mouseEventsStatic( int event, int x, int y, int flags, void* quadtree)
{
    static_cast<Quadtree*>(quadtree)->mouseEvents(event,x,y);
}

/*** MAIN ***/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Quadtree");
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
        dilate(mapProcessing,mapProcessing,element5);
        erode(mapProcessing,mapProcessing,element15);
    }
    else{
        cout<<"ERROR need argv[1]: map.png for example"<<endl;
    }

    if(! map.data )
    {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }


    namedWindow( "Map window quadtree", WINDOW_AUTOSIZE );
    startWindowThread();

    imshow( "Map window quadtree", map );
    //imshow( "Map processing window", mapProcessing );

    waitKey(100);

    Quadtree quadtree(map,mapProcessing);
    setMouseCallback("Map window quadtree", mouseEventsStatic, &quadtree);

    ros::Rate loop_rate(rate);
    int step = 0;
    int limit = 7;
    if(argc >= 3) limit = atoi(argv[2]);

    while(ros::ok()){
        if(mouseStep == 2 && step < limit){
            quadtree.loop();
            for(int i=quadtree._squares.size()-1;i>=0;i--)
            {
                rectangle(quadtree._map, Point(quadtree._squares[i]._y,quadtree._squares[i]._x), Point(quadtree._squares[i]._y+quadtree._squares[i]._heigh,quadtree._squares[i]._x+quadtree._squares[i]._width), CV_RGB( 100, 100, 100 ), 1, 8, 0);
            }
            waitKey(1000);
            if(step == limit-1)
            {
                for(int i=quadtree._squares.size()-1;i>=0;i--)
                {
                    //cout << quadtree._squares[i]._status<<" ";// << endl;
                    if(quadtree._squares[i]._status != 255){
                        quadtree._squares.erase(quadtree._squares.begin()+i);
                    }
                    else {
                        //quadtree._squares[i]._center[0] = round((2*quadtree._squares[i]._x + quadtree._squares[i]._width)/2);
                        //quadtree._squares[i]._center[1] = round((2*quadtree._squares[i]._y + quadtree._squares[i]._heigh)/2);
                        Vertex v = Vertex(round((2*quadtree._squares[i]._x + quadtree._squares[i]._width)/2), round((2*quadtree._squares[i]._y + quadtree._squares[i]._heigh)/2));
                        quadtree._vertices.push_back(v);
                        circle( quadtree._map, Point(v._y,v._x), 2, CV_RGB( 255, 50, 50 ), 1);
                        rectangle(quadtree._map, Point(quadtree._squares[i]._y,quadtree._squares[i]._x), Point(quadtree._squares[i]._y+quadtree._squares[i]._heigh,quadtree._squares[i]._x+quadtree._squares[i]._width), CV_RGB( 100, 100, 100 ), 1, 8, 0);
                    }
                }
                cout << "NB VERTICES: " << quadtree._vertices.size() << endl;
                quadtree.connectAllVertices();
                bool result = quadtree.addGoals();
                if(!result){
                    cout << "No solution" << endl;
                    quadtree._dijkstraEnd = 1;
                }
                quadtree._dijkstra->initNodes(quadtree._vertices);
                quadtree._dijkstra->initPath(quadtree._vertices.size()-2,quadtree._vertices.size()-1);
                rate = 1000;
            }
            step++;

        }
        else if(step == limit && quadtree._dijkstraEnd == 0)
        {
            quadtree._dijkstraEnd = quadtree._dijkstra->loop();
            if(quadtree._dijkstraEnd == 1)
            {
                vector<Vertex> path;
                quadtree._dijkstra->getPath(&path);
                cout << "path size: "<< path.size()<<endl;
                if(path.size() <= 1) break;
                for(int i=0;i<path.size()-1;i++)
                {
                    line(quadtree._map,Point(path[i]._y,path[i]._x),Point(path[i+1]._y,path[i+1]._x) ,CV_RGB( 100, 100, 100 ),2);
                }
                quadtree.smoothing(&path);
                for(int i=0;i<path.size()-1;i++){
                    line(quadtree._map,Point(path[i]._y,path[i]._x),Point(path[i+1]._y,path[i+1]._x) ,CV_RGB( 50, 250, 50 ),2);
                }
                final_path = quadtree.spline(path);
		cout<<"final path: "<<final_path.size()<<endl;
		//quadtree._suivi->initPath(final_path);

            }
        }
        //cout << "loop" <<endl;
        imshow( "Map window quadtree", quadtree._map );
        ros::spinOnce();
        loop_rate.sleep();
        waitKey(5);
    }
    destroyWindow("Map window quadtree");
    return 0;
}
