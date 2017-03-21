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
#include "path_finding/Path.h"



using namespace cv;
using namespace std;

int mouseStep = 0;
int rate = 1000;

vector<Point> final_path;



/**********************************************************/


/*** TREE ***/
class Tree
{
public:
    Tree();
    vector<Vertex> _vertex;
    void findPath(vector<Vertex> *path);

};

Tree::Tree()
{

}

void Tree::findPath(vector<Vertex> *path)
{

}

/**********************************************************/


/*** PRM ***/
class PRM
{
public:
    PRM(Mat map,int step, Mat mapProcessing);
    void loop();
    void addNewVertex();
    void connectAllVertices();
    Mat _map;
    Mat _mapProcessing;
    // tableau espace libre de vertex
    vector<Vertex> _freeSpace;
    void initFreeSpace();
    int distance(Vertex v1, Vertex v2);
    int nearestVertex(Vertex vrand);
    bool connectable(Vertex previous, Vertex vrand, Vertex* newVertex);
    int _step;
    int _sizeMax;
    bool _allConnected;
    int _dijkstraEnd;
    bool _continue;
    Vertex* _root;
    Vertex* _dest;
    void mouseEvents( int event, int x, int y);
    void smoothing(vector<Vertex> *path);
    void interpolation(vector<Vertex> path);
    vector<Point> spline(vector<Vertex> path);

private:
    Tree* _tree;
    Dijkstra* _dijkstra;
};

PRM::PRM(Mat map,int step, Mat mapProcessing): _map(map), _mapProcessing(mapProcessing), _step(step)
{
    _continue = true;
    _allConnected = false;
    _dijkstraEnd = 0;
    _tree = new Tree();
    _dijkstra = new Dijkstra();
    this->initFreeSpace();
}

void PRM::initFreeSpace()
{
    cout<<_mapProcessing.size()<<endl;
    cout<<"nb pixel: "<<_mapProcessing.rows*_mapProcessing.cols<<endl;
    for(int i=0;i<_mapProcessing.rows;i++){
        for(int j=0;j<_mapProcessing.cols;j++){
            if(_mapProcessing.at<uchar>(i,j) == 255){
                _freeSpace.push_back(Vertex(i,j));
            }
        }
    }
    cout<<"free space size: "<<_freeSpace.size()<<endl;

}

int PRM::distance(Vertex v1,Vertex v2)
{
    //cout << v1._x <<" "<<v1._y <<" "<<v2._x <<" "<<v2._y <<endl;
    //cout << sqrt(pow(v1._x-v2._x , 2) + pow(v1._y-v2._y,2))<<endl;
    int d = (int)sqrt(pow(v1._x-v2._x , 2) + pow(v1._y-v2._y , 2));
    return d;
}

int PRM::nearestVertex(Vertex v)
{
    int dist = 2*_mapProcessing.cols;
    int indexVertex = -1;
    for(int idx=0;idx<_tree->_vertex.size();idx++){
        int d_tmp =  distance(v, _tree->_vertex[idx]);
        if(d_tmp < dist && _step < d_tmp){
            dist = d_tmp;
            indexVertex = idx;
        }
        if(_step > d_tmp) return -1;
    }
    return indexVertex;
}

bool PRM::connectable(Vertex previous, Vertex vrand, Vertex* newVertex)
{
    bool connect = true;
    Point p1 = Point(previous._x,previous._y);
    Point p2 = Point(vrand._x,vrand._y);
    Point p12 = Point((p2.x-p1.x),(p2.y-p1.y));
    int normP12 = round(sqrt(pow(p12.x,2)+pow(p12.y,2)));
    if(normP12 == 0) return false;
    Point p13 = Point(round(_step*p12.x/normP12), round(_step*p12.y/normP12));
    Point p3 = Point(p13.x+p1.x,p13.y+p1.y);
    newVertex->_x = round(p3.x);
    newVertex->_y = round(p3.y);

    LineIterator it(_mapProcessing, Point(newVertex->_y,newVertex->_x), Point(previous._y,previous._x), 8);
    for(int i = 0; i < it.count; i++, ++it)
    {
        int val = _mapProcessing.at<uchar>(it.pos());
        if(val == 0) connect = false;

    }
    return connect;
}

void PRM::smoothing(vector<Vertex> *path)
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

void PRM::addNewVertex()
{
    int r = rand()%_freeSpace.size();
    Vertex vrand = _freeSpace[r];
    bool tooClose = false;
    for(int i=0;i<_tree->_vertex.size();i++)
    {
        if(distance(vrand,_tree->_vertex[i]) < _step)
            tooClose = true;
    }
    if(tooClose == false)
    {
        _tree->_vertex.push_back(vrand);
        circle( _map, Point(vrand._y,vrand._x), 3, CV_RGB( 250, 10, 10 ), 2);
        if(_tree->_vertex.size() == _sizeMax)
            cout << "All vertices added: "<<_sizeMax << endl;
    }
}

void PRM::connectAllVertices()
{
    for(int i=0;i<_tree->_vertex.size();i++)
    {
        for(int j=0;j<_tree->_vertex.size();j++)
        {
            if(i == j) continue;
            bool connect = true;
            LineIterator it(_mapProcessing, Point(_tree->_vertex[i]._y,_tree->_vertex[i]._x), Point(_tree->_vertex[j]._y,_tree->_vertex[j]._x), 8);
            for(int t = 0; t < it.count; t++, ++it)
            {
                int val = _mapProcessing.at<uchar>(it.pos());
                if(val == 0) connect = false;
            }
            int d = distance(_tree->_vertex[i],_tree->_vertex[j]);
            if(connect == true && _tree->_vertex[i].checkNeighbors(j) == false && d < round(2.5*_step)){
                vector<int> tmpj;
                tmpj.push_back(j);
                tmpj.push_back(d);
                vector<int> tmpi;
                tmpi.push_back(i);
                tmpi.push_back(d);
                _tree->_vertex[i]._neighborsIdx.push_back(tmpj);
                _tree->_vertex[j]._neighborsIdx.push_back(tmpi);
                Point a = Point(_tree->_vertex[i]._y,_tree->_vertex[i]._x);
                Point b = Point(_tree->_vertex[j]._y,_tree->_vertex[j]._x);
                line(_map,a,b ,CV_RGB( 10, 10, 10 ),1);
            }
        }
    }
}

vector<Point> PRM::spline(vector<Vertex> path)
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

void PRM::loop()
{
    if(_tree->_vertex.size() < _sizeMax)
    {
        addNewVertex();
    }
    else if(_allConnected == false)
    {
        _allConnected = true;
        connectAllVertices();
        _dijkstra->initNodes(_tree->_vertex);
        _dijkstra->initPath(0,1);
        //_dijkstra->loop();
    }
    else if(_dijkstraEnd == 0)
    {
        _dijkstraEnd = _dijkstra->loop();
        if(_dijkstraEnd == 1)
        {
            vector<Vertex> path;
            _dijkstra->getPath(&path);
            cout << "path size: "<< path.size()<<endl;
            if(path.size() <= 1) return;
            for(int i=0;i<path.size()-1;i++)
            {
                line(_map,Point(path[i]._y,path[i]._x),Point(path[i+1]._y,path[i+1]._x) ,CV_RGB( 100, 100, 100 ),2);
            }
            smoothing(&path);
            for(int i=0;i<path.size()-1;i++){
                line(_map,Point(path[i]._y,path[i]._x),Point(path[i+1]._y,path[i+1]._x) ,CV_RGB( 50, 250, 50 ),2);
            }
            final_path = spline(path);
        }
    }
}

void PRM::mouseEvents(int event, int x, int y)
{
    if( ( event == CV_EVENT_LBUTTONDOWN ) ) {
        if(mouseStep == 0){

            _root = new Vertex(y,x);
            _tree = new Tree();

            circle( _map, Point(_root->_y,_root->_x), 10, CV_RGB( 10, 10, 10 ), 5);
            mouseStep++;

            _tree->_vertex.push_back(*_root);
            cout << "root: "<< x <<", " << y << endl;

            

        }
        else if(mouseStep == 1){
            _dest = new Vertex(y,x);
            _tree->_vertex.push_back(*_dest);
            circle( _map, Point(_dest->_y,_dest->_x), 10, CV_RGB( 10, 10, 10 ), 5);
            mouseStep++;
            rate = 5000;
            cout << "destination: "<< x <<", " << y << endl;
        }
    }
    
}
/**********************************************************/

void mouseEventsStatic( int event, int x, int y, int flags, void* prm)
{
    static_cast<PRM*>(prm)->mouseEvents(event,x,y);
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

/*** MAIN ***/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "prm");
    ros::start();
    srand (time(NULL));


    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("path", sendPath);

    Mat map;

    Mat mapProcessing;
    if (argc >= 2){
        map = imread(argv[1], CV_LOAD_IMAGE_COLOR);
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

    cvNamedWindow( "Map window PRM", WINDOW_AUTOSIZE );
    cvStartWindowThread();
    imshow( "Map window PRM", map );
    waitKey(100);

    int step = 50;
    if(argc >= 3) step = atoi(argv[2]);

    PRM prm(map,step,mapProcessing);
    setMouseCallback("Map window PRM", mouseEventsStatic, &prm);

    int sizeMax = round(prm._freeSpace.size()/3700.0);
    if(argc >= 4) sizeMax = atoi(argv[3]);
    prm._sizeMax = sizeMax;

    ros::Rate loop_rate(rate);
    while(ros::ok()){
        if(mouseStep == 2){
            if(prm._dijkstraEnd >= 0)
                prm.loop();
        }
        //cout << "loop" <<endl;
        imshow( "Map window PRM", prm._map );
        ros::spinOnce();
        loop_rate.sleep();
    }
    cvDestroyWindow("Map window PRM");
    return 0;
}
