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
#include "nav_msgs/Path.h"



using namespace cv;
using namespace std;

int mouseStep = 0;
int rate = 1000;
bool inverse = false;

vector<Point> final_path;


/*** VERTEX ***/
class Vertex
{
public:
  Vertex(int x,int y);
  int _x;
  int _y;
  int _fatherIdx;
};

Vertex::Vertex(int x,int y): _x(x), _y(y){}
/**********************************************************/

/*** EDGE ***/
class Edge
{
public:
  Edge(Vertex v1, Vertex v2);
  Vertex _v1;
  Vertex _v2;

private:

};

Edge::Edge(Vertex v1,Vertex v2): _v1(v1),_v2(v2)
{

}
/**********************************************************/


/*** TREE ***/
class Tree
{
public:
  Tree(Vertex root);
  vector<Edge> _edges;
  vector<Vertex> _vertex;
  void addEdge(int idx, Vertex v1, Vertex* v2);
  int _nbEdges;
  void findPath(vector<Vertex> *path, Vertex* v2);

};

Tree::Tree(Vertex root)
{
  _vertex.push_back(root);
  _nbEdges = 0;
}

void Tree::addEdge(int idx, Vertex previous, Vertex* newVertex)
{
  newVertex->_fatherIdx = idx;//previous;//&previous;
  //cout << idx << endl;
  this->_vertex.push_back(*newVertex);
  this->_edges.push_back(Edge(previous,*newVertex));
  _nbEdges = _edges.size();
}

void Tree::findPath(vector<Vertex> *path, Vertex* v2)
{
  int securite = 0;

  path->push_back(*v2);
  int idx = v2->_fatherIdx;
  path->push_back(_vertex[idx]);

  while(idx != 0 && securite < 10000)
  {
    //cout << idx << endl;
    idx = _vertex[idx]._fatherIdx;
    path->push_back(_vertex[idx]);
    securite++;
  }
  return;
}

/**********************************************************/


/*** RRT ***/
class RRT_connect
{
public:
  RRT_connect(Mat map,int step,int sizeMax, Mat mapProcessing);
  bool loop(Tree *tree, Vertex *v);
  Mat _map;
  Mat _mapProcessing;
  // tableau espace libre de vertex
  vector<Vertex> _freeSpace;
  void initFreeSpace();
  void updateTree(Point p1, Point p2);
  int distance(Vertex v1, Vertex v2);
  int nearestVertex(Tree *tree, Vertex vrand);
  bool connectable(Vertex previous, Vertex vrand, Vertex* newVertex);
  int _step;
  bool _continue;
  Vertex* _root;
  Vertex* _dest;
  //int mouseStep;
  void mouseEvents( int event, int x, int y);
  void smoothing(vector<Vertex> *path);
  vector<Point> spline(vector<Vertex> path);
  Tree* _tree_root;
  Tree* _tree_dest;
  bool trees_connectable(Vertex v, Tree *tree, Vertex *vtree);
  ros::Publisher _pub_path;

private:
  int _sizeMax;
};

RRT_connect::RRT_connect(Mat map,int step, int sizeMax,Mat mapProcessing): _map(map), _mapProcessing(mapProcessing), _step(step), _sizeMax(sizeMax)
{
  _continue = true;
  this->initFreeSpace();
}

void RRT_connect::initFreeSpace()
{
  cout<<"Initiating free space"<<endl;
  //circle(_map, Point(j,i), 10, 1, 15);
  cout<<_mapProcessing.size()<<endl;
  cout<<"nb pixel: "<<_mapProcessing.rows*_mapProcessing.cols<<endl;
  for(int i=0;i<_mapProcessing.rows;i++){
    for(int j=0;j<_mapProcessing.cols;j++){
      if(_mapProcessing.at<uchar>(i,j) == 255){
        _freeSpace.push_back(Vertex(i,j));
        //circle(_map, Point(j,i), 10, 1, 15);
      }
    }
  }
  cout<<"free space size: "<<_freeSpace.size()<<endl;

}

void RRT_connect::updateTree(Point p1, Point p2)
{
  line(_map,Point(p1.y,p1.x),Point(p2.y,p2.x) ,150);
}

int RRT_connect::distance(Vertex v1,Vertex v2)
{
  //cout << v1._x <<" "<<v1._y <<" "<<v2._x <<" "<<v2._y <<endl;
  //cout << sqrt(pow(v1._x-v2._x , 2) + pow(v1._y-v2._y,2))<<endl;
  int d = (int)sqrt(pow(v1._x-v2._x , 2) + pow(v1._y-v2._y , 2));
  return d;
}

int RRT_connect::nearestVertex(Tree* tree,Vertex vrand)
{
  int dist = 2*_mapProcessing.cols;
  int indexVertex = -1;
  for(int idx=0;idx<tree->_vertex.size();idx++){
    int d_tmp =  distance(vrand, tree->_vertex[idx]);
    if(d_tmp < dist && _step < d_tmp){
      dist = d_tmp;
      indexVertex = idx;
    }
    if(_step > d_tmp) return -1;
  }
  return indexVertex;
}

bool RRT_connect::connectable(Vertex previous, Vertex vrand, Vertex* newVertex)
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

void RRT_connect::smoothing(vector<Vertex> *path)
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

vector<Point> RRT_connect::spline(vector<Vertex> path)
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

  return pathBezier;
}

bool RRT_connect::loop(Tree* tree, Vertex *newVertex)
{
  bool vertexAdded = false;
  if(tree->_nbEdges < _sizeMax && _continue == true)
  {
    int r = rand()%_freeSpace.size();
    Vertex vrand = _freeSpace[r];
    int idx = nearestVertex(tree, vrand);
    if(idx > -1){
      //Vertex newVertex = Vertex(0,0);
      bool result = connectable(tree->_vertex[idx],vrand,newVertex);
      if(result == true){
        tree->addEdge(idx,tree->_vertex[idx],newVertex);
        circle(_map, Point(newVertex->_y,newVertex->_x), 5, CV_RGB( 10, 10, 10 ), 1);
        //_freeSpace.erase(_freeSpace.begin()+ r);
        line(_map,Point(newVertex->_y,newVertex->_x),Point(tree->_vertex[idx]._y,tree->_vertex[idx]._x) ,CV_RGB( 10, 10, 10 ));
        vertexAdded = true;
        //v = newVertex;
      }
    }
  }
  return vertexAdded;
}

bool RRT_connect::trees_connectable(Vertex v, Tree *tree, Vertex *vtree)
{
  bool done = false;
  for(int idx=tree->_vertex.size()-1;idx>=0;idx--){
    bool connect = true;
    Vertex v_tree = tree->_vertex[idx];
    LineIterator it(_mapProcessing, Point(v._y,v._x), Point(v_tree._y,v_tree._x), 8);
    for(int t = 0; t < it.count; t++, ++it)
    {
      int val = _mapProcessing.at<uchar>(it.pos());
      if(val == 0) connect = false;
    }
    if(connect == true){
      _continue = false;
      done = true;
      vtree->_x = v_tree._x;
      vtree->_y = v_tree._y;
      vtree->_fatherIdx = v_tree._fatherIdx;
      break;
    }
  }
  return done;
}

void RRT_connect::mouseEvents(int event, int x, int y)
{
  if( ( event == EVENT_LBUTTONDOWN ) ) {
    cout << x <<", " << y << endl;

    if(mouseStep == 0){
      _root = new Vertex(y,x);
      _tree_root = new Tree(*_root);
      circle( _map, Point(_root->_y,_root->_x), 10, CV_RGB( 10, 10, 10 ), 5);
      mouseStep++;
    }
    else if(mouseStep == 1){
      _dest = new Vertex(y,x);
      _tree_dest = new Tree(*_dest);
      circle( _map, Point(_dest->_y,_dest->_x), 10, CV_RGB( 10, 10, 10 ), 5);
      mouseStep++;
      rate = 5000;
    }
  }
}
/**********************************************************/

void mouseEventsStatic( int event, int x, int y, int flags, void* rrt_connect)
{
  static_cast<RRT_connect*>(rrt_connect)->mouseEvents(event,x,y);
}

bool sendPathTurtle(path_finding::Path::Request  &req, path_finding::Path::Response &res)
{
  int j = 0;
  for(int i=0;i<final_path.size();i++)
  {
    if(inverse)
      j = final_path.size()-1 - i;
    else
      j = i;
    geometry_msgs::Point32 tmp;
    tmp.x = final_path[j].x;
    tmp.y = final_path[j].y;
    res.path.push_back(tmp);
  }
  return true;
}

/*void sendPath()
{
    nav_msgs::Path path;
    int j = 0;
    for(int i=0;i<final_path.size();i++)
    {
        if(inverse)
        j = final_path.size()-1 - i;
        else
        j = i;
        //geometry_msgs::Point32 tmp;
        geometry_msgs::PoseStamped tmp;

        tmp.pose.position.x = final_path[j].x;
        tmp.pose.position.y = final_path[j].y;
        path.poses.push_back(tmp);

    }
    rrt_connect._pub_path.publish(path);
}*/




/*** MAIN ***/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrt_connect");
  ros::start();
  srand (time(NULL));

  ros::NodeHandle n;
  ros::ServiceServer serviceTurtle = n.advertiseService("pathTurtle", sendPathTurtle);

  Mat map,mapProcessing,tmp,white,wall,free,clean;

  namedWindow( "RRT connect", WINDOW_AUTOSIZE );
  //namedWindow( "free", WINDOW_AUTOSIZE );
  //namedWindow( "wall", WINDOW_AUTOSIZE );
  //namedWindow( "clean", WINDOW_AUTOSIZE );
  //namedWindow( "map processing", WINDOW_AUTOSIZE );

  if (argc >= 2){
      map = imread(argv[1], cv::IMREAD_COLOR);
      mapProcessing = imread(argv[1], cv::IMREAD_GRAYSCALE);

      white = cv::Mat::zeros(mapProcessing.size(), mapProcessing.type());
      white = Scalar(255);
      clean = cv::Mat::zeros(mapProcessing.size(), mapProcessing.type());

      threshold(mapProcessing,tmp, 0, 10, CV_THRESH_BINARY);
      Mat element5 = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
      Mat element10 = getStructuringElement(MORPH_ELLIPSE,Size(10,10));
      erode(tmp,tmp,element10);
      dilate(tmp,tmp,element5);
      wall = tmp.mul(white);
      //imshow("wall",wall);

      dilate(mapProcessing,free,element5);
      erode(free,free,element10);
      //imshow("free",free);

      free.copyTo(clean, wall);

      //imshow("clean",clean);
      clean.copyTo(mapProcessing);
      threshold(mapProcessing,mapProcessing, 220, 255, CV_THRESH_BINARY);
      //imshow("map processing",mapProcessing);

  }
  else{
    cout<<"ERROR need argv[1]: map.png for example"<<endl;
  }

  if(! map.data )
  {
    cout <<  "Could not open or find the image" << endl ;
    return -1;
  }

  namedWindow( "RRT connect", WINDOW_AUTOSIZE );
  startWindowThread();
  imshow( "RRT connect", map );

  waitKey(100);

  int step = 20;
  if(argc >= 3) step = atoi(argv[2]);

  int sizeMax = 2000;
  if(argc >= 4) sizeMax = atoi(argv[3]);

  RRT_connect rrt_connect(map,step,sizeMax,mapProcessing);
  rrt_connect._pub_path = n.advertise<nav_msgs::Path>("path",10);

  setMouseCallback("RRT connect", mouseEventsStatic, &rrt_connect);

  ros::Rate loop_rate(rate);
  while(ros::ok()){
    if(mouseStep == 2){
      Vertex *v1 = new Vertex(0,0);
      Vertex *v2 = new Vertex(0,0);
      bool r1 = rrt_connect.loop(rrt_connect._tree_root,v1);
      bool r2 = rrt_connect.loop(rrt_connect._tree_dest,v2);
      Vertex *v_tree = new Vertex(0,0);
      bool done1 = false;
      bool done2 = false;

      if(r1 ==  true)
      done1 = rrt_connect.trees_connectable(*v1,rrt_connect._tree_dest,v_tree);
      if(r2 ==  true && done1 == false)
      done2 = rrt_connect.trees_connectable(*v2,rrt_connect._tree_root,v_tree);
      if(done1 == true){
        vector<Vertex> path1;
        vector<Vertex> path;

        rrt_connect._tree_dest->findPath(&path1,v_tree);
        for(int i = path1.size()-1; i>=0;i--){
          path.push_back(path1[i]);
        }
        rrt_connect._tree_root->findPath(&path,v1);
        for(int i=0;i<path.size()-1;i++){
          line(map,Point(path[i]._y,path[i]._x),Point(path[i+1]._y,path[i+1]._x) ,CV_RGB( 100, 100, 100 ),2);
        }
        rrt_connect.smoothing(&path);
        for(int i=0;i<path.size()-1;i++){
          line(map,Point(path[i]._y,path[i]._x),Point(path[i+1]._y,path[i+1]._x) ,CV_RGB( 50, 250, 50 ),2);
        }
        final_path = rrt_connect.spline(path);
      }
      else if(done2 == true){
        inverse = true;
        vector<Vertex> path1;
        vector<Vertex> path;

        rrt_connect._tree_root->findPath(&path1,v_tree);
        for(int i = path1.size()-1; i>=0;i--){
          path.push_back(path1[i]);
        }
        rrt_connect._tree_dest->findPath(&path,v2);
        for(int i=0;i<path.size()-1;i++){
          line(map,Point(path[i]._y,path[i]._x),Point(path[i+1]._y,path[i+1]._x) ,CV_RGB( 100, 100, 100 ),2);
        }
        rrt_connect.smoothing(&path);
        for(int i=0;i<path.size()-1;i++){
          line(map,Point(path[i]._y,path[i]._x),Point(path[i+1]._y,path[i+1]._x) ,CV_RGB( 50, 250, 50 ),2);
        }
        final_path = rrt_connect.spline(path);
      }
    }
    imshow( "RRT connect", rrt_connect._map );
    ros::spinOnce();
    loop_rate.sleep();
    waitKey(5);
  }
  destroyWindow("RRT connect");
  return 0;
}
