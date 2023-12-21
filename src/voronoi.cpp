#include <ros/ros.h>
#include "ros/console.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace cv;
using namespace std;

int mouseStep = 2;
float rate = 10;
Mat black;


struct mediatrice {
    Point2f l0;
    Point2f l1;
    Point2f p0;
    Point2f p1;
} ;

/**********************************************************/


/*** Voronoi ***/
class Voronoi
{
public:
    Voronoi(Mat map, Mat init);
    Mat _map,_init;
    Point* _root;
    Point* _dest;
    void mouseEvents( int event, int x, int y);
    vector<mediatrice > _lines;
    float distance(Point2f,Point2f);
    void findMediatrice(Point, Point);
    vector<Point> _points;
    void drawStraightLine(Point2f p1, Point2f p2, Scalar color);
    void intersection(int);
    long double slope(Point2f p1, Point2f p2);
    void resetMap();
    void removeLinesUseless();
    bool nearestPoint(mediatrice);
    float pDistance(Point2f p, mediatrice);
    bool belongsToSegment(mediatrice);

private:

};

Voronoi::Voronoi(Mat map, Mat init): _map(map), _init(init){}

void Voronoi::mouseEvents(int event, int x, int y)
{
    if( ( event == EVENT_LBUTTONDOWN ) ) {
        cout << "POINT: "<<x <<", " << y << endl;
        Point p = Point(x,y);
        _points.push_back(p);

        for(int i =0;i<_points.size()-1;i++)
        {
            findMediatrice(_points[i],p);
            intersection(_lines.size()-1);
            //imshow("Voronoi", _map);
            //removeLinesUseless();
        }
        //cout << "lines size: "<<_lines.size() <<endl<<endl;
        removeLinesUseless();
        resetMap();
    }
}

void Voronoi::resetMap()
{
    _map = _init.clone();
    for(int i=0;i<_points.size();i++)
    {
        circle( _map, _points[i], 6, Scalar(10,10,10), -1, 8, 0 );
    }
    for(int i=0;i<_lines.size();i++)
    {
        line(_map, _lines[i].l0, _lines[i].l1, CV_RGB( rand()%230, rand()%230, rand()%230 ), 3);
    }
    cout << "nb lines: "<<_lines.size()<<endl;
}

void Voronoi::findMediatrice(Point p1, Point p2)
{
    Point2f m1 = Point2f((p1.x+p2.x)/2,(p1.y+p2.y)/2);
    long double a = slope(p1,p2);
    if(a!=0)
        a = -1/a;
    //cout << "m1: "<<m1.x<<" "<< m1.y<< endl;
    double b = m1.y-a*m1.x;
    //cout << "a: "<<a<<" , b: "<<b<<endl;
    Point2f m2;
    if(p1.y == p2.y)
        m2 = Point2f(m1.x,m1.y-5);
    else if(p1.x == p2.x)
        m2 = Point2f(m1.x-5,m1.y);
    else
        m2 = Point2f(m1.x-2,a*(m1.x-2)+b);

    //cout << "m2: "<<m2.x<<" "<< m2.y<< endl<<endl;
    drawStraightLine(m1, m2, CV_RGB( 0, 0, 250 ));
    _lines[_lines.size()-1].p0 = p1;
    _lines[_lines.size()-1].p1 = p2;
}

void Voronoi::intersection(int idx)
{
    mediatrice m1 = _lines[idx];
    float a1 = (m1.l0.y-m1.l1.y)/(m1.l0.x-m1.l1.x);

    vector<Point2f> intersPoints;
    int size = _lines.size()-1;
    for(int i=0;i<size;i++)
    {
        mediatrice m2 = _lines[i];
        float a2 = (m2.l0.y-m2.l1.y)/(m2.l0.x-m2.l1.x);
        if(fabs(a2-a1) < 0.1)
            continue;

        Point2f x = m2.l0 - m1.l0;
        Point2f d1 = m1.l1 - m1.l0;
        Point2f d2 = m2.l1 - m2.l0;

        float cross = d1.x*d2.y - d1.y*d2.x;
        if (fabs(cross) < 1e-8)
            continue;

        double t1 = (x.x * d2.y - x.y * d2.x)/cross;
        Point2f r;
        r = m1.l0 + d1 * t1;

        float x1 = min(m1.l0.x,m1.l1.x);
        float x2 = max(m1.l0.x,m1.l1.x);
        float y1 = min(m1.l0.y,m1.l1.y);
        float y2 = max(m1.l0.y,m1.l1.y);

        if(r.x<x1 || x2<r.x || r.y<y1 || y2< r.y)
            continue;

        x1 = min(m2.l0.x,m2.l1.x);
        x2 = max(m2.l0.x,m2.l1.x);
        y1 = min(m2.l0.y,m2.l1.y);
        y2 = max(m2.l0.y,m2.l1.y);

        if(r.x<x1 || x2<r.x || r.y<y1 || y2< r.y)
            continue;

        intersPoints.push_back(r);
        //cout <<"p_inters: "<<r.x<<" , "<<r.y<<endl;

        // idx : ajouter les 2 demi-droites et supprimer la droite d'avant

        if(distance(r,_lines[idx].l1) > 5 && distance(r,_lines[idx].l0) > 5)
        {
            cout << "add 1"<<endl;
            mediatrice ltmp1;
            ltmp1.l0 = r;
            ltmp1.l1 = _lines[idx].l1;
            ltmp1.p0 = _lines[idx].p0;
            ltmp1.p1 = _lines[idx].p1;
            _lines[idx].l1 = r;
            _lines.push_back(ltmp1);
        }

        // i : ajouter les 2 demi-droites et supprimer la droite d'avant
        //cout << "r: "<<r.x<<" "<< r.y<< endl;
        //cout << "_lines[i].l1: "<<_lines[i].l1.x<<" "<< _lines[i].l1.y<< endl;
        if(distance(r,_lines[i].l1) > 5 && distance(r,_lines[i].l0) > 5)
        {
            cout << "add 2"<<endl;
            mediatrice ltmp2;
            ltmp2.l0 = r;
            ltmp2.l1 = _lines[i].l1;
            ltmp2.p0 = _lines[i].p0;
            ltmp2.p1 = _lines[i].p1;
            _lines[i].l1 = r;
            _lines.push_back(ltmp2);
        }

    }
    cout << "nb intersections: "<< intersPoints.size()<<endl;
    intersPoints.clear();
}

void Voronoi::removeLinesUseless()
{
    cout << "-------------------------"<<endl;
    cout << "REMOVE LINES"<<endl;
    vector<int> indexes;
    for(int i = 0;i<_lines.size();i++)
    {
        //float d = pDistance(_lines[i].p0, _lines[i]);
        bool r = belongsToSegment(_lines[i]);
        //cout << "d: "<<d<<endl;
        if(r == false) //(nearestPoint(_lines[i]))
        {
            cout << "i: "<<i<<endl;
            indexes.push_back(i);
        }
    }

    for(int i=indexes.size()-1;i>=0;i--)
    {
        cout<<i<<endl;
        _lines.erase(_lines.begin() + indexes[i]);
        cout << _lines.size() << endl;
    }
}

bool Voronoi::nearestPoint(mediatrice m)
{
    float dist = pDistance(m.p0,m);
    cout << "nearestPoint"<<endl;
    for(int i =0;i<_points.size();i++)
    {
        float d = pDistance(_points[i],m)+4;
        cout << "dist: "<< dist << " , d: "<<d<<endl;
        if(dist > d && d!=-1)
            return true;
    }
    return false;
}

bool Voronoi::belongsToSegment(mediatrice m)
{
    Point2f H = Point2f((m.p0.x+m.p1.x)/2,(m.p0.y+m.p1.y)/2);
    circle( _map, H, 6, Scalar(250,10,10), 3, 8, 0 );

    float x1 = min(m.l0.x,m.l1.x);
    float x2 = max(m.l0.x,m.l1.x);
    float y1 = min(m.l0.y,m.l1.y);
    float y2 = max(m.l0.y,m.l1.y);

    if(H.x<x1 || x2<H.x || H.y<y1 || y2< H.y)
        return false;
    else
        return true;

}

float Voronoi::pDistance(Point2f p, mediatrice m)
{
    Vec2f AB = Vec2f(m.l1.x-m.l0.x , m.l1.y-m.l0.y);
    float normAB = distance(m.l1,m.l0);
    cout << normAB <<endl;
    Vec2f AP = Vec2f(p.x-m.l0.x , p.y-m.l0.y);
    Vec2f AH = Vec2f((AP[0]*AB[0])/normAB , (AP[1]*AB[1])/normAB);

    Point2f H = Point2f(AH[0]+m.l0.x , AH[1]+m.l0.y);

    circle( _map, p, 6, Scalar(50,250,10), 3, 8, 0 );
    circle( _map, H, 6, Scalar(250,10,10), 3, 8, 0 );
    //imshow("Voronoi", _map);

    float x1 = min(m.l0.x,m.l1.x);
    float x2 = max(m.l0.x,m.l1.x);
    float y1 = min(m.l0.y,m.l1.y);
    float y2 = max(m.l0.y,m.l1.y);

    if(H.x<x1 || x2<H.x || H.y<y1 || y2< H.y)
        return -1;

    float normPH = distance(p,H);
    return normPH;
}

void Voronoi::drawStraightLine(Point2f p1, Point2f p2, Scalar color)
{
    Point2f p, q;
    if (p1.x != p2.x )
    {
        p.x = 0;
        q.x = _map.cols;
        float m = (p1.y - p2.y) / (p1.x - p2.x);
        float b = p1.y - (m * p1.x);
        p.y = m * p.x + b;
        q.y = m * q.x + b;
    }
    else
    {
        p.x = q.x = p2.x;
        p.y = 0;
        q.y = _map.rows;
    }

    //line(_map, p, q, color, 1);
    mediatrice l;
    l.l0 = p;
    l.l1 = q;
    _lines.push_back(l);
}

long double Voronoi::slope(Point2f p1, Point2f p2){
    return (double)(p2.y-p1.y)/(p2.x-p1.x);
}

float Voronoi::distance(Point2f p1,Point2f p2)
{
    float d = sqrt(pow(p1.x-p2.x , 2) + pow(p1.y-p2.y , 2));
    return d;
}


/**********************************************************/

void mouseEventsStatic( int event, int x, int y, int flags, void* voronoi)
{
    static_cast<Voronoi*>(voronoi)->mouseEvents(event,x,y);
}


/*** MAIN ***/


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Voronoi");
    ros::start();
    srand (time(NULL));

    Mat map,init;
    if (argc >= 2){
        map = imread(argv[1], IMREAD_COLOR);
        init = imread(argv[1], IMREAD_COLOR);
    }
    else{
        cout<<"ERROR need argv[1]: map.png for example"<<endl;
    }

    if(! map.data )
    {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }

    namedWindow( "Voronoi", WINDOW_AUTOSIZE);
    startWindowThread();
    waitKey(100);

    Voronoi voronoi(map,init);
    setMouseCallback("Voronoi", mouseEventsStatic, &voronoi);
    imshow("Voronoi", voronoi._map);
    ros::Rate loop_rate(rate);

    while(ros::ok()){
        imshow("Voronoi", voronoi._map);
        ros::spinOnce();
        loop_rate.sleep();
    }
    destroyWindow("Map window Voronoi");
    return 0;
}
