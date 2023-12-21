#include <ros/ros.h>
#include <vector>
#include <limits.h>
#include <boost/concept_check.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "path_finding/Path.h"
#include "nav_msgs/Odometry.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include <std_srvs/Empty.h>


using namespace std;
using namespace cv;

#define V 0.4 // m/S
#define Kt 0.05 // m
#define Kp 2.0
#define Kd 0.7

Mat mymap;


/*** ROBOT ***/
class Robot
{
public:
  Robot();
  turtlesim::Pose pose;
  vector<Point2f> path;
  int heigh_map;
  double p2m;
  bool reached;

private:
  ros::NodeHandle nh;
  ros::Publisher pub_cmd;
  ros::Subscriber sub_odom;
  void odomCallback(const turtlesim::Pose::ConstPtr&);
  double distance(Point2f p1, Point2f p2);
  int findClosest(double *);
  double curvature(int);
  double angle_traj(int);
  double prevTheta;

};

Robot::Robot()
{
  //sub_odom = nh.subscribe<nav_msgs::Odometry::ConstPtr>("odom", 1000, &Robot::odomCallback, this);
  sub_odom = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1000, &Robot::odomCallback, this);
  pub_cmd = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
  reached = false;
  prevTheta = 0;
}

//void Robot::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
void Robot::odomCallback(const turtlesim::Pose::ConstPtr& msg)
{

  if(pose.x !=0 ){
    Point p1 = Point(pose.x/p2m, heigh_map - pose.y/p2m);
    Point p2 = Point(msg->x/p2m, heigh_map - msg->y/p2m);
    line(mymap,p1,p2,CV_RGB( 250, 50, 50 ),2);
  }

  pose.x = msg->x;
  pose.y = msg->y;
  pose.theta = msg->theta;
  pose.linear_velocity = msg->linear_velocity;
  pose.angular_velocity = msg->angular_velocity;
  //cout << pose << endl;
  // GET CLOSEST POINT OF THE PATH TO THE ROBOT, s
  double d;
  int idx = findClosest(&d);

  if(idx<5) idx=5;
  // u2 = u1( c(s) - k1xd - k2xOe)
  // u2 = u1( -tan(Oe) - kxd)
  // u2 = u1(-c(s)xcos(Oe)/(1-dc(s)) - kxd)

  // C(s)
  double c = curvature(idx);
  if( isinf(c) ) c=0;

  double theta_traj = angle_traj(idx);

  //cout << "theta pose: "<< pose.theta<<endl;
 // cout << "theta traj: "<< theta_traj<<endl;

  double v1y = path[idx].y - pose.y;
  double v1x = path[idx].x - pose.x;
  double normeV1 = sqrt(v1x*v1x +v1y*v1y);
  v1x /= normeV1;
  v1y /= normeV1;

  double v2y = path[idx].y - path[idx-5].y;
  double v2x = path[idx].x - path[idx-5].x;
  double normeV2 = sqrt(v2x*v2x +v2y*v2y);
  v2x /= normeV2;
  v2y /= normeV2;

  double kt = Kt/d;
  if (kt > 1.0) kt=1;
  if (kt< 0.2) kt=0.2;
  double dy = ((1.0-kt)*v1y + kt*v2y);
  double dx = ((1.0-kt)*v1x + kt*v2x);

  double theta2 = atan2(dy,dx);

  //cout <<"theta2: "<<theta2<<endl;

  //double k1 = 1.0;
  double u1 = V;
  //double u2 =  u1*(c - k1*d - k2*theta_erreur);
  //double u2 = u1*(-k1*d - k2*tan(theta_erreur) );
  //double u2 = u1*(-k1*d*tan(theta_erreur) - k2*tan(theta_erreur) );
  //double u2 = u1*(-c*cos(theta_erreur)/(1-d*c) - k1*d);

  double theta = (theta2 - pose.theta);


  if(theta < -M_PI) theta += 2*M_PI;
  if(theta > M_PI) theta -= 2*M_PI;
  double u2 = Kp*theta + Kd*(theta - prevTheta);
  prevTheta = u2;

  if(idx == path.size()-1 )
  {
    if(!reached)
      cout << " destination reached "<<endl;
    u1 = 0;
    u2 = 0;
    reached = true;
  }
  else{
    cout << "d: "<<d << endl;
    //cout << "c: "<< c << endl;
    //cout << "theta erreur: "<<theta_erreur<<endl;
  }

  // PUBLISH CMD_VEL
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = u1 - fabs(u2)/10;
  twist_msg.angular.z = u2;
  pub_cmd.publish(twist_msg);
  if(!reached){
    cout << "linear x: "<<twist_msg.linear.x<<" , angular z: "<<twist_msg.angular.z<<endl;
    cout << "----------------------" <<endl;
  }
}

double Robot::angle_traj(int idx)
{
 // double theta = pose.theta;
  //cout << "theta : "<<theta<<endl;
   double r,x1,x2,y1,y2,xd1,yd1;
  x1 = path[idx].x;
  y1 = path[idx].y;
  x2 = path[idx-5].x;
  y2 = path[idx-5].y;

  xd1 = x1-x2;
  yd1 = y1-y2;

  double thetades = atan2(yd1,xd1);
  //cout << "theta desire: "<<thetades<<endl;
 // double erreur = theta - thetades;

  return thetades;
}

double Robot::curvature(int idx)
{
  if(idx < 2) return 0;
  double r,x1,x2,y1,y2,xd1,xd2,yd1,yd2,xdd,ydd;

  x1 = path[idx].x;
  y1 = path[idx].y;
  x2 = path[idx-1].x;
  y2 = path[idx-1].y;

  xd1 = x1-x2;
  yd1 = y1-y2;

  x1 = path[idx-1].x;
  y1 = path[idx-1].y;
  x2 = path[idx-2].x;
  y2 = path[idx-2].y;

  xd2 = x1-x2;
  yd2 = y1-y2;

  xdd = xd1-xd2;
  ydd = yd1-yd2;

  r = pow(xd1*xd1 + yd1*yd1 , 1.5) / (xd1*ydd - yd1*xdd);
  return 1.0/r;
}

int Robot::findClosest(double *d)
{
  double dmin = 10000.0;
  int idx = 0;
  for(int i=0; i<path.size();i++)
  {
    double dist = distance(path[i],Point2f(pose.x,pose.y));
    if(dist < dmin){
      dmin = dist;
      idx = i;
    }
  }
  *d = dmin;
  //cout << "idx: "<<idx<<endl;
  //cout << "dmin: "<<dmin<<endl;
  return idx;
}

double Robot::distance(Point2f p1,Point2f p2)
{
    double d = sqrt(pow(p1.x-p2.x , 2) + pow(p1.y-p2.y , 2));
    return d;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suivi_chemin");
  ros::start();

  ros::Duration(2.0).sleep();

  ros::NodeHandle n;


  if (argc >= 2){
        mymap = imread(argv[1], IMREAD_COLOR);
  }
  else{
      cout<<"ERROR need argv[1]: map.png for example"<<endl;
  }

  if(! mymap.data )
  {
      cout <<  "Could not open or find the image" << endl ;
      return -1;
  }

  namedWindow("path following", WINDOW_AUTOSIZE );
  startWindowThread();
  imshow("path following", mymap );
  waitKey(100);

  ros::service::waitForService("path");
  ros::ServiceClient client = n.serviceClient<path_finding::Path>("path");
  path_finding::Path srv;
  while(ros::ok() &&  (long int)srv.response.path.size() == 0)
  {
    client.call(srv);
  }
  Robot robot;

  robot.heigh_map = mymap.rows;
  robot.p2m = 11.0/robot.heigh_map;

  std_srvs::Empty empty;

  if (client.call(srv))
  {
    ROS_INFO("client: %ld", (long int)srv.response.path.size());
    for(int i=srv.response.path.size()-1;i>=0;i--)
    {
      Point2f p;
      p.x = srv.response.path[i].x *robot.p2m;
      p.y = (robot.heigh_map - srv.response.path[i].y) *robot.p2m;
      robot.path.push_back(p);

      if(i == srv.response.path.size()-1)
      {
        turtlesim::TeleportAbsolute srv_teleport;
        srv_teleport.request.x = p.x;
        srv_teleport.request.y = p.y;
        srv_teleport.request.theta = 1.8;
        cout<<"x:"<<p.x<<"y:"<<p.y<<endl;
        ros::service::call("/turtle1/teleport_absolute",srv_teleport);

        ros::service::call("/clear",empty);
        //ros::Duration(1.0).sleep();
      }
      if(robot.path.size()>1){
	         Point p1 = Point(srv.response.path[i].x,srv.response.path[i].y);
           Point p2 = Point(srv.response.path[i+1].x,srv.response.path[i+1].y);
           line(mymap,p1,p2,CV_RGB( 50, 250, 50 ),2);
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call service path");
    return 1;
  }


  ros::Rate loop_rate(100);
  while(ros::ok()){
      imshow("path following", mymap );
      ros::spinOnce();
      loop_rate.sleep();
      waitKey(10);
  }
  return 0;
}
