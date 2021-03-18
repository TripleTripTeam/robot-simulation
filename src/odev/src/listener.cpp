#include "ros/ros.h"
#include "std_msgs/String.h"

#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stdlib.h>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

#include <dynamic_reconfigure/server.h>

#include "odev/floatStamped.h"
#include "odev/sourcedOdom.h"
#include "odev/dyn_paramConfig.h"



using namespace message_filters;

//const double PI=3.1415926535897932;

int count = 0;
double axleLength =0.335; //distance between two wheel (meter)
double Wheelbase = 0.35; //distance from front to rear wheels (meter)

class pub_sub
{

  private:
  double x = 0.0;
  double y = 0.0;
  double steer_angle = 0.0;
  double theta = 0;

  double Ts;

  double linear_speed;
  double angular_speed;

  double time_left_wheel_old;
  double time_left_wheel_now;

  double velocityLeft = 0;
  double velocityRight = 0;


  bool diffdrive = false;

  ros::NodeHandle n;    

  ros::Publisher odom_pub;
  ros::Publisher custom_odom_pub;
  
  message_filters::Subscriber<odev::floatStamped> L_speed;
  message_filters::Subscriber<odev::floatStamped> R_speed;
  message_filters::Subscriber<sensor_msgs::Imu> IMU;
  
  
  dynamic_reconfigure::Server<odev::dyn_paramConfig> server;
  dynamic_reconfigure::Server<odev::dyn_paramConfig>::CallbackType f;
  
  tf::TransformBroadcaster odom_broadcaster;

  typedef sync_policies::ApproximateTime <odev::floatStamped, odev::floatStamped, sensor_msgs::Imu> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;
  

  public:

    pub_sub()
    {
      f = boost::bind(&pub_sub::dynamic_callback, this, _1, _2);
      server.setCallback(f);

      L_speed.subscribe(n, "speedL_stamped", 1);
      R_speed.subscribe(n,"speedR_stamped", 1);
      IMU.subscribe(n, "imu/data", 1);
      odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
      custom_odom_pub = n.advertise<odev::sourcedOdom>("custom_odom", 50);
  

      sync.reset(new Sync(MySyncPolicy(10), L_speed, R_speed, IMU));
      sync->registerCallback(boost::bind(&pub_sub::callback, this, _1, _2, _3));

    }

    void diffdrive_kinematic_forward(double right_wheel_speed, double left_wheel_speed){

      count=count+1;

      if (count == 1){
            Ts = time_left_wheel_now - time_left_wheel_now; 
        }
          else{
            Ts = time_left_wheel_now - time_left_wheel_old;
          }  
        
          linear_speed = (right_wheel_speed+left_wheel_speed)/2.0;
          angular_speed = (right_wheel_speed-left_wheel_speed)/axleLength;   
    }

 void ackerman_kinematic_forward(double right_wheel_speed, double left_wheel_speed, double angular_velocity ){

      count=count+1;

      if (count == 1){
            Ts = time_left_wheel_now - time_left_wheel_now; 
        }
          else{
            Ts = time_left_wheel_now - time_left_wheel_old;
          }  
        
          linear_speed = (right_wheel_speed+left_wheel_speed)/2.0;
          angular_speed = angular_velocity; //(linear_speed*tan(steer_angle))/Wheelbase; - old formula  
    }

  void dynamic_callback(odev::dyn_paramConfig &config, uint32_t level){
  
    diffdrive=config.bool_param;
    x= config.double_param_x;
    y= config.double_param_y;
    ROS_INFO("diffdrive is %d", config.bool_param);
  }

  void callback(const odev::floatStamped::ConstPtr& msg1, const odev::floatStamped::ConstPtr& msg2, const sensor_msgs::Imu::ConstPtr& msg3)
  {
    

    velocityLeft = msg1->data;
    velocityRight = msg2->data;
    angular_speed = (msg3->angular_velocity.z);

    time_left_wheel_now = msg1->header.stamp.toSec();
    
    ackerman_kinematic_forward(velocityRight, velocityLeft, angular_speed);
    
    x = x + linear_speed*Ts*cos(theta+((angular_speed*Ts)/2));
    y = y + linear_speed*Ts*sin(theta+((angular_speed*Ts)/2));  
   
    ROS_INFO ("[counter = %d] x_new (%lf) y_new (%lf) Ts (%lf) ",count, x , y, Ts);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

/*
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg1->header.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    //odom_broadcaster.sendTransform(odom_trans.transform, ros::Time::now(), "world", "turtle"));
    //next, we'll publish the odometry message over ROS
*/    
    nav_msgs::Odometry odom;
    
    odom.header.stamp = msg1->header.stamp;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = linear_speed*cos(theta);
    odom.twist.twist.linear.y = linear_speed*sin(theta);
    odom.twist.twist.angular.z = angular_speed;


    
    //publish the message
    odom_pub.publish(odom);

    customOdomPublis(odom);
    
    theta = theta + (angular_speed*Ts); 
    time_left_wheel_old = time_left_wheel_now;
  }
   
   
  void customOdomPublis(nav_msgs::Odometry custOdom){

    odev::sourcedOdom customOdomMsg;

    customOdomMsg.header = custOdom.header;
    customOdomMsg.child_frame_id = custOdom.child_frame_id;
    customOdomMsg.pose = custOdom.pose;
    customOdomMsg.twist = custOdom.twist;
    
    if (diffdrive){
      diffdrive_kinematic_forward(velocityRight, velocityLeft);
      customOdomMsg.source = "diffdrive";
    }
    else
    {
      ackerman_kinematic_forward(velocityRight, velocityLeft, angular_speed);
      customOdomMsg.source = "ackerman";
    }

  custom_odom_pub.publish(customOdomMsg); 
  }

};



int main(int argc, char **argv){
  	
	ros::init(argc, argv, "listener");

  pub_sub my_pub_sub;

  ros::spin();

  return 0;
}


