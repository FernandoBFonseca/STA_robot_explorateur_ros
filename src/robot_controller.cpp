#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>

#define SGN(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

double radius = 0.03;                              //Wheel radius, in m
double wheelbase = 0.192;                          //Wheelbase, in m
double two_pi = 6.28319;
double speed_est_linear = 0.0;
double speed_est_turning = 0.0;
double position_est_steering = 0.0;
double speed_req1 = 0.0;
double speed_req2 = 0.0;
double speed_dt = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
ros::Time current_time;
ros::Time speed_time(0.0);

//Function to calculate turning speed with steering angle and velocity
double steering_angle_trans_vel_2_rot_vel(double v, double phi){
  
  return v*tan(phi)/wheelbase;
}

//Message callback function to read position and speed estimations
void handle_speed( const geometry_msgs::Vector3Stamped& speed) {
  speed_est_linear = 3.86*trunc(speed.vector.x*100)/100;
  ROS_INFO("speed linear : %f\n", speed_est_linear);
  position_est_steering = trunc(speed.vector.y*100)/100;
  ROS_INFO("pos steering turning : %f\n", position_est_steering);
  speed_est_turning = 0.842*steering_angle_trans_vel_2_rot_vel(speed_est_linear,position_est_steering);
  ROS_INFO("speed turning : %f\n", speed_est_turning);
  speed_dt = speed.vector.z;
  speed_time = speed.header.stamp;
}



int main(int argc, char** argv){

  //ROS setup
  ros::init(argc, argv, "controller");

  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("est_vel", 50, handle_speed);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;  
  
  double rate = 10.0;
  double linear_scale_positive = 1.0;
  double linear_scale_negative = 1.0;
  double angular_scale_positive = 1.0;
  double angular_scale_negative = 1.0;
  bool publish_tf = true;
  double dt = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dth = 0.0;
  double dxy = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  char base_link[] = "/base_link";
  char odom[] = "/odom";
  char plate_link[] = "/base_plate";
  char steering_link[] = "/steering";
  //char kinect[] = "/kinect";
  //char camera_link[] = "/camera_link";
  ros::Duration d(1.0);
  nh_private_.getParam("publish_rate", rate);
  nh_private_.getParam("publish_tf", publish_tf);
  nh_private_.getParam("linear_scale_positive", linear_scale_positive);
  nh_private_.getParam("linear_scale_negative", linear_scale_negative);
  nh_private_.getParam("angular_scale_positive", angular_scale_positive);
  nh_private_.getParam("angular_scale_negative", angular_scale_negative);

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    current_time = speed_time;
    dt = speed_dt;					//Time in s
    ROS_INFO("dt : %f", dt);
    dxy = (speed_est_linear)*dt;
    ROS_INFO("dxy : %f", dxy);
    dth = (speed_est_turning)*dt;

    if (dth > 0) dth *= angular_scale_positive;
    if (dth < 0) dth *= angular_scale_negative;
    if (dxy > 0) dxy *= linear_scale_positive;
    if (dxy < 0) dxy *= linear_scale_negative;

    //Calculation of Δx and Δy variation
    dx = cos(dth) * dxy;
    dy = sin(dth) * dxy;

    //Calculation new position 
    x_pos += (cos(theta) * dx - sin(theta) * dy);
    y_pos += (sin(theta) * dx + cos(theta) * dy);
    theta += dth;

    //check if abs(theta) > 2pi
    if(theta >= two_pi) theta -= two_pi;
    if(theta <= -two_pi) theta += two_pi;

    //Creating rotation quaternion for baselink and steering bar
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::Quaternion steering_quat = tf::createQuaternionMsgFromYaw(position_est_steering);

    if(publish_tf) {
      //Filling up transformations (/odom -> /base_link) and (/base_link -> /steering)
      geometry_msgs::TransformStamped t;
      geometry_msgs::TransformStamped s;

      
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      t.transform.translation.x = x_pos;
      t.transform.translation.y = y_pos;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom_quat;
      t.header.stamp = current_time;
      
      s.header.frame_id = plate_link;
      s.child_frame_id = steering_link;
      s.transform.translation.x = 0.192;
      s.transform.translation.y = 0.0;
      s.transform.translation.z = 0.013;
      s.transform.rotation = steering_quat;
      s.header.stamp = current_time;

      broadcaster.sendTransform(t);
      broadcaster.sendTransform(s);
    }

    //Filling up odom message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    if (speed_est_linear == 0 && speed_est_turning == 0){
      odom_msg.pose.covariance[0] = 1e-9;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 1e-9;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e-9;
      odom_msg.twist.covariance[0] = 1e-9;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 1e-9;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e-9;
    }
    else{
      odom_msg.pose.covariance[0] = 1e-3;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 0.0;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e3;
      odom_msg.twist.covariance[0] = 1e-3;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 0.0;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e3;
    }
    vx = (dt == 0)?  0 : speed_est_linear;
    vth = (dt == 0)? 0 : speed_est_turning;
    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = dth;
    
    odom_pub.publish(odom_msg);
    r.sleep();
  }
}