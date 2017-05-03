/*
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    msg->header.stamp = ros::Time::now().toSec();
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
*/
/*#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

struct transform{
    double x;
    double y;
    double z;
};
struct transform answ;

// tube parameters
float bend_radius_tube = 0.15;
float tube_diameter = 0.025;
double num_points_1 = 100;
double num_points_2 = 200;
double num_points_3 = 100;
double num_points_4 = 40;
double dist_4 = 0.04;
unsigned int num_points = 30000;

// water workspace parameters
double wws_radius = 0.02;
double wws_points_1 = 50;
double wws_points_2 = 50;
unsigned int wws_num_points = 3000;

// ROS publishers and messages
ros::Publisher tube_obj_pub, wws_pub;
geometry_msgs::TransformStamped tube_obj, wws;
sensor_msgs::PointCloud tube_obj_cloud, wws_cloud;

void transform_rotation(double angle, double x_point, double y_point){
    answ.x = (cos(angle)*x_point);
    answ.y = y_point;
    answ.z = (sin(-angle)*x_point);
}

void tubeDataGeneration(){
    int tube_count_1 = 0;
    int tube_count_2 = 0;

    tube_obj_cloud.header.stamp = ros::Time::now();
    tube_obj_cloud.header.frame_id = "tube_surface";

    tube_obj_cloud.points.resize(num_points);

    tube_obj_cloud.channels.resize(1);
    tube_obj_cloud.channels[0].name = "intensities";
    tube_obj_cloud.channels[0].values.resize(num_points);

    for (uint32_t i = 0; i < num_points_1; ++i){
        tube_obj_cloud.points[i + (tube_count_1*num_points_2)].x = 0;
        tube_obj_cloud.points[i + (tube_count_1*num_points_2)].y = (tube_diameter*cos(i / num_points_1 * 2 * M_PI)) + bend_radius_tube;
        tube_obj_cloud.points[i + (tube_count_1*num_points_2)].z = tube_diameter*sin(i / num_points_1 * 2 * M_PI);

        for (uint32_t j = 0; j < num_points_2; ++j){
            double angle = (j / num_points_2 * 0.5 * M_PI);
            transform_rotation(angle, tube_obj_cloud.points[i + (tube_count_1*num_points_2)].y, tube_obj_cloud.points[i + (tube_count_1*num_points_2)].z);
            tube_obj_cloud.points[(i + (tube_count_1*num_points_2))+j].y = answ.x;
            tube_obj_cloud.points[(i + (tube_count_1*num_points_2))+j].x = answ.z;
            tube_obj_cloud.points[(i + (tube_count_1*num_points_2))+j].z = answ.y;

        }
        tube_count_1 = tube_count_1 + 1;
    }
    for (uint32_t k = 0; k < num_points_3; ++k){
        tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)].x = 0;
        tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)].y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
        tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)].z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);

        for (uint32_t l = 0; l < num_points_4; ++l){
            double distance = ((l / num_points_4) * dist_4);
            tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)+l].x = distance;
            tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)+l].y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
            tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)+l].z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);
        }
        tube_count_2 = tube_count_2 + 1;
    }
}

void waterWorkspaceDataGeneration(){
    wws_cloud.header.stamp = ros::Time::now();
    wws_cloud.header.frame_id = "water_workspace";

    wws_cloud.points.resize(wws_num_points);

    wws_cloud.channels.resize(1);
    wws_cloud.channels[0].name = "intensities";
    wws_cloud.channels[0].values.resize(wws_num_points);

    int wws_count = 0;

    for (uint32_t i = 0; i < wws_points_1; ++i){
        wws_cloud.points[i + (wws_count*wws_points_2)].x = 0;
        wws_cloud.points[i + (wws_count*wws_points_2)].y = wws_radius*cos(i / wws_points_1 * 2 * M_PI);
        wws_cloud.points[i + (wws_count*wws_points_2)].z = wws_radius*sin(i / wws_points_1 * 2 * M_PI);

        for (uint32_t j = 0; j < wws_points_2; ++j){
            double angle = (j / wws_points_2 * M_PI);
            transform_rotation(angle, wws_cloud.points[i + (wws_count*wws_points_2)].y, wws_cloud.points[i + (wws_count*wws_points_2)].z);
            wws_cloud.points[(i + (wws_count*wws_points_2))+j].y = answ.x;
            wws_cloud.points[(i + (wws_count*wws_points_2))+j].x = answ.z;
            wws_cloud.points[(i + (wws_count*wws_points_2))+j].z = answ.y;

            wws_cloud.channels[0].values[(i + (wws_count*wws_points_2))+j] = 20;
        }
        wws_count = wws_count + 1;
    }
}

void tf_Transform(geometry_msgs::TransformStamped msg){
    static tf::TransformBroadcaster transformer;
    transformer.sendTransform(msg);
}

void tube_Transform(){
    tube_obj.header.stamp = ros::Time::now();
    tube_obj.header.frame_id = "map";
    tube_obj.child_frame_id = "tube_surface_frame";
    tube_obj.transform.translation.x = 0.0;
    tube_obj.transform.translation.y = -bend_radius_tube;
    tube_obj.transform.translation.z = 0.0;
    tube_obj.transform.rotation.x = 0.0;
    tube_obj.transform.rotation.y = 0.0;
    tube_obj.transform.rotation.z = 0.0;
    tube_obj.transform.rotation.w = 1.0;
    tube_obj_pub.publish(tube_obj);
    tf_Transform(tube_obj);
}

void water_workspace_Transform(){
    wws.header.stamp = ros::Time::now();
    wws.header.frame_id = "nozzle_frame";
    wws.child_frame_id = "wws_frame";
    wws.transform.translation.x = 0.18;
    wws.transform.translation.y = 0.0;
    wws.transform.translation.z = 0.0;
    wws.transform.rotation.x = 0.0;
    wws.transform.rotation.y = 0.0;
    wws.transform.rotation.z = 0.0;
    wws.transform.rotation.w = 1.0;
    wws_pub.publish(wws);
    tf_Transform(wws);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "Visualisation_objects_in_space");

  ros::NodeHandle nh;
  ros::Publisher cloud_pub_surface = nh.advertise<sensor_msgs::PointCloud>("/tube_surface_cloud", 10);
  ros::Publisher cloud_pub_wws = nh.advertise<sensor_msgs::PointCloud>("/water_wss_cloud", 10);
  tube_obj_pub = nh.advertise<geometry_msgs::TransformStamped>("/tube_frame_geometry_transform",1);
  wws_pub = nh.advertise<geometry_msgs::TransformStamped>("/water_workspace_frame_geometry_transform",1);

  tubeDataGeneration();
  waterWorkspaceDataGeneration();
  tube_Transform();
  water_workspace_Transform();
  cloud_pub_surface.publish(tube_obj);
  cloud_pub_wws.publish(wws_cloud);

  ros::Rate r(10.0);
  while(ros::ok()){
      tube_obj_cloud.header.stamp = ros::Time::now();
      cloud_pub_surface.publish(tube_obj);

      wws_cloud.header.stamp = ros::Time::now();
      cloud_pub_wws.publish(wws_cloud);

      tube_Transform();
      water_workspace_Transform();
      r.sleep();

  }
}*/

//================================

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

struct transform{
    double x;
    double y;
    double z;
};
struct transform answ;

// tube parameters
float bend_radius_tube = 0.15;
float tube_diameter = 0.025;
double num_points_1 = 100;
double num_points_2 = 200;
double num_points_3 = 100;
double num_points_4 = 40;
double dist_4 = 0.04;
unsigned int num_points = 30000;

// water workspace parameters
double wws_radius = 0.02;
double wws_points_1 = 50;
double wws_points_2 = 50;
unsigned int wws_num_points = 3000;

// ROS publishers and messages
ros::Publisher tube_obj_pub, wws_pub;
geometry_msgs::TransformStamped tube_obj, wws;
sensor_msgs::PointCloud tube_obj_cloud, wws_cloud;

//new
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void transform_rotation(double angle, double x_point, double y_point){
    answ.x = (cos(angle)*x_point);
    answ.y = y_point;
    answ.z = (sin(-angle)*x_point);
}

void tubeDataGeneration(){
    int tube_count_1 = 0;
    int tube_count_2 = 0;

    tube_obj_cloud.header.stamp = ros::Time::now();
    tube_obj_cloud.header.frame_id = "tube_surface";

    tube_obj_cloud.points.resize(num_points);

    tube_obj_cloud.channels.resize(1);
    tube_obj_cloud.channels[0].name = "intensities";
    tube_obj_cloud.channels[0].values.resize(num_points);

    for (uint32_t i = 0; i < num_points_1; ++i){
        tube_obj_cloud.points[i + (tube_count_1*num_points_2)].x = 0;
        tube_obj_cloud.points[i + (tube_count_1*num_points_2)].y = (tube_diameter*cos(i / num_points_1 * 2 * M_PI)) + bend_radius_tube;
        tube_obj_cloud.points[i + (tube_count_1*num_points_2)].z = tube_diameter*sin(i / num_points_1 * 2 * M_PI);

        for (uint32_t j = 0; j < num_points_2; ++j){
            double angle = (j / num_points_2 * 0.5 * M_PI);
            transform_rotation(angle, tube_obj_cloud.points[i + (tube_count_1*num_points_2)].y, tube_obj_cloud.points[i + (tube_count_1*num_points_2)].z);
            tube_obj_cloud.points[(i + (tube_count_1*num_points_2))+j].y = answ.x;
            tube_obj_cloud.points[(i + (tube_count_1*num_points_2))+j].x = answ.z;
            tube_obj_cloud.points[(i + (tube_count_1*num_points_2))+j].z = answ.y;

        }
        tube_count_1 = tube_count_1 + 1;
    }
    for (uint32_t k = 0; k < num_points_3; ++k){
        tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)].x = 0;
        tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)].y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
        tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)].z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);

        for (uint32_t l = 0; l < num_points_4; ++l){
            double distance = ((l / num_points_4) * dist_4);
            tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)+l].x = distance;
            tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)+l].y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
            tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)+l].z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);
        }
        tube_count_2 = tube_count_2 + 1;
    }
}

void waterWorkspaceDataGeneration(){
    wws_cloud.header.stamp = ros::Time::now();
    wws_cloud.header.frame_id = "water_workspace";

    wws_cloud.points.resize(wws_num_points);

    wws_cloud.channels.resize(1);
    wws_cloud.channels[0].name = "intensities";
    wws_cloud.channels[0].values.resize(wws_num_points);

    int wws_count = 0;

    for (uint32_t i = 0; i < wws_points_1; ++i){
        wws_cloud.points[i + (wws_count*wws_points_2)].x = 0;
        wws_cloud.points[i + (wws_count*wws_points_2)].y = wws_radius*cos(i / wws_points_1 * 2 * M_PI);
        wws_cloud.points[i + (wws_count*wws_points_2)].z = wws_radius*sin(i / wws_points_1 * 2 * M_PI);

        for (uint32_t j = 0; j < wws_points_2; ++j){
            double angle = (j / wws_points_2 * M_PI);
            transform_rotation(angle, wws_cloud.points[i + (wws_count*wws_points_2)].y, wws_cloud.points[i + (wws_count*wws_points_2)].z);
            wws_cloud.points[(i + (wws_count*wws_points_2))+j].y = answ.x;
            wws_cloud.points[(i + (wws_count*wws_points_2))+j].x = answ.z;
            wws_cloud.points[(i + (wws_count*wws_points_2))+j].z = answ.y;

            wws_cloud.channels[0].values[(i + (wws_count*wws_points_2))+j] = 20;
        }
        wws_count = wws_count + 1;
    }
}

void tf_Transform(geometry_msgs::TransformStamped msg){
    static tf::TransformBroadcaster transformer;
    transformer.sendTransform(msg);
}

void tube_Transform(){
    tube_obj.header.stamp = ros::Time::now();
    tube_obj.header.frame_id = "map";
    tube_obj.child_frame_id = "tube_surface_frame";
    tube_obj.transform.translation.x = 0.0;
    tube_obj.transform.translation.y = -bend_radius_tube;
    tube_obj.transform.translation.z = 0.0;
    tube_obj.transform.rotation.x = 0.0;
    tube_obj.transform.rotation.y = 0.0;
    tube_obj.transform.rotation.z = 0.0;
    tube_obj.transform.rotation.w = 1.0;
    tube_obj_pub.publish(tube_obj);
    tf_Transform(tube_obj);
}

void water_workspace_Transform(){
    wws.header.stamp = ros::Time::now();
    wws.header.frame_id = "nozzle_frame";
    wws.child_frame_id = "wws_frame";
    wws.transform.translation.x = 0.18;
    wws.transform.translation.y = 0.0;
    wws.transform.translation.z = 0.0;
    wws.transform.rotation.x = 0.0;
    wws.transform.rotation.y = 0.0;
    wws.transform.rotation.z = 0.0;
    wws.transform.rotation.w = 1.0;
    wws_pub.publish(wws);
    tf_Transform(wws);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "Visualisation_objects_in_space");

  ros::NodeHandle nh;
  //new
  ros::Publisher pub = nh.advertise<PointCloud>("point_col", 1);

  ros::Publisher cloud_pub_surface = nh.advertise<sensor_msgs::PointCloud>("/tube_surface_cloud", 10);
  ros::Publisher cloud_pub_wws = nh.advertise<sensor_msgs::PointCloud>("/water_wss_cloud", 10);
  tube_obj_pub = nh.advertise<geometry_msgs::TransformStamped>("/tube_frame_geometry_transform",1);
  wws_pub = nh.advertise<geometry_msgs::TransformStamped>("/water_workspace_frame_geometry_transform",1);

  tubeDataGeneration();
  waterWorkspaceDataGeneration();
  tube_Transform();
  water_workspace_Transform();
  cloud_pub_surface.publish(tube_obj_cloud);
  cloud_pub_wws.publish(wws_cloud);

  //new

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  //new

  ros::Rate r(10.0);
  while(ros::ok()){

      //new
      msg->header.stamp = ros::Time::now().toSec();
      pub.publish (msg);
      //

      tube_obj_cloud.header.stamp = ros::Time::now();
      cloud_pub_surface.publish(tube_obj_cloud);

      wws_cloud.header.stamp = ros::Time::now();
      cloud_pub_wws.publish(wws_cloud);

      tube_Transform();
      water_workspace_Transform();
      r.sleep();

  }
}
