/*
//trail code to use pointCloud2


#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
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

struct init_point{
    double x;
    double y;
    double z;
};
struct init_point data_gen;

// tube parameters
float bend_radius_tube = 0.15;
float tube_diameter = 0.025;
double num_points_1 = 100;
double num_points_2 = 200;
double num_points_3 = 100;
double num_points_4 = 40;
double dist_4 = 0.04;
unsigned int tube_num_points = 30000;

// water workspace parameters
double wws_radius = 0.02;
double wws_points_1 = 50;
double wws_points_2 = 50;
unsigned int wws_num_points = 3000;

// ROS publishers and messages
ros::Publisher tube_obj_pub, wws_pub;
geometry_msgs::TransformStamped tube_obj, wws;
sensor_msgs::PointCloud2 tube_obj_cloud, wws_cloud;


void transform_rotation(double angle, double x_point, double y_point){
    answ.x = (cos(angle)*x_point);
    answ.y = y_point;
    answ.z = (sin(-angle)*x_point);
}

void tubeDataGeneration(){

    tube_obj_cloud.header.stamp = ros::Time::now();
    tube_obj_cloud.header.frame_id = "tube_surface";

    tube_obj_cloud.height = 1;
    tube_obj_cloud.width = tube_num_points;

    sensor_msgs::PointCloud2Iterator<float> iter_x(tube_obj_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(tube_obj_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(tube_obj_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(tube_obj_cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(tube_obj_cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(tube_obj_cloud, "b");

    for (uint32_t i = 0; i < num_points_1; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b){
        *iter_x = data_gen.x = 0;
        *iter_y = data_gen.y = (tube_diameter*cos(i / num_points_1 * 2 * M_PI)) + bend_radius_tube;
        *iter_z = data_gen.z = tube_diameter*sin(i / num_points_1 * 2 * M_PI);
        *iter_r = 0;
        *iter_g = 255;
        *iter_b = 0;

        for (uint32_t j = 0; j < num_points_2; ++j, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b){
            double angle = (j / num_points_2 * 0.5 * M_PI);
            transform_rotation(angle, data_gen.y, data_gen.z);
            *iter_x = answ.x;
            *iter_y = answ.z;
            *iter_z = answ.y;
            *iter_r = 0;
            *iter_g = 255;
            *iter_b = 0;
        }
    }
    for (uint32_t k = 0; k < num_points_3; ++k, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b){
        *iter_x = 0;
        *iter_y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
        *iter_z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);
        *iter_r = 0;
        *iter_g = 255;
        *iter_b = 0;

        for (uint32_t l = 0; l < num_points_4; ++l, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b){
            double distance = ((l / num_points_4) * dist_4);
            *iter_x = distance;
            *iter_y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
            *iter_z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);
            *iter_r = 0;
            *iter_g = 255;
            *iter_b = 0;
        }
    }
}

void waterWorkspaceDataGeneration(){
    wws_cloud.header.stamp = ros::Time::now();
    wws_cloud.header.frame_id = "water_workspace";

    wws_cloud.height = 1;
    wws_cloud.width = wws_num_points;

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
}
*/

// original code --> working

#include <ros/ros.h>
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
double tube_dist_offset = 0.04;
unsigned int tube_num_points = 30000;

// water workspace parameters
double wws_radius = 0.02;
double wws_points_1 = 50;
double wws_points_2 = 50;
unsigned int wws_num_points = 3000;

// nozzle workspace parameters
double nozzle_radius = 0.015;
double nozzle_length = 0.07;
double nozzle_points_t_1 = 50;
double nozzle_points_t_2 = 100;
double nozzle_points_b_1 = 50;
double nozzle_points_b_2 = 50;
double nozzle_points_f_1 = 50;
double nozzle_points_f_2 = 50;
unsigned int nozzle_num_points = 11000;

// ROS publishers and messages
ros::Publisher tube_obj_pub, wws_pub, nozzle_pub;
geometry_msgs::TransformStamped tube_obj, wws, nozzle;
sensor_msgs::PointCloud tube_obj_cloud, wws_cloud, nozzle_cloud;

void transform_rotation(double angle, double x_point, double y_point){
    answ.x = (cos(angle)*x_point);
    answ.y = y_point;
    answ.z = (sin(-angle)*x_point);
}

void tubeDataGeneration(){
    int tube_count_1 = 0;
    int tube_count_2 = 0;

    tube_obj_cloud.header.stamp = ros::Time::now();
    tube_obj_cloud.header.frame_id = "tube_surface_frame";
    tube_obj_cloud.points.resize(tube_num_points);
    tube_obj_cloud.channels.resize(2);
    tube_obj_cloud.channels[0].name = "intensities";
    tube_obj_cloud.channels[0].values.resize(tube_num_points);  
    tube_obj_cloud.channels[1].name = "AxisColor";
    tube_obj_cloud.channels[1].values.resize(tube_num_points);

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

            tube_obj_cloud.channels[1].values[(i + (tube_count_1*num_points_2))+j] = 200;

        }
        tube_count_1 = tube_count_1 + 1;
    }
    for (uint32_t k = 0; k < num_points_3; ++k){
        tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)].x = 0;
        tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)].y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
        tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)].z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);

        for (uint32_t l = 0; l < num_points_4; ++l){
            double distance = ((l / num_points_4) * tube_dist_offset);
            tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)+l].x = distance;
            tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)+l].y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
            tube_obj_cloud.points[k + ((tube_count_1*num_points_2)+(tube_count_2*num_points_4)+num_points_1)+l].z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);
        }
        tube_count_2 = tube_count_2 + 1;
    }
}

void waterWorkspaceDataGeneration(){
    int wws_count = 0;

    wws_cloud.header.stamp = ros::Time::now();
    wws_cloud.header.frame_id = "wws_frame";
    wws_cloud.points.resize(wws_num_points);
    wws_cloud.channels.resize(1);
    wws_cloud.channels[0].name = "intensities";
    wws_cloud.channels[0].values.resize(wws_num_points);

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

void nozzleWorkspaceDataGeneration(){
    int nozzle_count_1 = 0;
    int nozzle_count_2 = 0;
    int nozzle_count_3 = 0;

    nozzle_cloud.header.stamp = ros::Time::now();
    nozzle_cloud.header.frame_id = "nozzle_frame";
    nozzle_cloud.points.resize(nozzle_num_points);
    nozzle_cloud.channels.resize(1);
    nozzle_cloud.channels[0].name = "intensities";
    nozzle_cloud.channels[0].values.resize(nozzle_num_points);

    for (uint32_t i = 0; i < nozzle_points_t_1; ++i){
        nozzle_cloud.points[i + (nozzle_count_1*nozzle_points_t_2)].x = 0;
        nozzle_cloud.points[i + (nozzle_count_1*nozzle_points_t_2)].y = nozzle_radius*cos(i / nozzle_points_t_1 * 2 * M_PI);
        nozzle_cloud.points[i + (nozzle_count_1*nozzle_points_t_2)].z = nozzle_radius*sin(i / nozzle_points_t_1 * 2 * M_PI);

        for (uint32_t j = 0; j < nozzle_points_t_2; ++j){
            double distance = ((j / nozzle_points_t_2) * nozzle_length);
            nozzle_cloud.points[(i + (nozzle_count_1*nozzle_points_t_2))+j].x = distance;
            nozzle_cloud.points[(i + (nozzle_count_1*nozzle_points_t_2))+j].y = nozzle_radius*cos(i / nozzle_points_t_1 * 2 * M_PI);
            nozzle_cloud.points[(i + (nozzle_count_1*nozzle_points_t_2))+j].z = nozzle_radius*sin(i / nozzle_points_t_1 * 2 * M_PI);

            //nozzle_cloud.channels[0].values[(i + (nozzle_count_1*nozzle_points_t_2))+j] = 20;
        }
        nozzle_count_1 = nozzle_count_1 + 1;
    }
    for (uint32_t k = 0; k < nozzle_points_b_1; ++k){
        nozzle_cloud.points[k + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+nozzle_points_t_1)].x = 0;
        nozzle_cloud.points[k + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+nozzle_points_t_1)].y = nozzle_radius*cos(k / nozzle_points_b_1 * 2 * M_PI);
        nozzle_cloud.points[k + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+nozzle_points_t_1)].z = nozzle_radius*sin(k / nozzle_points_b_1 * 2 * M_PI);

        for (uint32_t l = 0; l < nozzle_points_b_2; ++l){
            double offset = ((l / nozzle_points_b_2) * nozzle_radius);
            nozzle_cloud.points[k + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+nozzle_points_t_1)+l].x = 0;
            nozzle_cloud.points[k + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+nozzle_points_t_1)+l].y = offset*cos(k / nozzle_points_b_1 * 2 * M_PI);
            nozzle_cloud.points[k + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+nozzle_points_t_1)+l].z = offset*sin(k / nozzle_points_b_1 * 2 * M_PI);
        }
        nozzle_count_2 = nozzle_count_2 + 1;
    }
    for (uint32_t m = 0; m < nozzle_points_f_1; ++m){
        nozzle_cloud.points[m + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+(nozzle_count_3*nozzle_points_f_2)+nozzle_points_t_1+nozzle_points_b_1)].x = nozzle_length;
        nozzle_cloud.points[m + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+(nozzle_count_3*nozzle_points_f_2)+nozzle_points_t_1+nozzle_points_b_1)].y = nozzle_radius*cos(m / nozzle_points_f_1 * 2 * M_PI);
        nozzle_cloud.points[m + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+(nozzle_count_3*nozzle_points_f_2)+nozzle_points_t_1+nozzle_points_b_1)].z = nozzle_radius*sin(m / nozzle_points_f_1 * 2 * M_PI);

        for (uint32_t n = 0; n < nozzle_points_f_2; ++n){
            double offset = ((n / nozzle_points_f_2) * nozzle_radius);
            nozzle_cloud.points[m + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+(nozzle_count_3*nozzle_points_f_2)+nozzle_points_t_1+nozzle_points_b_1)+n].x = nozzle_length;
            nozzle_cloud.points[m + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+(nozzle_count_3*nozzle_points_f_2)+nozzle_points_t_1+nozzle_points_b_1)+n].y = offset*cos(m / nozzle_points_f_1 * 2 * M_PI);
            nozzle_cloud.points[m + ((nozzle_count_1*nozzle_points_t_2)+(nozzle_count_2*nozzle_points_b_2)+(nozzle_count_3*nozzle_points_f_2)+nozzle_points_t_1+nozzle_points_b_1)+n].z = offset*sin(m / nozzle_points_f_1 * 2 * M_PI);
        }
        nozzle_count_3 = nozzle_count_3 + 1;
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
    wws.transform.translation.x = -0.18;
    wws.transform.translation.y = 0.0;
    wws.transform.translation.z = 0.0;
    wws.transform.rotation.x = 0.0;
    wws.transform.rotation.y = 0.0;
    wws.transform.rotation.z = 0.0;
    wws.transform.rotation.w = 1.0;
    wws_pub.publish(wws);
    tf_Transform(wws);
}

void nozzle_Transform(){
    nozzle.header.stamp = ros::Time::now();
    nozzle.header.frame_id = "nozzle_RPYpos_frame";
    nozzle.child_frame_id = "nozzle_frame";
    nozzle.transform.translation.x = 0.0;
    nozzle.transform.translation.y = 0.0;
    nozzle.transform.translation.z = 0.0;
    nozzle.transform.rotation.x = 0.0;
    nozzle.transform.rotation.y = 0.0;
    nozzle.transform.rotation.z = 0.0;
    nozzle.transform.rotation.w = 1.0;
    nozzle_pub.publish(nozzle);
    tf_Transform(nozzle);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "Visualisation_objects_in_space");

  ros::NodeHandle nh;
  ros::Publisher cloud_pub_surface = nh.advertise<sensor_msgs::PointCloud>("/tube_surface_cloud", 1);
  ros::Publisher cloud_pub_wws = nh.advertise<sensor_msgs::PointCloud>("/water_wss_cloud", 1);
  ros::Publisher cloud_pub_nozzle = nh.advertise<sensor_msgs::PointCloud>("/nozzle_cloud", 1);
  tube_obj_pub = nh.advertise<geometry_msgs::TransformStamped>("/tube_frame_geometry_transform",1);
  wws_pub = nh.advertise<geometry_msgs::TransformStamped>("/water_workspace_frame_geometry_transform",1);
  nozzle_pub = nh.advertise<geometry_msgs::TransformStamped>("/nozzle_frame_geometry_transform",1);

  tubeDataGeneration();
  waterWorkspaceDataGeneration();
  nozzleWorkspaceDataGeneration();
  tube_Transform();
  water_workspace_Transform();
  nozzle_Transform();
  cloud_pub_surface.publish(tube_obj_cloud);
  cloud_pub_wws.publish(wws_cloud);
  cloud_pub_nozzle.publish(nozzle_cloud);

  ros::Rate r(10.0);
  while(ros::ok()){
      tube_obj_cloud.header.stamp = ros::Time::now();
      cloud_pub_surface.publish(tube_obj_cloud);

      wws_cloud.header.stamp = ros::Time::now();
      cloud_pub_wws.publish(wws_cloud);

      nozzle_cloud.header.stamp = ros::Time::now();
      cloud_pub_nozzle.publish(nozzle_cloud);

      tube_Transform();
      water_workspace_Transform();
      nozzle_Transform();
      r.sleep();

  }
}

/*
#include <pcl/point_types.h>

#include <pcl/2d/Convolution.h>
#include <pcl/2d/Edge.h>
#include <pcl/2d/Kernel.h>
#include <pcl/2d/Morphology.h>
#include <pcl/pcl_base.h>

using namespace pcl;

void example_edge ()
{
  Edge<pcl::PointXYZRGB> edge;

  //dummy clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  //example 1
  edge.output_type_ = Edge<pcl::PointXYZRGB>::OUTPUT_X_Y;
  edge.detectEdgeRoberts (*output_cloud, *input_cloud);

  //example 2
  edge.hysteresis_threshold_low_ = 20;
  edge.hysteresis_threshold_high_ = 80;
  edge.non_max_suppression_radius_x_ = 3;
  edge.non_max_suppression_radius_y_ = 3;
  edge.detectEdgeCanny (*output_cloud, *input_cloud);

  //example 3
  edge.detector_kernel_type_ = Edge<pcl::PointXYZRGB>::PREWITT;
  edge.hysteresis_thresholding_ = true;
  edge.hysteresis_threshold_low_ = 20;
  edge.hysteresis_threshold_high_ = 80;
  edge.non_maximal_suppression_ = true;
  edge.non_max_suppression_radius_x_ = 1;
  edge.non_max_suppression_radius_y_ = 1;
  edge.output_type_ = Edge<pcl::PointXYZRGB>::OUTPUT_X_Y;
  edge.detectEdge (*output_cloud, *input_cloud);
}

void example_convolution ()
{
  Kernel<pcl::PointXYZRGB> kernel;
  Convolution<pcl::PointXYZRGB> convolution;

  //dummy clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kernel_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  //example 1 : Gaussian Smoothing
  kernel.sigma_ = 2.0;
  kernel.kernel_size_ = 3;
  kernel.gaussianKernel (*kernel_cloud);
  convolution.kernel_ = *kernel_cloud;
  convolution.convolve (*output_cloud, *input_cloud);

  //example 2 : forward derivative in X direction
  kernel.kernel_type_ = Kernel<pcl::PointXYZRGB>::DERIVATIVE_FORWARD_X;
  kernel.fetchKernel (*kernel_cloud);
  convolution.kernel_ = *kernel_cloud;
  convolution.convolve (*output_cloud, *input_cloud);

  //example 3
  kernel.kernel_type_ = Kernel<pcl::PointXYZRGB>::DERIVATIVE_FORWARD_X;
  kernel.fetchKernel (convolution.kernel_);
  convolution.convolve (*output_cloud, *input_cloud);
}

void example_morphology ()
{
  Morphology<pcl::PointXYZRGB> morphology;

  //dummy clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr structuring_element_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  //example 1 : Gaussian Smoothing
  morphology.structuringElementCircular (*structuring_element_cloud, 3);
  morphology.structuring_element_ = *structuring_element_cloud;
  morphology.operator_type_ = Morphology<pcl::PointXYZRGB>::EROSION_GRAY;
  morphology.applyMorphologicalOperation (*output_cloud, *input_cloud);

  //example 2 : forward derivative in X direction
  morphology.structuringElementCircular (morphology.structuring_element_, 3);
  morphology.operator_type_ = Morphology<pcl::PointXYZRGB>::EROSION_GRAY;
  morphology.applyMorphologicalOperation (*output_cloud, *input_cloud);

}

int main(char *args, int argv)
{
  return 0;
}
*/
/*
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void draw_cloud(
    const std::string &text,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    pcl::visualization::CloudViewer viewer(text);
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr img_to_cloud(
        const cv::Mat& image,
        const cv::Mat &coords)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (int y=0;y<image.rows;y++)
    {
        for (int x=0;x<image.cols;x++)
        {
            pcl::PointXYZRGB point;
            point.x = coords.at<double>(0,y*image.cols+x);
            point.y = coords.at<double>(1,y*image.cols+x);
            point.z = coords.at<double>(2,y*image.cols+x);

            cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
            uint8_t r = (color[2]);
            uint8_t g = (color[1]);
            uint8_t b = (color[0]);

            int32_t rgb = (r << 16) | (g << 8) | b;
            point.rgb = *reinterpret_cast<float*>(&rgb);

            cloud->points.push_back(point);
        }
    }
    return cloud;
}

void cloud_to_img(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        cv::Mat &coords,
        cv::Mat &image)
{
    coords = cv::Mat(3, cloud->points.size(), CV_64FC1);
    image = cv::Mat(480, 640, CV_8UC3);
    for(int y=0;y<image.rows;y++)
    {
        for(int x=0;x<image.cols;x++)
        {
            coords.at<double>(0,y*image.cols+x) = cloud->points.at(y*image.cols+x).x;
            coords.at<double>(1,y*image.cols+x) = cloud->points.at(y*image.cols+x).y;
            coords.at<double>(2,y*image.cols+x) = cloud->points.at(y*image.cols+x).z;

            cv::Vec3b color = cv::Vec3b(
                    cloud->points.at(y*image.cols+x).b,
                    cloud->points.at(y*image.cols+x).g,
                    cloud->points.at(y*image.cols+x).r);

            image.at<cv::Vec3b>(cv::Point(x,y)) = color;
        }
    }
}

int main(int argc, char *argv[])
{
    cv::Mat image = cv::imread("test.png");
    cv::resize(image, image, cv::Size(640, 480));
    cv::imshow("initial", image);

    cv::Mat coords(3, image.cols * image.rows, CV_64FC1);
    for (int col = 0; col < coords.cols; ++col)
    {
        coords.at<double>(0, col) = col % image.cols;
        coords.at<double>(1, col) = col / image.cols;
        coords.at<double>(2, col) = 10;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = img_to_cloud(image, coords);
    draw_cloud("points", cloud);

    cloud_to_img(cloud, coords, image);
    cv::imshow("returned", image);

    cv::waitKey();
    return 0;
}
*/
/*



I am trying to convert a pointcloud with rgb information from pcl format to cv::Mat and back to pcl. I have found convert mat to pointcloud on stackoverflow.

Code Updated

I therefore used that code found on stackoverflow as a starting point. And now have the following:

//input pcl::PointCloud<pcl::PointXYZRGB> point_cloud_ptr
cv::Mat OpenCVPointCloud(3, point_cloud_ptr.points.size(), CV_64FC1);
OpenCVPointCloudColor = cv::Mat(480, 640, CV_8UC3);
for(int y=0;y<OpenCVPointCloudColor.rows;y++)
{
    for(int x=0;x<OpenCVPointCloudColor.cols;x++)
    {
        OpenCVPointCloud.at<double>(0,y*OpenCVPointCloudColor.cols+x) = point_cloud_ptr.points.at(y*OpenCVPointCloudColor.cols+x).x;
        OpenCVPointCloud.at<double>(1,y*OpenCVPointCloudColor.cols+x) = point_cloud_ptr.points.at(y*OpenCVPointCloudColor.cols+x).y;
        OpenCVPointCloud.at<double>(2,y*OpenCVPointCloudColor.cols+x) = point_cloud_ptr.points.at(y*OpenCVPointCloudColor.cols+x).z;

        cv::Vec3b color = cv::Vec3b(point_cloud_ptr.points.at(y*OpenCVPointCloudColor.cols+x).b,point_cloud_ptr.points.at(y*OpenCVPointCloudColor.cols+x).g,point_cloud_ptr.points.at(y*OpenCVPointCloudColor.cols+x).r);

        OpenCVPointCloudColor.at<cv::Vec3b>(cv::Point(x,y)) = color;
    }
}

cv::imshow("view 2", OpenCVPointCloudColor);
cv::waitKey(30);

Displaying the image above ensured me that the colours are extracted correctly (the image is compared by eye with the raw image). I then want to convert it back to a pointcloud:

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

for(int y=0;y<OpenCVPointCloudColor.rows;y++)
{
    for(int x=0;x<OpenCVPointCloudColor.cols;x++)
    {
        pcl::PointXYZRGB point;
        point.x = OpencVPointCloud.at<double>(0,y*OpenCVPointCloudColor.cols+x);
        point.y = OpencVPointCloud.at<double>(1,y*OpenCVPointCloudColor.cols+x);
        point.z = OpencVPointCloud.at<double>(2,y*OpenCVPointCloudColor.cols+x);

            cv::Vec3b color = OpenCVPointCloudColor.at<cv::Vec3b>(cv::Point(x,y));
        //Try 1 not working
        //uint8_t r = (color[2]);
        //uint8_t g = (color[1]);
        //uint8_t b = (color[0]);

        //Try 2 not working
        //point.r = color[2];
        //point.g = color[1];
        //point.b = color[0];

        //Try 3 not working
        //uint8_t r = (color.val[2]);
        //uint8_t g = (color.val[1]);
        //uint8_t b = (color.val[0]);

        //test working WHY DO THE ABOVE CODES NOT WORK :/
        uint8_t r = 255;
        uint8_t g = 0;
        uint8_t b = 0;
        int32_t rgb = (r << 16) | (g << 8) | b;
        point.rgb = *reinterpret_cast<float*>(&rgb);

        point_cloud_ptr -> points.push_back(point);
    }
}
*/
/*
#include <iostream>
#include <cstdlib>

// ROS
#include <ros/ros.h>

// PCL
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> visor;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2 cloud;

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer and add properties-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return viewer;
}

void updateVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->removePointCloud();
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb);
  viewer->spinOnce();
}

void showPCL(sensor_msgs::PointCloud2 points){
  cloud = points;
  pcl::fromROSMsg<pcl::PointXYZRGB>(cloud, *(ptCloud.get()));
  updateVis(visor, ptCloud);
}

int main(int argc, char **argv) {
  using namespace ros;

  visor = createVis();

  init(argc,argv,"kinQual");
  NodeHandle nh;
  Subscriber subDepth = nh.subscribe<>("camera/depth_registered/points",2000,showPCL);
  while(!visor->wasStopped()){
   sleep(1);
   spin();

  }

  return 0;
}
*/
/*
 * PointCloud2 msg;

msg.height = 240;
msg.width = 320;

msg.point_step = 8;
msg.row_step = msg.width*msg.point_step;
msg.is_bigendian = false;

bool is_dense = true;

msg.fields[0].name = "depth";
msg.fields[0].offset = 0;
msg.fields[0].datatype = FLOAT32;
msg.fields[0].count = ???;

msg.fields[1].name = "intensity";
msg.fields[1].offset = 0;
msg.fields[1].datatype = FLOAT32;
msg.fields[1].count = ???;

int byteIdx = 0
for(i = 0; i < depth.size();i++)
{
  msg.data[byteIdx++].count = (uint8_t)(depth[i] >>0);
  msg.data[byteIdx++].count = (uint8_t)(depth[i] >>8);
  msg.data[byteIdx++].count = (uint8_t)(depth[i] >>16);
  msg.data[byteIdx++].count = (uint8_t)(depth[i] >>24);
  msg.data[byteIdx++].count = (uint8_t)(intensity[i] >>0);
  msg.data[byteIdx++].count = (uint8_t)(intensity[i] >>8);
  msg.data[byteIdx++].count = (uint8_t)(intensity[i] >>16);
  msg.data[byteIdx++].count = (uint8_t)(intensity[i] >>24);
}*/
/*
Tools for manipulating sensor_msgs.

This file provides a type to enum mapping for the different PointField types and methods to read and write in a PointCloud2 buffer for the different PointField types.

Convert between the old (sensor_msgs::PointCloud) and the new (sensor_msgs::PointCloud2) format.

This file provides two sets of utilities to modify and parse PointCloud2 The first set allows you to conveniently set the fields by hand:

  #include <sensor_msgs/point_cloud_iterator.h>
  // Create a PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;
  // Fill some internals of the PoinCloud2 like the header/width/height ...
  cloud_msgs.height = 1;  cloud_msgs.width = 4;
  // Set the point fields to xyzrgb and resize the vector with the following command
  // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
  // the number of occurrences of the type in the PointField, the type of the PointField
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                           "y", 1, sensor_msgs::PointField::FLOAT32,
                                           "z", 1, sensor_msgs::PointField::FLOAT32,
                                           "rgb", 1, sensor_msgs::PointField::FLOAT32);
  // For convenience and the xyz, rgb, rgba fields, you can also use the following overloaded function.
  // You have to be aware that the following function does add extra padding for backward compatibility though
  // so it is definitely the solution of choice for PointXYZ and PointXYZRGB
  // 2 is for the number of fields to add
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  // You can then reserve / resize as usual
  modifier.resize(100);

The second set allow you to traverse your PointCloud using an iterator:

  // Define some raw data we'll put in the PointCloud2
  float point_data[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  uint8_t color_data[] = {40, 80, 120, 160, 200, 240, 20, 40, 60, 80, 100, 120};
  // Define the iterators. When doing so, you define the Field you would like to iterate upon and
  // the type of you would like returned: it is not necessary the type of the PointField as sometimes
  // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
  // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
  // and RGBA as A,R,G,B)
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
  // Fill the PointCloud2
  for(size_t i=0; i<n_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
    *iter_x = point_data[3*i+0];
    *iter_y = point_data[3*i+1];
    *iter_z = point_data[3*i+2];
    *iter_r = color_data[3*i+0];
    *iter_g = color_data[3*i+1];
    *iter_b = color_data[3*i+2];
  }

Author
    Radu Bogdan Rusu
    Sebastian Pütz
*/
/*
 void
RingColors::convertPoints(const sensor_msgs::PointCloud2ConstPtr &inMsg)
{
  if (output_.getNumSubscribers() == 0)         // no one listening?
    return;                                     // do nothing

  // allocate an PointXYZRGB message with same time and frame ID as
  // input data
  sensor_msgs::PointCloud2::Ptr outMsg(new sensor_msgs::PointCloud2());
  sensor_msgs::PointCloud2Modifier modifier(*outMsg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(inMsg->height * inMsg->width);

  outMsg->header.stamp = inMsg->header.stamp;
  outMsg->header.frame_id = inMsg->header.frame_id;
  outMsg->height = 1;

  sensor_msgs::PointCloud2Iterator<float> out_x(*outMsg, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(*outMsg, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(*outMsg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*outMsg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*outMsg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*outMsg, "b");

  sensor_msgs::PointCloud2ConstIterator<float> in_x(*inMsg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> in_y(*inMsg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> in_z(*inMsg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint16_t> in_ring(*inMsg, "ring");

  for (size_t i = 0; i < inMsg->height * inMsg->width; ++i, ++out_x, ++out_y, ++out_z, ++out_r, ++out_g, ++out_b,
    ++in_x, ++in_y, ++in_z, ++in_ring)
    {
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;

      // color lasers with the rainbow array
      int color = *in_ring % N_COLORS;
      *out_r = (rainbow[color] >> 16) & 0x0000ff;
      *out_g = (rainbow[color] >> 8) & 0x0000ff;
      *out_b = rainbow[color] & 0x0000ff;
    }

  output_.publish(outMsg);
}
*/
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
/*
program trial usage PCL library --> not working

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
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
unsigned int tube_num_points = 30000;

// water workspace parameters
double wws_radius = 0.02;
double wws_points_1 = 50;
double wws_points_2 = 50;
unsigned int wws_num_points = 3000;

// ROS publishers and messages
ros::Publisher tube_obj_pub, wws_pub;
geometry_msgs::TransformStamped tube_obj, wws;
//sensor_msgs::PointCloud tube_obj_cloud, wws_cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
PointCloud::Ptr tube_obj_cloud (new PointCloud);
PointCloud::Ptr wws_cloud (new PointCloud);
sensor_msgs::PointCloud2 cloud;
ros::Publisher cloud_msg2;

void transform_rotation(double angle, double x_point, double y_point){
    answ.x = (cos(angle)*x_point);
    answ.y = y_point;
    answ.z = (sin(-angle)*x_point);
}

void tubeDataGeneration(){

    tube_obj_cloud->header.stamp = ros::Time::now().toSec();
    tube_obj_cloud->header.frame_id = "tube_surface";
    tube_obj_cloud->height = tube_num_points;
    tube_obj_cloud->width = 1;
    pcl::PointXYZRGB init_point;
    pcl::PointXYZRGB calc_point;

    for (uint32_t i = 0; i < num_points_1; ++i){
        init_point.x = 0;
        init_point.y = (tube_diameter*cos(i / num_points_1 * 2 * M_PI)) + bend_radius_tube;
        init_point.z = tube_diameter*sin(i / num_points_1 * 2 * M_PI);

        for (uint32_t j = 0; j < num_points_2; ++j){
            double angle = (j / num_points_2 * 0.5 * M_PI);
            transform_rotation(angle, init_point.y, init_point.z);
            calc_point.y = answ.x;
            calc_point.x = answ.z;
            calc_point.z = answ.y;
            uint8_t r = 255;
            uint8_t g = 0;
            uint8_t b = 0;
            int32_t rgb = (r << 16) | (g << 8) | b;
            calc_point.rgb = *reinterpret_cast<float*>(&rgb);
            tube_obj_cloud->points.push_back(calc_point);
        }
    }
    for (uint32_t k = 0; k < num_points_3; ++k){
        init_point.x = 0;
        init_point.y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
        init_point.z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);

        for (uint32_t l = 0; l < num_points_4; ++l){
            double distance = ((l / num_points_4) * dist_4);
            calc_point.x = distance;
            calc_point.y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
            calc_point.z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);
            uint8_t r = 255;
            uint8_t g = 0;
            uint8_t b = 0;
            int32_t rgb = (r << 16) | (g << 8) | b;
            calc_point.rgb = *reinterpret_cast<float*>(&rgb);
            tube_obj_cloud->points.push_back(calc_point);
        }
    }
    cloud.height = tube_num_points;
    cloud.width = 1;
    pcl::toROSMsg(*tube_obj_cloud, cloud);
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "tube_surface2";


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

void waterWorkspaceDataGeneration(){

    wws_cloud->header.stamp = ros::Time::now().toSec();
    wws_cloud->header.frame_id = "water_workspace";
    wws_cloud->height = 1;
    wws_cloud->width = wws_num_points;
    pcl::PointXYZRGB init_point;
    pcl::PointXYZRGB calc_point;

    for (uint32_t i = 0; i < wws_points_1; ++i){
        init_point.x = 0;
        init_point.y = wws_radius*cos(i / wws_points_1 * 2 * M_PI);
        init_point.z = wws_radius*sin(i / wws_points_1 * 2 * M_PI);

        for (uint32_t j = 0; j < wws_points_2; ++j){
            double angle = (j / wws_points_2 * M_PI);
            transform_rotation(angle, init_point.y, init_point.z);
            calc_point.y = answ.x;
            calc_point.x = answ.z;
            calc_point.z = answ.y;
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 255;
            int32_t rgb = (r << 16) | (g << 8) | b;
            calc_point.rgb = *reinterpret_cast<float*>(&rgb);
            wws_cloud->points.push_back(calc_point);
        }
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

  //ros::Publisher colour_pub_pub = nh.advertise<PointCloud>("point_col", 1);
  //ros::Publisher cloud_pub_surface = nh.advertise<sensor_msgs::PointCloud>("/tube_surface_cloud", 10);
  ros::Publisher cloud_pub_surface = nh.advertise<PointCloud>("/tube_surface_cloud", 1000);
  //ros::Publisher cloud_pub_wws = nh.advertise<sensor_msgs::PointCloud>("/water_wss_cloud", 10);

  //ros::Publisher cloud_pub_wws = nh.advertise<PointCloud>("/water_wss_cloud", 1000);

  cloud_msg2 = nh.advertise<sensor_msgs::PointCloud2>("/surface2", 1);

  tube_obj_pub = nh.advertise<geometry_msgs::TransformStamped>("/tube_frame_geometry_transform",1);
  wws_pub = nh.advertise<geometry_msgs::TransformStamped>("/water_workspace_frame_geometry_transform",1);

  tubeDataGeneration();
  waterWorkspaceDataGeneration();
  tube_Transform();
  water_workspace_Transform();
  cloud_pub_surface.publish(tube_obj_cloud);
  //cloud_pub_wws.publish(wws_cloud);

  cloud_msg2.publish(cloud);

  ros::Rate rate(10.0);
  while(ros::ok()){

      tube_obj_cloud->header.stamp = ros::Time::now().toSec();
      cloud_pub_surface.publish(tube_obj_cloud);

      cloud.header.stamp = ros::Time::now();
      cloud_msg2.publish(cloud);

      tube_Transform();
      water_workspace_Transform();
      rate.sleep();

  }
}
*/
/*
// program + Usage PCL library


#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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
typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_ptr;

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
  ros::Publisher Pub2 = nh.advertise<point_cloud_ptr>("point_rgb", 1);
  //

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
  point_cloud_ptr::Ptr point (new pcl::PointCloud<pcl::PointXYZRGB>);
  point.x = 1.0;
  point.y = 3.0;
  point.z = 2.0;
  uint8_t r = 0;
  uint8_t g = 255;
  uint8_t b = 0;
  int32_t rgb = (r << 16) | (g << 8) | b;
  point.rgb = *reinterpret_cast<float*>(&rgb);
  point_cloud_ptr -> points.push_back(point);
  point.header.frame_id = "some_tf_frame_2";

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
  //new

  ros::Rate rate(10.0);
  while(ros::ok()){

      //new
      msg->header.stamp = ros::Time::now().toSec();
      pub.publish (msg);

      point.header.stamp = ros::Time::now();
      Pub2.publish (point);
      //

      tube_obj_cloud.header.stamp = ros::Time::now();
      cloud_pub_surface.publish(tube_obj_cloud);

      wws_cloud.header.stamp = ros::Time::now();
      cloud_pub_wws.publish(wws_cloud);

      tube_Transform();
      water_workspace_Transform();
      rate.sleep();

  }
}*/
