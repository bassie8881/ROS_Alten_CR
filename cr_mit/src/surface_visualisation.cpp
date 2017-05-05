/*#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

geometry_msgs::TransformStamped marker;

float bend_radius_tube = 0.15;
float tube_diameter = 0.025;

void surfaceTransform(){
    static tf::TransformBroadcaster transformer;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.child_frame_id = "marked_surface";
    marker.transform.translation.x = 0;
    marker.transform.translation.y = 0;
    marker.transform.translation.z = 0;
    marker.transform.rotation.x = 0;
    marker.transform.rotation.y = 0;
    marker.transform.rotation.z = 0;
    marker.transform.rotation.w = 1;
    transformer.sendTransform(marker);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "surface_visualisation");
    ros::NodeHandle nh;

    ros::Publisher surface_pub = nh.advertise<visualization_msgs::Marker>("visualization_surface_marker", 10);
    ros::Rate rate(10.0);

    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "/marked_surface";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.001;
    points.scale.y = 0.001;
    line_strip.scale.x = 0.001;

    points.color.g = 1.0f;
    points.color.a = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    while(ros::ok()){
        surfaceTransform();

        for (uint32_t i = 1; i < 201; ++i){
            float z = tube_diameter*sin(i / 200.0f * 2 * M_PI);
            float y = tube_diameter*cos(i / 200.0f * 2 * M_PI) + bend_radius_tube;


            for (uint32_t j = 0; j < 200; ++j){
                y = y - tube_diameter*sin(j / 200.0f * 0.5 * M_PI);
                float x = tube_diameter*sin(j / 200.0f * 0.5 * M_PI);

                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = z;

                points.points.push_back(p);
                line_strip.points.push_back(p);
                ROS_INFO_STREAM("points = "<<points);
            }


        }
        surface_pub.publish(points);
        surface_pub.publish(line_strip);
        rate.sleep();
    }
    return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>

struct ans{
    double x;
    double y;
    double z;
};

float bend_radius_tube = 0.15;
float tube_diameter = 0.025;
struct ans answ;

void transform_rotation(double angle, double x_point, double y_point){
    answ.x = (cos(angle)*x_point);
    answ.y = y_point;
    answ.z = (sin(-angle)*x_point);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "surface_visualisation");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 10);

  int count = 0;
  double num_points_1 = 100;
  double num_points_2 = 100;
  unsigned int num_points = 20000;

  ros::Rate r(10.0);
  while(ros::ok()){
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "inner_surface";

    cloud.points.resize(num_points);

    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);


    for (uint32_t i = 0; i < num_points_1; ++i){
        cloud.points[i + (count*num_points_2)].x = 0;
        cloud.points[i + (count*num_points_2)].y = (tube_diameter*cos(i / num_points_1 * 2 * M_PI)) + bend_radius_tube;
        cloud.points[i + (count*num_points_2)].z = tube_diameter*sin(i / num_points_1 * 2 * M_PI);

        for (uint32_t j = 0; j < num_points_2; ++j){
            double angle = (j / num_points_2 * 0.5 * M_PI);
            transform_rotation(angle, cloud.points[i + (count*num_points_2)].y, cloud.points[i + (count*num_points_2)].z);
            cloud.points[(i + (count*num_points_2))+j].y = answ.x;
            cloud.points[(i + (count*num_points_2))+j].x = answ.z;
            cloud.points[(i + (count*num_points_2))+j].z = answ.y;

            //cloud.channels[0].values[(i + (count*num_points_2))+j] = 1;
        }
        count = count + 1;
    }
    count = 0;
    cloud_pub.publish(cloud);
    r.sleep();
  }
}*/
/*
Channels

PointClouds can have any number of channels associated with them. If you're using a LaserScan display, the only available channel will be the "Intensity" channel.

This section explains how the color/position of a point is computed for each channel type.

Intensity

Valid channel names: intensity, intensities

Intensity only affects the color of the point.

The intensity channel uses 4 values to compute the final color of the point:

Min Intensity


min_i

Max Intensity


max_i

Min Color


min_c

Max Color


max_c

For each point:

    To compute the color value, we first compute a normalized intensity value based on min_i and max_i:

norm_i = (i - min_i) / (max_i - min_i)

    Then to compute the color from that normalized intensity:

final_c = (norm_i * max_c) + ((1 - norm_i) * min_c)

RGB

Valid channel names: rgb (1 channel), r, g, b (3 channel)

RGB only affects the color of the point.

There are two ways to specify RGB:

    3 channels, named "r", "g", and "b", with floating point values between 0 and 1.

    1 channel, with the float in the channel reinterpreted as 3 single-byte values with ranges from 0 to 255. 0xff0000 is red, 0xff00 is green, 0xff is blue.

        In C++, int rgb = 0xff0000; float float_rgb = *reinterpret_cast<float*>(&rgb);

        In Python,  float_rgb = struct.unpack('f', struct.pack('i', 0xff0000))[0]

Normal Sphere

Valid channel names: nx, ny, nz (all 3 required)

Normal Sphere only affects the position of the point.

For this channel, the "nx", "ny" and "nz" channels will be used to position the points instead of the values in the points array.

Curvature

Valid channel names: curvature, curvatures

Curvature colors in the same way intensity does.

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Empty.h>

struct ans{
    double x;
    double y;
    double z;
};

float bend_radius_tube = 0.15;
float tube_diameter = 0.025;

struct ans answ;

double wws_radius = 0.02;

int count = 0;
int count2 = 0;
double num_points_1 = 100;
double num_points_2 = 200;
double num_points_3 = 100;
double num_points_4 = 40;
double dist_4 = 0.04;
unsigned int num_points = 30000;

int wws_count = 0;
double wws_points_1 = 50;
double wws_points_2 = 50;
unsigned int wws_num_points = 3000;

bool set_timer = false;
ros::Time timer;
double duration;
double pub_interval = 10;
double cleaned_percentage = 0;

bool flip_cloud = false;

ros::Publisher fix_obj, water_workspace;
geometry_msgs::TransformStamped fixed_object, wws;
sensor_msgs::PointCloud cleaned_surface, cloud, wws_cloud;

double degrees_to_radian(double deg){
    double ans = (deg * (M_PI/180));
    return ans;
}

void completedMessage(const std_msgs::Empty& flip_cloud_msg){
    flip_cloud = true;
}

float det_procentage_cleaned(){
    float cloud_points = 0;
    float cleaned_points = 0;
    float percentage = 0;
    for(size_t i=0; i<cloud.points.size(); ++i){
        if(cloud.points[i].x !=0 || cloud.points[i].y !=0 || cloud.points[i].z !=0){
            cloud_points = cloud_points + 1;
            if(cloud.channels[0].values[i] == 2000){
                cleaned_points = cleaned_points + 1;
            }
        }
    }
    percentage = (cleaned_points/cloud_points)*100;
    return percentage;
}

void updateCleanedSurface(){
    for(size_t i=0; i<cloud.points.size(); ++i){
        if(cleaned_surface.points[i].x !=0 || cleaned_surface.points[i].y !=0 || cleaned_surface.points[i].z !=0){
            cloud.points[i] = cleaned_surface.points[i];
            cloud.channels[0].values[i] = cleaned_surface.channels[0].values[i];
        }
    }
}

void cleanedSurfaceCallback(const sensor_msgs::PointCloud& msg){
    cleaned_surface = msg;
}

void transform_rotation(double angle, double x_point, double y_point){
    answ.x = (cos(angle)*x_point);
    answ.y = y_point;
    answ.z = (sin(-angle)*x_point);
}

void tf_Transform(geometry_msgs::TransformStamped msg){
    static tf::TransformBroadcaster transformer;
    transformer.sendTransform(msg);
}

void object_geometry_Transform(){
    fixed_object.header.stamp = ros::Time::now();
    fixed_object.header.frame_id = "map";
    fixed_object.child_frame_id = "inner_surface";
    fixed_object.transform.translation.x = 0.0;
    fixed_object.transform.translation.y = -bend_radius_tube;
    fixed_object.transform.translation.z = 0.0;
    fixed_object.transform.rotation.x = 0.0;
    fixed_object.transform.rotation.y = 0.0;
    fixed_object.transform.rotation.z = 0.0;
    fixed_object.transform.rotation.w = 1.0;
    fix_obj.publish(fixed_object);
    tf_Transform(fixed_object);
}

void fliped_cloud_geometry_Transform(){
    fixed_object.header.stamp = ros::Time::now();
    fixed_object.header.frame_id = "map";
    fixed_object.child_frame_id = "inner_surface";
    fixed_object.transform.translation.x = 0.0;
    fixed_object.transform.translation.y = bend_radius_tube;
    fixed_object.transform.translation.z = 0.0;
    double angle = degrees_to_radian(90);
    q = tf::createQuaternionFromRPY(0, 0, angle);
    fixed_object.transform.rotation.x = q[0];
    fixed_object.transform.rotation.y = q[1];
    fixed_object.transform.rotation.z = q[2];
    fixed_object.transform.rotation.w = q[3];
    fix_obj.publish(fixed_object);
    tf_Transform(fixed_object);
}

void water_workspace_Transform(){
    wws.header.stamp = ros::Time::now();
    wws.header.frame_id = "00_spuitkop";
    wws.child_frame_id = "water_workspace";
    wws.transform.translation.x = 0.0;
    wws.transform.translation.y = 0.0;
    wws.transform.translation.z = 0.0;
    wws.transform.rotation.x = 0.0;
    wws.transform.rotation.y = 0.0;
    wws.transform.rotation.z = 0.0;
    wws.transform.rotation.w = 1.0;
    water_workspace.publish(wws);
    tf_Transform(wws);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "visualisation");

  ros::NodeHandle n;
  ros::Publisher cloud_pub_surface = n.advertise<sensor_msgs::PointCloud>("/surface_cloud", 10);
  ros::Publisher cloud_pub_wws = n.advertise<sensor_msgs::PointCloud>("/water_wss_cloud", 10);
  fix_obj = n.advertise<geometry_msgs::TransformStamped>("/surface_visualisation_base",1);
  water_workspace = n.advertise<geometry_msgs::TransformStamped>("/water_workspace_base",1);
  ros::Subscriber sub_cleaned_surface = n.subscribe("/cleaned_surface", 1, cleanedSurfaceCallback);
  ros::Subscriber<std_msgs::Empty> sub("/flip_cloud", &completedMessage);

  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = "inner_surface";

  cloud.points.resize(num_points);

  cloud.channels.resize(1);
  cloud.channels[0].name = "intensities";
  cloud.channels[0].values.resize(num_points);

  wws_cloud.header.stamp = ros::Time::now();
  wws_cloud.header.frame_id = "water_workspace";

  wws_cloud.points.resize(wws_num_points);

  wws_cloud.channels.resize(1);
  wws_cloud.channels[0].name = "intensities";
  wws_cloud.channels[0].values.resize(wws_num_points);

  for (uint32_t i = 0; i < num_points_1; ++i){
      cloud.points[i + (count*num_points_2)].x = 0;
      cloud.points[i + (count*num_points_2)].y = (tube_diameter*cos(i / num_points_1 * 2 * M_PI)) + bend_radius_tube;
      cloud.points[i + (count*num_points_2)].z = tube_diameter*sin(i / num_points_1 * 2 * M_PI);

      for (uint32_t j = 0; j < num_points_2; ++j){
          double angle = (j / num_points_2 * 0.5 * M_PI);
          transform_rotation(angle, cloud.points[i + (count*num_points_2)].y, cloud.points[i + (count*num_points_2)].z);
          cloud.points[(i + (count*num_points_2))+j].y = answ.x;
          cloud.points[(i + (count*num_points_2))+j].x = answ.z;
          cloud.points[(i + (count*num_points_2))+j].z = answ.y;

          //cloud.channels[0].values[(i + (count*num_points_2))+j] = 1;
      }
      count = count + 1;
  }
  for (uint32_t k = 0; k < num_points_3; ++k){
      cloud.points[k + ((count*num_points_2)+(count2*num_points_4)+num_points_1)].x = 0;
      cloud.points[k + ((count*num_points_2)+(count2*num_points_4)+num_points_1)].y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
      cloud.points[k + ((count*num_points_2)+(count2*num_points_4)+num_points_1)].z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);

      for (uint32_t l = 0; l < num_points_4; ++l){
          double distance = ((l / num_points_4) * dist_4);
          cloud.points[k + ((count*num_points_2)+(count2*num_points_4)+num_points_1)+l].x = distance;
          cloud.points[k + ((count*num_points_2)+(count2*num_points_4)+num_points_1)+l].y = (tube_diameter*cos(k / num_points_3 * 2 * M_PI)) + bend_radius_tube;
          cloud.points[k + ((count*num_points_2)+(count2*num_points_4)+num_points_1)+l].z = tube_diameter*sin(k / num_points_3 * 2 * M_PI);
      }
      count2 = count2 + 1;
  }
  count = 0;
  count2 = 0;

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

  object_geometry_Transform();
  water_workspace_Transform();
  cloud_pub_surface.publish(cloud);
  cloud_pub_wws.publish(wws_cloud);

  ros::Rate r(10.0);
  while(ros::ok()){

      ros::spinOnce();

      if(!cleaned_surface.points.empty()){
          updateCleanedSurface();
      }

      if(!cleaned_surface.points.empty()&& set_timer ==false){
          set_timer = true;
          timer = ros::Time::now();
      }

      cloud.header.stamp = ros::Time::now();
      cloud_pub_surface.publish(cloud);

      wws_cloud.header.stamp = ros::Time::now();
      cloud_pub_wws.publish(wws_cloud);

      water_workspace_Transform();

      if(flip_cloud == true){
          fliped_cloud_geometry_Transform();
      }
      else{
          object_geometry_Transform();
      }

      duration = (ros::Time::now()-timer).toSec();
      if(set_timer == true && (duration >= pub_interval)){
          timer = ros::Time::now();
          cleaned_percentage = det_procentage_cleaned();
          ROS_INFO_STREAM("Percentage of surface cleaned = "<<cleaned_percentage);
      }

      r.sleep();

  }
}
