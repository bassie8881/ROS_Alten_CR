#include <ros/ros.h>
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
