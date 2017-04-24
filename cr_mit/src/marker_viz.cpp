#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Point compare;
geometry_msgs::TransformStamped marker;

int num_points = 0;

void markerTransform(){
    static tf::TransformBroadcaster transformer;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.child_frame_id = "my_frame";
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
    ros::init(argc, argv, "trajectory_visualisation");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while(nh.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/00_spuitkop", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        visualization_msgs::Marker points, line_strip;
        points.header.frame_id = line_strip.header.frame_id = "/my_frame";
        points.header.stamp = line_strip.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = "points_and_lines";
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        points.scale.x = 0.005;
        points.scale.y = 0.005;
        line_strip.scale.x = 0.005;

        points.color.g = 1.0f;
        points.color.a = 1.0;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Message
        geometry_msgs::Point p;
        p.x = transform.getOrigin().x();
        p.y = transform.getOrigin().y();
        p.z = transform.getOrigin().z();

        markerTransform();

        if((p.x != compare.x)||(p.y != compare.y)||(p.z != compare.z)){
            points.points.push_back(p);
            line_strip.points.push_back(p);
            compare = p;
            num_points = num_points +1;

            if(num_points >= 15){
                marker_pub.publish(points);
                marker_pub.publish(line_strip);
            }
        }
        rate.sleep();
    }
    return 0;
}
