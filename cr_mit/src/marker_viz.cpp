#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

void poseTransform(visualization_msgs::Marker msg){
    static tf::TransformBroadcaster transformer;
    geometry_msgs::TransformStamped marker_trans;
    marker_trans.header.frame_id = "my_frame";
    marker_trans.child_frame_id = "map";
    marker_trans.transform.translation.x = msg.pose.position.x;
    marker_trans.transform.translation.y = msg.pose.position.y;
    marker_trans.transform.translation.z = msg.pose.position.z;
    marker_trans.transform.rotation.x = msg.pose.orientation.x;
    marker_trans.transform.rotation.y = msg.pose.orientation.y;
    marker_trans.transform.rotation.z = msg.pose.orientation.z;
    marker_trans.transform.rotation.w = msg.pose.orientation.w;
    transformer.sendTransform(marker_trans);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "marker_visualisation_node");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    uint32_t shape = visualization_msgs::Marker::CUBE;

    while(ros::ok()){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time();
        marker.ns = "visual_shapes";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;

        marker.lifetime = ros::Duration();

        marker_pub.publish(marker);
        poseTransform(marker);
    }
}
