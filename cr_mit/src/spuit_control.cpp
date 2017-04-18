#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

ros::Publisher pub_spuit;
sensor_msgs::JointState spuit_msgs;

int main(int argc, char **argv){
    ros::init(argc, argv, "spraygun_control_node");
    ros::NodeHandle nh;

    sensor_msgs::JointState end_eff_spuit;
    end_eff_spuit.header.frame_id = "/00_spuitkop";
    end_eff_spuit.header.stamp = ros::Time();
    end_eff_spuit.name.push_back("spuitkop");
    end_eff_spuit.position.push_back(2);

    pub_spuit = nh.advertise<sensor_msgs::JointState>("/joint_state_publisher",1);

    while(ros::ok()){
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
