#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

double angleR_spuit = 0;
double angleP_spuit = 0;
double angleY_spuit = 0;
double disX_spuit = 0;
double disY_spuit = 0;
double disZ_spuit = 0;
double angleR_head = 0;
double angleP_head = 0;
double angleY_head = 0;
double disX_head = 0;
double disY_head = 0;
double disZ_head = 0;
double angleR_base = 0;
double angleP_base = 0;
double angleY_base = 0;
double disX_base = 0;
double disY_base = 0;
double disZ_base = 0;

double time_interval = 4;
double const_ = 101;

tf::Quaternion q1, q2, q3;

ros::Publisher pos, fix_obj, base, head;
geometry_msgs::TransformStamped odom, fixed_object, spuit_base, spuit_head;

void tf_Transform(geometry_msgs::TransformStamped msg){
    static tf::TransformBroadcaster transformer;
    transformer.sendTransform(msg);
}

void spuit_geometry_Transform(){
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "00_head";
    odom.child_frame_id = "00_spuitkop";
    odom.transform.translation.x = disX_spuit;
    odom.transform.translation.y = disY_spuit;
    odom.transform.translation.z = disZ_spuit;
    q1 = tf::createQuaternionFromRPY(angleP_spuit, angleR_spuit, angleY_spuit);
    odom.transform.rotation.x = q1[0];
    odom.transform.rotation.y = q1[1];
    odom.transform.rotation.z = q1[2];
    odom.transform.rotation.w = q1[3];
    pos.publish(odom);
    tf_Transform(odom);
}

void head_geometry_Transform(){
    spuit_head.header.stamp = ros::Time::now();
    spuit_head.header.frame_id = "00_base";
    spuit_head.child_frame_id = "00_head";
    spuit_head.transform.translation.x = disX_head;
    spuit_head.transform.translation.y = disY_head;
    spuit_head.transform.translation.z = disZ_head;
    q3 = tf::createQuaternionFromRPY(angleP_head, angleR_head, angleY_head);
    spuit_head.transform.rotation.x = q3[0];
    spuit_head.transform.rotation.y = q3[1];
    spuit_head.transform.rotation.z = q3[2];
    spuit_head.transform.rotation.w = q3[3];
    head.publish(spuit_head);
    tf_Transform(spuit_head);
}

void base_geometry_Transform(){
    spuit_base.header.stamp = ros::Time::now();
    spuit_base.header.frame_id = "map";
    spuit_base.child_frame_id = "00_base";
    spuit_base.transform.translation.x = disX_base;
    spuit_base.transform.translation.y = disY_base;
    spuit_base.transform.translation.z = disZ_base;
    q2 = tf::createQuaternionFromRPY(angleP_base, angleR_base, angleY_base);
    spuit_base.transform.rotation.x = q2[0];
    spuit_base.transform.rotation.y = q2[1];
    spuit_base.transform.rotation.z = q2[2];
    spuit_base.transform.rotation.w = q2[3];
    base.publish(spuit_base);
    tf_Transform(spuit_base);
}

void object_geometry_Transform(){
    fixed_object.header.stamp = ros::Time::now();
    fixed_object.header.frame_id = "map";
    fixed_object.child_frame_id = "90elbow";
    fixed_object.transform.translation.x = 0.0;
    fixed_object.transform.translation.y = 0.0;
    fixed_object.transform.translation.z = 0.0;
    fixed_object.transform.rotation.x = 0.0;
    fixed_object.transform.rotation.y = 0.0;
    fixed_object.transform.rotation.z = 0.0;
    fixed_object.transform.rotation.w = 1.0;
    fix_obj.publish(fixed_object);
    tf_Transform(fixed_object);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "spraygun_control_node");
    ros::NodeHandle nh;

    fix_obj = nh.advertise<geometry_msgs::TransformStamped>("/fix_obj",1);
    pos = nh.advertise<geometry_msgs::TransformStamped>("/spuit_pos",1);
    head = nh.advertise<geometry_msgs::TransformStamped>("/spuit_head",1);
    base = nh.advertise<geometry_msgs::TransformStamped>("/spuit_base",1);

    while(ros::ok()){
        spuit_geometry_Transform();
        object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();

        double value_z_pos = 0.04;
        for(int i=0;i!=const_;i++){
            disZ_spuit = (value_z_pos/const_)*i;
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration(time_interval/const_).sleep();
        }

        double value_R_angle = 0.2;
        for(int i=0;i!=const_;i++){
            angleR_head = -(value_R_angle/const_)*i;
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration(time_interval/const_).sleep();
        }

        double value_P_angle = 1;
        for(int i=0; i!=const_; i++){
            angleP_base = angleP_base + (value_P_angle/const_);
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration(time_interval/const_).sleep();
        }
    }
    return 0;
}
