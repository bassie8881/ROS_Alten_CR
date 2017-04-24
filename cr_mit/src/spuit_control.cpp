#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

struct frames{
    double angleR_spuit;
    double angleP_spuit;
    double angleY_spuit;
    double disX_spuit;
    double disY_spuit;
    double disZ_spuit;
    double angleR_head;
    double angleP_head;
    double angleY_head;
    double disX_head;
    double disY_head;
    double disZ_head;
    double angleR_base;
    double angleP_base;
    double angleY_base;
    double disX_base;
    double disY_base;
    double disZ_base;
};

frames nozzle_frames;
frames empty_instance;

double time_interval = 4;
double const_ = 101;

tf::Quaternion q1, q2, q3;

ros::Publisher pos, fix_obj, base, head;
geometry_msgs::TransformStamped odom, fixed_object, spuit_base, spuit_head;

double degrees_to_radian(double deg){
    double ans = (deg * (M_PI/180));
    return ans;
}

void tf_Transform(geometry_msgs::TransformStamped msg){
    static tf::TransformBroadcaster transformer;
    transformer.sendTransform(msg);
}

void spuit_geometry_Transform(){
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "00_head";
    odom.child_frame_id = "00_spuitkop";
    odom.transform.translation.x = nozzle_frames.disX_spuit;
    odom.transform.translation.y = nozzle_frames.disY_spuit;
    odom.transform.translation.z = nozzle_frames.disZ_spuit;
    q1 = tf::createQuaternionFromRPY(nozzle_frames.angleP_spuit, nozzle_frames.angleR_spuit, nozzle_frames.angleY_spuit);
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
    spuit_head.transform.translation.x = nozzle_frames.disX_head;
    spuit_head.transform.translation.y = nozzle_frames.disY_head;
    spuit_head.transform.translation.z = nozzle_frames.disZ_head;
    q3 = tf::createQuaternionFromRPY(nozzle_frames.angleP_head, nozzle_frames.angleR_head, nozzle_frames.angleY_head);
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
    spuit_base.transform.translation.x = nozzle_frames.disX_base;
    spuit_base.transform.translation.y = nozzle_frames.disY_base;
    spuit_base.transform.translation.z = nozzle_frames.disZ_base;
    q2 = tf::createQuaternionFromRPY(nozzle_frames.angleP_base, nozzle_frames.angleR_base, nozzle_frames.angleY_base);
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

        nozzle_frames = empty_instance;

        double value_z_pos = 0.04;
        double value_R_angle = degrees_to_radian(10);
        for(int i=0;i!=const_;i++){
            nozzle_frames.disZ_spuit = (value_z_pos/const_)*i;
            nozzle_frames.angleR_head = -(value_R_angle/const_)*i;
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration(time_interval/const_).sleep();
        }
        double value_P_angle = degrees_to_radian(360);
        for(int i=0; i!=(const_*3); i++){
            nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle/(const_*3));
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration((time_interval*3)/(const_*3)).sleep();
        }
        for(int i=0; i!=const_;i++){
            nozzle_frames.angleR_head = nozzle_frames.angleR_head + (value_R_angle/const_);
            nozzle_frames.disZ_spuit = nozzle_frames.disZ_spuit - (value_z_pos/const_);
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration(time_interval/const_).sleep();
        }
        double value_x_pos = 0.12;
        for(int i=0;i!=const_;i++){
            nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos/const_);
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration((time_interval*1.5)/const_).sleep();
        }
        double value_Y_angle_spuit = degrees_to_radian(9);
        double value_Y_pos_spuit = 0.02;
        for(int i=0;i!=const_;i++){
            nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit + (value_Y_angle_spuit/const_);
            nozzle_frames.disY_head = nozzle_frames.disY_head - (value_Y_pos_spuit/const_);
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration(time_interval/const_).sleep();
        }
        double value_P_angle_base = degrees_to_radian(360);
        for(int i=0;i!=(const_*3);i++){
            nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base/(const_*3));
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration((time_interval*3)/(const_*3)).sleep();
        }
        for(int i=0;i!=(const_*3*10);i++){
            nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base/(const_*3));
            nozzle_frames.disX_head = nozzle_frames.disX_head + (value_x_pos/(const_*3*10));
            spuit_geometry_Transform();
            object_geometry_Transform();
            base_geometry_Transform();
            head_geometry_Transform();
            ros::Duration((time_interval*3*10)/(const_*3*10)).sleep();
        }
    }
    return 0;
}
