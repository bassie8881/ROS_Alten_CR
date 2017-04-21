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
double time_interval = 4;
double a = 101;
double b = 101;
tf::Quaternion q;

ros::Publisher pos, fix_obj;
geometry_msgs::TransformStamped odom, fixed_object;

void spuit_tf_Transform(geometry_msgs::TransformStamped msg){
    static tf::TransformBroadcaster transformer;
    transformer.sendTransform(msg);
}

void spuit_geometry_Transform(){
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "00_spuitkop";
    odom.transform.translation.x = disX_spuit;
    odom.transform.translation.y = disY_spuit;
    odom.transform.translation.z = disZ_spuit;
    q = tf::createQuaternionFromRPY(angleP_spuit, angleR_spuit, angleY_spuit);
    odom.transform.rotation.x = q[0];
    odom.transform.rotation.y = q[1];
    odom.transform.rotation.z = q[2];
    odom.transform.rotation.w = q[3];
    pos.publish(odom);
    spuit_tf_Transform(odom);
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
    spuit_tf_Transform(fixed_object);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "spraygun_control_node");
    ros::NodeHandle nh;

    pos = nh.advertise<geometry_msgs::TransformStamped>("/spuit_pos",1);
    fix_obj = nh.advertise<geometry_msgs::TransformStamped>("/fix_obj",1);

    while(ros::ok()){
        spuit_geometry_Transform();
        object_geometry_Transform();

        double value_z_pos = 0.04;
        double value_R_angle = 0.2;
        for(int i=0;i!=a;i++)
        {
            disZ_spuit = (value_z_pos/a)*i;
            angleR_spuit = -(value_R_angle/a)*i;
            spuit_geometry_Transform();
            object_geometry_Transform();
            ros::Duration(time_interval/a).sleep();
        }
        double value_P_angle = 1;
        for(int j=0; j!=b; j++){
            angleP_spuit = angleP_spuit + (value_P_angle/b);
            spuit_geometry_Transform();
            object_geometry_Transform();
            ros::Duration(time_interval/b).sleep();
        }
    }
    return 0;
}
