#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Empty.h>

struct frames{
    double angleR_spuit;
    double angleP_spuit;
    double angleY_spuit;
    double disX_spuit;
    double disY_spuit;
    double disZ_spuit;
    double angleR_rot_axis;
    double angleP_rot_axis;
    double angleY_rot_axis;
    double disX_rot_axis;
    double disY_rot_axis;
    double disZ_rot_axis;
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

tf::Quaternion q1, q2, q3, q4;

ros::Publisher pos, fix_obj, base, rot, head;
geometry_msgs::TransformStamped odom, fixed_object, spuit_base, rot_axis, spuit_head;

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
    odom.header.frame_id = "00_rot_axis";
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

void rotation_axis_geometry_Transform(){
    rot_axis.header.stamp = ros::Time::now();
    rot_axis.header.frame_id = "00_head";
    rot_axis.child_frame_id = "00_rot_axis";
    rot_axis.transform.translation.x = nozzle_frames.disX_rot_axis;
    rot_axis.transform.translation.y = nozzle_frames.disY_rot_axis;
    rot_axis.transform.translation.z = nozzle_frames.disZ_rot_axis;
    q4 = tf::createQuaternionFromRPY(nozzle_frames.angleP_rot_axis, nozzle_frames.angleR_rot_axis, nozzle_frames.angleY_rot_axis);
    rot_axis.transform.rotation.x = q4[0];
    rot_axis.transform.rotation.y = q4[1];
    rot_axis.transform.rotation.z = q4[2];
    rot_axis.transform.rotation.w = q4[3];
    head.publish(rot_axis);
    tf_Transform(rot_axis);
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
/*
void init_trajectory(){

    //init trajectory

    double value_z_pos = 0.04;
    double value_R_angle = degrees_to_radian(10);
    for(int i=0;i!=const_;i++){
        nozzle_frames.disZ_spuit = (value_z_pos/const_)*i;
        nozzle_frames.angleR_head = -(value_R_angle/const_)*i;
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }
    double value_P_angle = degrees_to_radian(360);
    for(int i=0; i!=(const_*3); i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle/(const_*3));
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration((time_interval*3)/(const_*3)).sleep();
    }
    for(int i=0; i!=const_;i++){
        nozzle_frames.angleR_head = nozzle_frames.angleR_head + (value_R_angle/const_);
        nozzle_frames.disZ_spuit = nozzle_frames.disZ_spuit - (value_z_pos/const_);
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }
    double value_x_pos = 0.12;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos/const_);
        spuit_geometry_Transform();
        //object_geometry_Transform();
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
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }
    double value_P_angle_base = degrees_to_radian(360);
    for(int i=0;i!=(const_*3);i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base/(const_*3));
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration((time_interval*3)/(const_*3)).sleep();
    }
    for(int i=0;i!=(const_*3*10);i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base/(const_*3));
        nozzle_frames.disX_head = nozzle_frames.disX_head + (value_x_pos/(const_*3*10));
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration((time_interval*3*10)/(const_*3*10)).sleep();
    }
}
*/

/*
void optional_trajectory(){
    double value_x_pos_back = -0.03;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_back/const_);
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration((time_interval*1.5)/const_).sleep();
    }
    double value_z_pos_up = 0.04;
    double value_R_angle_up = degrees_to_radian(10);
    for(int i=0;i!=const_;i++){
        nozzle_frames.disZ_spuit = (value_z_pos_up/const_)*i;
        nozzle_frames.angleR_head = -(value_R_angle_up/const_)*i;
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }
    double value_P_angle_rot = degrees_to_radian(360);
    for(int i=0; i!=(const_*3); i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_rot/(const_*3));
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration((time_interval*3)/(const_*3)).sleep();
    }
    for(int i=0; i!=const_;i++){
        nozzle_frames.angleR_head = nozzle_frames.angleR_head + (value_R_angle_up/const_);
        nozzle_frames.disZ_spuit = nozzle_frames.disZ_spuit - (value_z_pos_up/const_);
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }
    double value_x_pos_into_obj = 0.15;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_into_obj/const_);
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration((time_interval*1.5)/const_).sleep();
    }
    double value_Y_angle_spuit_inside_comp = degrees_to_radian(9);
    double value_Y_pos_spuit_inside_comp = 0.02;
    for(int i=0;i!=const_;i++){
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit + (value_Y_angle_spuit_inside_comp/const_);
        nozzle_frames.disY_head = nozzle_frames.disY_head - (value_Y_pos_spuit_inside_comp/const_);
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }
    double value_P_angle_base_rot_base = degrees_to_radian(360);
    for(int i=0;i!=(const_*3);i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base_rot_base/(const_*3));
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration((time_interval*3)/(const_*3)).sleep();
    }
    for(int i=0;i!=(const_*3*10);i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base_rot_base/(const_*3));
        nozzle_frames.disX_head = nozzle_frames.disX_head + (value_x_pos_into_obj/(const_*3*10));
        spuit_geometry_Transform();
        //object_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        ros::Duration((time_interval*3*10)/(const_*3*10)).sleep();
    }
}
*/

//5-DOF
void most_optimal_trajectory(){

    //moving nozzle backwards
    double value_x_pos_back = -0.03;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_back/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*0.75)/const_).sleep();
    }

    // moving nozzle upwards + rotate in upwards movement
    double value_z_pos_up = 0.04;
    double value_R_angle_up = degrees_to_radian(10);
    for(int i=0;i!=const_;i++){
        nozzle_frames.disZ_spuit = (value_z_pos_up/const_)*i;
        nozzle_frames.angleR_head = -(value_R_angle_up/const_)*i;
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval*0.75/const_).sleep();
    }

    // rotate nozzle 360 degrees (positioned in angled way)
    double value_P_angle_rot = degrees_to_radian(360);
    for(int i=0; i!=(const_*3); i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_rot/(const_*3));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*3)/(const_*3)).sleep();
    }

    // moving nozzle to middle line
    for(int i=0; i!=const_;i++){
        nozzle_frames.angleR_head = nozzle_frames.angleR_head + (value_R_angle_up/const_);
        nozzle_frames.disZ_spuit = nozzle_frames.disZ_spuit - (value_z_pos_up/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval*0.75/const_).sleep();
    }

    // moving nozzle into the object
    double value_x_pos_into_obj = 0.135;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_into_obj/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval)/const_).sleep();
    }

    // put nozzle in angled position to clean most inner surface
    double value_Y_angle_spuit_inside_comp = degrees_to_radian(-1.5);
    double value_Y_angle_rot_axis_init = degrees_to_radian(15);
    double value_Y_pos_spuit_inside_comp = 0.035;
    double value_Y_pos_frame_ins_comp = 0.01;
    for(int i=0;i!=const_;i++){
        nozzle_frames.angleY_rot_axis = nozzle_frames.angleY_rot_axis + (value_Y_angle_rot_axis_init/const_);
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit + (value_Y_angle_spuit_inside_comp/const_);
        nozzle_frames.disY_head = nozzle_frames.disY_head - (value_Y_pos_spuit_inside_comp/const_);
        //nozzle_frames.disY_rot_axis = nozzle_frames.disY_rot_axis + (value_Y_pos_frame_ins_comp/const_);
        nozzle_frames.disY_spuit = nozzle_frames.disY_spuit + (value_Y_pos_frame_ins_comp/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }

    // rotate to clean spot that most hard reachable
    double value_P_angle_base_rot_base = degrees_to_radian(360);
    for(int i=0;i!=(const_*3);i++){
        nozzle_frames.angleP_rot_axis = nozzle_frames.angleP_rot_axis + (value_P_angle_base_rot_base/(const_*3));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*1.5)/(const_*3)).sleep();
    }

    // moving out of the elbow (1)
    double value_x_pos_out_obj_1 = 0.05;
    for(int i=0;i!=(const_*4);i++){
        nozzle_frames.disX_spuit = nozzle_frames.disX_spuit + (value_x_pos_out_obj_1/(const_*4));
        nozzle_frames.angleP_rot_axis = nozzle_frames.angleP_rot_axis + (value_P_angle_base_rot_base/(const_));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*0.5)/const_*4).sleep();
    }
    for(int i=0;i!=const_;i++){
        nozzle_frames.angleY_rot_axis = nozzle_frames.angleY_rot_axis - (value_Y_angle_rot_axis_init/const_);
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit - (value_Y_angle_spuit_inside_comp/const_);
        nozzle_frames.disY_head = nozzle_frames.disY_head + (value_Y_pos_spuit_inside_comp/const_);
        //nozzle_frames.disY_rot_axis = nozzle_frames.disY_rot_axis + (value_Y_pos_frame_ins_comp/const_);
        nozzle_frames.disY_spuit = nozzle_frames.disY_spuit - (value_Y_pos_frame_ins_comp/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }

    // move out of the elbow (2)

    //moving nozzle a little bit forward
    double value_x_pos_forw = 0.01;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_forw/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*0.75)/const_).sleep();
    }

    // angled positioned to move out the last part
    double value_Y_angle_spuit_2 = degrees_to_radian(-1.5);
    double value_Y_pos_spuit_2 = -0.0125;
    for(int i=0;i!=const_;i++){
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit + (value_Y_angle_spuit_2/const_);
        nozzle_frames.disY_head = nozzle_frames.disY_head - (value_Y_pos_spuit_2/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }

    // rotate + move out of object to clean first part of the object
     double value_x_pos_into_obj_2 = 0.105;
    for(int i=0;i!=(const_*3*10);i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base_rot_base/(const_*3));
        nozzle_frames.disX_head = nozzle_frames.disX_head + (value_x_pos_into_obj_2/(const_*3*10));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*3*10*0.65)/(const_*3*10)).sleep();
    }
}

void DOF2_movement(){
    nozzle_frames.angleY_spuit = degrees_to_radian(8);
    nozzle_frames.angleY_rot_axis = degrees_to_radian(-10);
    nozzle_frames.disY_rot_axis = 0.015;


    //moving nozzle backwards
    double value_x_pos_back = -0.03;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_back/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*0.75)/const_).sleep();
    }

    // rotate nozzle 360 degrees
    double value_P_angle_rot = degrees_to_radian(360);
    for(int i=0; i!=(const_*3); i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_rot/(const_*3));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*3)/(const_*3)).sleep();
    }

    // moving nozzle into the object
    double value_x_pos_into_obj = 0.145;
    double value_P_angle_base_rot_base = degrees_to_radian(360);
    for(int i=0;i!=(const_*10);i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_into_obj/(const_*10));
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base_rot_base/(const_));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*10*2.2)/(const_*10)).sleep();
    }

}

void DOF3_movement(){
    nozzle_frames.angleY_spuit = degrees_to_radian(8);
    nozzle_frames.angleY_rot_axis = degrees_to_radian(-10);
    nozzle_frames.disY_rot_axis = 0.015;

    //moving nozzle backwards
    double value_x_pos_back = -0.03;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_back/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*0.75)/const_).sleep();
    }

    // rotate nozzle 360 degrees
    double value_P_angle_rot = degrees_to_radian(360);
    for(int i=0; i!=(const_*3); i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_rot/(const_*3));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*3)/(const_*3)).sleep();
    }

    // moving nozzle into the object
    double value_x_pos_into_obj = 0.145;
    double value_P_angle_base_rot_base = degrees_to_radian(360);
    double added_distance = 0.0009;
    double added_distance2 = 0.00075;
    double added_angle = degrees_to_radian(0.085);
    for(int i=0;i!=(const_*10);i++){
        nozzle_frames.disY_rot_axis = nozzle_frames.disY_rot_axis + (added_distance*sin(i / const_ * 2 * M_PI));
        nozzle_frames.disY_spuit = nozzle_frames.disY_spuit - (added_distance2*sin(i / const_ * 2 * M_PI));
        nozzle_frames.angleY_rot_axis = nozzle_frames.angleY_rot_axis - (added_angle*sin(i / const_ * 2 * M_PI));
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_into_obj/(const_*10));
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base_rot_base/(const_));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*10*2.5)/(const_*10)).sleep();
    }
}

void DOF4_movement(){

    nozzle_frames.angleY_spuit = degrees_to_radian(8);
    nozzle_frames.angleY_rot_axis = degrees_to_radian(-10);
    nozzle_frames.disY_rot_axis = 0.015;

    //moving nozzle backwards
    double value_x_pos_back = -0.03;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_back/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*0.75)/const_).sleep();
    }

    // rotate nozzle 360 degrees
    double value_P_angle_rot = degrees_to_radian(360);
    for(int i=0; i!=(const_*3); i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_rot/(const_*3));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*3)/(const_*3)).sleep();
    }
    /*
    //moving nozzle backwards
    double value_x_pos_back = -0.03;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_back/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*0.75)/const_).sleep();
    }

    // put nozzle in angled position to clean most inner surface
    double value_Y_angle_spuit_inside_comp = degrees_to_radian(-1.5);
    double value_Y_angle_rot_axis_init = degrees_to_radian(15);
    double value_Y_pos_spuit_inside_comp = 0.035;
    double value_Y_pos_frame_ins_comp = 0.01;
    for(int i=0;i!=const_;i++){
        nozzle_frames.angleY_rot_axis = nozzle_frames.angleY_rot_axis + (value_Y_angle_rot_axis_init/const_);
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit + (value_Y_angle_spuit_inside_comp/const_);
        //nozzle_frames.disY_head = nozzle_frames.disY_head - (value_Y_pos_spuit_inside_comp/const_);
        nozzle_frames.disY_spuit = nozzle_frames.disY_spuit + (value_Y_pos_frame_ins_comp/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }

    // rotate to clean spot that most hard reachable
    double value_P_angle_base_rot_base = degrees_to_radian(360);
    for(int i=0;i!=(const_*3);i++){
        nozzle_frames.angleP_head = nozzle_frames.angleP_head + (value_P_angle_base_rot_base/(const_*3));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*1.85)/(const_*3)).sleep();
    }

    // moving nozzle to middle line
    for(int i=0; i!=const_;i++){
        nozzle_frames.angleY_rot_axis = nozzle_frames.angleY_rot_axis - (value_Y_angle_rot_axis_init/const_);
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit - (value_Y_angle_spuit_inside_comp/const_);
        //nozzle_frames.disY_head = nozzle_frames.disY_head + (value_Y_pos_spuit_inside_comp/const_);
        nozzle_frames.disY_spuit = nozzle_frames.disY_spuit - (value_Y_pos_frame_ins_comp/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }
*/
    // moving nozzle to middle line
    for(int i=0; i!=const_;i++){
        nozzle_frames.angleY_rot_axis = nozzle_frames.angleY_rot_axis + (degrees_to_radian(10)/const_);
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit - (degrees_to_radian(8)/const_);
        nozzle_frames.disY_rot_axis = nozzle_frames.disY_rot_axis - (0.015/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }

    // moving nozzle into the object
    double value_x_pos_into_obj = 0.135;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_into_obj/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval)/const_).sleep();
    }

    // put nozzle in angled position to clean most inner surface
    double value_Y_angle_spuit_inside_comp = degrees_to_radian(-1.5);
    double value_Y_angle_rot_axis_init = degrees_to_radian(15);
    double value_Y_pos_spuit_inside_comp = 0.035;
    double value_Y_pos_frame_ins_comp = 0.01;
    for(int i=0;i!=const_;i++){
        nozzle_frames.angleY_rot_axis = nozzle_frames.angleY_rot_axis + (value_Y_angle_rot_axis_init/const_);
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit + (value_Y_angle_spuit_inside_comp/const_);
        nozzle_frames.disY_head = nozzle_frames.disY_head - (value_Y_pos_spuit_inside_comp/const_);
        nozzle_frames.disY_spuit = nozzle_frames.disY_spuit + (value_Y_pos_frame_ins_comp/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }

    // rotate to clean spot that most hard reachable
    double value_P_angle_base_rot_base = degrees_to_radian(360);
    for(int i=0;i!=(const_*3);i++){
        nozzle_frames.angleP_rot_axis = nozzle_frames.angleP_rot_axis + (value_P_angle_base_rot_base/(const_*3));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*1.5)/(const_*3)).sleep();
    }

    // moving out of the elbow (1)
    double value_x_pos_out_obj_1 = 0.05;
    for(int i=0;i!=(const_*4);i++){
        nozzle_frames.disX_spuit = nozzle_frames.disX_spuit + (value_x_pos_out_obj_1/(const_*4));
        nozzle_frames.angleP_rot_axis = nozzle_frames.angleP_rot_axis + (value_P_angle_base_rot_base/(const_));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*0.5)/const_*4).sleep();
    }
    for(int i=0;i!=const_;i++){
        nozzle_frames.angleY_rot_axis = nozzle_frames.angleY_rot_axis - (value_Y_angle_rot_axis_init/const_);
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit - (value_Y_angle_spuit_inside_comp/const_);
        nozzle_frames.disY_head = nozzle_frames.disY_head + (value_Y_pos_spuit_inside_comp/const_);
        //nozzle_frames.disY_rot_axis = nozzle_frames.disY_rot_axis + (value_Y_pos_frame_ins_comp/const_);
        nozzle_frames.disY_spuit = nozzle_frames.disY_spuit - (value_Y_pos_frame_ins_comp/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }

    // move out of the elbow (2)

    //moving nozzle a little bit forward
    double value_x_pos_forw = 0.01;
    for(int i=0;i!=const_;i++){
        nozzle_frames.disX_head = nozzle_frames.disX_head - (value_x_pos_forw/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*0.75)/const_).sleep();
    }

    // angled positioned to move out the last part
    double value_Y_angle_spuit_2 = degrees_to_radian(-1.5);
    double value_Y_pos_spuit_2 = -0.0125;
    for(int i=0;i!=const_;i++){
        nozzle_frames.angleY_spuit = nozzle_frames.angleY_spuit + (value_Y_angle_spuit_2/const_);
        nozzle_frames.disY_head = nozzle_frames.disY_head - (value_Y_pos_spuit_2/const_);
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration(time_interval/const_).sleep();
    }

    // rotate + move out of object to clean first part of the object
     double value_x_pos_into_obj_2 = 0.105;
    for(int i=0;i!=(const_*3*10);i++){
        nozzle_frames.angleP_base = nozzle_frames.angleP_base + (value_P_angle_base_rot_base/(const_*3));
        nozzle_frames.disX_head = nozzle_frames.disX_head + (value_x_pos_into_obj_2/(const_*3*10));
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();
        ros::Duration((time_interval*3*10*0.65)/(const_*3*10)).sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "spraygun_control_node");
    ros::NodeHandle nh;
    std_msgs::Empty flip_msg;

    //fix_obj = nh.advertise<geometry_msgs::TransformStamped>("/fix_obj",1);
    pos = nh.advertise<geometry_msgs::TransformStamped>("/spuit_pos",1);
    head = nh.advertise<geometry_msgs::TransformStamped>("/spuit_head",1);
    rot = nh.advertise<geometry_msgs::TransformStamped>("/rot_axis",1);
    base = nh.advertise<geometry_msgs::TransformStamped>("/spuit_base",1);
    ros::Publisher send_flip_command = nh.advertise<std_msgs::Empty>("/flip_cloud", 1);

    while(ros::ok()){
        spuit_geometry_Transform();
        base_geometry_Transform();
        head_geometry_Transform();
        rotation_axis_geometry_Transform();

        nozzle_frames = empty_instance;

        //init_trajectory();
        //optional_trajectory();

        //DOF2_movement();
        //DOF3_movement();
        DOF4_movement();
        //most_optimal_trajectory(); //5DOF
        send_flip_command.publish(flip_msg);

    }
    return 0;
}
