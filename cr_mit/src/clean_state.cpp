#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>

struct boundaries{
    double beg_x;
    double end_x;
    double beg_y;
    double end_y;
    double beg_z;
    double end_z;
};

struct boundaries bounds;
bool DO = true;
double d = 0;
sensor_msgs::PointCloud surface_cloud, workspace_cloud, surface_cloud_map, workspace_cloud_map, cleaned_surface;
tf::StampedTransform transform_wws, transform_surf;
ros::Publisher pub_cleaned_surface;

void surfaceCallback(const sensor_msgs::PointCloud& msg){
    surface_cloud = msg;
}

void workspaceCallback(const sensor_msgs::PointCloud& msg){
    workspace_cloud = msg;
}

void findBounds(){
    d = workspace_cloud.points[1].x;
    for(size_t i=0; i<(workspace_cloud.points.size()-1); ++i){
        workspace_cloud_map.points.resize(workspace_cloud.points.size()-1);
        workspace_cloud_map.points[i].x = workspace_cloud.points[i+1].x + transform_wws.getOrigin().x();
        workspace_cloud_map.points[i].y = workspace_cloud.points[i+1].y + transform_wws.getOrigin().y();
        workspace_cloud_map.points[i].z = workspace_cloud.points[i+1].z + transform_wws.getOrigin().z();
    }
    bounds.beg_x = transform_wws.getOrigin().x() - d;
    bounds.end_x = transform_wws.getOrigin().x() + d;
    bounds.beg_y = transform_wws.getOrigin().y() - d;
    bounds.end_y = transform_wws.getOrigin().y() + d;
    bounds.beg_z = transform_wws.getOrigin().z() - d;
    bounds.end_z = transform_wws.getOrigin().z() + d;
    /*ROS_INFO_STREAM("boundaries begx "<<bounds.beg_x);
    ROS_INFO_STREAM("boundaries begy "<<bounds.beg_y);
    ROS_INFO_STREAM("boundaries begz "<<bounds.beg_z);
    ROS_INFO_STREAM("boundaries endx "<<bounds.end_x);
    ROS_INFO_STREAM("boundaries endy "<<bounds.end_y);
    ROS_INFO_STREAM("boundaries endz "<<bounds.end_z);*/
    ROS_INFO_STREAM("delta value "<<d);
}

void findCleanedSurface(){
    for(size_t i=0; i<surface_cloud_map.points.size(); ++i){
        if((surface_cloud_map.points[i].y >= bounds.beg_y)&&(surface_cloud_map.points[i].y <= bounds.end_y)){
            if((surface_cloud_map.points[i].x >= bounds.beg_x)&&(surface_cloud_map.points[i].x <= bounds.end_x)){
                if((surface_cloud_map.points[i].z >= bounds.beg_z)&&(surface_cloud_map.points[i].z <= bounds.end_z)){
                    double x_b = surface_cloud_map.points[i].x - transform_surf.getOrigin().x();
                    double y_b = surface_cloud_map.points[i].y - transform_surf.getOrigin().y();
                    double z_b = surface_cloud_map.points[i].z - transform_surf.getOrigin().z();
                    if(abs(cbrt((x_b*x_b)+(y_b*y_b)+(z_b*z_b))) <= d){
                        cleaned_surface.header.stamp = ros::Time::now();
                        cleaned_surface.header.frame_id = "cleaned_surface";
                        cleaned_surface.channels.resize(1);
                        cleaned_surface.channels[0].name = "intensities";
                        cleaned_surface.channels[0].values.resize(surface_cloud_map.points.size());
                        cleaned_surface.points.resize(surface_cloud_map.points.size());
                        //data
                        cleaned_surface.points[i].x = surface_cloud.points[i].x;
                        cleaned_surface.points[i].y = surface_cloud.points[i].y;
                        cleaned_surface.points[i].z = surface_cloud.points[i].z;
                        cleaned_surface.channels[0].values[i] = 50;
                        pub_cleaned_surface.publish(cleaned_surface);
                        ROS_INFO("A part of the surface is now cleaned");
                    }
                }
            }
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "clean_state_determination");
    ros::NodeHandle nh;

    tf::TransformListener listener_wws, listener_surf;
    ros::Subscriber sub_surface = nh.subscribe("/surface_cloud",1,surfaceCallback);
    ros::Subscriber sub_workspace = nh.subscribe("/water_wss_cloud",1,workspaceCallback);
    pub_cleaned_surface = nh.advertise<sensor_msgs::PointCloud>("/cleaned_surface", 10);

    ros::Rate rate(10.0);
    ros::spinOnce();

    while(ros::ok()){

        if(!surface_cloud.points.empty()&&!workspace_cloud.points.empty()){
            if(DO == true){
                try{
                    listener_surf.lookupTransform("/map", "/inner_surface", ros::Time(0), transform_surf);
                    DO = false;
                }
                catch (tf::TransformException &ex){
                    ROS_WARN("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    DO = true;
                }
                for(size_t i=0; i<surface_cloud.points.size(); ++i){
                    surface_cloud_map.points.resize(surface_cloud.points.size());
                    surface_cloud_map.points[i].x = surface_cloud.points[i].x + transform_surf.getOrigin().x();
                    surface_cloud_map.points[i].y = surface_cloud.points[i].y + transform_surf.getOrigin().y();
                    surface_cloud_map.points[i].z = surface_cloud.points[i].z + transform_surf.getOrigin().z();
                }
            }
            try{
                listener_wws.lookupTransform("/map", "/water_workspace", ros::Time(0), transform_wws);
                //algorithms
                findBounds();
                findCleanedSurface();
            }
            catch (tf::TransformException &ex){
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        else{
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ROS_INFO("NO data??");
        }
        rate.sleep();
    }
    return 0;
}
