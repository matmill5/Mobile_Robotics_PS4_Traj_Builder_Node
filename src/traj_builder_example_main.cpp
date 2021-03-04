#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <des_state_publisher_service/ServiceMsg.h>
#include <string>
#include <iostream>

//global vars, equivalent to member vars of class
geometry_msgs::Twist g_halt_twist;
nav_msgs::Odometry g_end_state;
nav_msgs::Odometry g_start_state;
geometry_msgs::PoseStamped g_start_pose;
geometry_msgs::PoseStamped g_end_pose;

void do_inits() { //similar to a constructor  
    //define a halt state; zero speed and spin, and fill with viable coords
    g_halt_twist.linear.x = 0.0;
    g_halt_twist.linear.y = 0.0;
    g_halt_twist.linear.z = 0.0;
    g_halt_twist.angular.x = 0.0;
    g_halt_twist.angular.y = 0.0;
    g_halt_twist.angular.z = 0.0;

    //default values; can be overridden
    g_start_state.twist.twist = g_halt_twist;
    g_start_state.pose.pose.position.x = 0;
    g_start_state.pose.pose.position.y = 0;
    g_start_state.pose.pose.position.z = 0;
}

void halt_mode() {
    //define a halt state; zero speed and spin, and fill with viable coords
    g_halt_twist.linear.x = 0.0;
    g_halt_twist.linear.y = 0.0;
    g_halt_twist.linear.z = 0.0;
    g_halt_twist.angular.x = 0.0;
    g_halt_twist.angular.y = 0.0;
    g_halt_twist.angular.z = 0.0;
}

void forward_travel_mode(){
    //define a forward travel state; one speed and zero spin, and fill with viable coords
    g_halt_twist.linear.x = 1.0;
    g_halt_twist.linear.y = 0.0;
    g_halt_twist.linear.z = 0.0;
    g_halt_twist.angular.x = 0.0;
    g_halt_twist.angular.y = 0.0;
    g_halt_twist.angular.z = 0.0;
}

void spin_mode(){
    //define a spin-in-place state; zero speed and one spin, and fill with viable coords
    g_halt_twist.linear.x = 0.0;
    g_halt_twist.linear.y = 0.0;
    g_halt_twist.linear.z = 0.0;
    g_halt_twist.angular.x = 0.0;
    g_halt_twist.angular.y = 0.0;
    g_halt_twist.angular.z = 1.0;
}

bool gracefulBrakingTrajCallback(){
    // Publish a graceful braking traj
}

bool desiredStateServiceCallback(des_state_publisher_service::ServiceMsgRequest& request, des_state_publisher_service::ServiceMsgRequest& response){
    ROS_INFO("Des_State_Publisher_Service Callback Activated.");
    //Request Types
    //geometry_msgs::PoseStamped start_pose,
    //geometry_msgs::PoseStamped end_pose,
    //Response Type
    //std::vector<nav_msgs::Odometry> &vec_of_states
    geometry_msgs::PoseStamped start_pos = request.start_pos;
    geometry_msgs::PoseStamped goal_pos = request.goal_pos;
    
    // Reasons for failure - encounter a lidar alarm
    // Failure to converge on the goal within some tolerance
    // Initialize response to failed (expect it will fail)
    response.success = false;

    // Subscribe to lidar
    // 03/02/2021 - Matthew
    lidar_alarm_subscriber = n.subscribe("mobot/lidar", 1, lidarAlarmCallback);

    double dt = 0.02;
    //will stream (publish) desired states to this topic with this message type:
    ros::Publisher des_state_publisher = n.advertise<nav_msgs::Odometry>("/desState", 1);
    ros::Publisher des_psi_publisher = n.advertise<std_msgs::Float64>("/desPsi", 1);
    //can use this to drive a robot around, open loop:
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    ros::Rate looprate(1 / dt); //timer for fixed publication rate   
    TrajBuilder trajBuilder; //instantiate one of these
    trajBuilder.set_dt(dt); //make sure trajectory builder and main use the same time step
    trajBuilder.set_alpha_max(1.0);
    //hard code two poses; more generally, would get poses from a nav_msgs/Path message.
    double psi_start = 0.0;
    double psi_end = 0.0; //3.0;
    g_start_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
    g_end_state = g_start_state;
    g_end_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);

    g_start_pose.pose.position.x = 0.0;
    g_start_pose.pose.position.y = 0.0;
    g_start_pose.pose.position.z = 0.0;
    g_start_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
    g_end_pose = g_start_pose; //includes copying over twist with all zeros
    //don't really care about orientation, since this will follow from 
    // point-and-go trajectory; 
    g_end_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);
    g_end_pose.pose.position.x = 5.0; //set goal coordinates
    g_end_pose.pose.position.y = 0.0; //-4.0;

    double des_psi;
    std_msgs::Float64 psi_msg;
    std::vector<nav_msgs::Odometry> vec_of_states;
    //trajBuilder.build_triangular_spin_traj(g_start_pose,g_end_pose,vec_of_states);
    //trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);

    nav_msgs::Odometry des_state;
    nav_msgs::Odometry last_state;
    geometry_msgs::PoseStamped last_pose;

    /*timing test: takes about 1.3 msec to compute a point-and-go trajectory
    ROS_INFO("timing test: constructing 10000 point-and-go trajectories");
    for (int ipaths=0;ipaths<10000;ipaths++) {
        trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);
    }
    ROS_INFO("done computing trajectories");
     * */

    // main loop; publish a desired state every iteration    
    while (ros::ok()) {
        ROS_INFO("building traj from start to end");
        trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);
        ROS_INFO("publishing desired states and open-loop cmd_vel");
        for (int i = 0; i < vec_of_states.size(); i++) {
            des_state = vec_of_states[i];
            des_state.header.stamp = ros::Time::now();
            des_state_publisher.publish(des_state);
            des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
            psi_msg.data = des_psi;
            des_psi_publisher.publish(psi_msg);
            twist_commander.publish(des_state.twist.twist); //FOR OPEN-LOOP CTL ONLY!

            looprate.sleep(); //sleep for defined sample period, then do loop again
        }
        ROS_INFO("building traj from end to start");
        last_state = vec_of_states.back();
        last_pose.header = last_state.header;
        last_pose.pose = last_state.pose.pose;
        trajBuilder.build_point_and_go_traj(last_pose, g_start_pose, vec_of_states);
        for (int i = 0; i < vec_of_states.size(); i++) {
            des_state = vec_of_states[i];
            des_state.header.stamp = ros::Time::now();
            des_state_publisher.publish(des_state);
            des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
            psi_msg.data = des_psi;
            des_psi_publisher.publish(psi_msg);
            twist_commander.publish(des_state.twist.twist); //FOR OPEN-LOOP CTL ONLY!
            looprate.sleep(); //sleep for defined sample period, then do loop again
        }
        last_state = vec_of_states.back();
        g_start_pose.header = last_state.header;
        g_start_pose.pose = last_state.pose.pose;
    }

}

ros::Subscriber lidar_alarm_subscriber;
bool lidarAlarmCallback(std_msgs::Bool lidar_alarm, des_state_publisher_service::ServiceMsgRequest& response){
    if(lidar_alarm.data == true){
        response.success == false;
        return lidar_alarm.data;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher_service");
    ros::NodeHandle n;

    ros::ServiceServer desiredStatePublisherService = n.advertiseService("desStatePubServ", desiredStateServiceCallback);
    ROS_INFO("Ready to command build trajectories for desired positions.");
    ros::spin();
}

